//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lightning_math.hpp"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    gravity_aligned_map_ = yaml["fasterlio"]["gravity_align_init"] &&
                           yaml["fasterlio"]["gravity_align_init"].as<bool>();
    map_save_root_ = yaml["system"]["map_save_root"]
                         ? yaml["system"]["map_save_root"].as<std::string>()
                         : "./data";
    if (map_save_root_ == "~" || map_save_root_.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home != nullptr) {
            map_save_root_ = std::string(home) + map_save_root_.substr(1);
        }
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        // if (options_.with_2dvisualization_) {
        //     g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
        //         cv::Mat image = map->ToCV();
        //         cv::imshow("map", image);

        //         if (options_.step_on_kf_) {
        //             cv::waitKey(0);

        //         } else {
        //             cv::waitKey(10);
        //         }
        //     });
        // }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");
        if (options_.with_gridmap_) {
            map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(1).reliable().transient_local());
        }

        // 诊断话题：把 LIO 去畸变后的当前帧摆到世界系发布出来，方便在 RViz 里
        // 直接看清 SLAM 实际拿什么点云在做配准、配准结果落在哪里。
        // 默认关闭；纯旁路输出，不回写任何算法状态。
        // 开关来自 yaml 的 system 段，而不是只靠 ROS 参数：本可执行文件用 gflags
        // 解析 argv（run_slam_online.cc 的 ParseCommandLineFlags），遇到 --ros-args
        // 这类未知 flag 会直接报错退出，因此 launch 侧无法用 "-p name:=value" 传参，
        // 只能经 --config 的 yaml 下发。仍保留 ROS 参数声明便于 ros2 param get 查看。
        // as<T>(default) 在键缺失时返回默认值，兼容老配置文件。
        pub_registered_scan_ = node_->declare_parameter<bool>(
            "pub_registered_scan", yaml["system"]["pub_registered_scan"].as<bool>(false));
        registered_scan_leaf_ = node_->declare_parameter<double>(
            "registered_scan_leaf", yaml["system"]["registered_scan_leaf"].as<double>(0.0));
        registered_scan_frame_ = node_->declare_parameter<std::string>(
            "registered_scan_frame",
            yaml["system"]["registered_scan_frame"].as<std::string>("map"));
        if (pub_registered_scan_) {
            scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/lightning/registered_scan", rclcpp::QoS(2));
            LOG(INFO) << "publishing registered scan on /lightning/registered_scan, frame="
                      << registered_scan_frame_ << ", leaf=" << registered_scan_leaf_;
        }

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);
        // IMU 200 Hz 与雷达处理共用单线程 executor，单帧雷达处理可达 100+ ms；深度 10（50 ms）
        // 会在回调被堵时丢 IMU，快转时旋转预测缺失导致航向跳变。内核 UDP 缓冲不足造成的丢包
        // 另见 robot_bringup/system/60-dds-buffers.conf（必须安装，否则队列再大也没用）。
        rclcpp::QoS imu_qos(100);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    if (g2p5_) {
        // A single callback serves both ROS and the optional OpenCV UI.
        // Capture ROS handles by value: the asynchronous callback owns their lifetime.
        auto publisher = map_pub_;
        auto clock = node_ ? node_->get_clock() : rclcpp::Clock::SharedPtr{};
        const bool show_ui = options_.with_2dvisualization_;
        const bool step = options_.step_on_kf_;
        g2p5_->SetMapUpdateCallback([publisher, clock, show_ui, step](g2p5::G2P5MapPtr map) {
            if (!map) return;
            if (publisher && clock && rclcpp::ok()) {
                auto message = map->ToROS();
                if (message.info.width > 0 && message.info.height > 0 && !message.data.empty()) {
                    message.header.frame_id = "map";
                    message.header.stamp = clock->now();  // Snapshot publication time.
                    message.info.map_load_time = message.header.stamp;
                    // ToROS supplies XY origin but leaves its quaternion at all zeros.
                    message.info.origin.orientation.w = 1.0;
                    publisher->publish(message);
                }
            }
            if (show_ui) {
                cv::imshow("map", map->ToCV());
                cv::waitKey(step ? 0 : 10);
            }
        });
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    const std::filesystem::path map_id(map_name_);
    if (map_name_.empty() || map_name_ == "." || map_name_ == ".." ||
        map_id.filename() != map_id) {
        LOG(ERROR) << "invalid map ID: " << map_name_;
        response->response = 1;
        return;
    }
    std::string save_path =
        (std::filesystem::path(map_save_root_) / map_id).string();

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = (std::filesystem::path(map_save_root_) / map_name_).string();
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    if (gravity_aligned_map_) {
        // 地图系重力上方向。定位的 lidar_loc.gravity_constrain 只在存在该文件时启用。
        std::ofstream(save_path + "/gravity_up.txt") << "0 0 1\n";
    }
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::PublishRegisteredScan() {
    if (!pub_registered_scan_ || scan_pub_ == nullptr || lio_ == nullptr) {
        return;
    }

    // scan_undistort_ 是 IMU 逐点去畸变后、位于扫描结束时刻【雷达系】的点云，
    // 也正是送进 LIO 点面配准的那份数据。
    CloudPtr scan = lio_->GetScanUndist();
    if (scan == nullptr || scan->empty()) {
        return;
    }

    // 与 Lightning 自身投影关键帧、拼全局地图时的约定保持一致：直接用 LIO 位姿
    // 左乘点。注意该约定未显式乘 lidar->IMU 外参，这里刻意不做“修正”，
    // 以保证发布的点云与 Lightning 内部地图完全同源、可直接叠加比对。
    const NavState state = lio_->GetState();
    const SE3 pose = state.GetPose();

    PointCloudType world;
    world.points.reserve(scan->size());
    const double leaf = registered_scan_leaf_;
    for (const auto& pt : scan->points) {
        const Vec3d p = pose * Vec3d(pt.x, pt.y, pt.z);
        PointType out;
        out.x = static_cast<float>(p.x());
        out.y = static_cast<float>(p.y());
        out.z = static_cast<float>(p.z());
        out.intensity = pt.intensity;
        out.time = pt.time;
        world.points.emplace_back(out);
    }

    CloudPtr cloud(new PointCloudType);
    if (leaf > 0.0) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(world.makeShared());
        voxel.filter(*cloud);
    } else {
        *cloud = world;
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = registered_scan_frame_;
    msg.header.stamp = math::FromSec(state.timestamp_);
    scan_pub_->publish(msg);
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    // 放在关键帧判定之前，保证每一帧都能看到，而不是只有关键帧才更新。
    PublishRegisteredScan();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    // 放在关键帧判定之前，保证每一帧都能看到，而不是只有关键帧才更新。
    PublishRegisteredScan();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
