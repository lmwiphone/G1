//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

    if (options_.pub_tf_) {
        base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
        lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "mid360_link");
        body_frame_ = node_->declare_parameter<std::string>("body_frame", "body_link");
        footprint_frame_ = node_->declare_parameter<std::string>("footprint_frame", "base_footprint");
        if (base_frame_ == lidar_frame_ || base_frame_.empty() || lidar_frame_.empty()) {
            throw std::runtime_error("base_frame and lidar_frame must be distinct, nonempty frame IDs");
        }
        static_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        static_tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
            [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { CacheStaticExtrinsic(*msg); });
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { PublishBaseTF(pose); });
    }

    // SLAM 建图以雷达起始位置为 map 原点，地面因此位于 z = g2p5.floor_height（约 -1.2 m），
    // 导致 base_link 落在 map 平面下方约 1.2 m。这里把 map 的 z=0 平移到地面，
    // 使 base_link 贴合地面、与 map_server 发布的 2D 栅格（origin.z 恒为 0）对齐。
    // 平面 base_link 模式下 TF 不再使用该偏移（base_link 恒贴地 z=0），仅用于 /base_link_pose 与旧 6DoF 模式。
    try {
        map_z_offset_ = -yaml.GetValue<double>("g2p5", "floor_height");
    } catch (const std::exception &) {
        map_z_offset_ = 0.0;
    }
    loc_->SetMapZOffset(map_z_offset_);
    LOG(INFO) << "map z offset (ground-referenced map frame): " << map_z_offset_;

    // 诊断话题：把配准后的点云（LIO 去畸变 + 定位位姿，与 UI 同源）发布到 map 系，
    // 便于在 RViz 里直接看 SLAM 用的是什么点云、配准结果落在哪。默认关闭。
    //
    // 开关放在 yaml 的 system.pub_registered_scan，而不是只靠 ROS 参数：
    // 本可执行文件用 gflags 解析 argv（run_loc_online.cc:20 ParseCommandLineFlags），
    // 遇到 --ros-args 这类未知 flag 会直接报错退出，所以 launch 侧无法用
    // "-p name:=value" 传参，只能经 --config 的 yaml 下发。
    // 仍保留 ROS 参数声明，便于 ros2 param get 查看当前值。
    // 键缺失时 YAML_IO::GetValue 会抛 YAML::TypedBadConversion，必须兜底。
    bool pub_scan = false;
    try {
        pub_scan = yaml.GetValue<bool>("system", "pub_registered_scan");
    } catch (const std::exception &) {
        pub_scan = false;  // 老配置文件没有该键，保持关闭
    }
    pub_scan = node_->declare_parameter<bool>("pub_registered_scan", pub_scan);
    if (pub_scan) {
        scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/registered_scan", rclcpp::QoS(2));
        loc_->SetPointcloudWorldCallback([this](const sensor_msgs::msg::PointCloud2 &cloud) {
            auto msg = cloud;
            msg.header.frame_id = lidar_frame_;  // 雷达系 + 扫描时刻，下游按该时刻查 TF
            scan_pub_->publish(msg);
        });
        LOG(INFO) << "publishing deskewed scan on /lightning/registered_scan (frame=" << lidar_frame_ << ")";
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::CacheStaticExtrinsic(const tf2_msgs::msg::TFMessage& msg) {
    std::lock_guard<std::mutex> lock(extrinsic_mutex_);
    if (extrinsic_ready_) return;  // Fixed mounting: read once per process.
    for (const auto& transform : msg.transforms) {
        static_tf_buffer_->setTransform(transform, "static_extrinsic", true);
    }
    try {
        // No timeout, and only on static-TF reception, never in the localization callback.
        // 优先 body_link（平面 base_link 模式）；旧 URDF 没有 body_link 时退回 base_link。
        const bool has_body = !body_frame_.empty() && body_frame_ != base_frame_ &&
                              static_tf_buffer_->canTransform(body_frame_, lidar_frame_, tf2::TimePointZero);
        const std::string parent = has_body ? body_frame_ : base_frame_;
        const auto transform = static_tf_buffer_->lookupTransform(
            parent, lidar_frame_, tf2::TimePointZero);
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        Quatd q(r.w, r.x, r.y, r.z);
        const Vec3d translation(t.x, t.y, t.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !translation.allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid static LiDAR extrinsic; TF output disabled");
            return;
        }
        lidar_to_base_ = SE3(q.normalized(), translation).inverse();
        planar_base_ = has_body;
        extrinsic_ready_ = true;
        if (planar_base_) {
            // base_footprint 与平面 base_link 重合（collision_monitor 使用）
            geometry_msgs::msg::TransformStamped fp;
            fp.header.stamp = node_->now();
            fp.header.frame_id = base_frame_;
            fp.child_frame_id = footprint_frame_;
            fp.transform.rotation.w = 1.0;
            static_tf_broadcaster_->sendTransform(fp);
        }
        RCLCPP_INFO(node_->get_logger(), "Cached static extrinsic %s <- %s; %s TF output enabled",
                    parent.c_str(), lidar_frame_.c_str(),
                    planar_base_ ? "planar base_link + dynamic body tilt" : "6DoF base_link");
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Waiting for static base/LiDAR extrinsic: %s", error.what());
    }
}

void LocSystem::PublishBaseTF(const geometry_msgs::msg::TransformStamped& lidar_pose) {
    SE3 extrinsic;
    {
        std::lock_guard<std::mutex> lock(extrinsic_mutex_);
        if (!extrinsic_ready_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                "Static extrinsic unavailable: NOT publishing map -> base_link");
            return;
        }
        extrinsic = lidar_to_base_;
    }
    // The algorithm output is T_map_lidar, regardless of its legacy child-frame label.
    const auto& t = lidar_pose.transform.translation;
    const auto& r = lidar_pose.transform.rotation;
    const Quatd q(r.w, r.x, r.y, r.z);
    const Vec3d translation(t.x, t.y, t.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !translation.allFinite()) return;
    // planar_base_ 时 extrinsic 为 T_lidar_body，否则为 T_lidar_base。
    const SE3 map_to_parent = SE3(q.normalized(), translation) * extrinsic;
    auto fill = [](geometry_msgs::msg::TransformStamped& msg, const SE3& T, double dz) {
        msg.transform.translation.x = T.translation().x();
        msg.transform.translation.y = T.translation().y();
        msg.transform.translation.z = T.translation().z() + dz;
        const auto r = T.unit_quaternion();
        msg.transform.rotation.x = r.x();
        msg.transform.rotation.y = r.y();
        msg.transform.rotation.z = r.z();
        msg.transform.rotation.w = r.w();
    };
    auto output = lidar_pose;  // Preserve algorithm timestamp and map frame.
    output.child_frame_id = base_frame_;
    if (!planar_base_) {
        fill(output, map_to_parent, map_z_offset_);
        tf_broadcaster_->sendTransform(output);
        return;
    }
    // 平面导航系：躯干 x 轴在地图水平面上的投影作为航向，位置取躯干原点（站立旋转中心的地面投影）。
    const Mat3d R = map_to_parent.rotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const SE3 map_to_base(SO3::rotZ(yaw), map_to_parent.translation());
    const SE3 base_to_body(map_to_base.so3().inverse() * map_to_parent.so3(), Vec3d::Zero());
    // base_link 贴地：body_link 原点按 URDF 即机器人脚下地面，因此 map 系中 z 恒为 0。
    // 固定的 map_z_offset_ 依赖建图起点 IMU 高度，每张图不同（实测 1789713472324 偏 0.19m，
    // 整片地面被 costmap 当成障碍），不能用于导航高度基准。
    fill(output, map_to_base, -map_to_base.translation().z());
    auto tilt = lidar_pose;
    tilt.header.frame_id = base_frame_;
    tilt.child_frame_id = body_frame_;
    fill(tilt, base_to_body, 0.0);
    tf_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{output, tilt});
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
