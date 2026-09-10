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
        if (base_frame_ == lidar_frame_ || base_frame_.empty() || lidar_frame_.empty()) {
            throw std::runtime_error("base_frame and lidar_frame must be distinct, nonempty frame IDs");
        }
        static_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        static_tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
            [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { CacheStaticExtrinsic(*msg); });
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { PublishBaseTF(pose); });
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
        const auto transform = static_tf_buffer_->lookupTransform(
            base_frame_, lidar_frame_, tf2::TimePointZero);
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        Quatd q(r.w, r.x, r.y, r.z);
        const Vec3d translation(t.x, t.y, t.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !translation.allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid static LiDAR extrinsic; TF output disabled");
            return;
        }
        lidar_to_base_ = SE3(q.normalized(), translation).inverse();
        extrinsic_ready_ = true;
        RCLCPP_INFO(node_->get_logger(), "Cached static extrinsic %s <- %s; base TF output enabled",
                    base_frame_.c_str(), lidar_frame_.c_str());
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
    const SE3 map_to_base = SE3(q.normalized(), translation) * extrinsic;
    auto output = lidar_pose;  // Preserve algorithm timestamp and map frame.
    output.child_frame_id = base_frame_;
    output.transform.translation.x = map_to_base.translation().x();
    output.transform.translation.y = map_to_base.translation().y();
    output.transform.translation.z = map_to_base.translation().z();
    const auto rotation = map_to_base.unit_quaternion();
    output.transform.rotation.x = rotation.x();
    output.transform.rotation.y = rotation.y();
    output.transform.rotation.z = rotation.z();
    output.transform.rotation.w = rotation.w();
    tf_broadcaster_->sendTransform(output);
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
