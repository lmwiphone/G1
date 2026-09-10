#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <deque>

class TransformPublisher : public rclcpp::Node
{
public:
    TransformPublisher() : Node("transform_publisher"), max_length_(100.0), path_distance_interval_(0.1)
    {
        this->declare_parameter<double>("max_length", 100.0);
        max_length_ = this->get_parameter("max_length").as_double();
        this->declare_parameter<double>("path_distance_interval", 0.1);
        path_distance_interval_ = this->get_parameter("path_distance_interval").as_double();
        

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link_pose", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/robot_path", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TransformPublisher::publish_transformed_pose, this));
        path_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TransformPublisher::publish_path, this));
        
        clear_path_service_ = this->create_service<std_srvs::srv::Trigger>(
            "clear_path",
            std::bind(&TransformPublisher::handle_clear_path, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void publish_transformed_pose()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_out = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePoint());

            geometry_msgs::msg::PoseStamped pose_out;
            pose_out.header.frame_id = "map";
            pose_out.header.stamp = this->now();
            pose_out.pose.position.x = transform_out.transform.translation.x;
            pose_out.pose.position.y = transform_out.transform.translation.y;
            pose_out.pose.position.z = transform_out.transform.translation.z;
            pose_out.pose.orientation = transform_out.transform.rotation;

            pose_publisher_->publish(pose_out);
            record_path_point(pose_out);
        }
        catch (tf2::TransformException &ex)
        {
            static auto clock = rclcpp::Clock(RCL_ROS_TIME);
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 50000,
                         "Failed to lookup transform: %s", ex.what());
        }
    }

    void record_path_point(const geometry_msgs::msg::PoseStamped& pose)
    {
        if (path_.empty() || distance(pose.pose.position, path_.back().pose.position) >= path_distance_interval_)
        {
            path_.push_back(pose);

            if (path_.size() > 1)
            {
                double total_length = calculate_path_length();
                while (total_length > max_length_ && path_.size() > 1)
                {
                    path_.pop_front();
                    total_length = calculate_path_length();
                }
            }
        }
    }

    double calculate_path_length()
    {
        double length = 0.0;
        for (size_t i = 1; i < path_.size(); ++i)
        {
            length += distance(path_[i].pose.position, path_[i - 1].pose.position);
        }
        return length;
    }

    double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void publish_path()
    {
        if (!path_.empty())
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = this->now();
            path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>(path_.begin(), path_.end());

            path_publisher_->publish(path_msg);
        }
    }

    void handle_clear_path(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "Clearing path history");
        path_.clear();
        response->success = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_path_service_;

    std::deque<geometry_msgs::msg::PoseStamped> path_;
    double max_length_;
    double path_distance_interval_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
