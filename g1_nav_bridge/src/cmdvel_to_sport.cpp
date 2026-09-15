#include <array>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/response.hpp>

#include "g1_nav_bridge/unitree/g1_loco_client.hpp"

namespace g1_nav_bridge {

class CmdVelToSport final : public rclcpp::Node {
 public:
  CmdVelToSport() : Node("g1_cmdvel_to_sport"), client_(this) {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    declare_parameter<double>("duration", 0.5);
    duration_ = get_parameter("duration").as_double();
    if (!std::isfinite(duration_) || duration_ <= 0.0) {
      throw std::invalid_argument("duration must be positive and finite");
    }

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_topic").as_string(), 10,
        std::bind(&CmdVelToSport::OnCmdVel, this, std::placeholders::_1));
    response_subscription_ = create_subscription<unitree_api::msg::Response>(
        "/api/sport/response", rclcpp::QoS(1),
        [this](const unitree_api::msg::Response::SharedPtr response) {
          if (response->header.identity.api_id !=
              ROBOT_API_ID_LOCO_SET_VELOCITY) {
            return;
          }
          if (response->header.status.code == 0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "SetVelocity response: id=%lld code=0",
                static_cast<long long>(response->header.identity.id));
          } else {
            RCLCPP_ERROR(get_logger(),
                         "SetVelocity response: id=%lld code=%d",
                         static_cast<long long>(response->header.identity.id),
                         response->header.status.code);
          }
        });
    worker_ = std::thread(&CmdVelToSport::Run, this);
  }

  ~CmdVelToSport() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr message) {
    std::array<float, 3> velocity{
        static_cast<float>(message->linear.x),
        static_cast<float>(message->linear.y),
        static_cast<float>(message->angular.z)};
    if (!std::isfinite(velocity[0]) || !std::isfinite(velocity[1]) ||
        !std::isfinite(velocity[2])) {
      RCLCPP_ERROR(get_logger(), "Invalid cmd_vel; sending zero velocity");
      velocity.fill(0.0F);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      velocity_ = velocity;
      pending_ = true;
    }
    condition_.notify_one();
  }

  void Run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      condition_.wait(lock, [this] { return stopping_ || pending_; });
      if (stopping_) {
        return;
      }

      const auto velocity = velocity_;
      pending_ = false;
      lock.unlock();
      const auto ret = client_.SetVelocity(
          velocity[0], velocity[1], velocity[2],
          static_cast<float>(duration_));
      if (ret != 0) {
        RCLCPP_ERROR(get_logger(), "SetVelocity failed: %d", ret);
      }
      lock.lock();
    }
  }

  ::unitree::robot::g1::LocoClient client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr
      response_subscription_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::array<float, 3> velocity_{};
  double duration_{0.5};
  bool pending_{false};
  bool stopping_{false};
};

}  // namespace g1_nav_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_nav_bridge::CmdVelToSport>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
