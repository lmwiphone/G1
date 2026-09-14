#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <g1/g1_loco_client.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace g1_nav_bridge {

using namespace std::chrono_literals;

class CmdVelToSport final : public rclcpp::Node {
 public:
  CmdVelToSport() : Node("g1_cmdvel_to_sport"), client_(this) {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    declare_parameter<std::string>("emergency_stop_topic", "/g1/emergency_stop");
    declare_parameter<std::string>("status_topic", "/g1_cmdvel_to_sport/status");
    declare_parameter<double>("rate_hz", 30.0);
    declare_parameter<double>("timeout_s", 0.20);
    declare_parameter<double>("max_vx", 1.0);
    declare_parameter<double>("max_vy", 0.0);
    declare_parameter<double>("max_wz", 0.25);
    declare_parameter<double>("max_ax", 1.0);
    declare_parameter<double>("max_ay", 0.35);
    declare_parameter<double>("max_awz", 0.60);
    declare_parameter<bool>("enabled_on_start", false);
    declare_parameter<std::vector<int64_t>>(
        "allowed_fsm_ids", std::vector<int64_t>{500, 501, 801, 802});

    rate_hz_ = get_parameter("rate_hz").as_double();
    timeout_s_ = get_parameter("timeout_s").as_double();
    limits_ = {get_parameter("max_vx").as_double(),
               get_parameter("max_vy").as_double(),
               get_parameter("max_wz").as_double()};
    accel_limits_ = {get_parameter("max_ax").as_double(),
                     get_parameter("max_ay").as_double(),
                     get_parameter("max_awz").as_double()};
    allowed_fsm_ids_ = get_parameter("allowed_fsm_ids").as_integer_array();
    validate_parameters();

    // 官方 BaseClient::Call() 会同步等待 /api/sport/response。外部回调放在
    // 独立 callback group；官方临时 response subscription 使用默认 group。
    external_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = external_callback_group_;

    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_topic").as_string(), rclcpp::QoS(10),
        std::bind(&CmdVelToSport::on_cmd_vel, this, std::placeholders::_1),
        subscription_options);
    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
        get_parameter("emergency_stop_topic").as_string(), rclcpp::QoS(10),
        std::bind(&CmdVelToSport::on_emergency_stop, this,
                  std::placeholders::_1),
        subscription_options);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        get_parameter("status_topic").as_string(), rclcpp::QoS(10));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "~/enable",
        std::bind(&CmdVelToSport::on_enable, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, external_callback_group_);
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop",
        std::bind(&CmdVelToSport::on_stop, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, external_callback_group_);
    status_timer_ = create_wall_timer(
        500ms, std::bind(&CmdVelToSport::publish_status, this),
        external_callback_group_);

    requested_enable_on_start_ = get_parameter("enabled_on_start").as_bool();
    last_tick_ = std::chrono::steady_clock::now();
    // 与官方 loco_client_example 一样，RPC 在工作线程执行，executor 专门处理 ROS 回调。
    command_thread_ = std::thread(&CmdVelToSport::command_loop, this);

    const std::string rmw_identifier = rmw_get_implementation_identifier();
    if (rmw_identifier != "rmw_cyclonedds_cpp") {
      RCLCPP_ERROR(
          get_logger(),
          "Current RMW is %s, but Unitree requires rmw_cyclonedds_cpp. "
          "Restart after exporting RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
          rmw_identifier.c_str());
    }
    RCLCPP_WARN(
        get_logger(),
        "G1 bridge uses official unitree_ros2 LocoClient on "
        "/api/sport/request and /api/sport/response; RMW=%s; no FSM transition is performed",
        rmw_identifier.c_str());
  }

  ~CmdVelToSport() override {
    stopping_.store(true);
    command_cv_.notify_all();
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
    // executor 停止后不能再同步等待 response。正常禁用、急停和 watchdog
    // 都会调用 StopMove()；Move() 的官方默认有效期为 1 秒。
  }

 private:
  static bool is_zero(const std::array<double, 3>& velocity) {
    return velocity[0] == 0.0 && velocity[1] == 0.0 && velocity[2] == 0.0;
  }

  static double clamp(double value, double limit) {
    return limit > 0.0 ? std::clamp(value, -limit, limit) : 0.0;
  }

  void validate_parameters() const {
    const auto positive_finite = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    if (!positive_finite(rate_hz_) || !positive_finite(timeout_s_)) {
      throw std::invalid_argument("rate_hz and timeout_s must be positive and finite");
    }
    for (double value : limits_) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(
            "velocity limits must be finite and non-negative");
      }
    }
    for (double value : accel_limits_) {
      if (!positive_finite(value)) {
        throw std::invalid_argument(
            "acceleration limits must be positive and finite");
      }
    }
    if (allowed_fsm_ids_.empty()) {
      throw std::invalid_argument("allowed_fsm_ids must not be empty");
    }
  }

  void reset_command_state_locked() {
    target_.fill(0.0);
    output_.fill(0.0);
    have_command_ = false;
    moving_ = false;
    last_tick_ = std::chrono::steady_clock::now();
  }

  int32_t call_stop_move() {
    std::lock_guard<std::mutex> client_lock(client_mutex_);
    return client_.StopMove();
  }

  int32_t call_move(const std::array<double, 3>& velocity) {
    std::lock_guard<std::mutex> client_lock(client_mutex_);
    return client_.Move(static_cast<float>(velocity[0]),
                        static_cast<float>(velocity[1]),
                        static_cast<float>(velocity[2]));
  }

  bool fsm_allows_motion(int* fsm_out) {
    int fsm_id = -1;
    int32_t result = 0;
    {
      std::lock_guard<std::mutex> client_lock(client_mutex_);
      result = client_.GetFsmId(fsm_id);
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      last_fsm_id_ = fsm_id;
      last_sdk_result_ = result;
    }
    if (fsm_out != nullptr) {
      *fsm_out = fsm_id;
    }
    if (result != 0) {
      RCLCPP_ERROR(get_logger(), "official LocoClient GetFsmId failed: %d",
                   result);
      return false;
    }
    return std::find(allowed_fsm_ids_.begin(), allowed_fsm_ids_.end(),
                     fsm_id) != allowed_fsm_ids_.end();
  }

  void record_stop_result(int32_t result) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_sdk_result_ = result;
    output_.fill(0.0);
    moving_ = false;
  }

  void disable_and_stop() {
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled_ = false;
      reset_command_state_locked();
    }
    command_cv_.notify_all();
    record_stop_result(call_stop_move());
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->angular.z)) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      target_.fill(0.0);
      have_command_ = false;
      RCLCPP_ERROR(get_logger(), "Rejected non-finite cmd_vel");
      return;
    }

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      target_ = {clamp(msg->linear.x, limits_[0]),
                 clamp(msg->linear.y, limits_[1]),
                 clamp(msg->angular.z, limits_[2])};
      last_cmd_ = std::chrono::steady_clock::now();
      have_command_ = true;
    }
    command_cv_.notify_all();
  }

  void on_enable(const std_srvs::srv::SetBool::Request::SharedPtr request,
                 std_srvs::srv::SetBool::Response::SharedPtr response) {
    if (!request->data) {
      disable_and_stop();
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      response->success = last_sdk_result_ == 0;
      response->message =
          last_sdk_result_ == 0
              ? "disabled; official LocoClient StopMove succeeded"
              : "disabled but StopMove failed: " +
                    std::to_string(last_sdk_result_);
      return;
    }

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      reset_command_state_locked();
      if (emergency_stop_latched_) {
        enabled_ = false;
        response->success = false;
        response->message =
            "enable rejected; /g1/emergency_stop is latched. Publish false, "
            "then enable again";
        return;
      }
    }

    const auto request_subscribers = count_subscribers("/api/sport/request");
    const auto response_publishers = count_publishers("/api/sport/response");
    if (request_subscribers == 0 || response_publishers == 0) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled_ = false;
      response->success = false;
      response->message =
          "enable rejected; Unitree sport service endpoints are missing: "
          "request_subscribers=" +
          std::to_string(request_subscribers) + " response_publishers=" +
          std::to_string(response_publishers);
      return;
    }

    int fsm_id = -1;
    if (!fsm_allows_motion(&fsm_id)) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled_ = false;
      response->success = false;
      response->message =
          "enable rejected; current FSM is unavailable or not motion-capable: " +
          std::to_string(fsm_id);
      return;
    }

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled_ = true;
      last_tick_ = std::chrono::steady_clock::now();
    }
    command_cv_.notify_all();
    response->success = true;
    response->message = "enabled; waiting for a new cmd_vel; fsm_id=" +
                        std::to_string(fsm_id);
  }

  void on_stop(const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
    disable_and_stop();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    response->success = last_sdk_result_ == 0;
    response->message =
        last_sdk_result_ == 0
            ? "official LocoClient StopMove succeeded; bridge disabled"
            : "StopMove failed: " + std::to_string(last_sdk_result_);
  }

  void on_emergency_stop(const std_msgs::msg::Bool::SharedPtr message) {
    if (!message->data) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      emergency_stop_latched_ = false;
      enabled_ = false;
      reset_command_state_locked();
      RCLCPP_WARN(get_logger(),
                  "Software emergency-stop latch cleared; bridge remains disabled");
      return;
    }

    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      emergency_stop_latched_ = true;
    }
    disable_and_stop();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (last_sdk_result_ == 0) {
      RCLCPP_ERROR(get_logger(),
                   "Software emergency stop latched; StopMove succeeded");
    } else {
      RCLCPP_FATAL(get_logger(),
                   "Software emergency stop latched but StopMove failed: %d",
                   last_sdk_result_);
    }
  }

  void command_loop() {
    // 官方例程同样先等待 executor 开始 spin，再在工作线程调用 LocoClient。
    std::this_thread::sleep_for(500ms);

    if (requested_enable_on_start_ && !stopping_.load()) {
      int fsm_id = -1;
      const bool allowed = fsm_allows_motion(&fsm_id);
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled_ = allowed && !emergency_stop_latched_;
      if (!enabled_) {
        RCLCPP_ERROR(get_logger(),
                     "enabled_on_start rejected; current fsm_id=%d", fsm_id);
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    std::unique_lock<std::mutex> wait_lock(wait_mutex_);
    while (!stopping_.load()) {
      command_cv_.wait_for(wait_lock, period,
                           [this] { return stopping_.load(); });
      if (stopping_.load()) {
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      std::array<double, 3> command{};
      bool should_stop = false;
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!enabled_ || emergency_stop_latched_) {
          continue;
        }

        const bool fresh =
            have_command_ &&
            std::chrono::duration<double>(now - last_cmd_).count() <= timeout_s_;
        if (!fresh || is_zero(target_)) {
          should_stop = moving_;
          output_.fill(0.0);
          moving_ = false;
        } else {
          const double dt = std::clamp(
              std::chrono::duration<double>(now - last_tick_).count(), 0.0,
              0.1);
          for (std::size_t index = 0; index < output_.size(); ++index) {
            const double max_delta = accel_limits_[index] * dt;
            const double error = target_[index] - output_[index];
            output_[index] += std::clamp(error, -max_delta, max_delta);
          }
          command = output_;
        }
        last_tick_ = now;
      }

      if (should_stop) {
        const int32_t result = call_stop_move();
        record_stop_result(result);
        if (result != 0) {
          RCLCPP_ERROR(get_logger(), "watchdog StopMove failed: %d", result);
        }
        continue;
      }
      if (is_zero(command)) {
        continue;
      }

      const int32_t result = call_move(command);
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_sdk_result_ = result;
        moving_ = result == 0;
        if (result != 0) {
          enabled_ = false;
          output_.fill(0.0);
        }
      }
      if (result != 0) {
        RCLCPP_ERROR(get_logger(),
                     "official LocoClient Move failed: %d; bridge disabled",
                     result);
        const int32_t stop_result = call_stop_move();
        if (stop_result != 0) {
          RCLCPP_FATAL(get_logger(),
                       "StopMove after Move failure also failed: %d", stop_result);
        }
      }
    }
  }

  void publish_status() {
    bool enabled = false;
    bool emergency_stop_latched = false;
    bool fresh_command = false;
    int fsm_id = -1;
    int32_t sdk_result = 0;
    std::array<double, 3> output{};
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      enabled = enabled_;
      emergency_stop_latched = emergency_stop_latched_;
      fresh_command =
          have_command_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_cmd_)
                  .count() <= timeout_s_;
      fsm_id = last_fsm_id_;
      sdk_result = last_sdk_result_;
      output = output_;
    }

    std_msgs::msg::String message;
    message.data =
        "enabled=" + std::string(enabled ? "true" : "false") +
        " estop_latched=" +
        std::string(emergency_stop_latched ? "true" : "false") +
        " fresh_cmd=" + std::string(fresh_command ? "true" : "false") +
        " fsm_id=" + std::to_string(fsm_id) +
        " sdk_result=" + std::to_string(sdk_result) + " output=[" +
        std::to_string(output[0]) + "," + std::to_string(output[1]) + "," +
        std::to_string(output[2]) + "]";
    status_publisher_->publish(message);
  }

  unitree::robot::g1::LocoClient client_;
  rclcpp::CallbackGroup::SharedPtr external_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      emergency_stop_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::thread command_thread_;
  std::atomic<bool> stopping_{false};
  std::condition_variable command_cv_;
  std::mutex wait_mutex_;
  std::mutex client_mutex_;
  std::mutex state_mutex_;

  bool requested_enable_on_start_{false};
  bool enabled_{false};
  bool emergency_stop_latched_{false};
  bool have_command_{false};
  bool moving_{false};
  double rate_hz_{30.0};
  double timeout_s_{0.20};
  int last_fsm_id_{-1};
  int32_t last_sdk_result_{0};
  std::array<double, 3> limits_{};
  std::array<double, 3> accel_limits_{};
  std::array<double, 3> target_{};
  std::array<double, 3> output_{};
  std::vector<int64_t> allowed_fsm_ids_;
  std::chrono::steady_clock::time_point last_cmd_{};
  std::chrono::steady_clock::time_point last_tick_{};
};

}  // namespace g1_nav_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<g1_nav_bridge::CmdVelToSport>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_cmdvel_to_sport"), "%s",
                 error.what());
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
