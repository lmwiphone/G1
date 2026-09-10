/******************************************************************************
 * Copyright (c) 2023 dongfang chen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of chassis_comm nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <boost/asio.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "aid_robot_msgs/srv/control_launch.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "aid_robot_msgs/srv/get_string.hpp"
#include "aid_robot_msgs/srv/map_operation.hpp"
#include "aid_robot_msgs/srv/status_change.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "lightning/srv/save_map.hpp"
#include <filesystem>

using namespace std::chrono_literals;
using namespace std::placeholders;

class StatusManagerNode : public rclcpp::Node {
 private:
  std::string maping_launch_file =
      "/robot_bringup/share/robot_bringup/launch/g1_mapping.launch.py";
  std::string localization_launch_file =
      "/robot_bringup/share/robot_bringup/launch/g1_localization.launch.py";
  std::string navigation_launch_file =
      "/aid_navigation2/share/aid_navigation2/launch/navigation2.launch.py";
  std::string map_filepath_;
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string set_status_;
  std::string control_model_;
  std::string slam_status_;
  std::string map_filename_;
  bool navigation_started_ = false;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Service<aid_robot_msgs::srv::StatusChange>::SharedPtr
      status_change_server_;
  rclcpp::Service<aid_robot_msgs::srv::MapOperation>::SharedPtr
      map_save_server_;
  rclcpp::Client<aid_robot_msgs::srv::ControlLaunch>::SharedPtr   
   start_launch_client_;
  rclcpp::Client<aid_robot_msgs::srv::ControlLaunch>::SharedPtr  
   stop_launch_client_;
  rclcpp::Client<lightning::srv::SaveMap>::SharedPtr lightning_save_map_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;

  rclcpp::Client<aid_robot_msgs::srv::GetCurrentMap>::SharedPtr get_current_map_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      init_pose_sub_;
  rclcpp::Service<aid_robot_msgs::srv::GetString>::SharedPtr ip_server_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_map_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
      lifecycle_manager_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      lifecycle_navigation_is_active_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motion_bridge_enable_client_;
  void GetIpHandleRequest(
      const std::shared_ptr<aid_robot_msgs::srv::GetString::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::GetString::Response> response) {
    (void)request;

    std::string ip_address = GetIpAddress();

    if (ip_address.empty()) {
      response->success = false;
      response->message = "Failed to get IP address";
      RCLCPP_ERROR(get_logger(), "Failed to get IP address");
    } else {
      response->success = true;
      response->message = "Success";
      response->result = ip_address;
      RCLCPP_INFO(get_logger(), "IP Address: %s", ip_address.c_str());
    }
  }

  bool ChangeMap(const std::string& map_filepath) {

    if (!change_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(node_->get_logger(), "change map service not available");
      return false;
    }
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_filepath;

    auto future = change_map_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(node_, future);

    auto status = future.get()->result;
    if (status != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Change map request failed!");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Change map request was successful!");
      return true;
    }
    return true;
  }

  std::string GetIpAddress() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    std::string ip;
    if (sockfd == -1) {
      perror("socket");
      return "";
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == -1) {
      perror("ioctl");
      close(sockfd);
      return "";
    }

    close(sockfd);

    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    char ip_address[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr->sin_addr, ip_address, INET_ADDRSTRLEN);
    ip.append(ip_address);
    return ip;
  }

  bool GetCurrentMap(std::string &filename) {

    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentMap::Request>();

    if (false == get_current_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(
          rclcpp::get_logger("get_current_map_client"),
          "Wait finish_trajectory failed,service not available, return ...");
      return false;
    }
    auto result = get_current_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_current_map_client"),
                         "message:success");
      auto service_response = result.get();
      if(service_response->success==false){
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "get current map failed");
        return false;
      }

      if(service_response->map_file.empty()){
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "map file name is empty");
        return false;
      }
      filename = service_response->map_file;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_client"),
                   "Failed to call service get_current_map_idp");
      return false;
    }
    return true;
  }

  bool IsLightningMapComplete(const std::string &directory) const {
    const std::filesystem::path path(directory);
    return std::filesystem::exists(path / "index.txt") &&
           std::filesystem::exists(path / "map.yaml");
  }

  bool LightningSaveMap(const std::string &requested_path) {
    std::filesystem::path path(requested_path);
    std::string map_id = path.filename().string();
    if (map_id.empty()) {
      map_id = path.parent_path().filename().string();
    }
    if (map_id.empty() || map_id == "." || map_id == "..") {
      RCLCPP_ERROR(get_logger(), "Invalid map path: %s", requested_path.c_str());
      return false;
    }

    if (!lightning_save_map_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /lightning/save_map is unavailable");
      return false;
    }
    auto request = std::make_shared<lightning::srv::SaveMap::Request>();
    request->map_id = map_id;
    auto result = lightning_save_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(60)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Timed out while saving Lightning map %s", map_id.c_str());
      return false;
    }
    if (result.get()->response != 0) {
      RCLCPP_ERROR(get_logger(), "Lightning map save failed, code=%u", result.get()->response);
      return false;
    }

    const std::filesystem::path target =
        std::filesystem::path("/opt/G1/maps") / map_id / "lightning";
    if (!std::filesystem::exists(target / "index.txt") ||
        !std::filesystem::exists(target / "map.yaml")) {
      RCLCPP_ERROR(get_logger(), "Lightning returned success but map files are incomplete: %s",
                   target.c_str());
      return false;
    }

    const char *home = std::getenv("HOME");
    if (home == nullptr || requested_path.empty() || requested_path.front() != '/') {
      RCLCPP_ERROR(get_logger(), "Legacy map path must start with '/': %s", requested_path.c_str());
      return false;
    }
    const std::filesystem::path alias = std::string(home) + requested_path;
    std::error_code ec;
    std::filesystem::create_directories(alias.parent_path(), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Cannot create map parent directory: %s", ec.message().c_str());
      return false;
    }
    if (std::filesystem::is_symlink(alias, ec)) {
      std::filesystem::remove(alias, ec);
    } else if (std::filesystem::exists(alias, ec)) {
      RCLCPP_ERROR(get_logger(), "Refusing to replace existing non-symlink path: %s", alias.c_str());
      return false;
    }
    std::filesystem::create_directory_symlink(target, alias, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Cannot create map compatibility link: %s", ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Map compatibility path: %s -> %s", alias.c_str(), target.c_str());
    return true;
  }
  // Legacy map loading helpers were removed. Lightning localization loads its
  // tiled map and Nav2's map_server owns the 2D occupancy map.

  void TimerCallback() {  // RCLCPP_INFO(this->get_logger(), "Timer callback");
    std_msgs::msg::String status;
    status.data = slam_status_ + "+" + control_model_;
    status_pub_->publish(status);
  }

  void InitPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header = pose->header;
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose = pose->pose;
    initial_pose.pose.covariance[0] = 0.25;
    initial_pose.pose.covariance[7] = 0.25;
    initial_pose.pose.covariance[35] = 0.0685;
    initial_pose_pub_->publish(initial_pose);
  }

  bool EnableMotionBridge(bool enable) {
    if (!motion_bridge_enable_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "G1 motion bridge enable service is unavailable");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    auto result = motion_bridge_enable_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(3)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "G1 motion bridge enable request timed out");
      return false;
    }
    return result.get()->success;
  }

  bool ControlNavigation(uint8_t command) {

    auto request =
        std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = command;

    if (!lifecycle_manager_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
      return false;
    }

    auto result = lifecycle_manager_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                          std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed");
      return false;
    }
    return result.get()->success;
  }

  bool PauseNavigation() {
    return ControlNavigation(
        nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE);
  }

  bool ResumeNavigation() {
    return ControlNavigation(
        nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME);
  }
  bool IsNavigationActive() {
    auto request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    if (!lifecycle_navigation_is_active_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
      return false;
    }

    auto result = lifecycle_navigation_is_active_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                          std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed");
      return false;
    }
    return result.get()->success;
  }

  void SaveMapCallback(
      const std::shared_ptr<aid_robot_msgs::srv::MapOperation::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::MapOperation::Response> response) {
    if (slam_status_ == "mapping") {
      if (!LightningSaveMap(request->map_file_name)) {
        response->message = "lightning map save failed";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           response->message);
        response->success = false;
        return;
      }

      StopLaunch(maping_launch_file);
      slam_status_ = "idle";
      response->success = true;
      return;
    }

    response->message = "must change mode to mapping first";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       response->message);
    response->success = false;
  }
  /**
   * @brief 设置机器人工作模式
   *
   * 根据传入的模式字符串设置机器人的工作模式，并启动或停止相应的服务。
   *
   * @param status 模式字符串，可选值为 "mapping", "localization", "patrol", "remote_control", "idle"
   * @return 如果模式设置成功，则返回 true；否则返回 false
   */
  bool ModeSet(std::string status) {
    bool run_status = false;
    set_status_ = status;
    if (set_status_ == "mapping") {
      EnableMotionBridge(false);
      if (slam_status_ == "localization") {
        StopLaunch(localization_launch_file);
      } else {
        StopLaunch(maping_launch_file);
      }
      if (navigation_started_ && IsNavigationActive()) {
        PauseNavigation();
      }
      

      run_status = StartLaunch(maping_launch_file);
      if (run_status == true) {
        slam_status_ = set_status_;
      }

    } else if (set_status_ == "localization") {
      EnableMotionBridge(false);
      if (navigation_started_ && IsNavigationActive()) {
        PauseNavigation();
      }
      if (slam_status_ == "mapping") {
        StopLaunch(maping_launch_file);
      } else {
        StopLaunch(localization_launch_file);
      }

      if (!GetCurrentMap(map_filename_) || !IsLightningMapComplete(map_filename_)) {
        RCLCPP_ERROR(get_logger(),
                     "No complete Lightning map is selected; mapping must be saved first");
        return false;
      }
      run_status = StartLaunch(localization_launch_file,
                                "map_dir:=" + map_filename_);
      if (run_status == true) {
        slam_status_ = set_status_;
        if (!navigation_started_) {
          navigation_started_ = StartLaunch(
              navigation_launch_file, "map:=" + map_filename_ + "/map.yaml");
          if (!navigation_started_) {
            RCLCPP_ERROR(get_logger(), "Failed to start Nav2 for selected map");
            return false;
          }
        }
      }
      
    } else if (set_status_ == "patrol") {
      
      if (slam_status_ == "localization") {
        run_status = ChangeMap(map_filename_+"/map.yaml");
        if (run_status == true) {
          if (!IsNavigationActive() && !ResumeNavigation()) {
            RCLCPP_ERROR(get_logger(), "Failed to resume Nav2");
            return false;
          }
          run_status = EnableMotionBridge(true);
          if (run_status) {
            control_model_ = set_status_;
          }
        }
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                           "can not set to patrol model");
      }
    } else if (set_status_ == "remote_control") {
      run_status = EnableMotionBridge(true);
      if (run_status) {
        control_model_ = set_status_;
      }
    } else if (set_status_ == "idle") {
      EnableMotionBridge(false);
      if (navigation_started_ && IsNavigationActive()) {
        PauseNavigation();
      }
      StopLaunch(maping_launch_file);
      control_model_ = set_status_;
      run_status = true;
    }
    return run_status;
  }
  /**
   * @brief 设置模式回调函数
   *
   * 根据传入的请求，设置机器人的运行状态，并更新响应信息。
   *
   * @param request 模式设置请求，包含要设置的状态
   * @param response 模式设置响应，包含操作结果信息
   */
  void ModeSetCallback(
      const std::shared_ptr<aid_robot_msgs::srv::StatusChange::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::StatusChange::Response> response) {
    bool run_status = false;
    std::string set_status = request->action;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager"),
                       "set model to" << set_status);
    run_status = ModeSet(set_status);
    if (run_status)
      response->message = "ok";
    else
      response->message = "err";
    return;
  }

 public:
  StatusManagerNode() : Node("robot_status_manager_node") {
    control_model_ = "idle";
    node_ = rclcpp::Node::make_shared("robot_status_server_manager_node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    const std::string bringup_share =
        ament_index_cpp::get_package_share_directory("robot_bringup");
    map_filepath_ = bringup_share + "/maps";
    maping_launch_file = bringup_share + "/launch/g1_mapping.launch.py";
    localization_launch_file = bringup_share + "/launch/g1_localization.launch.py";
    navigation_launch_file =
        ament_index_cpp::get_package_share_directory("aid_navigation2") +
        "/launch/navigation2.launch.py";
    status_change_server_ =
        nh_->create_service<aid_robot_msgs::srv::StatusChange>(
            "mode_set",
            std::bind(&StatusManagerNode::ModeSetCallback, this, _1, _2));
    ip_server_ = nh_->create_service<aid_robot_msgs::srv::GetString>(
        "get_ip_addresses",
        std::bind(&StatusManagerNode::GetIpHandleRequest, this, _1, _2));
    map_save_server_ = nh_->create_service<aid_robot_msgs::srv::MapOperation>(
        "aid_save_map",
        std::bind(&StatusManagerNode::SaveMapCallback, this, _1, _2));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StatusManagerNode::TimerCallback, this));

    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "aid_init_pose", 10,
        std::bind(&StatusManagerNode::InitPoseCallback, this,
                  std::placeholders::_1));

    status_pub_ =
        nh_->create_publisher<std_msgs::msg::String>("robot_status", 10);
    initial_pose_pub_ = nh_->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    lifecycle_manager_client_ =
      node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
          "/lifecycle_manager_navigation/manage_nodes");
    lifecycle_navigation_is_active_client_ = 
      node_->create_client<std_srvs::srv::Trigger>(
          "/lifecycle_manager_navigation/is_active");
    motion_bridge_enable_client_ =
      node_->create_client<std_srvs::srv::SetBool>(
          "/g1_cmdvel_to_sport/enable");
    start_launch_client_ =
        node_->create_client<aid_robot_msgs::srv::ControlLaunch>("start_launch");
    start_launch_client_ = node_->create_client<aid_robot_msgs::srv::ControlLaunch>(
      "start_launch",
      rmw_qos_profile_services_default, 
      callback_group_
    );

    stop_launch_client_ =
        node_->create_client<aid_robot_msgs::srv::ControlLaunch>("stop_launch");  
    lightning_save_map_client_ =
        node_->create_client<lightning::srv::SaveMap>("/lightning/save_map");
    get_current_map_client_ =
        node_->create_client<aid_robot_msgs::srv::GetCurrentMap>("get_current_map_id");
    change_map_client_ =
        node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    slam_status_ = "idle";
    

    const bool have_lightning_map =
        GetCurrentMap(map_filename_) && IsLightningMapComplete(map_filename_);
    if (have_lightning_map) {
      const bool run_status = StartLaunch(localization_launch_file,
                                          "map_dir:=" + map_filename_);
      if (run_status) {
        slam_status_ = "localization";
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to start Lightning localization: %s",
                     map_filename_.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(),
                  "No complete Lightning map is selected; starting idle for first mapping");
      map_filename_ = map_filepath_ + "/default";
    }

    if (have_lightning_map) {
      navigation_started_ = StartLaunch(
          navigation_launch_file, "map:=" + map_filename_ + "/map.yaml");
      if (!navigation_started_) {
        RCLCPP_ERROR(get_logger(), "Navigation launch failed");
      }
    }
    // Physical G1 must not accept velocity commands merely because bringup
    // completed. The frontend explicitly selects patrol or remote_control to
    // enable the Unitree motion bridge.
    EnableMotionBridge(false);
    if (navigation_started_ && IsNavigationActive()) {
      PauseNavigation();
    }
    RCLCPP_INFO(this->get_logger(), "robot_status init success");

  }

  bool StartLaunch(std::string file, std::string param = "") {
  while (!start_launch_client_->wait_for_service(std::chrono::seconds(5))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_status_manager_node"), "Interruped while waiting for the server.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("robot_status_manager_node"), "Server not available, waiting again...");
    }
    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = file;
    request->parameter = param;
    auto future = start_launch_client_->async_send_request(request);
    auto result = callback_group_executor_->spin_until_future_complete(future);
    //处理返回结果
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "Received response from start_launch service. result: "<< response->success
                << ", message: " << response->message);
      if(response->success){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "start launch success!:  " << response->message);
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_status_manager_node"), "start launch failed!" << response->message);
        return false;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("robot_status_manager_node"), "start launch time out!");
      return false;
    }
    return true;
  }
  bool StopLaunch(std::string file) {

    auto request =
        std::make_shared<aid_robot_msgs::srv::ControlLaunch::Request>();
    request->launch_file = file;

    if (!stop_launch_client_->wait_for_service(5s)) {
      RCLCPP_INFO(rclcpp::get_logger("stop_launch_client"),
                  "service stop_launch not available");
      return false;
    }

    auto result = stop_launch_client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("stop_launch_client"),
                         "message:" << result.get()->message);
      rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stop_launch_client"),
                   "Failed to call service stop_launch");
      return false;
    }
    return true;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusManagerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
