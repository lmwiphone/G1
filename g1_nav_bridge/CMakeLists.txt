cmake_minimum_required(VERSION 3.8)
project(g1_nav_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)

# 直接使用工作区同级 unitree_ros2-master 中的官方 G1 LocoClient ROS 封装。
# 也可通过 -DUNITREE_ROS2_EXAMPLE_INCLUDE_DIR=... 显式指定。
set(UNITREE_ROS2_EXAMPLE_INCLUDE_DIR "" CACHE PATH
  "Path to unitree_ros2 example/src/include")
if(NOT UNITREE_ROS2_EXAMPLE_INCLUDE_DIR)
  foreach(UNITREE_ROS2_INCLUDE_CANDIDATE
      "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_ros2-master/example/src/include"
      "/home/unitree/unitree_ros2/example/src/include"
      "/opt/G1/lighting_ws/unitree_ros2-master/example/src/include")
    if(EXISTS "${UNITREE_ROS2_INCLUDE_CANDIDATE}/g1/g1_loco_client.hpp")
      set(UNITREE_ROS2_EXAMPLE_INCLUDE_DIR
        "${UNITREE_ROS2_INCLUDE_CANDIDATE}" CACHE PATH
        "Path to unitree_ros2 example/src/include" FORCE)
      break()
    endif()
  endforeach()
endif()
if(NOT EXISTS "${UNITREE_ROS2_EXAMPLE_INCLUDE_DIR}/g1/g1_loco_client.hpp")
  message(FATAL_ERROR
    "Official unitree_ros2 g1_loco_client.hpp was not found. "
    "Set UNITREE_ROS2_EXAMPLE_INCLUDE_DIR to example/src/include.")
endif()
message(STATUS "Using official Unitree ROS2 headers: ${UNITREE_ROS2_EXAMPLE_INCLUDE_DIR}")

include_directories(include)

add_executable(cmdvel_to_sport src/cmdvel_to_sport.cpp)
target_compile_features(cmdvel_to_sport PRIVATE cxx_std_20)
target_include_directories(cmdvel_to_sport PRIVATE
  "${UNITREE_ROS2_EXAMPLE_INCLUDE_DIR}"
  "${UNITREE_ROS2_EXAMPLE_INCLUDE_DIR}/common"
  "${UNITREE_ROS2_EXAMPLE_INCLUDE_DIR}/nlohmann")
ament_target_dependencies(cmdvel_to_sport
  geometry_msgs rclcpp rmw std_msgs std_srvs unitree_api)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS cmdvel_to_sport
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS g1_nav_bridge/sport_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sport_to_odom)
install(PROGRAMS g1_nav_bridge/tf_to_current_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_to_current_pose)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md ODOMETRY.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
