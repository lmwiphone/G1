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
find_package(sensor_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)

include_directories(
  include
  include/g1_nav_bridge/unitree
  include/g1_nav_bridge/unitree/common
  include/nlohmann
)

add_executable(cmdvel_to_sport src/cmdvel_to_sport.cpp)
target_compile_features(cmdvel_to_sport PRIVATE cxx_std_20)
target_include_directories(cmdvel_to_sport PRIVATE
  include
  include/g1_nav_bridge/unitree
  include/g1_nav_bridge/unitree/common
  include/nlohmann
)
ament_target_dependencies(cmdvel_to_sport
  geometry_msgs rclcpp unitree_api)

add_executable(battery_state_bridge src/battery_state_bridge.cpp)
ament_target_dependencies(battery_state_bridge
  rclcpp sensor_msgs unitree_hg)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS cmdvel_to_sport battery_state_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
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
install(FILES third_party/unitree_ros2/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME unitree_ros2-BSD-3-Clause.txt)

ament_export_include_directories(include)

ament_package()
