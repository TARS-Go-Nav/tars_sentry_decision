project(tars_decision)
cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tars_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

add_executable(behavior_test_node
    behavior_test.cpp
)

ament_target_dependencies(behavior_test_node
  rclcpp rclcpp_action nav2_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs std_msgs tars_msgs message_filters auto_aim_interfaces visualization_msgs cv_bridge OpenCV
)

target_include_directories(behavior_test_node PUBLIC ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(behavior_test_node ${TINYXML2_LIBRARIES})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS behavior_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
