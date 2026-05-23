cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. Add the executable
add_executable(arm_moveit_node src/arm_moveit_node.cpp)

# 3. USE THIS INSTEAD OF target_link_libraries
# This macro handles all the "::" target resolution for you
ament_target_dependencies(arm_moveit_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)

# 4. Install the executable
install(TARGETS arm_moveit_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()