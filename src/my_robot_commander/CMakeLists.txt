cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(
  test_moveit rclcpp moveit_ros_planning_interface)

add_executable(test2_moveit src/test2_moveit.cpp)
ament_target_dependencies(
  test2_moveit rclcpp moveit_ros_planning_interface)

add_executable(test3_moveit src/test2_moveit.cpp)
ament_target_dependencies(
  test3_moveit rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander.cpp)
ament_target_dependencies(
  commander rclcpp moveit_ros_planning_interface )


install(
  TARGETS test_moveit test2_moveit test3_moveit commander
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

