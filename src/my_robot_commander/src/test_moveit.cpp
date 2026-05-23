#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread{[&executor]() { executor.spin(); }};

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

//    // Named Goal
//-----------------------------------------------------------------------------
//    arm.setStartStateToCurrentState();
//    arm.setNamedTarget("pose1");
//
//    moveit::planning_interface::MoveGroupInterface::Plan plan1;
//    
//    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
//
//    if (success1) {
//        arm.execute(plan1);
//    } else {
//        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
//    }
//
//    arm.setStartStateToCurrentState();
//    arm.setNamedTarget("home");
//
//    moveit::planning_interface::MoveGroupInterface::Plan plan2;
//    
//    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
//
//    if (success2) {
//        arm.execute(plan2);
//    } else {
//        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
//    }
//----------------------------------------------------------------------------------

//// joint goal
//-------------------------------------------------------------------------------

//std::vector<double> joints = { 1.5,1.5,1.5};
//arm.setStartStateToCurrentState();
//arm.setJointValueTarget(joints);
//
//moveit::planning_interface::MoveGroupInterface::Plan plan1;
//bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
//
//   if (success1) {
//       arm.execute(plan1);
//   } else {
//       RCLCPP_ERROR(node->get_logger(), "Planning failed!");
//  }
//---------------------------------------------------------------------------

//pose goal
//-----------------------------------------------------------------------

geometry_msgs::msg::PoseStamped target_pose;
target_pose.header.frame_id = "world";

target_pose.pose.position.x = -0.527;
target_pose.pose.position.y =  0.038;
target_pose.pose.position.z =  0.123;

tf2::Quaternion q(0.802, -0.029, -0.596, -0.022);
q.normalize();

target_pose.pose.orientation.x = q.getX();
target_pose.pose.orientation.y = q.getY();
target_pose.pose.orientation.z = q.getZ();
target_pose.pose.orientation.w = q.getW();

arm.setStartStateToCurrentState();
arm.setPoseTarget(target_pose);

moveit::planning_interface::MoveGroupInterface::Plan plan1;
bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

if (success1) {
    arm.execute(plan1);
} else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
}




    rclcpp::shutdown();
    if (spinner.joinable()) {
        spinner.join();
    }
    
    return 0;
}