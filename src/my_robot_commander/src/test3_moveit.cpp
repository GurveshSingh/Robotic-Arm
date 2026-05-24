#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::msg::PoseStamped latest_pose;
bool pose_received = false;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10,
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            latest_pose = *msg;
            pose_received = true;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){ executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        if (pose_received) {
            pose_received = false;

            arm.setStartStateToCurrentState();
            arm.setPoseTarget(latest_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan1;
            bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                arm.execute(plan1);
            } else {
                RCLCPP_WARN(node->get_logger(), "Planning failed!");
            }
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}