#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){ executor.spin(); });

    //as we need two threads so if we normally spin any one thread
    //the code gets stuck there hence use executor to spin

    auto arm = moveit::planning_interface::MoveGroupInterface(node,"arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    std::vector<double> joints = { 0.5,0.0,0.5};

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool sucess = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    //executing
    
    if (sucess) {
        arm.execute(plan1);
    }





    rclcpp::shutdown();
    spinner.join();
    return 0;
}

