#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

class Commander : public rclcpp::Node
{
public:
    Commander() : Node("commander")
    {
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState &msg) {
                for (size_t i = 0; i < msg.name.size(); i++) {
                    if (msg.name[i] == "base_yaw_joint") q_[0] = msg.position[i];
                    if (msg.name[i] == "arm_joint1")     q_[1] = msg.position[i];
                    if (msg.name[i] == "arm_joint2")     q_[2] = msg.position[i];
                }
                got_joints_ = true;
            });

        error_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/visual_error", 10,
            std::bind(&Commander::errorCallback, this, std::placeholders::_1));

        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        Kp_x_     = 0.3;    // yaw gain — flip sign if arm goes wrong way
        Kp_y_     = -0.2;    // pitch gain — flip sign if arm goes wrong way
        deadzone_ = 0.05;

        RCLCPP_INFO(get_logger(), "Commander ready.");
    }

private:
    void publishJoints(double q0, double q1, double q2, double duration_sec = 0.5)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = now();
        traj.joint_names = {"base_yaw_joint", "arm_joint1", "arm_joint2"};

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {q0, q1, q2};
        pt.velocities = {0.0, 0.0, 0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

        traj.points.push_back(pt);
        traj_pub_->publish(traj);
    }

    void errorCallback(const geometry_msgs::msg::Point &msg)
    {
        if (!got_joints_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for joint states...");
            return;
        }

        if (msg.z < 0.5) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No bottle detected.");
            return;
        }

        double err_x = msg.x;
        double err_y = msg.y;

        if (std::abs(err_x) < deadzone_ && std::abs(err_y) < deadzone_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Centered!");
            return;
        }

        // Direct joint-space correction
        // err_x > 0 = bottle right of center → yaw positive (flip Kp_x_ if wrong)
        // err_y > 0 = bottle below center    → pitch up, decrease q1
        double new_q0 = std::clamp(q_[0] + Kp_x_ * err_x, -3.14159, 3.14159);
        double new_q1 = std::clamp(q_[1] - Kp_y_ * err_y, -1.5, 1.5);

        RCLCPP_INFO(get_logger(),
            "err_x:%.2f err_y:%.2f → q0:%.3f q1:%.3f",
            err_x, err_y, new_q0, new_q1);

        publishJoints(new_q0, new_q1, q_[2]);
    }

    double q_[3]     = {0.0, 0.0, 0.0};
    bool got_joints_ = false;
    double Kp_x_, Kp_y_, deadzone_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr     error_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Commander>());
    rclcpp::shutdown();
    return 0;
}