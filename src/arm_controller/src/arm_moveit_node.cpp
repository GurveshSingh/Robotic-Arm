#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class ArmMoveItNode : public rclcpp::Node {
public:
  ArmMoveItNode() : Node("arm_moveit_node") {

    // 2. Subscribe to the YOLO target (PoseStamped)
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&ArmMoveItNode::target_callback, this, std::placeholders::_1));

    // 3. Subscribe to a raw Point topic for simple XYZ commanding
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_point", 10,
      std::bind(&ArmMoveItNode::point_callback, this, std::placeholders::_1));

    // 4. Init MoveIt after node is fully up
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ArmMoveItNode::init_moveit, this));

    RCLCPP_INFO(this->get_logger(),
      "Arm Controller Node Started. Listening for targets...");
  }

private:
  // ── MoveIt initialisation ────────────────────────────────────────────────
  void init_moveit() {
    timer_->cancel();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");

    move_group_->setPoseReferenceFrame("world");

    RCLCPP_INFO(this->get_logger(), "Planning frame:    %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "MoveGroup Interface Connected.");

    // Move to your recorded pose on startup
    move_to_pose(-0.528, 0.000, 0.105,
                  0.815, 0.000, -0.579, 0.000);
  }

  // ── Plan and execute a full pose (xyz + quaternion) ──────────────────────
  void move_to_pose(double x,  double y,  double z,
                    double qx, double qy, double qz, double qw) {
    if (!move_group_ || is_moving_) return;

    RCLCPP_INFO(this->get_logger(),
      "Moving to pose: x=%.3f y=%.3f z=%.3f | qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
      x, y, z, qx, qy, qz, qw);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x    = x;
    target_pose.position.y    = y;
    target_pose.position.z    = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    is_moving_ = true;
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success =
      (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan found - executing.");
      move_group_->execute(my_plan);
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed!");
    }

    is_moving_ = false;
  }

  // ── Build a Pose from (x,y,z) using your recorded orientation ────────────
  void move_to_position(double x, double y, double z) {
    // Reuse your recorded orientation as the default
    move_to_pose(x, y, z,
                 0.815, 0.000, -0.579, 0.000);
  }

  // ── Callback: full PoseStamped ────────────────────────────────────────────
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!move_group_ || is_moving_) return;
    RCLCPP_INFO(this->get_logger(),
      "PoseStamped target: x=%.2f, y=%.2f, z=%.2f",
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    move_to_pose(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
  }

  // ── Callback: plain Point (xyz only, uses default orientation) ───────────
  void point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!move_group_ || is_moving_) return;
    RCLCPP_INFO(this->get_logger(),
      "Point target: x=%.2f, y=%.2f, z=%.2f",
      msg->x, msg->y, msg->z);

    move_to_position(msg->x, msg->y, msg->z);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr        point_sub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    move_group_;
  bool is_moving_ = false;
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmMoveItNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}