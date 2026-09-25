#include <chrono>
#include <cmath>
#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"),
  control_(
    this->declare_parameter("lookahead_distance", 0.8),
    this->declare_parameter("goal_tolerance", 0.25),
    this->declare_parameter("linear_speed", 0.45),
    this->declare_parameter("max_angular_speed", 1.5))
{
  lidar_to_wheel_center_ = this->declare_parameter("lidar_to_wheel_center", 1.3);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = *msg;
  have_path_ = !current_path_.poses.empty();
  if (!have_path_) {
    publishStop();
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Shift lidar pose to the wheel centre.
  wheel_pose_ = msg->pose.pose;
  const auto & q = wheel_pose_.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  wheel_pose_.position.x -= lidar_to_wheel_center_ * std::cos(yaw);
  wheel_pose_.position.y -= lidar_to_wheel_center_ * std::sin(yaw);
  wheel_pose_.position.z = 0.0;
  have_odom_ = true;
}

void ControlNode::controlLoop()
{
  if (!have_path_ || !have_odom_) {
    return;
  }

  if (control_.isGoalReached(current_path_, wheel_pose_)) {
    have_path_ = false;
    publishStop();
    return;
  }

  const auto target = control_.findLookaheadPoint(current_path_, wheel_pose_);
  if (!target) {
    publishStop();
    return;
  }
  cmd_vel_pub_->publish(control_.computeCommand(*target, wheel_pose_, current_path_.poses.back().pose.position));
}

void ControlNode::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
