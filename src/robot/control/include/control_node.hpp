#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "control_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
  robot::ControlCore control_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Pose wheel_pose_;
  double lidar_to_wheel_center_;
  bool have_path_{false};
  bool have_odom_{false};

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();
  void publishStop();
};

#endif
