#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <cstdint>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"
#include "rclcpp/rclcpp.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose wheel_pose_;
  double lidar_to_wheel_center_;
  bool have_map_{false};
  bool have_goal_{false};
  bool have_odom_{false};
  enum class State { WAITING_FOR_GOAL, NAVIGATING };
  State state_{State::WAITING_FOR_GOAL};
  double goal_tolerance_;
  double replan_period_;
  int64_t last_plan_ns_{0};

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void planPath();
  void publishStopPath();
};

#endif
