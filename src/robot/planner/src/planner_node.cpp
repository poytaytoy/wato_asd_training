#include <chrono>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner")
{
  lidar_to_wheel_center_ = this->declare_parameter("lidar_to_wheel_center", 1.3);

  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.30);
  replan_period_ = this->declare_parameter("replan_period", 2.0);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  have_map_ = robot::PlannerCore::isValidMap(*msg);
  if (state_ == State::NAVIGATING && !have_map_) {
    publishStopPath();
  }
  if (state_ == State::NAVIGATING && have_map_) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  state_ = State::NAVIGATING;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

  if (state_ == State::NAVIGATING) {
    const double distance = std::hypot(goal_.point.x - wheel_pose_.position.x, goal_.point.y - wheel_pose_.position.y);
    if (distance <= goal_tolerance_) {
      state_ = State::WAITING_FOR_GOAL;
      have_goal_ = false;
      publishStopPath();
      RCLCPP_INFO(this->get_logger(), "Goal reached");
    }
  }
}

void PlannerNode::timerCallback()
{
  if (state_ != State::NAVIGATING || !have_goal_ || !have_map_ || !have_odom_) {
    return;
  }

  const int64_t now_ns = this->now().nanoseconds();
  if (last_plan_ns_ == 0 || static_cast<double>(now_ns - last_plan_ns_) / 1e9 >= replan_period_)
  {
    planPath();
  }
}

void PlannerNode::planPath()
{
  if (!have_map_ || !have_goal_ || !have_odom_) {
    return;
  }
  std::string failure_reason;
  const auto points = planner_.planPath(
    current_map_, wheel_pose_.position, goal_.point, &failure_reason);
  last_plan_ns_ = this->now().nanoseconds();
  if (!points) {
    RCLCPP_WARN(this->get_logger(), "Planning failed: %s", failure_reason.c_str());
    // Clear the old path to stop control.
    publishStopPath();
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id.empty() ? goal_.header.frame_id : current_map_.header.frame_id;
  path.poses.reserve(points->size());
  for (const auto & point : *points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

void PlannerNode::publishStopPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
