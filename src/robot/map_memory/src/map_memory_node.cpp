#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(
    this->declare_parameter("resolution", 0.10),
    this->declare_parameter("map_size", 30.0),
    this->declare_parameter("map_frame", std::string("sim_world")),
    this->declare_parameter("arena_size", 0.0))
{
  update_distance_ = this->declare_parameter("update_distance", 1.5);
  const double update_period = this->declare_parameter("update_period", 1.0);
  if (!std::isfinite(update_period) || update_period <= 0.0) {
    throw std::invalid_argument("update_period must be positive and finite");
  }

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(update_period), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Mapping uses the lidar pose.
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  const auto & q = msg->pose.pose.orientation;
  current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  have_costmap_ = !msg->data.empty();
  costmap_updated_ = have_costmap_;
}

void MapMemoryNode::updateMap()
{
  if (have_odom_ && costmap_updated_) {
    const double moved = std::hypot(current_x_ - last_update_x_, current_y_ - last_update_y_);
    if (!initialized_ || moved >= update_distance_) {
      // Map immediately at startup, then after movement.
      if (!map_memory_.integrateCostmap(latest_costmap_, current_x_, current_y_, current_yaw_)) {
        RCLCPP_WARN(this->get_logger(), "Ignoring invalid local costmap or pose");
        publishMap();
        return;
      }
      last_update_x_ = current_x_;
      last_update_y_ = current_y_;
      initialized_ = true;
      costmap_updated_ = false;
    }
  }

  // Publish from startup.
  publishMap();
}

void MapMemoryNode::publishMap()
{
  auto map = map_memory_.map();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
