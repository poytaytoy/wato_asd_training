#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif
