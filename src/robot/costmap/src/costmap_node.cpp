#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(
    this->declare_parameter("resolution", 0.10),
    this->declare_parameter("map_size", 15.0),
    this->declare_parameter("inflation_radius", 1.0))
{
  pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  pub_->publish(costmap_.build(*msg));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
