#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{
class CostmapCore
{
public:
  explicit CostmapCore(double resolution = 0.10, double map_size = 15.0,
    double inflation_radius = 1.0);
  nav_msgs::msg::OccupancyGrid build(const sensor_msgs::msg::LaserScan & scan);

private:
  nav_msgs::msg::OccupancyGrid grid_msg_;
  double resolution_;
  double map_size_;
  double inflation_radius_;
  void inflateObstacle(int grid_x, int grid_y);
};
}  // namespace robot

#endif
