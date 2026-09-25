#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{
class MapMemoryCore
{
public:
  explicit MapMemoryCore(double resolution = 0.10, double map_size = 30.0,
    const std::string & frame = "sim_world", double arena_size = 0.0);
  // Returns false for invalid input.
  bool integrateCostmap(const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x, double robot_y, double robot_yaw);
  const nav_msgs::msg::OccupancyGrid & map() const { return global_map_; }

private:
  // Zero disables fixed-arena shading.
  double arena_size_;
  bool insideArena(int x, int y) const;
  nav_msgs::msg::OccupancyGrid global_map_;
};
}  // namespace robot

#endif
