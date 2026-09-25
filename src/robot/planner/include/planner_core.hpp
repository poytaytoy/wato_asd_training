#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <optional>
#include <string>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{
class PlannerCore
{
public:
  static bool isValidMap(const nav_msgs::msg::OccupancyGrid & map);

  // Returns no path on failure.
  std::optional<std::vector<geometry_msgs::msg::Point>> planPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Point & start_world,
    const geometry_msgs::msg::Point & goal_world,
    std::string * failure_reason = nullptr) const;

private:
  bool worldToCell(const nav_msgs::msg::OccupancyGrid & map,
    double world_x, double world_y, int & cell_x, int & cell_y) const;
  bool isBlocked(const nav_msgs::msg::OccupancyGrid & map, int cell_x, int cell_y) const;
};
}  // namespace robot

#endif
