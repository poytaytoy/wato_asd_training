#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "costmap_core.hpp"

namespace robot
{
CostmapCore::CostmapCore(double resolution, double map_size, double inflation_radius)
: resolution_(resolution), map_size_(map_size), inflation_radius_(inflation_radius)
{
  if (!std::isfinite(resolution) || resolution <= 0 ||
      !std::isfinite(map_size) || map_size <= 0 ||
      !std::isfinite(inflation_radius) || inflation_radius <= 0) {
    throw std::invalid_argument("Invalid costmap parameters");
  }
  const auto cells = static_cast<uint32_t>(std::ceil(map_size_ / resolution_));
  grid_msg_.info.resolution = static_cast<float>(resolution_);
  grid_msg_.info.width = cells;
  grid_msg_.info.height = cells;
  // Centre on the lidar.
  grid_msg_.info.origin.position.x = -0.5 * cells * resolution_;
  grid_msg_.info.origin.position.y = -0.5 * cells * resolution_;
  grid_msg_.info.origin.orientation.w = 1.0;
  grid_msg_.data.assign(static_cast<size_t>(cells) * cells, 0);
}

nav_msgs::msg::OccupancyGrid CostmapCore::build(const sensor_msgs::msg::LaserScan & scan)
{
  // Reset each scan. Unseen cells also start free.
  std::fill(grid_msg_.data.begin(), grid_msg_.data.end(), 0);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);
    const int grid_x = static_cast<int>(std::floor((x - grid_msg_.info.origin.position.x) / resolution_));
    const int grid_y = static_cast<int>(std::floor((y - grid_msg_.info.origin.position.y) / resolution_));
    inflateObstacle(grid_x, grid_y);
  }

  grid_msg_.header = scan.header;
  return grid_msg_;
}

void CostmapCore::inflateObstacle(int grid_x, int grid_y)
{
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const int width = static_cast<int>(grid_msg_.info.width);
  const int height = static_cast<int>(grid_msg_.info.height);

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = grid_x + dx;
      const int y = grid_y + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const double distance = std::hypot(dx * resolution_, dy * resolution_);
      if (distance > inflation_radius_) {
        continue;
      }
      // Hits are 100. Padding stays between 1 and 99.
      const int cost = (dx == 0 && dy == 0) ? 100 : static_cast<int>(std::round(99.0 * (1.0 - distance / inflation_radius_)));
      auto & cell = grid_msg_.data[static_cast<size_t>(y) * width + x];
      cell = std::max<int8_t>(cell, static_cast<int8_t>(std::max(1, cost)));
    }
  }
}

}  // namespace robot
