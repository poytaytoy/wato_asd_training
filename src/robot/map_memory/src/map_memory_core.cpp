#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "map_memory_core.hpp"

namespace robot
{
MapMemoryCore::MapMemoryCore(double resolution, double map_size, const std::string & frame,
  double arena_size)
: arena_size_(arena_size)
{
  if (!std::isfinite(resolution) || resolution <= 0 || !std::isfinite(map_size) || map_size <= 0) {
    throw std::invalid_argument("Invalid map parameters");
  }
  if (!std::isfinite(arena_size) || arena_size < 0 || arena_size > map_size) {
    throw std::invalid_argument("Arena size must be between zero and map size");
  }
  const auto cells = static_cast<uint32_t>(std::ceil(map_size / resolution));
  global_map_.header.frame_id = frame;
  global_map_.info.resolution = static_cast<float>(resolution);
  global_map_.info.width = cells;
  global_map_.info.height = cells;
  global_map_.info.origin.position.x = -0.5 * cells * resolution;
  global_map_.info.origin.position.y = -0.5 * cells * resolution;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(static_cast<size_t>(cells) * cells, -1);
  // Optional fixed-arena shading.
  if (arena_size_ > 0) {
    for (uint32_t y = 0; y < cells; ++y) {
      for (uint32_t x = 0; x < cells; ++x) {
        if (insideArena(x, y)) {
          global_map_.data[static_cast<size_t>(y) * cells + x] = 0;
        }
      }
    }
  }
}

bool MapMemoryCore::insideArena(int x, int y) const
{
  const double world_x = global_map_.info.origin.position.x + (x + 0.5) * global_map_.info.resolution;
  const double world_y = global_map_.info.origin.position.y + (y + 0.5) * global_map_.info.resolution;
  const double half = arena_size_ * 0.5;
  return world_x >= -half && world_x < half && world_y >= -half && world_y < half;
}

bool MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid & costmap,
  double robot_x, double robot_y, double robot_yaw)
{
  if (!std::isfinite(costmap.info.resolution) || costmap.info.resolution <= 0 ||
      costmap.info.width == 0 || costmap.info.height == 0 ||
      costmap.data.size() != static_cast<size_t>(costmap.info.width) * costmap.info.height ||
      !std::isfinite(robot_x) || !std::isfinite(robot_y) || !std::isfinite(robot_yaw)) {
    return false;
  }
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  const int local_width = static_cast<int>(costmap.info.width);
  const int local_height = static_cast<int>(costmap.info.height);
  const int global_width = static_cast<int>(global_map_.info.width);
  const int global_height = static_cast<int>(global_map_.info.height);
  const double local_resolution = costmap.info.resolution;
  const double global_resolution = global_map_.info.resolution;

  for (int row = 0; row < local_height; ++row) {
    for (int col = 0; col < local_width; ++col) {
      const auto cell = costmap.data[static_cast<size_t>(row) * local_width + col];
      if (cell < 0) {
        continue;
      }

      const double local_x = costmap.info.origin.position.x + (static_cast<double>(col) + 0.5) * local_resolution;
      const double local_y = costmap.info.origin.position.y + (static_cast<double>(row) + 0.5) * local_resolution;
      // Transform using the lidar pose.
      const double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      const double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      const int grid_x = static_cast<int>(std::floor((world_x - global_map_.info.origin.position.x) / global_resolution));
      const int grid_y = static_cast<int>(std::floor((world_y - global_map_.info.origin.position.y) / global_resolution));
      if (grid_x < 0 || grid_y < 0 || grid_x >= global_width || grid_y >= global_height) {
        continue;
      }

      // Clip free shading only, not obstacles.
      if (cell == 0 && arena_size_ > 0 && !insideArena(grid_x, grid_y)) {
        continue;
      }

      auto & destination = global_map_.data[static_cast<size_t>(grid_y) * global_width + grid_x];
      // Keep old obstacles. Free readings cannot erase them.
      destination = std::max(destination, cell);
    }
  }
  return true;
}

}  // namespace robot
