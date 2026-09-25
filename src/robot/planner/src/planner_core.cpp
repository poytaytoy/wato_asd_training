#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "planner_core.hpp"

namespace
{
struct CellIndex
{
  int x;
  int y;

  bool operator==(const CellIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & index) const
  {
    return std::hash<int>{}(index.x) ^ (std::hash<int>{}(index.y) << 1);
  }
};

struct OpenNode
{
  CellIndex index;
  double f_score;
};

// Lowest f score first.
struct CompareF
{
  bool operator()(const OpenNode & a, const OpenNode & b) const
  {
    return a.f_score > b.f_score;
  }
};

double heuristic(const CellIndex & a, const CellIndex & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

namespace robot
{
bool PlannerCore::isValidMap(const nav_msgs::msg::OccupancyGrid & map)
{
  return std::isfinite(map.info.resolution) && map.info.resolution > 0.0 &&
    map.info.width > 0 && map.info.height > 0 &&
    map.info.width <= static_cast<unsigned>(std::numeric_limits<int>::max()) &&
    map.info.height <= static_cast<unsigned>(std::numeric_limits<int>::max()) &&
    map.data.size() == static_cast<size_t>(map.info.width) * map.info.height;
}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid & map, double world_x, double world_y, int & cell_x, int & cell_y) const
{
  const double resolution = map.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }
  const double x = std::floor((world_x - map.info.origin.position.x) / resolution);
  const double y = std::floor((world_y - map.info.origin.position.y) / resolution);
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
      x >= map.info.width || y >= map.info.height) {
    return false;
  }
  cell_x = static_cast<int>(x);
  cell_y = static_cast<int>(y);
  return cell_x >= 0 && cell_y >= 0 && cell_x < static_cast<int>(map.info.width) && cell_y < static_cast<int>(map.info.height);
}

bool PlannerCore::isBlocked(const nav_msgs::msg::OccupancyGrid & map, int cell_x, int cell_y) const
{
  if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(map.info.width) || cell_y >= static_cast<int>(map.info.height)) {
    return true;
  }
  const auto value = map.data[static_cast<size_t>(cell_y) * map.info.width + cell_x];
  // Reject goals inside padding.
  return value > 0;
}

std::optional<std::vector<geometry_msgs::msg::Point>> PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Point & start_world,
  const geometry_msgs::msg::Point & goal_world, std::string * failure_reason) const
{
  if (failure_reason) {
    failure_reason->clear();
  }
  const auto fail = [failure_reason](const char * reason)
    -> std::optional<std::vector<geometry_msgs::msg::Point>> {
    if (failure_reason) {
      *failure_reason = reason;
    }
    return std::nullopt;
  };
  if (!isValidMap(map)) {
    return fail("Invalid map dimensions, data length, or resolution");
  }

  int start_x;
  int start_y;
  int goal_x;
  int goal_y;
  if (!worldToCell(map, start_world.x, start_world.y, start_x, start_y)) {
    return fail("Start is outside the map or has invalid coordinates");
  }
  if (!worldToCell(map, goal_world.x, goal_world.y, goal_x, goal_y)) {
    return fail("Goal is outside the map or has invalid coordinates");
  }

  const CellIndex start{start_x, start_y};
  const CellIndex goal{goal_x, goal_y};
  if (isBlocked(map, goal.x, goal.y)) {
    return fail("Goal is inside an obstacle or its padding");
  }

  const auto costAt = [&map](const CellIndex & cell) -> int {
    if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(map.info.width) ||
        cell.y >= static_cast<int>(map.info.height)) {
      return 100;
    }
    // Allow unknown space.
    return std::max(0, static_cast<int>(map.data[
      static_cast<size_t>(cell.y) * map.info.width + cell.x]));
  };
  if (costAt(start) >= 100) {
    return fail("Start is on a recorded obstacle hit, not just padding");
  }
  const auto canStep = [&costAt](const CellIndex & from, const CellIndex & to) {
    const int next_cost = costAt(to);
    // Escape through equal or lower costs. Never enter hits.
    return next_cost < 100 && next_cost <= costAt(from);
  };

  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;
  open.push({start, heuristic(start, goal)});
  g_score[start] = 0.0;

  constexpr std::array<std::array<int, 2>, 8> directions{{
    {{-1, -1}}, {{0, -1}}, {{1, -1}}, {{-1, 0}},
    {{1, 0}}, {{-1, 1}}, {{0, 1}}, {{1, 1}}
  }};

  bool found = false;
  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();
    // Skip duplicate queue entries.
    if (closed.find(current) != closed.end()) {
      continue;
    }
    if (current == goal) {
      found = true;
      break;
    }
    closed.insert(current);

    for (const auto & direction : directions) {
      const int dx = direction[0];
      const int dy = direction[1];
      const CellIndex next{current.x + dx, current.y + dy};
      if (!canStep(current, next) || closed.find(next) != closed.end()) {
        continue;
      }
      // Prevent diagonal corner cutting.
      if (dx != 0 && dy != 0 &&
          (!canStep(current, CellIndex{current.x + dx, current.y}) ||
           !canStep(current, CellIndex{current.x, current.y + dy}))) {
        continue;
      }

      const double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative = g_score[current] + step_cost;

      const auto score = g_score.find(next);
      if (score == g_score.end() || tentative < score->second) {
        g_score[next] = tentative;
        came_from[next] = current;
        open.push({next, tentative + heuristic(next, goal)});
      }
    }
  }

  if (!found) {
    return fail(costAt(start) > 0 ?
      "No route to goal using equal or lower padding costs from the start" :
      "No free route connects start and goal");
  }

  // Rebuild the path from the goal.
  std::vector<CellIndex> cells;
  for (CellIndex current = goal; !(current == start); current = came_from.at(current)) {
    cells.push_back(current);
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(cells.size());
  for (const auto & cell : cells) {
    geometry_msgs::msg::Point point;
    point.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
    point.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
    points.push_back(point);
  }
  return points;
}

}  // namespace robot
