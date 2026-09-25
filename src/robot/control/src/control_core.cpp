#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "control_core.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
}  // namespace

namespace robot
{
ControlCore::ControlCore(double lookahead_distance, double goal_tolerance,
  double linear_speed, double max_angular_speed, double turn_kp)
: lookahead_distance_(lookahead_distance), goal_tolerance_(goal_tolerance),
  linear_speed_(linear_speed), max_angular_speed_(max_angular_speed), turn_kp_(turn_kp)
{
  if (!std::isfinite(lookahead_distance) || lookahead_distance <= 0 ||
      !std::isfinite(goal_tolerance) || goal_tolerance <= 0 ||
      !std::isfinite(linear_speed) || linear_speed < 0 ||
      !std::isfinite(max_angular_speed) || max_angular_speed <= 0 ||
      !std::isfinite(turn_kp) || turn_kp <= 0) {
    throw std::invalid_argument("Invalid control parameters");
  }
}

bool ControlCore::isGoalReached(const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & robot_pose) const
{
  return !path.poses.empty() &&
    distance(robot_pose.position, path.poses.back().pose.position) <= goal_tolerance_;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot_pose) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }
  const auto & robot = robot_pose.position;
  // Search forward from the nearest waypoint.
  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double candidate_distance = distance(robot, path.poses[i].pose.position);
    if (candidate_distance < closest_distance) {
      closest_distance = candidate_distance;
      closest_index = i;
    }
  }

  for (size_t i = closest_index; i < path.poses.size(); ++i) {
    if (distance(robot, path.poses[i].pose.position) >= lookahead_distance_) {
      return path.poses[i];
    }
  }
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeCommand(const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::Pose & robot_pose, const geometry_msgs::msg::Point & goal)
{
  const auto & robot = robot_pose.position;
  const double yaw = yawFromQuaternion(robot_pose.orientation);
  const double dx = target.pose.position.x - robot.x;
  const double dy = target.pose.position.y - robot.y;
  const double y_body = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double squared_distance = dx * dx + dy * dy;
  const double heading_error = normalizeAngle(std::atan2(dy, dx) - yaw);

  geometry_msgs::msg::Twist command;
  // Start turning above 45 degrees. Finish below 10 degrees.
  const double heading_magnitude = std::abs(heading_error);
  if (turning_) {
    turning_ = heading_magnitude > kPi / 18.0;
  } else {
    turning_ = heading_magnitude > kPi / 4.0;
  }
  if (turning_) {
    // we use proportional here to control the issue of overshooting
    command.angular.z = std::clamp(turn_kp_ * heading_error, -max_angular_speed_, max_angular_speed_);
    return command;
  }

  const double goal_distance = distance(robot, goal);
  // Slow down near the goal and during turns.
  const double speed_scale = std::clamp(goal_distance / lookahead_distance_, 0.25, 1.0);
  const double heading_scale = std::clamp(std::cos(heading_error), 0.2, 1.0);
  command.linear.x = linear_speed_ * speed_scale * heading_scale;

  const double curvature = squared_distance > 1e-6 ? 2.0 * y_body / squared_distance : 0.0;
  command.angular.z = std::clamp(command.linear.x * curvature, -max_angular_speed_, max_angular_speed_);
  return command;
}

double ControlCore::distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace robot
