#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{
class ControlCore
{
public:
  explicit ControlCore(double lookahead_distance = 0.8, double goal_tolerance = 0.25,
    double linear_speed = 0.45, double max_angular_speed = 1.5, double turn_kp = 0.8);
  bool isGoalReached(const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & robot_pose) const;
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & robot_pose) const;
  geometry_msgs::msg::Twist computeCommand(const geometry_msgs::msg::PoseStamped & target,
    const geometry_msgs::msg::Pose & robot_pose, const geometry_msgs::msg::Point & goal);
  void resetTurning() { turning_ = false; }

private:
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;
  double max_angular_speed_;
  double turn_kp_;
  bool turning_{false};
  static double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion);
};
}  // namespace robot

#endif
