#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control") {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!robot_odom_) {
    return;
  }
  // No path: stop once (so manual Teleop still works afterwards)
  if (!current_path_ || current_path_->poses.empty()) {
    if (driving_) {
      stopRobot();
    }
    return;
  }

  const auto & pos = robot_odom_->pose.pose.position;
  const auto & end = current_path_->poses.back().pose.position;
  if (std::hypot(end.x - pos.x, end.y - pos.y) < goal_tolerance_) {
    if (driving_) {
      stopRobot();
    }
    return;
  }

  auto target = findLookaheadPoint();
  if (!target) {
    return;
  }
  cmd_vel_pub_->publish(computeVelocity(*target));
  driving_ = true;
}

void ControlNode::stopRobot() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  driving_ = false;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const {
  const auto & poses = current_path_->poses;
  const auto & pos = robot_odom_->pose.pose.position;

  // Closest path point to the robot
  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < poses.size(); ++i) {
    const double d = std::hypot(poses[i].pose.position.x - pos.x, poses[i].pose.position.y - pos.y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }

  // First point after it that is at least lookahead_distance_ away
  for (size_t i = closest; i < poses.size(); ++i) {
    const double d = std::hypot(poses[i].pose.position.x - pos.x, poses[i].pose.position.y - pos.y);
    if (d >= lookahead_distance_) {
      return poses[i];
    }
  }
  return poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped & target) const {
  const auto & pos = robot_odom_->pose.pose.position;
  const double yaw = extractYaw(robot_odom_->pose.pose.orientation);

  const double dx = target.pose.position.x - pos.x;
  const double dy = target.pose.position.y - pos.y;
  const double dist = std::hypot(dx, dy);

  // Angle between where the robot faces and where the target is, in [-pi, pi]
  double alpha = std::atan2(dy, dx) - yaw;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));

  geometry_msgs::msg::Twist cmd;
  if (std::abs(alpha) > 0.8) {
    // Target is well off to the side or behind: turn in place first
    cmd.linear.x = 0.0;
    cmd.angular.z = std::clamp(1.5 * alpha, -max_angular_speed_, max_angular_speed_);
  } else if (dist > 1e-3) {
    // Pure pursuit: curvature = 2 sin(alpha) / distance
    cmd.linear.x = linear_speed_;
    cmd.angular.z = std::clamp(2.0 * linear_speed_ * std::sin(alpha) / dist,
                               -max_angular_speed_, max_angular_speed_);
  }
  return cmd;
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}