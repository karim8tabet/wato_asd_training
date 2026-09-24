#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void controlLoop();
    void stopRobot();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped & target) const;
    static double extractYaw(const geometry_msgs::msg::Quaternion & q);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    bool driving_ = false;

    const double lookahead_distance_ = 0.8;  // meters
    const double goal_tolerance_ = 0.4;      // meters
    const double linear_speed_ = 0.5;        // m/s
    const double max_angular_speed_ = 1.0;   // rad/s
};

#endif