#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool goalReached() const;
    void planPath();
    void publishEmptyPath();
    bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
    bool aStar(int sx, int sy, int gx, int gy, std::vector<std::pair<int, int>> & out) const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;
    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::PointStamped goal_;
    rclcpp::Time goal_time_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool have_map_ = false;
    bool have_odom_ = false;

    const double goal_tolerance_ = 0.5;     // meters
    const double timeout_seconds_ = 120.0;  // give up after this long
    const int obstacle_threshold_ = 50;     // cells at or above this cost are walls
    const double cost_weight_ = 10.0;       // higher = less avoidance of high-cost cells
};

#endif