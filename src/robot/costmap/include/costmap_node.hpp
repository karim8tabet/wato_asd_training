#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Runs every time a new lidar scan arrives
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;

    // Listens to the /lidar topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Sends the finished costmap out on /costmap
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Grid settings: 400 x 400 cells, 0.1 m each = 40 m x 40 m,
    // with the lidar at the center
    const double resolution_ = 0.1;   // meters per cell
    const int width_ = 400;           // cells
    const int height_ = 400;          // cells
    const double origin_x_ = -20.0;   // bottom-left corner, in meters
    const double origin_y_ = -20.0;
    const double inflation_radius_ = 1.5;  // meters of "danger zone" around obstacles
    const double max_cost_ = 100.0;        // cost right at an obstacle

    // The grid itself: one number per cell (0 = free, 100 = obstacle)
    std::vector<int8_t> grid_;
};

#endif