#include <chrono>
#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscribe to /lidar. Every time a scan arrives, call laserCallback with it.
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Publisher for the costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // 1. Start from an empty grid: every cell set to 0 (free)
  grid_.assign(width_ * height_, 0);

  // 2. Go through every beam and mark where it hit
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // Skip invalid readings: infinite, NaN, too close, or at/over max range
    if (!std::isfinite(range) || range < scan->range_min || range >= scan->range_max) {
      continue;
    }

    // Angle of this beam
    double angle = scan->angle_min + i * scan->angle_increment;

    // Where the beam hit, in meters, relative to the lidar
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Convert meters to grid cell indices
    int gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

    // Skip points that fall outside the grid
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
      continue;
    }

    // Mark the cell as an obstacle
    grid_[gy * width_ + gx] = 100;
  }

  // 3. Package the grid into a message and publish it
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = scan->header;                 // same frame and time as the scan
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;       // "no rotation"
  msg.data = grid_;

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}