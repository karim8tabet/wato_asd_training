#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

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

  // Remember every obstacle cell, so we can inflate around them afterwards
  std::vector<std::pair<int, int>> obstacles;

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

    // Mark the cell as an obstacle (only record it once, even if several beams hit it)
    int index = gy * width_ + gx;
    if (grid_[index] != 100) {
      grid_[index] = 100;
      obstacles.push_back({gx, gy});
    }
  }

  // 3. Inflate: give cells near obstacles a cost that fades with distance
  int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));  // 10 cells

  for (const auto & cell : obstacles) {
    int ox = cell.first;   // obstacle's column
    int oy = cell.second;  // obstacle's row

    // Look at every cell in a square around the obstacle
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;

        // Skip cells outside the grid
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }

        // Real distance from the obstacle, in meters
        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

        // Skip the corners of the square that are beyond the radius
        if (distance > inflation_radius_) {
          continue;
        }

        // Cost fades from max_cost_ at the obstacle to 0 at the edge of the radius
        int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));

        // Only raise a cell's cost, never lower it
        int n_index = ny * width_ + nx;
        if (cost > grid_[n_index]) {
          grid_[n_index] = static_cast<int8_t>(cost);
        }
      }
    }
  }

  // 4. Package the grid into a message and publish it
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