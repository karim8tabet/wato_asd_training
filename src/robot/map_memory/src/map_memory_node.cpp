#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory") {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width_ * height_, -1);  // -1 = unknown
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_odom_ = true;

  if (std::hypot(robot_x_ - last_x_, robot_y_ - last_y_) >= distance_threshold_) {
    should_update_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_ && have_costmap_ && have_odom_) {
    integrateCostmap();
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_ = false;
  }
  // Publish every second so the planner always has a map
  global_map_.header.stamp = this->now();
  map_pub_->publish(global_map_);
}

void MapMemoryNode::integrateCostmap() {
  const auto & cm = latest_costmap_;
  const double cm_res = cm.info.resolution;
  const int cm_w = static_cast<int>(cm.info.width);
  const int cm_h = static_cast<int>(cm.info.height);
  const double cm_ox = cm.info.origin.position.x;
  const double cm_oy = cm.info.origin.position.y;
  if (cm_w == 0 || cm_h == 0 || cm.data.size() != static_cast<size_t>(cm_w * cm_h)) {
    return;
  }

  const double c = std::cos(robot_yaw_);
  const double s = std::sin(robot_yaw_);

  // Only visit global cells the costmap could cover
  const double far_x = std::max(std::abs(cm_ox), std::abs(cm_ox + cm_w * cm_res));
  const double far_y = std::max(std::abs(cm_oy), std::abs(cm_oy + cm_h * cm_res));
  const double reach = std::hypot(far_x, far_y);

  const int min_gx = std::max(0, static_cast<int>(std::floor((robot_x_ - reach - origin_x_) / resolution_)));
  const int max_gx = std::min(width_ - 1, static_cast<int>(std::floor((robot_x_ + reach - origin_x_) / resolution_)));
  const int min_gy = std::max(0, static_cast<int>(std::floor((robot_y_ - reach - origin_y_) / resolution_)));
  const int max_gy = std::min(height_ - 1, static_cast<int>(std::floor((robot_y_ + reach - origin_y_) / resolution_)));

  for (int gy = min_gy; gy <= max_gy; ++gy) {
    for (int gx = min_gx; gx <= max_gx; ++gx) {
      // Center of this global cell in world coordinates
      const double wx = origin_x_ + (gx + 0.5) * resolution_;
      const double wy = origin_y_ + (gy + 0.5) * resolution_;

      // World -> lidar frame (inverse of the robot's rotation and position)
      const double dx = wx - robot_x_;
      const double dy = wy - robot_y_;
      const double lx = c * dx + s * dy;
      const double ly = -s * dx + c * dy;

      const int cx = static_cast<int>(std::floor((lx - cm_ox) / cm_res));
      const int cy = static_cast<int>(std::floor((ly - cm_oy) / cm_res));
      if (cx < 0 || cx >= cm_w || cy < 0 || cy >= cm_h) {
        continue;
      }

      const int8_t value = cm.data[cy * cm_w + cx];
      if (value < 0) {
        continue;
      }

      // Keep the highest cost seen, so remembered obstacles are never erased
      int8_t & cell = global_map_.data[gy * width_ + gx];
      if (value > cell) {
        cell = value;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}