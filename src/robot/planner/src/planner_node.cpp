#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner") {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
  have_map_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }
  if ((this->now() - goal_time_).seconds() > timeout_seconds_) {
    RCLCPP_WARN(this->get_logger(), "Timed out before reaching goal");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
    return;
  }
  planPath();  // replan from the current position with the latest map
}

bool PlannerNode::goalReached() const {
  return std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_) < goal_tolerance_;
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  path_pub_->publish(path);
}

bool PlannerNode::worldToGrid(double wx, double wy, int & gx, int & gy) const {
  gx = static_cast<int>(std::floor((wx - map_.info.origin.position.x) / map_.info.resolution));
  gy = static_cast<int>(std::floor((wy - map_.info.origin.position.y) / map_.info.resolution));
  return gx >= 0 && gx < static_cast<int>(map_.info.width) &&
         gy >= 0 && gy < static_cast<int>(map_.info.height);
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or odometry");
    return;
  }

  int sx, sy, gx, gy;
  if (!worldToGrid(robot_x_, robot_y_, sx, sy) || !worldToGrid(goal_.point.x, goal_.point.y, gx, gy)) {
    RCLCPP_WARN(this->get_logger(), "Start or goal is outside the map");
    return;
  }

  std::vector<std::pair<int, int>> cells;
  if (!aStar(sx, sy, gx, gy, cells)) {
    RCLCPP_WARN(this->get_logger(), "No path found");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_.header.frame_id;
  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;

  for (const auto & cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ox + (cell.first + 0.5) * res;
    pose.pose.position.y = oy + (cell.second + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

bool PlannerNode::aStar(int sx, int sy, int gx, int gy,
                        std::vector<std::pair<int, int>> & out) const {
  const int w = static_cast<int>(map_.info.width);
  const int h = static_cast<int>(map_.info.height);
  const int n = w * h;
  const int start = sy * w + sx;
  const int goal = gy * w + gx;

  std::vector<double> g_score(n, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(n, -1);
  std::vector<bool> closed(n, false);

  using Item = std::pair<double, int>;  // (f score, cell index)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;

  auto heuristic = [gx, gy](int x, int y) { return std::hypot(x - gx, y - gy); };

  g_score[start] = 0.0;
  open.push({heuristic(sx, sy), start});

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    if (current == goal) {
      break;
    }

    const int cx = current % w;
    const int cy = current / w;

    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dxs[k];
      const int ny = cy + dys[k];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
        continue;
      }
      const int neighbor = ny * w + nx;
      if (closed[neighbor]) {
        continue;
      }

      int cost = map_.data[neighbor];
      if (cost < 0) {
        cost = 0;  // unknown = free
      }
      if (cost >= obstacle_threshold_ && neighbor != goal) {
        continue;
      }

      const double step = (k < 4) ? 1.0 : std::sqrt(2.0);
      const double tentative = g_score[current] + step * (1.0 + cost / cost_weight_);
      if (tentative < g_score[neighbor]) {
        g_score[neighbor] = tentative;
        came_from[neighbor] = current;
        open.push({tentative + heuristic(nx, ny), neighbor});
      }
    }
  }

  if (start != goal && came_from[goal] == -1) {
    return false;
  }

  out.clear();
  for (int c = goal; c != -1; c = came_from[c]) {
    out.push_back({c % w, c / w});
  }
  std::reverse(out.begin(), out.end());
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}