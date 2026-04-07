#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════
//  2D Grid cell
// ═══════════════════════════════════════════════════════════════
struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell & o) const { return x == o.x && y == o.y; }
};

struct GridCellHash
{
  size_t operator()(const GridCell & c) const
  {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 16);
  }
};

// ═══════════════════════════════════════════════════════════════
//  A* priority queue entry
// ═══════════════════════════════════════════════════════════════
struct AStarNode
{
  GridCell cell;
  double f_cost;  // g + h

  bool operator>(const AStarNode & o) const { return f_cost > o.f_cost; }
};

// ═══════════════════════════════════════════════════════════════
//  Planner Node
// ═══════════════════════════════════════════════════════════════
class OctomapPlannerNode : public rclcpp::Node
{
public:
  OctomapPlannerNode()
  : Node("octomap_planner_node")
  {
    // ── Parameters ──
    cruise_altitude_ = this->declare_parameter<double>("cruise_altitude", 0.45);
    grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.1);
    inflation_radius_ = this->declare_parameter<double>("inflation_radius", 0.15);
    max_plan_distance_ = this->declare_parameter<double>("max_plan_distance", 2.0);
    altitude_slice_thickness_ = this->declare_parameter<double>("altitude_slice_thickness", 0.3);
    replan_interval_seconds_ = this->declare_parameter<double>("replan_interval_seconds", 2.0);
    path_check_step_ = this->declare_parameter<double>("path_check_step", 0.05);

    // ── Subscribers ──
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapPlannerNode::octomapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&OctomapPlannerNode::goalCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose",
      rclcpp::SensorDataQoS(),
      std::bind(&OctomapPlannerNode::poseCallback, this, std::placeholders::_1));

    // ── Publishers ──
    planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/planned_path", 10);
    planned_path_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/planned_path_viz", 10);

    // ── Replan timer ──
    replan_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(replan_interval_seconds_)),
      std::bind(&OctomapPlannerNode::replanCheck, this));

    last_replan_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "octomap_planner_node started");
    RCLCPP_INFO(this->get_logger(),
      "cruise_alt=%.2f grid_res=%.2f inflate=%.2f max_dist=%.2f",
      cruise_altitude_, grid_resolution_, inflation_radius_, max_plan_distance_);
  }

private:
  // ═══════════════════════════════════════════════════════════════
  //  Callbacks
  // ═══════════════════════════════════════════════════════════════

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);

    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(*msg);
    if (!tree) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert OctoMap message");
      return;
    }

    octomap::OcTree * oc_tree = dynamic_cast<octomap::OcTree *>(tree);
    if (!oc_tree) {
      RCLCPP_WARN(this->get_logger(), "OctoMap is not an OcTree");
      delete tree;
      return;
    }

    tree_.reset(oc_tree);
    map_received_ = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "OctoMap received. Resolution: %.3f", tree_->getResolution());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    have_pose_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!map_received_) {
      RCLCPP_WARN(this->get_logger(), "No OctoMap yet — ignoring goal");
      return;
    }

    if (!have_pose_) {
      RCLCPP_WARN(this->get_logger(), "No drone pose yet — ignoring goal");
      return;
    }

    // Goal is in map frame (swapped coords from mavros_tf_node)
    goal_map_x_ = msg->pose.position.x;
    goal_map_y_ = msg->pose.position.y;

    // Drone position in map frame (swap MAVROS coords)
    double drone_map_x = current_pose_.pose.position.y;
    double drone_map_y = current_pose_.pose.position.x;

    // Distance check
    double dist = std::sqrt(
      std::pow(goal_map_x_ - drone_map_x, 2) +
      std::pow(goal_map_y_ - drone_map_y, 2));

    if (dist > max_plan_distance_) {
      RCLCPP_WARN(this->get_logger(),
        "Goal too far: %.2fm (max: %.2fm) — ignoring",
        dist, max_plan_distance_);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Goal received: map=(%.2f, %.2f) drone_map=(%.2f, %.2f) dist=%.2f",
      goal_map_x_, goal_map_y_, drone_map_x, drone_map_y, dist);
    RCLCPP_INFO(this->get_logger(),
      "  Drone MAVROS pose: x=%.2f y=%.2f z=%.2f",
      current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);

    have_goal_ = true;
    planPath(drone_map_x, drone_map_y, goal_map_x_, goal_map_y_);
  }

  // ═══════════════════════════════════════════════════════════════
  //  Planning
  // ═══════════════════════════════════════════════════════════════

  void planPath(double start_x, double start_y, double goal_x, double goal_y)
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);

    if (!tree_) {
      RCLCPP_ERROR(this->get_logger(), "No OctoMap for planning");
      return;
    }

    // ── Step 1: Build 2D occupancy grid at drone's actual flight altitude ──
    // Drone's map Z = -MAVROS Z (because mavros_tf_node negates Z)
    double drone_map_z = -current_pose_.pose.position.z;
    std::unordered_set<GridCell, GridCellHash> occupied_cells;
    buildOccupancyGrid(occupied_cells, drone_map_z);

    // ── Step 2: Inflate obstacles by drone radius ──
    std::unordered_set<GridCell, GridCellHash> inflated_cells;
    inflateObstacles(occupied_cells, inflated_cells);

    // ── Step 3: Check if straight line is clear (Phase 3) ──
    GridCell start_cell = worldToGrid(start_x, start_y);
    GridCell goal_cell = worldToGrid(goal_x, goal_y);

    if (isLineClear(start_cell, goal_cell, inflated_cells)) {
      RCLCPP_INFO(this->get_logger(), "Direct path is clear — no A* needed");

      // Build simple 2-point path
      std::vector<GridCell> path_cells;
      path_cells.push_back(start_cell);
      path_cells.push_back(goal_cell);

      publishPath(path_cells);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Direct path blocked — running A*");

    // ── Step 4: Run A* (Phase 4) ──
    std::vector<GridCell> path_cells;
    bool found = runAStar(start_cell, goal_cell, inflated_cells, path_cells);

    if (!found) {
      RCLCPP_WARN(this->get_logger(), "A* failed — no path found to goal");
      return;
    }

    // ── Step 5: Simplify path ──
    std::vector<GridCell> simplified;
    simplifyPath(path_cells, inflated_cells, simplified);

    RCLCPP_INFO(this->get_logger(),
      "Path found: %zu raw waypoints -> %zu simplified",
      path_cells.size(), simplified.size());

    // ── Step 6: Publish ──
    publishPath(simplified);
    last_planned_cells_ = simplified;
  }

  // ═══════════════════════════════════════════════════════════════
  //  Build 2D occupancy grid from OctoMap slice
  // ═══════════════════════════════════════════════════════════════

  void buildOccupancyGrid(
    std::unordered_set<GridCell, GridCellHash> & occupied,
    double z_center) const
  {
    // Query OctoMap at z_center +/- half thickness
    // z_center should be in map frame (negated MAVROS Z)
    double z_min = z_center - altitude_slice_thickness_ / 2.0;
    double z_max = z_center + altitude_slice_thickness_ / 2.0;

    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (tree_->isNodeOccupied(*it)) {
        double vx = it.getX();
        double vy = it.getY();
        double vz = it.getZ();

        // Check if voxel is at cruise altitude slice
        if (vz >= z_min && vz <= z_max) {
          GridCell cell = worldToGrid(vx, vy);
          occupied.insert(cell);
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Occupancy grid: %zu occupied cells at z=[%.2f, %.2f]",
      occupied.size(), z_min, z_max);
  }

  // ═══════════════════════════════════════════════════════════════
  //  Inflate obstacles
  // ═══════════════════════════════════════════════════════════════

  void inflateObstacles(
    const std::unordered_set<GridCell, GridCellHash> & occupied,
    std::unordered_set<GridCell, GridCellHash> & inflated) const
  {
    int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / grid_resolution_));

    inflated = occupied;  // start with original obstacles

    for (const auto & cell : occupied) {
      for (int dx = -inflate_cells; dx <= inflate_cells; dx++) {
        for (int dy = -inflate_cells; dy <= inflate_cells; dy++) {
          double dist = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
          if (dist <= inflation_radius_) {
            GridCell inflated_cell{cell.x + dx, cell.y + dy};
            inflated.insert(inflated_cell);
          }
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Inflated: %zu -> %zu cells (radius=%.2fm)",
      occupied.size(), inflated.size(), inflation_radius_);
  }

  // ═══════════════════════════════════════════════════════════════
  //  Line-of-sight check (Phase 3)
  // ═══════════════════════════════════════════════════════════════

  bool isLineClear(
    const GridCell & start,
    const GridCell & goal,
    const std::unordered_set<GridCell, GridCellHash> & obstacles) const
  {
    // Bresenham-style line check
    int dx = std::abs(goal.x - start.x);
    int dy = std::abs(goal.y - start.y);
    int sx = (start.x < goal.x) ? 1 : -1;
    int sy = (start.y < goal.y) ? 1 : -1;
    int err = dx - dy;

    int cx = start.x;
    int cy = start.y;

    while (true) {
      GridCell check{cx, cy};
      if (obstacles.count(check) > 0) {
        return false;  // blocked
      }

      if (cx == goal.x && cy == goal.y) {
        break;
      }

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        cx += sx;
      }
      if (e2 < dx) {
        err += dx;
        cy += sy;
      }
    }

    return true;  // clear
  }

  // ═══════════════════════════════════════════════════════════════
  //  A* search (Phase 4)
  // ═══════════════════════════════════════════════════════════════

  GridCell findNearestFreeCell(
    const GridCell & cell,
    const std::unordered_set<GridCell, GridCellHash> & obstacles,
    int max_search_radius = 8) const
  {
    // BFS outward to find nearest non-obstacle cell
    for (int r = 1; r <= max_search_radius; r++) {
      for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;  // only check border
          GridCell candidate{cell.x + dx, cell.y + dy};
          if (obstacles.count(candidate) == 0) {
            return candidate;
          }
        }
      }
    }
    return cell;  // fallback: return original
  }

  bool runAStar(
    const GridCell & start_in,
    const GridCell & goal_in,
    const std::unordered_set<GridCell, GridCellHash> & obstacles,
    std::vector<GridCell> & path) const
  {
    GridCell start = start_in;
    GridCell goal = goal_in;

    // If start is inside inflated obstacle, find nearest free cell
    if (obstacles.count(start) > 0) {
      RCLCPP_WARN(this->get_logger(),
        "Start (%d, %d) is inside inflated obstacle — searching for nearest free cell",
        start.x, start.y);
      start = findNearestFreeCell(start, obstacles);
      if (obstacles.count(start) > 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not find free cell near start");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
        "Using adjusted start: (%d, %d)", start.x, start.y);
    }

    // If goal is inside inflated obstacle, find nearest free cell
    if (obstacles.count(goal) > 0) {
      RCLCPP_WARN(this->get_logger(),
        "Goal (%d, %d) is inside inflated obstacle — searching for nearest free cell",
        goal.x, goal.y);
      goal = findNearestFreeCell(goal, obstacles);
      if (obstacles.count(goal) > 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not find free cell near goal");
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
        "Using adjusted goal: (%d, %d)", goal.x, goal.y);
    }

    // Priority queue (min-heap by f_cost)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;

    std::unordered_map<GridCell, double, GridCellHash> g_cost;
    std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
    std::unordered_set<GridCell, GridCellHash> closed;

    g_cost[start] = 0.0;
    double h = heuristic(start, goal);
    open.push({start, h});

    // 8-connected grid neighbors
    const int dx8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const double cost8[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

    int iterations = 0;
    const int max_iterations = 50000;

    while (!open.empty() && iterations < max_iterations) {
      iterations++;

      AStarNode current = open.top();
      open.pop();

      if (current.cell == goal) {
        // Reconstruct path
        GridCell trace = goal;
        while (!(trace == start)) {
          path.push_back(trace);
          trace = came_from[trace];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        RCLCPP_INFO(this->get_logger(),
          "A* found path: %zu cells, %d iterations", path.size(), iterations);
        return true;
      }

      if (closed.count(current.cell) > 0) {
        continue;
      }
      closed.insert(current.cell);

      for (int i = 0; i < 8; i++) {
        GridCell neighbor{current.cell.x + dx8[i], current.cell.y + dy8[i]};

        if (closed.count(neighbor) > 0) continue;
        if (obstacles.count(neighbor) > 0) continue;

        double tentative_g = g_cost[current.cell] + cost8[i];

        if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
          g_cost[neighbor] = tentative_g;
          came_from[neighbor] = current.cell;
          double f = tentative_g + heuristic(neighbor, goal);
          open.push({neighbor, f});
        }
      }
    }

    RCLCPP_WARN(this->get_logger(),
      "A* exhausted after %d iterations — no path found", iterations);
    return false;
  }

  double heuristic(const GridCell & a, const GridCell & b) const
  {
    // Euclidean distance heuristic
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // ═══════════════════════════════════════════════════════════════
  //  Path simplification (remove unnecessary waypoints)
  // ═══════════════════════════════════════════════════════════════

  void simplifyPath(
    const std::vector<GridCell> & raw_path,
    const std::unordered_set<GridCell, GridCellHash> & obstacles,
    std::vector<GridCell> & simplified) const
  {
    if (raw_path.size() <= 2) {
      simplified = raw_path;
      return;
    }

    simplified.push_back(raw_path.front());

    size_t current = 0;
    while (current < raw_path.size() - 1) {
      // Find the furthest visible cell from current
      size_t furthest = current + 1;
      for (size_t test = raw_path.size() - 1; test > current + 1; test--) {
        if (isLineClear(raw_path[current], raw_path[test], obstacles)) {
          furthest = test;
          break;
        }
      }

      simplified.push_back(raw_path[furthest]);
      current = furthest;
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Replan check (Phase 5)
  // ═══════════════════════════════════════════════════════════════

  void replanCheck()
  {
    return; //disables planning?

    if (!have_goal_ || !map_received_ || !have_pose_) {
      return;
    }

    if (last_planned_cells_.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(tree_mutex_);
    if (!tree_) return;

    // Check if the current planned path is still clear
    double drone_map_z = -current_pose_.pose.position.z;
    std::unordered_set<GridCell, GridCellHash> occupied_cells;
    buildOccupancyGrid(occupied_cells, drone_map_z);

    std::unordered_set<GridCell, GridCellHash> inflated_cells;
    inflateObstacles(occupied_cells, inflated_cells);

    bool path_still_clear = true;
    for (const auto & cell : last_planned_cells_) {
      if (inflated_cells.count(cell) > 0) {
        path_still_clear = false;
        break;
      }
    }

    if (!path_still_clear) {
      RCLCPP_WARN(this->get_logger(), "Planned path is now blocked — replanning");

      double drone_map_x = current_pose_.pose.position.y;
      double drone_map_y = current_pose_.pose.position.x;

      // Replan from current position to original goal
      // Release lock before calling planPath (it acquires its own lock)
      // Actually planPath acquires the lock too, so we need to unlock first
      // But we already have the inflated cells, so let's plan inline

      GridCell start_cell = worldToGrid(drone_map_x, drone_map_y);
      GridCell goal_cell = worldToGrid(goal_map_x_, goal_map_y_);

      if (isLineClear(start_cell, goal_cell, inflated_cells)) {
        RCLCPP_INFO(this->get_logger(), "Replan: direct path now clear");
        std::vector<GridCell> path_cells;
        path_cells.push_back(start_cell);
        path_cells.push_back(goal_cell);
        publishPath(path_cells);
        last_planned_cells_ = path_cells;
      } else {
        std::vector<GridCell> path_cells;
        bool found = runAStar(start_cell, goal_cell, inflated_cells, path_cells);

        if (found) {
          std::vector<GridCell> simplified;
          simplifyPath(path_cells, inflated_cells, simplified);
          RCLCPP_INFO(this->get_logger(),
            "Replan success: %zu waypoints", simplified.size());
          publishPath(simplified);
          last_planned_cells_ = simplified;
        } else {
          RCLCPP_WARN(this->get_logger(), "Replan failed — no path found");
        }
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Coordinate conversion
  // ═══════════════════════════════════════════════════════════════

  GridCell worldToGrid(double wx, double wy) const
  {
    return GridCell{
      static_cast<int>(std::round(wx / grid_resolution_)),
      static_cast<int>(std::round(wy / grid_resolution_))
    };
  }

  void gridToWorld(const GridCell & cell, double & wx, double & wy) const
  {
    wx = cell.x * grid_resolution_;
    wy = cell.y * grid_resolution_;
  }

  // ═══════════════════════════════════════════════════════════════
  //  Publish planned path
  // ═══════════════════════════════════════════════════════════════

  void publishPath(const std::vector<GridCell> & path_cells)
  {
    // ── Path for flight node (MAVROS frame — un-swapped) ──
    nav_msgs::msg::Path mavros_path;
    mavros_path.header.stamp = this->now();
    mavros_path.header.frame_id = "map";

    // ── Path for RViz visualization (map frame — as-is) ──
    nav_msgs::msg::Path viz_path;
    viz_path.header.stamp = this->now();
    viz_path.header.frame_id = "map";

    for (const auto & cell : path_cells) {
      double map_x, map_y;
      gridToWorld(cell, map_x, map_y);

      // Visualization path (map frame for RViz)
      geometry_msgs::msg::PoseStamped viz_pose;
      viz_pose.header.stamp = this->now();
      viz_pose.header.frame_id = "map";
      viz_pose.pose.position.x = map_x;
      viz_pose.pose.position.y = map_y;
      viz_pose.pose.position.z = -current_pose_.pose.position.z;  // actual drone map Z
      viz_pose.pose.orientation.w = 1.0;
      viz_path.poses.push_back(viz_pose);

      // Flight path (MAVROS frame — un-swap)
      // map_x = MAVROS_y, map_y = MAVROS_x
      // So: MAVROS_x = map_y, MAVROS_y = map_x
      geometry_msgs::msg::PoseStamped mavros_pose;
      mavros_pose.header.stamp = this->now();
      mavros_pose.header.frame_id = "map";
      mavros_pose.pose.position.x = map_y;   // un-swap
      mavros_pose.pose.position.y = map_x;   // un-swap
      mavros_pose.pose.position.z = current_pose_.pose.position.z;  // actual MAVROS Z
      mavros_pose.pose.orientation.w = 1.0;
      mavros_path.poses.push_back(mavros_pose);

      RCLCPP_INFO(this->get_logger(),
        "  WP map=(%.2f, %.2f) -> MAVROS=(%.2f, %.2f)",
        map_x, map_y, mavros_pose.pose.position.x, mavros_pose.pose.position.y);
    }

    planned_path_pub_->publish(mavros_path);
    planned_path_viz_pub_->publish(viz_path);

    RCLCPP_INFO(this->get_logger(),
      "Published planned path: %zu waypoints", path_cells.size());
  }

  // ═══════════════════════════════════════════════════════════════
  //  Parameters
  // ═══════════════════════════════════════════════════════════════
  double cruise_altitude_;
  double grid_resolution_;
  double inflation_radius_;
  double max_plan_distance_;
  double altitude_slice_thickness_;
  double replan_interval_seconds_;
  double path_check_step_;

  // ═══════════════════════════════════════════════════════════════
  //  ROS interfaces
  // ═══════════════════════════════════════════════════════════════
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_viz_pub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;

  // ═══════════════════════════════════════════════════════════════
  //  State
  // ═══════════════════════════════════════════════════════════════
  std::mutex tree_mutex_;
  std::unique_ptr<octomap::OcTree> tree_;
  bool map_received_{false};
  bool have_pose_{false};
  bool have_goal_{false};

  geometry_msgs::msg::PoseStamped current_pose_;
  double goal_map_x_{0.0};
  double goal_map_y_{0.0};

  rclcpp::Time last_replan_time_;
  std::vector<GridCell> last_planned_cells_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
