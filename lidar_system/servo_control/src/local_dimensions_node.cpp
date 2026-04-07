#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"

#include "servo_control/srv/get_local_dimensions.hpp"

struct XYOffset
{
  double dx;
  double dy;
};

enum class CellState
{
  FREE,
  OCCUPIED,
  UNKNOWN
};

class LocalDimensionsNode : public rclcpp::Node
{
public:
  LocalDimensionsNode()
  : Node("local_dimensions_node")
  {
    octomap_topic_ = this->declare_parameter<std::string>("octomap_topic", "/octomap_binary");
    drone_radius_ = this->declare_parameter<double>("drone_radius", 0.50);
    drone_height_ = this->declare_parameter<double>("drone_height", 0.20);
    footprint_step_ = this->declare_parameter<double>("footprint_step", 0.10);
    search_up_distance_ = this->declare_parameter<double>("search_up_distance", 3.0);
    search_down_distance_ = this->declare_parameter<double>("search_down_distance", 3.0);
    unknown_is_blocked_ = this->declare_parameter<bool>("unknown_is_blocked", false);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LocalDimensionsNode::octomapCallback, this, std::placeholders::_1));

    service_ = this->create_service<servo_control::srv::GetLocalDimensions>(
      "get_local_dimensions",
      std::bind(
        &LocalDimensionsNode::handleService,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "LocalDimensionsNode started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to OctoMap topic: %s", octomap_topic_.c_str());
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (!tree) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert OctoMap message to AbstractOcTree");
      return;
    }

    octomap::OcTree* oc_tree = dynamic_cast<octomap::OcTree*>(tree);
    if (!oc_tree) {
      RCLCPP_WARN(this->get_logger(), "Received OctoMap is not an OcTree");
      delete tree;
      return;
    }

    tree_.reset(oc_tree);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Received OctoMap. Resolution: %.3f",
      tree_->getResolution());
  }

  void handleService(
    const std::shared_ptr<servo_control::srv::GetLocalDimensions::Request> request,
    std::shared_ptr<servo_control::srv::GetLocalDimensions::Response> response)
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);

    if (!tree_) {
      response->success = false;
      response->message = "No OctoMap received yet";
      return;
    }

    QueryResult result = evaluateLocalDimensions(request->x, request->y, request->z);
    RCLCPP_INFO(
      this->get_logger(),
      "Query at (%.2f, %.2f, %.2f) -> success=%d | floor=%.2f ceiling=%.2f height=%.2f | below=%.2f above=%.2f",
      request->x,
      request->y,
      request->z,
      result.success,
      result.floor_z,
      result.ceiling_z,
      result.room_height,
      result.clearance_below,
      result.clearance_above
    );
    response->success = result.success;
    response->message = result.message;
    response->floor_z = result.floor_z;
    response->ceiling_z = result.ceiling_z;
    response->room_height = result.room_height;
    response->clearance_below = result.clearance_below;
    response->clearance_above = result.clearance_above;
  }

  struct QueryResult
  {
    bool success = false;
    std::string message;
    double floor_z = 0.0;
    double ceiling_z = 0.0;
    double room_height = 0.0;
    double clearance_below = 0.0;
    double clearance_above = 0.0;
  };

  std::vector<XYOffset> makeFootprintSamples(double radius, double step) const
  {
    std::vector<XYOffset> samples;

    if (step <= 0.0) {
      step = radius;
    }

    for (double x = -radius; x <= radius; x += step) {
      for (double y = -radius; y <= radius; y += step) {
        if ((x * x + y * y) <= radius * radius) {
          samples.push_back({x, y});
        }
      }
    }

    samples.push_back({0.0, 0.0});
    return samples;
  }

  CellState getCellState(double x, double y, double z) const
  {
    octomap::OcTreeNode* node = tree_->search(x, y, z);

    if (!node) {
      return CellState::UNKNOWN;
    }

    if (tree_->isNodeOccupied(node)) {
      return CellState::OCCUPIED;
    }

    return CellState::FREE;
  }

  bool searchVerticalSurface(
    double x,
    double y,
    double z_start,
    double z_limit,
    double step,
    bool search_up,
    double& hit_z) const
  {
    if (step <= 0.0) {
      return false;
    }

    if (search_up) {
      for (double z = z_start; z <= z_limit; z += step) {
        CellState state = getCellState(x, y, z);

        if (state == CellState::OCCUPIED) {
          hit_z = z;
          return true;
        }

        if (state == CellState::UNKNOWN && unknown_is_blocked_) {
          hit_z = z;
          return true;
        }
      }
    } else {
      for (double z = z_start; z >= z_limit; z -= step) {
        CellState state = getCellState(x, y, z);

        if (state == CellState::OCCUPIED) {
          hit_z = z;
          return true;
        }

        if (state == CellState::UNKNOWN && unknown_is_blocked_) {
          hit_z = z;
          return true;
        }
      }
    }

    return false;
  }

  bool isCylinderBodyFree(double cx, double cy, double cz) const
  {
    const double res = tree_->getResolution();
    const double z_step = res;
    const double xy_step = std::max(res, footprint_step_);

    const double z_min = cz - 0.5 * drone_height_;
    const double z_max = cz + 0.5 * drone_height_;

    std::vector<XYOffset> samples = makeFootprintSamples(drone_radius_, xy_step);

    for (const auto& s : samples) {
      const double x = cx + s.dx;
      const double y = cy + s.dy;

      for (double z = z_min; z <= z_max; z += z_step) {
        CellState state = getCellState(x, y, z);

        if (state == CellState::OCCUPIED) {
          return false;
        }

        if (state == CellState::UNKNOWN && unknown_is_blocked_) {
          return false;
        }
      }
    }

    return true;
  }

  QueryResult evaluateLocalDimensions(double cx, double cy, double cz) const
  {
    QueryResult result;

    if (!isCylinderBodyFree(cx, cy, cz)) {
      result.success = false;
      result.message = "Candidate body volume is occupied or unknown";
      return result;
    }

    const double res = tree_->getResolution();
    const double z_step = res;
    const double xy_step = std::max(res, footprint_step_);

    const double body_z_min = cz - 0.5 * drone_height_;
    const double body_z_max = cz + 0.5 * drone_height_;

    const double search_z_min = body_z_min - search_down_distance_;
    const double search_z_max = body_z_max + search_up_distance_;

    bool any_floor = false;
    bool any_ceiling = false;

    double conservative_floor = -std::numeric_limits<double>::infinity();
    double conservative_ceiling = std::numeric_limits<double>::infinity();

    std::vector<XYOffset> samples = makeFootprintSamples(drone_radius_, xy_step);

    for (const auto& s : samples) {
      const double x = cx + s.dx;
      const double y = cy + s.dy;

      double floor_hit = 0.0;
      double ceiling_hit = 0.0;

      bool floor_found = searchVerticalSurface(
        x, y, body_z_min, search_z_min, z_step, false, floor_hit);

      bool ceiling_found = searchVerticalSurface(
        x, y, body_z_max, search_z_max, z_step, true, ceiling_hit);

      if (floor_found) {
        any_floor = true;
        conservative_floor = std::max(conservative_floor, floor_hit);
      }

      if (ceiling_found) {
        any_ceiling = true;
        conservative_ceiling = std::min(conservative_ceiling, ceiling_hit);
      }
    }

    if (!any_floor) {
      result.success = false;
      result.message = "No floor found below query location";
      return result;
    }

    if (!any_ceiling) {
      result.success = false;
      result.message = "No ceiling found above query location";
      return result;
    }

    if (conservative_floor >= conservative_ceiling) {
      result.success = false;
      result.message = "Invalid geometry: floor is above or equal to ceiling";
      return result;
    }

    result.success = true;
    result.message = "Local dimensions found";
    result.floor_z = conservative_floor;
    result.ceiling_z = conservative_ceiling;
    result.room_height = conservative_ceiling - conservative_floor;
    result.clearance_below = body_z_min - conservative_floor;
    result.clearance_above = conservative_ceiling - body_z_max;
    return result;
  }

  std::string octomap_topic_;
  double drone_radius_;
  double drone_height_;
  double footprint_step_;
  double search_up_distance_;
  double search_down_distance_;
  bool unknown_is_blocked_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Service<servo_control::srv::GetLocalDimensions>::SharedPtr service_;

  mutable std::mutex tree_mutex_;
  std::unique_ptr<octomap::OcTree> tree_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalDimensionsNode>());
  rclcpp::shutdown();
  return 0;
}
