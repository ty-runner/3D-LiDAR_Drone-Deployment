#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class TargetTrackerNode : public rclcpp::Node
{
public:
  TargetTrackerNode()
  : Node("target_tracker_node"),
    scan_count_(0),
    capture_next_(false),
    bg_valid_(false)
  {
    // Detection parameters
    min_range_m_       = this->declare_parameter<double>("min_range_m", 0.25);
    max_range_m_       = this->declare_parameter<double>("max_range_m", 9.0);
    delta_range_m_     = this->declare_parameter<double>("delta_range_m", 0.20);
    min_cluster_beams_ = this->declare_parameter<int>("min_cluster_beams", 5);

    // Sector filter (degrees, in laser_frame)
    sector_min_deg_ = this->declare_parameter<double>("sector_min_deg", -180.0);
    sector_max_deg_ = this->declare_parameter<double>("sector_max_deg", 180.0);

    // Bearing offset to align LiDAR 0deg with physical forward direction
    bearing_offset_deg_ = this->declare_parameter<double>("bearing_offset_deg", 180.0);

    // Debug
    debug_enabled_ = this->declare_parameter<bool>("debug_enabled", true);
    debug_every_n_ = this->declare_parameter<int>("debug_every_n", 10);

    // Publishers
    bearing_pub_  = this->create_publisher<std_msgs::msg::Float32>("/target_bearing_deg", 10);
    range_pub_    = this->create_publisher<std_msgs::msg::Float32>("/target_range_m", 10);
    detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/target_detected", 10);
    width_pub_    = this->create_publisher<std_msgs::msg::Float32>("/target_cluster_width_deg", 10);
    marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TargetTrackerNode::scan_callback, this, std::placeholders::_1));

    capture_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/tracker/capture_bg", 10,
      std::bind(&TargetTrackerNode::capture_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "TargetTracker (BG snapshot): delta=%.3fm, cluster=%d, range=[%.1f, %.1f]m",
      delta_range_m_, min_cluster_beams_, min_range_m_, max_range_m_);
  }

private:
  bool is_valid_range(float r) const
  {
    return std::isfinite(r) && r >= min_range_m_ && r <= max_range_m_;
  }

  float beam_angle_deg(int idx, float angle_min, float angle_inc) const
  {
    return (angle_min + idx * angle_inc) * 180.0f / M_PI;
  }

  void capture_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      capture_next_ = true;
      RCLCPP_INFO(this->get_logger(), "BG CAPTURE requested");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_count_++;
    std::vector<float> curr(msg->ranges.begin(), msg->ranges.end());
    int num_beams = static_cast<int>(curr.size());

    // --- Background capture: accumulate N scans, then average ---
    if (capture_next_)
    {
      if (bg_accumulator_.empty())
      {
        // First scan — initialize accumulator
        bg_accumulator_.resize(num_beams, 0.0f);
        bg_count_.resize(num_beams, 0);
        bg_scans_collected_ = 0;
        RCLCPP_INFO(this->get_logger(), "BG CAPTURE: collecting %d scans...", bg_scans_needed_);
      }

      // Accumulate valid ranges per beam
      for (int i = 0; i < num_beams; i++)
      {
        if (std::isfinite(curr[i]) && curr[i] >= min_range_m_ && curr[i] <= max_range_m_)
        {
          bg_accumulator_[i] += curr[i];
          bg_count_[i]++;
        }
      }
      bg_scans_collected_++;

      if (bg_scans_collected_ >= bg_scans_needed_)
      {
        // Finalize: compute average per beam
        bg_ranges_.resize(num_beams);
        for (int i = 0; i < num_beams; i++)
        {
          if (bg_count_[i] > 0)
            bg_ranges_[i] = bg_accumulator_[i] / bg_count_[i];
          else
            bg_ranges_[i] = std::numeric_limits<float>::infinity();
        }
        bg_valid_ = true;
        capture_next_ = false;
        bg_accumulator_.clear();
        bg_count_.clear();
        RCLCPP_INFO(this->get_logger(),
          "BG CAPTURED: %d beams averaged over %d scans", num_beams, bg_scans_needed_);
      }

      std_msgs::msg::Bool det_msg;
      det_msg.data = false;
      detected_pub_->publish(det_msg);
      return;
    }

    // --- Detection: compare against frozen background ---
    bool detected = false;
    float best_angle_rad = 0.0f;
    float best_range_m = 0.0f;
    int best_cluster_size = 0;
    float best_width_deg = 0.0f;

    if (bg_valid_ && static_cast<int>(bg_ranges_.size()) == num_beams)
    {
      int run = 0;
      int run_start = -1;
      int max_run = 0;
      int max_start = -1;
      int max_end = -1;

      for (int i = 0; i < num_beams; i++)
      {
        float ang_deg = beam_angle_deg(i, msg->angle_min, msg->angle_increment);

        if (ang_deg < sector_min_deg_ || ang_deg > sector_max_deg_)
        {
          if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
          run = 0; run_start = -1;
          continue;
        }

        float r_cur = curr[i];
        float r_bg = bg_ranges_[i];

        if (!is_valid_range(r_cur))
        {
          if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
          run = 0; run_start = -1;
          continue;
        }

        // Compare against background
        bool changed = false;
        if (is_valid_range(r_bg))
        {
          // Object is CLOSER than background (blocking the background)
          // Only detect things closer, not further — avoids detecting
          // when background object is removed
          changed = (r_bg - r_cur) >= delta_range_m_;
        }
        else
        {
          // Background had no return (inf), now something is there
          changed = true;
        }

        if (changed)
        {
          if (run == 0) run_start = i;
          run++;
        }
        else
        {
          if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
          run = 0; run_start = -1;
        }
      }
      if (run > max_run) { max_run = run; max_start = run_start; max_end = num_beams - 1; }

      if (max_run >= min_cluster_beams_ && max_start >= 0 && max_end >= max_start)
      {
        detected = true;
        best_cluster_size = max_run;

        float sum_r = 0.0f;
        int count = 0;
        for (int j = max_start; j <= max_end; j++)
        {
          if (is_valid_range(curr[j])) { sum_r += curr[j]; count++; }
        }
        best_range_m = (count > 0) ? sum_r / count : 0.0f;

        int center_idx = (max_start + max_end) / 2;
        best_angle_rad = msg->angle_min + center_idx * msg->angle_increment;

        best_width_deg = (max_end - max_start + 1) * msg->angle_increment * 180.0f / M_PI;
      }
    }

    // Publish detection state
    std_msgs::msg::Bool det_msg;
    det_msg.data = detected;
    detected_pub_->publish(det_msg);

    if (detected)
    {
      float bearing_deg = best_angle_rad * 180.0f / M_PI;

      bearing_deg += static_cast<float>(bearing_offset_deg_);
      while (bearing_deg > 180.0f) bearing_deg -= 360.0f;
      while (bearing_deg < -180.0f) bearing_deg += 360.0f;

      std_msgs::msg::Float32 bearing_msg;
      bearing_msg.data = bearing_deg;
      bearing_pub_->publish(bearing_msg);

      std_msgs::msg::Float32 range_msg;
      range_msg.data = best_range_m;
      range_pub_->publish(range_msg);

      std_msgs::msg::Float32 width_msg;
      width_msg.data = best_width_deg;
      width_pub_->publish(width_msg);

      publish_marker(msg->header, best_angle_rad, best_range_m);
    }
    else
    {
      delete_marker(msg->header);
    }

    // Debug logging
    if (debug_enabled_ && (scan_count_ % debug_every_n_ == 0))
    {
      if (detected)
      {
        RCLCPP_INFO(this->get_logger(),
          "TARGET: bearing=%.1f° range=%.2fm width=%.1f° cluster=%d [bg_mode]",
          best_angle_rad * 180.0f / M_PI, best_range_m, best_width_deg,
          best_cluster_size);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
          "TARGET: none (bg_valid=%s, cluster=%d)",
          bg_valid_ ? "yes" : "no", best_cluster_size);
      }
    }
  }

  void publish_marker(const std_msgs::msg::Header& header, float angle_rad, float range_m)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "target";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = range_m * std::cos(angle_rad);
    marker.pose.position.y = range_m * std::sin(angle_rad);
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 500000000;
    marker_pub_->publish(marker);
  }

  void delete_marker(const std_msgs::msg::Header& header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "target";
    marker.id = 1;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr width_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  double min_range_m_, max_range_m_, delta_range_m_;
  int min_cluster_beams_;
  double sector_min_deg_, sector_max_deg_;
  double bearing_offset_deg_;
  bool debug_enabled_;
  int debug_every_n_;

  // State
  std::vector<float> bg_ranges_;
  std::vector<float> bg_accumulator_;
  std::vector<int> bg_count_;
  int bg_scans_collected_{0};
  int bg_scans_needed_{10};  // Average over 10 scans (~1 second at 10Hz)
  bool bg_valid_;
  bool capture_next_;
  int scan_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
