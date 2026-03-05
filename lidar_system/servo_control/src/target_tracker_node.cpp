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
    motion_streak_(0),
    scan_count_(0),
    last_detect_(false)
  {
    // Detection parameters
    min_range_m_     = this->declare_parameter<double>("min_range_m", 0.25);
    max_range_m_     = this->declare_parameter<double>("max_range_m", 9.0);
    delta_range_m_   = this->declare_parameter<double>("delta_range_m", 0.05);
    min_cluster_beams_ = this->declare_parameter<int>("min_cluster_beams", 3);
    min_motion_frames_ = this->declare_parameter<int>("min_motion_frames", 2);

    // Sector filter (degrees, in laser_frame)
    sector_min_deg_ = this->declare_parameter<double>("sector_min_deg", -180.0);
    sector_max_deg_ = this->declare_parameter<double>("sector_max_deg", 180.0);

    // Debug
    debug_enabled_ = this->declare_parameter<bool>("debug_enabled", true);
    debug_every_n_ = this->declare_parameter<int>("debug_every_n", 10);

    // Publishers
    bearing_pub_  = this->create_publisher<std_msgs::msg::Float32>("/target_bearing_deg", 10);
    range_pub_    = this->create_publisher<std_msgs::msg::Float32>("/target_range_m", 10);
    detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/target_detected", 10);
    marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

    // Subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TargetTrackerNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "TargetTracker started: delta=%.3fm, cluster=%d, frames=%d, range=[%.1f, %.1f]m",
      delta_range_m_, min_cluster_beams_, min_motion_frames_, min_range_m_, max_range_m_);
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
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_count_++;
        std::vector<float> curr(msg->ranges.begin(), msg->ranges.end());
        int num_beams = static_cast<int>(curr.size());

        bool detected_now = false;
        float best_angle_rad = 0.0f;
        float best_range_m = 0.0f;
        int best_cluster_size = 0;

        // Compare with previous scan
        if (!prev_ranges_.empty() && static_cast<int>(prev_ranges_.size()) == num_beams)
        {
            // Find largest cluster of beams with range change > delta
            int run = 0;
            int run_start = -1;
            int max_run = 0;
            int max_start = -1;
            int max_end = -1;

            for (int i = 0; i < num_beams; i++)
            {
                float ang_deg = beam_angle_deg(i, msg->angle_min, msg->angle_increment);

                // Sector filter
                if (ang_deg < sector_min_deg_ || ang_deg > sector_max_deg_)
                {
                    if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
                    run = 0; run_start = -1;
                    continue;
                }

                float r_cur = curr[i];
                float r_prev = prev_ranges_[i];

                if (!is_valid_range(r_cur) || !is_valid_range(r_prev))
                {
                    if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
                    run = 0; run_start = -1;
                    continue;
                }

                if (std::abs(r_cur - r_prev) >= delta_range_m_)
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
            // Check final run
            if (run > max_run) { max_run = run; max_start = run_start; max_end = num_beams - 1; }

            if (max_run >= min_cluster_beams_ && max_start >= 0 && max_end >= max_start)
            {
                detected_now = true;
                best_cluster_size = max_run;
                int center_idx = (max_start + max_end) / 2;
                best_angle_rad = msg->angle_min + center_idx * msg->angle_increment;
                best_range_m = curr[center_idx];

                if (!is_valid_range(best_range_m))
                {
                    // Fallback: average of cluster
                    float sum_r = 0.0f;
                    int count = 0;
                    for (int j = max_start; j <= max_end; j++)
                    {
                        if (is_valid_range(curr[j])) { sum_r += curr[j]; count++; }
                    }
                    best_range_m = (count > 0) ? sum_r / count : 0.0f;
                }
            }
        }

        // Motion streak filter: weighted average for smoother motion streak
        if (detected_now)
        {
            motion_streak_ = std::min(10, motion_streak_ + 1); // cap the streak to 10 to avoid overflow
        }
        else
        {
            motion_streak_ = std::max(0, motion_streak_ - 1); // decrement if no detection
        }

        // Apply smoothing with history: weighted decision on detection status
        bool detected = (motion_streak_ >= min_motion_frames_);

        // Publish detection state
        std_msgs::msg::Bool det_msg;
        det_msg.data = detected;
        detected_pub_->publish(det_msg);

        // Publish bearing and range if detected
        if (detected && detected_now)
        {
            float bearing_deg = best_angle_rad * 180.0f / M_PI;

            std_msgs::msg::Float32 bearing_msg;
            bearing_msg.data = bearing_deg;
            bearing_pub_->publish(bearing_msg);

            std_msgs::msg::Float32 range_msg;
            range_msg.data = best_range_m;
            range_pub_->publish(range_msg);

            // Publish RViz marker
            publish_marker(msg->header, best_angle_rad, best_range_m);
        }
        else
        {
            // Delete marker when no target
            delete_marker(msg->header);
        }

        // Store for next comparison
        prev_ranges_ = curr;
        last_detect_ = detected;

        // Debug logging with smoothing
        if (debug_enabled_ && (scan_count_ % debug_every_n_ == 0))
        {
            if (detected)
            {
                RCLCPP_INFO(this->get_logger(),
                    "TARGET detected: bearing=%.1f° range=%.2fm cluster=%d streak=%d",
                    best_angle_rad * 180.0f / M_PI, best_range_m, best_cluster_size, motion_streak_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                    "TARGET: none (streak=%d, last_cluster=%d)",
                    motion_streak_, best_cluster_size);
            }
        }
    }
  /*void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_count_++;
    std::vector<float> curr(msg->ranges.begin(), msg->ranges.end());
    int num_beams = static_cast<int>(curr.size());

    bool detected_now = false;
    float best_angle_rad = 0.0f;
    float best_range_m = 0.0f;
    int best_cluster_size = 0;

    // Compare with previous scan
    if (!prev_ranges_.empty() && static_cast<int>(prev_ranges_.size()) == num_beams)
    {
      // Find largest cluster of beams with range change > delta
      int run = 0;
      int run_start = -1;
      int max_run = 0;
      int max_start = -1;
      int max_end = -1;

      for (int i = 0; i < num_beams; i++)
      {
        float ang_deg = beam_angle_deg(i, msg->angle_min, msg->angle_increment);

        // Sector filter
        if (ang_deg < sector_min_deg_ || ang_deg > sector_max_deg_)
        {
          if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
          run = 0; run_start = -1;
          continue;
        }

        float r_cur = curr[i];
        float r_prev = prev_ranges_[i];

        if (!is_valid_range(r_cur) || !is_valid_range(r_prev))
        {
          if (run > max_run) { max_run = run; max_start = run_start; max_end = i - 1; }
          run = 0; run_start = -1;
          continue;
        }

        if (std::abs(r_cur - r_prev) >= delta_range_m_)
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
      // Check final run
      if (run > max_run) { max_run = run; max_start = run_start; max_end = num_beams - 1; }

      if (max_run >= min_cluster_beams_ && max_start >= 0 && max_end >= max_start)
      {
        detected_now = true;
        best_cluster_size = max_run;
        int center_idx = (max_start + max_end) / 2;
        best_angle_rad = msg->angle_min + center_idx * msg->angle_increment;
        best_range_m = curr[center_idx];
        if (!is_valid_range(best_range_m))
        {
          // Fallback: average of cluster
          float sum_r = 0.0f;
          int count = 0;
          for (int j = max_start; j <= max_end; j++)
          {
            if (is_valid_range(curr[j])) { sum_r += curr[j]; count++; }
          }
          best_range_m = (count > 0) ? sum_r / count : 0.0f;
        }
      }
    }

    // Motion streak filter
    if (detected_now)
      motion_streak_++;
    else
      motion_streak_ = std::max(0, motion_streak_ - 1);

    bool detected = motion_streak_ >= min_motion_frames_;

    // Publish detection state
    std_msgs::msg::Bool det_msg;
    det_msg.data = detected;
    detected_pub_->publish(det_msg);

    // Publish bearing and range if detected
    if (detected && detected_now)
    {
      float bearing_deg = best_angle_rad * 180.0f / M_PI;

      std_msgs::msg::Float32 bearing_msg;
      bearing_msg.data = bearing_deg;
      bearing_pub_->publish(bearing_msg);

      std_msgs::msg::Float32 range_msg;
      range_msg.data = best_range_m;
      range_pub_->publish(range_msg);

      // Publish rviz marker
      publish_marker(msg->header, best_angle_rad, best_range_m);
    }
    else
    {
      // Delete marker when no target
      delete_marker(msg->header);
    }

    // Store for next comparison
    prev_ranges_ = curr;
    last_detect_ = detected;

    // Debug logging
    if (debug_enabled_ && (scan_count_ % debug_every_n_ == 0))
    {
      if (detected)
      {
        RCLCPP_INFO(this->get_logger(),
          "TARGET detected: bearing=%.1f° range=%.2fm cluster=%d streak=%d",
          best_angle_rad * 180.0f / M_PI, best_range_m, best_cluster_size, motion_streak_);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
          "TARGET: none (streak=%d, last_cluster=%d)",
          motion_streak_, best_cluster_size);
      }
    }
  }*/

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
    marker.lifetime.nanosec = 500000000;  // 500ms
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

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  double min_range_m_, max_range_m_, delta_range_m_;
  int min_cluster_beams_, min_motion_frames_;
  double sector_min_deg_, sector_max_deg_;
  bool debug_enabled_;
  int debug_every_n_;

  // State
  std::vector<float> prev_ranges_;
  int motion_streak_;
  int scan_count_;
  bool last_detect_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
