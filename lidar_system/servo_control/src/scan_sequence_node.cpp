#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <string>

class ScanSequenceNode : public rclcpp::Node
{
public:
  ScanSequenceNode()
  : Node("scan_sequence_node"),
    // Mapping state
    pan_angle_(0.0), tilt_angle_(-45.0),
    tilt_step_(10.0), pan_step_(10.0),
    map_state_(MapState::MOVE),
    settle_start_(this->get_clock()->now()),
    scan_received_(false),
    // Tracking state
    mode_(Mode::MAPPING),
    target_bearing_deg_(0.0),
    target_detected_(false),
    track_pan_cmd_(0.0),
    last_target_time_(this->get_clock()->now())
  {
    // Tracking parameters
    track_tilt_deg_  = this->declare_parameter<double>("track_tilt_deg", 0.0);
    track_kp_        = this->declare_parameter<double>("track_kp", 0.45);
    track_max_step_  = this->declare_parameter<double>("track_max_step_deg", 8.0);
    track_deadband_  = this->declare_parameter<double>("track_deadband_deg", 0.8);
    target_timeout_  = this->declare_parameter<double>("target_timeout_sec", 1.0);
    pan_min_deg_     = this->declare_parameter<double>("pan_min_deg", -90.0);
    pan_max_deg_     = this->declare_parameter<double>("pan_max_deg", 90.0);
    track_rate_hz_   = this->declare_parameter<double>("track_rate_hz", 12.0);

    // Servo angle publisher (used by both modes)
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angles", 10);

    // Mapping subscribers
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ScanSequenceNode::scan_callback, this, std::placeholders::_1));

    // Tracking subscribers
    bearing_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_bearing_deg", 10,
      std::bind(&ScanSequenceNode::bearing_callback, this, std::placeholders::_1));

    detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/target_detected", 10,
      std::bind(&ScanSequenceNode::detected_callback, this, std::placeholders::_1));

    //3d_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    //  "/3d_target_detected", 10,
    //  std::bind(&ScanSequenceNode::3d_detected_callback, this, std::placeholders::_1));

    // Mapping timer (step-and-scan at 20Hz check rate)
    map_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ScanSequenceNode::map_timer_callback, this));

    // Tracking timer (P-control loop)
    double track_period_ms = 1000.0 / track_rate_hz_;
    track_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(track_period_ms)),
      std::bind(&ScanSequenceNode::track_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "ScanSequence with MAPPING + TRACKING modes. Publish 'map' or 'track' to /tracker/mode");
    RCLCPP_INFO(this->get_logger(),
      "Tracking: kp=%.2f, max_step=%.1f°, deadband=%.1f°, timeout=%.1fs",
      track_kp_, track_max_step_, track_deadband_, target_timeout_);
  }

private:
  // ======================= MODE MANAGEMENT =======================
  enum class Mode { MAPPING, TRACKING };

  // ======================= MAPPING MODE =======================
  enum class MapState { MOVE, SETTLING, WAIT_FOR_SCAN };

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/)
  {
    if (mode_ == Mode::MAPPING && map_state_ == MapState::WAIT_FOR_SCAN) {
      scan_received_ = true;
    }
  }

  void map_timer_callback()
  {
    if (mode_ != Mode::MAPPING) return;

    switch (map_state_) {
      case MapState::MOVE:
      {
        publish_angles(pan_angle_, tilt_angle_);
        settle_start_ = this->get_clock()->now();
        map_state_ = MapState::SETTLING;
        break;
      }

      case MapState::SETTLING:
      {
        auto elapsed = (this->get_clock()->now() - settle_start_).seconds();
        if (elapsed >= 0.4) {
          scan_received_ = false;
          map_state_ = MapState::WAIT_FOR_SCAN;
        }
        publish_angles(pan_angle_, tilt_angle_);
        break;
      }

      case MapState::WAIT_FOR_SCAN:
      {
        publish_angles(pan_angle_, tilt_angle_);
        if (scan_received_) {
          advance_mapping_angles();
          map_state_ = MapState::MOVE;
        }
        break;
      }
    }
  }

  void advance_mapping_angles()
  {
    // Zigzag tilt (no big jumps)
    tilt_angle_ += tilt_step_;

    if (tilt_angle_ > 90.0 || tilt_angle_ < -45.0) {
      tilt_step_ *= -1;  // Reverse tilt direction
      tilt_angle_ += tilt_step_;
      pan_angle_ += pan_step_;

      if (pan_angle_ > 45.0 || pan_angle_ < -45.0) {
        pan_step_ *= -1;  // Reverse pan direction
        pan_angle_ += pan_step_;
        RCLCPP_INFO(this->get_logger(), "Full scan cycle complete, restarting");
      }
      RCLCPP_INFO(this->get_logger(), "Pan stepped to %.2f°, Tilt at %.2f°",
                  pan_angle_, tilt_angle_);
    }
  }

  // ======================= TRACKING MODE =======================
  void bearing_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_bearing_deg_ = msg->data;
    last_target_time_ = this->get_clock()->now();
  }

  void detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    target_detected_ = false;//msg->data;
    if(target_detected_ == last_detection_state)
      frames_to_expire--;
    if (target_detected_ && mode_ != Mode::TRACKING && frames_to_expire <= 0)
    {
      frames_to_expire = FRAME_COUNT; //reset
      mode_ = Mode::TRACKING;
      // Initialize tracking pan from current mapping pan
      track_pan_cmd_ = pan_angle_;
      RCLCPP_INFO(this->get_logger(), "MODE -> TRACKING (pan starting at %.1f°)", track_pan_cmd_);
    }
    else if (!target_detected_ && mode_ != Mode::MAPPING && frames_to_expire <= 0)
    {
      mode_ = Mode::MAPPING;
      frames_to_expire = FRAME_COUNT; //reset
      // Reset mapping state
      map_state_ = MapState::MOVE;
      RCLCPP_INFO(this->get_logger(), "MODE -> MAPPING");
    }
    last_detection_state = target_detected_;
  }

  void track_timer_callback()
  {
    if (mode_ != Mode::TRACKING) return;

    double now_sec = this->get_clock()->now().seconds();
    double target_age = now_sec - last_target_time_.seconds();

    if (target_detected_ && target_age <= target_timeout_)
    {
      // Target bearing is in laser_frame (relative to current pan).
      // Desired pan = current pan + target bearing
      // Negative because if target is at +30° in laser frame,
      // we need to pan by +30° to center it.
      //if(track_tilt_deg_ > 45.0 || track_tilt_deg_ < -15.0)
        //track_tilt_increment_ *= -1;
      //track_tilt_deg_ += track_tilt_increment_;

      double desired_pan = track_pan_cmd_ + target_bearing_deg_;
      desired_pan = clamp(desired_pan, pan_min_deg_, pan_max_deg_);

      double error = desired_pan - track_pan_cmd_;

      if (std::abs(error) > track_deadband_)
      {
        double delta = track_kp_ * error;
        delta = clamp(delta, -track_max_step_, track_max_step_);
        track_pan_cmd_ = clamp(track_pan_cmd_ + delta, pan_min_deg_, pan_max_deg_);
      }
    }
    // If no target, hold current pan position (don't drift)

    publish_angles(track_pan_cmd_, track_tilt_deg_);
  }

  // ======================= UTILITIES =======================
  void publish_angles(double pan, double tilt)
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.push_back(pan);
    msg.data.push_back(tilt);
    angle_publisher_->publish(msg);
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bearing_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr track_timer_;

  // Mapping state
  double pan_angle_;
  double tilt_angle_;
  double tilt_step_;
  double pan_step_;
  MapState map_state_;
  rclcpp::Time settle_start_;
  bool scan_received_;

  // Tracking state
  Mode mode_;
  bool last_detection_state = false;
  int FRAME_COUNT {10};
  int frames_to_expire {FRAME_COUNT};

  double target_bearing_deg_;
  bool target_detected_;
  double track_pan_cmd_;
  rclcpp::Time last_target_time_;

  // Tracking parameters
  double track_tilt_deg_;
  double track_tilt_increment_;
  double track_kp_;
  double track_max_step_;
  double track_deadband_;
  double target_timeout_;
  double pan_min_deg_;
  double pan_max_deg_;
  double track_rate_hz_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSequenceNode>());
  rclcpp::shutdown();
  return 0;
}
