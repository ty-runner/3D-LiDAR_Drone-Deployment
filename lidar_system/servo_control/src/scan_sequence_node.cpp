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
    // Mode
    mode_(Mode::MAPPING),
    // Tracking state
    target_bearing_deg_(0.0),
    target_range_m_(0.0),
    target_width_deg_(0.0),
    target_detected_(false),
    track_pan_cmd_(0.0),
    track_state_(TrackState::PAN_SETTLE),
    pan_settle_start_(this->get_clock()->now()),
    detect_start_(this->get_clock()->now()),
    // Tilt sweep state
    tilt_sweep_angle_(0.0),
    sweep_phase_(SweepPhase::UP),
    tilt_top_deg_(0.0),
    tilt_bottom_deg_(0.0),
    consecutive_misses_(0),
    tilt_scan_received_(false),
    tilt_settle_start_(this->get_clock()->now()),
    last_target_time_(this->get_clock()->now())
  {
    // ===================== PARAMETERS =====================
    // Tracking pan parameters
    track_tilt_deg_    = this->declare_parameter<double>("track_tilt_deg", 0.0);
    track_kp_          = this->declare_parameter<double>("track_kp", 0.45);
    track_max_step_    = this->declare_parameter<double>("track_max_step_deg", 8.0);
    track_deadband_    = this->declare_parameter<double>("track_deadband_deg", 0.8);
    target_timeout_    = this->declare_parameter<double>("target_timeout_sec", 1.0);
    pan_min_deg_       = this->declare_parameter<double>("pan_min_deg", -90.0);
    pan_max_deg_       = this->declare_parameter<double>("pan_max_deg", 90.0);
    track_rate_hz_     = this->declare_parameter<double>("track_rate_hz", 12.0);

    // Step-and-detect parameters
    lock_bearing_thresh_deg_ = this->declare_parameter<double>("lock_bearing_thresh_deg", 20.0);
    pan_settle_sec_          = this->declare_parameter<double>("pan_settle_sec", 3.0);
    search_timeout_sec_      = this->declare_parameter<double>("search_timeout_sec", 3.0);
    search_step_deg_         = this->declare_parameter<double>("search_step_deg", 10.0);

    // Tilt sweep parameters
    tilt_step_deg_       = this->declare_parameter<double>("tilt_step_deg", 3.0);
    tilt_max_up_deg_     = this->declare_parameter<double>("tilt_max_up_deg", 45.0);
    tilt_max_down_deg_   = this->declare_parameter<double>("tilt_max_down_deg", -25.0);
    tilt_settle_sec_     = this->declare_parameter<double>("tilt_settle_sec", 0.5);
    range_tolerance_m_   = this->declare_parameter<double>("range_tolerance_m", 0.3);
    bearing_pad_deg_     = this->declare_parameter<double>("bearing_pad_deg", 5.0);
    min_tilt_beams_      = this->declare_parameter<int>("min_tilt_beams", 2);
    miss_limit_          = this->declare_parameter<int>("miss_limit", 3);

    // Bearing offset (must match target_tracker_node's offset)
    bearing_offset_deg_  = this->declare_parameter<double>("bearing_offset_deg", 180.0);

    // ===================== PUBLISHERS =====================
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angles", 10);
    height_pub_      = this->create_publisher<std_msgs::msg::Float32>("/target_height_m", 10);
    track_state_pub_ = this->create_publisher<std_msgs::msg::String>("/tracker/state", 10);
    capture_bg_pub_  = this->create_publisher<std_msgs::msg::Bool>("/tracker/capture_bg", 10);

    // ===================== SUBSCRIBERS =====================
    // Scan data (used for mapping sync + tilt sweep analysis)
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ScanSequenceNode::scan_callback, this, std::placeholders::_1));

    // Target tracker outputs
    bearing_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_bearing_deg", 10,
      std::bind(&ScanSequenceNode::bearing_callback, this, std::placeholders::_1));

    detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/target_detected", 10,
      std::bind(&ScanSequenceNode::detected_callback, this, std::placeholders::_1));

    range_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_range_m", 10,
      std::bind(&ScanSequenceNode::range_callback, this, std::placeholders::_1));

    width_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_cluster_width_deg", 10,
      std::bind(&ScanSequenceNode::width_callback, this, std::placeholders::_1));

    // *** BUG FIX: Actually wire the mode subscriber ***
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/tracker/mode", 10,
      std::bind(&ScanSequenceNode::mode_callback, this, std::placeholders::_1));

    // ===================== TIMERS =====================
    // Mapping timer (step-and-scan)
    map_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(15),
      std::bind(&ScanSequenceNode::map_timer_callback, this));

    // Tracking timer (pan P-control + tilt state machine)
    double track_period_ms = 1000.0 / track_rate_hz_;
    track_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(track_period_ms)),
      std::bind(&ScanSequenceNode::track_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "ScanSequence ready. Publish 'map' or 'track' to /tracker/mode");
    RCLCPP_INFO(this->get_logger(),
      "Step-and-detect: kp=%.2f, max_step=%.1f°, settle=%.1fs, search_timeout=%.1fs",
      track_kp_, track_max_step_, pan_settle_sec_, search_timeout_sec_);
    RCLCPP_INFO(this->get_logger(),
      "Tilt sweep: step=%.1f°, range_tol=%.2fm, pad=%.1f°, min_beams=%d, miss_limit=%d",
      tilt_step_deg_, range_tolerance_m_, bearing_pad_deg_, min_tilt_beams_, miss_limit_);
  }

private:
  // ======================= ENUMS =======================
  enum class Mode { MAPPING, TRACKING };
  enum class MapState { MOVE, SETTLING, WAIT_FOR_SCAN };
  enum class TrackState { PAN_SETTLE, PAN_DETECT, TILT_MOVE, TILT_SETTLE, TILT_SCAN_WAIT, TILT_DONE, TRACK_IDLE };
  enum class SweepPhase { UP, DOWN };

  // ======================= MODE MANAGEMENT =======================
  void mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string cmd = msg->data;
    if (cmd == "map" && mode_ != Mode::MAPPING)
    {
      mode_ = Mode::MAPPING;
      map_state_ = MapState::MOVE;
      RCLCPP_INFO(this->get_logger(), "MODE -> MAPPING (manual)");
    }
    else if (cmd == "track")
    {
      // If already actively tracking (not IDLE), ignore repeated 'track' messages
      if (mode_ == Mode::TRACKING && track_state_ != TrackState::TRACK_IDLE)
      {
        RCLCPP_INFO(this->get_logger(), "Already tracking, ignoring repeated 'track' command");
        return;
      }

      // Start fresh from MAPPING or restart from IDLE
      double start_pan = 0.0;  // Always start facing forward
      if (mode_ != Mode::TRACKING) {
        bg_captured_ = false;  // New session — need fresh background
      }
      mode_ = Mode::TRACKING;
      track_state_ = TrackState::PAN_SETTLE;
      track_pan_cmd_ = start_pan;
      had_recent_detection_ = false;
      pan_settle_start_ = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(),
        "MODE -> TRACKING / PAN_SETTLE (pan starting at %.1f°)", track_pan_cmd_);
    }
    publish_state();
  }

  // ======================= SCAN CALLBACK =======================
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Always store latest scan for tilt analysis
    latest_scan_ = msg;

    // Mapping sync flag
    if (mode_ == Mode::MAPPING && map_state_ == MapState::WAIT_FOR_SCAN) {
      scan_received_ = true;
    }

    // Tilt sweep scan flag
    if (mode_ == Mode::TRACKING && track_state_ == TrackState::TILT_SCAN_WAIT) {
      tilt_scan_received_ = true;
    }
  }

  // ======================= TARGET DATA CALLBACKS =======================
  void bearing_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_bearing_deg_ = msg->data;
    last_target_time_ = this->get_clock()->now();
  }

  void detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // *** BUG FIX: Actually use the message data ***
    target_detected_ = msg->data;
  }

  void range_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_range_m_ = msg->data;
  }

  void width_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_width_deg_ = msg->data;
  }

  // ======================= MAPPING MODE =======================
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
    tilt_angle_ += tilt_step_;
    if (tilt_angle_ > 90.0 || tilt_angle_ < -45.0) {
      tilt_step_ *= -1;
      tilt_angle_ += tilt_step_;
      pan_angle_ += pan_step_;
      if (pan_angle_ > 45.0 || pan_angle_ < -45.0) {
        pan_step_ *= -1;
        pan_angle_ += pan_step_;
        RCLCPP_INFO(this->get_logger(), "Full scan cycle complete, restarting");
      }
      RCLCPP_INFO(this->get_logger(), "Pan stepped to %.2f°, Tilt at %.2f°",
                  pan_angle_, tilt_angle_);
    }
  }

  // ======================= TRACKING MODE =======================
  void track_timer_callback()
  {
    if (mode_ != Mode::TRACKING) return;

    switch (track_state_)
    {
      case TrackState::PAN_SETTLE:
        handle_pan_settle();
        break;
      case TrackState::PAN_DETECT:
        handle_pan_detect();
        break;
      case TrackState::TILT_MOVE:
        handle_tilt_move();
        break;
      case TrackState::TILT_SETTLE:
        handle_tilt_settle();
        break;
      case TrackState::TILT_SCAN_WAIT:
        handle_tilt_scan_wait();
        break;
      case TrackState::TILT_DONE:
        handle_tilt_done();
        break;
      case TrackState::TRACK_IDLE:
        handle_track_idle();
        break;
    }
  }

  // --- PAN_SETTLE: Hold pan still, capture bg once, then listen ---
  void handle_pan_settle()
  {
    publish_angles(track_pan_cmd_, 0.0);

    double elapsed = (this->get_clock()->now() - pan_settle_start_).seconds();
    if (elapsed >= pan_settle_sec_)
    {
      // Capture background ONCE at the very start, never again
      if (!bg_captured_)
      {
        std_msgs::msg::Bool cap_msg;
        cap_msg.data = true;
        capture_bg_pub_->publish(cap_msg);
        bg_captured_ = true;
        RCLCPP_INFO(this->get_logger(), "BG CAPTURED at pan=%.1f°", track_pan_cmd_);
      }

      target_detected_ = false;
      detect_start_ = this->get_clock()->now();
      track_state_ = TrackState::PAN_DETECT;
      RCLCPP_INFO(this->get_logger(),
        "PAN_DETECT: listening at pan=%.1f°", track_pan_cmd_);
    }
  }

  // --- PAN_DETECT: Pan stable, listen for real detections ---
  void handle_pan_detect()
  {
    publish_angles(track_pan_cmd_, 0.0);

    double now_sec = this->get_clock()->now().seconds();
    double target_age = now_sec - last_target_time_.seconds();

    if (target_detected_ && target_age <= target_timeout_)
    {
      double abs_bearing = std::abs(target_bearing_deg_);

      // Remember we found something near here
      had_recent_detection_ = true;
      last_detection_pan_ = track_pan_cmd_;

      // Target is centered enough → lock on
      if (abs_bearing < lock_bearing_thresh_deg_ && target_range_m_ > 0.1)
      {
        // Lock on — record target parameters for tilt sweep
        lock_range_m_      = target_range_m_;
        lock_bearing_rad_  = (target_bearing_deg_ - bearing_offset_deg_) * M_PI / 180.0;
        lock_width_rad_    = (target_width_deg_ + 2.0 * bearing_pad_deg_) * M_PI / 180.0;

        // Initialize tilt sweep
        tilt_sweep_angle_ = 0.0;
        sweep_phase_ = SweepPhase::UP;
        tilt_top_deg_ = 0.0;
        tilt_bottom_deg_ = 0.0;
        consecutive_misses_ = 0;
        track_state_ = TrackState::TILT_MOVE;

        RCLCPP_INFO(this->get_logger(),
          "LOCK ON: range=%.2fm bearing=%.1f° width=%.1f° -> starting tilt sweep",
          lock_range_m_, target_bearing_deg_, target_width_deg_);
        publish_state();
        return;
      }

      // Target detected but not centered → step toward it
      double step = clamp(target_bearing_deg_ * track_kp_,
                          -track_max_step_, track_max_step_);
      track_pan_cmd_ = clamp(track_pan_cmd_ + step, pan_min_deg_, pan_max_deg_);

      RCLCPP_INFO(this->get_logger(),
        "PAN_STEP: bearing=%.1f° -> stepping pan to %.1f° (delta=%.1f°)",
        target_bearing_deg_, track_pan_cmd_, step);

      pan_settle_start_ = this->get_clock()->now();
      track_state_ = TrackState::PAN_SETTLE;
      return;
    }

    // No detection — just wait. Background snapshot detects instantly when someone walks in.
  }

  // --- TILT_MOVE: Command servo to next tilt angle ---
  void handle_tilt_move()
  {
    publish_angles(track_pan_cmd_, tilt_sweep_angle_);  // Pan frozen, tilt to sweep angle
    tilt_settle_start_ = this->get_clock()->now();
    tilt_scan_received_ = false;
    track_state_ = TrackState::TILT_SETTLE;

    RCLCPP_INFO(this->get_logger(),
      "TILT_SWEEP [%s]: commanding tilt=%.1f° (pan frozen at %.1f°)",
      (sweep_phase_ == SweepPhase::UP ? "UP" : "DOWN"),
      tilt_sweep_angle_, track_pan_cmd_);
  }

  // --- TILT_SETTLE: Wait for servo to reach position ---
  void handle_tilt_settle()
  {
    publish_angles(track_pan_cmd_, tilt_sweep_angle_);
    double elapsed = (this->get_clock()->now() - tilt_settle_start_).seconds();
    if (elapsed >= tilt_settle_sec_)
    {
      tilt_scan_received_ = false;
      track_state_ = TrackState::TILT_SCAN_WAIT;
    }
  }

  // --- TILT_SCAN_WAIT: Wait for scan, then analyze ---
  void handle_tilt_scan_wait()
  {
    publish_angles(track_pan_cmd_, tilt_sweep_angle_);

    if (!tilt_scan_received_ || !latest_scan_) return;

    // Analyze scan using range gate
    int hit_count = count_target_beams(*latest_scan_);

    RCLCPP_INFO(this->get_logger(),
      "TILT_ANALYZE: tilt=%.1f° hit_count=%d (need>=%d) expected_range=%.2fm",
      tilt_sweep_angle_, hit_count, min_tilt_beams_,
      lock_range_m_ / std::cos(tilt_sweep_angle_ * M_PI / 180.0));

    if (hit_count >= min_tilt_beams_)
    {
      // Target present at this tilt angle
      consecutive_misses_ = 0;

      if (sweep_phase_ == SweepPhase::UP)
        tilt_top_deg_ = tilt_sweep_angle_;
      else
        tilt_bottom_deg_ = tilt_sweep_angle_;

      // Step to next tilt angle
      advance_tilt_sweep();
    }
    else
    {
      // No target at this angle
      consecutive_misses_++;

      if (consecutive_misses_ >= miss_limit_)
      {
        // Edge found
        if (sweep_phase_ == SweepPhase::UP)
        {
          RCLCPP_INFO(this->get_logger(),
            "TOP EDGE found at tilt=%.1f° (last hit=%.1f°)",
            tilt_sweep_angle_, tilt_top_deg_);

          // Switch to sweeping down from 0°
          sweep_phase_ = SweepPhase::DOWN;
          tilt_sweep_angle_ = 0.0;
          consecutive_misses_ = 0;

          // Step one down immediately since 0° was already confirmed
          tilt_sweep_angle_ = -tilt_step_deg_;
          track_state_ = TrackState::TILT_MOVE;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(),
            "BOTTOM EDGE found at tilt=%.1f° (last hit=%.1f°)",
            tilt_sweep_angle_, tilt_bottom_deg_);

          // Both edges found — done
          track_state_ = TrackState::TILT_DONE;
          publish_state();
        }
      }
      else
      {
        // Not enough misses yet, keep going
        advance_tilt_sweep();
      }
    }
  }

  // Step tilt angle in current sweep direction, check limits
  void advance_tilt_sweep()
  {
    if (sweep_phase_ == SweepPhase::UP)
    {
      tilt_sweep_angle_ += tilt_step_deg_;
      if (tilt_sweep_angle_ > tilt_max_up_deg_)
      {
        RCLCPP_WARN(this->get_logger(),
          "Tilt UP hit limit (%.1f°), switching to DOWN", tilt_max_up_deg_);
        sweep_phase_ = SweepPhase::DOWN;
        tilt_sweep_angle_ = -tilt_step_deg_;
        consecutive_misses_ = 0;
      }
    }
    else
    {
      tilt_sweep_angle_ -= tilt_step_deg_;
      if (tilt_sweep_angle_ < tilt_max_down_deg_)
      {
        RCLCPP_WARN(this->get_logger(),
          "Tilt DOWN hit limit (%.1f°), finishing sweep", tilt_max_down_deg_);
        track_state_ = TrackState::TILT_DONE;
        publish_state();
        return;
      }
    }
    track_state_ = TrackState::TILT_MOVE;
  }

  // --- TILT_DONE: Compute height, go to IDLE (no auto-loop) ---
  void handle_tilt_done()
  {
    // Compute target height from tilt edges and locked range
    double top_rad = tilt_top_deg_ * M_PI / 180.0;
    double bot_rad = tilt_bottom_deg_ * M_PI / 180.0;
    double height_m = lock_range_m_ * (std::tan(top_rad) - std::tan(bot_rad));

    RCLCPP_INFO(this->get_logger(),
      "TILT SWEEP COMPLETE: top=%.1f° bottom=%.1f° range=%.2fm -> height=%.2fm",
      tilt_top_deg_, tilt_bottom_deg_, lock_range_m_, height_m);

    // Publish height estimate
    std_msgs::msg::Float32 h_msg;
    h_msg.data = static_cast<float>(std::abs(height_m));
    height_pub_->publish(h_msg);

    // Return tilt to 0° and go to IDLE — NO auto-loop
    tilt_sweep_angle_ = 0.0;
    publish_angles(track_pan_cmd_, 0.0);
    track_state_ = TrackState::TRACK_IDLE;

    RCLCPP_INFO(this->get_logger(),
      "IDLE — sweep done. Send 'track' to re-scan or 'map' to return to mapping.");
    publish_state();
  }

  // --- TRACK_IDLE: Hold position, wait for manual command ---
  void handle_track_idle()
  {
    // Just hold current pan at tilt=0°, do nothing else
    publish_angles(track_pan_cmd_, 0.0);
  }

  // ======================= TILT SCAN ANALYSIS =======================
  // Count beams near target bearing that match expected range at current tilt
  int count_target_beams(const sensor_msgs::msg::LaserScan& scan)
  {
    double tilt_rad = tilt_sweep_angle_ * M_PI / 180.0;
    double cos_tilt = std::cos(tilt_rad);

    // Avoid division by zero at extreme tilt
    if (std::abs(cos_tilt) < 0.1) return 0;

    double expected_range = lock_range_m_ / cos_tilt;
    double half_margin = lock_width_rad_ / 2.0;  // already includes padding

    int count = 0;
    int num_beams = static_cast<int>(scan.ranges.size());

    for (int i = 0; i < num_beams; i++)
    {
      float beam_angle = scan.angle_min + i * scan.angle_increment;

      // Angular mask: only look at beams near target bearing
      if (std::abs(beam_angle - lock_bearing_rad_) > half_margin) continue;

      float r = scan.ranges[i];
      if (!std::isfinite(r) || r < 0.1) continue;

      // Range gate: is this beam's range close to expected?
      if (std::abs(r - expected_range) <= range_tolerance_m_) {
        count++;
      }
    }
    return count;
  }

  // ======================= UTILITIES =======================
  void publish_angles(double pan, double tilt)
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.push_back(pan);
    msg.data.push_back(tilt);
    angle_publisher_->publish(msg);
  }

  void publish_state()
  {
    auto msg = std_msgs::msg::String();
    if (mode_ == Mode::MAPPING)
      msg.data = "MAPPING";
    else
    {
      switch (track_state_) {
        case TrackState::PAN_SETTLE:    msg.data = "TRACK/PAN_SETTLE"; break;
        case TrackState::PAN_DETECT:    msg.data = "TRACK/PAN_DETECT"; break;
        case TrackState::TILT_MOVE:     msg.data = "TRACK/TILT_SWEEP"; break;
        case TrackState::TILT_SETTLE:   msg.data = "TRACK/TILT_SWEEP"; break;
        case TrackState::TILT_SCAN_WAIT:msg.data = "TRACK/TILT_SWEEP"; break;
        case TrackState::TILT_DONE:     msg.data = "TRACK/TILT_DONE"; break;
        case TrackState::TRACK_IDLE:    msg.data = "TRACK/IDLE"; break;
      }
    }
    track_state_pub_->publish(msg);
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  // ======================= MEMBERS =======================
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr track_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr capture_bg_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bearing_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr range_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr width_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr track_timer_;

  // Latest scan (for tilt analysis)
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // Mapping state
  double pan_angle_;
  double tilt_angle_;
  double tilt_step_;
  double pan_step_;
  MapState map_state_;
  rclcpp::Time settle_start_;
  bool scan_received_;

  // Mode
  Mode mode_;

  // Tracking: target data from subscribers
  double target_bearing_deg_;
  double target_range_m_;
  double target_width_deg_;
  bool target_detected_;
  rclcpp::Time last_target_time_;

  // Tracking: pan state (step-and-detect)
  double track_pan_cmd_;
  TrackState track_state_;
  rclcpp::Time pan_settle_start_;
  rclcpp::Time detect_start_;
  bool had_recent_detection_{false};
  double last_detection_pan_{0.0};
  bool bg_captured_{false};

  // Tracking: locked target parameters (captured at lock-on)
  double lock_range_m_{0.0};
  double lock_bearing_rad_{0.0};
  double lock_width_rad_{0.0};

  // Tracking: tilt sweep state
  double tilt_sweep_angle_;
  SweepPhase sweep_phase_;
  double tilt_top_deg_;
  double tilt_bottom_deg_;
  int consecutive_misses_;
  bool tilt_scan_received_;
  rclcpp::Time tilt_settle_start_;

  // Parameters: tracking pan
  double track_tilt_deg_;
  double track_kp_;
  double track_max_step_;
  double track_deadband_;
  double target_timeout_;
  double pan_min_deg_;
  double pan_max_deg_;
  double track_rate_hz_;

  // Parameters: step-and-detect
  double lock_bearing_thresh_deg_;
  double pan_settle_sec_;
  double search_timeout_sec_;
  double search_step_deg_;

  // Parameters: tilt sweep
  double tilt_step_deg_;
  double tilt_max_up_deg_;
  double tilt_max_down_deg_;
  double tilt_settle_sec_;
  double range_tolerance_m_;
  double bearing_pad_deg_;
  int min_tilt_beams_;
  int miss_limit_;
  double bearing_offset_deg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSequenceNode>());
  rclcpp::shutdown();
  return 0;
}
