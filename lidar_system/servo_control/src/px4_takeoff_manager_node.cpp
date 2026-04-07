#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "servo_control/srv/get_local_dimensions.hpp"

using namespace std::chrono_literals;

class Px4TakeoffManagerNode : public rclcpp::Node
{
public:
  Px4TakeoffManagerNode()
  : Node("px4_takeoff_manager_node")
  {
    // ── Topic / service names ──
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/mavros/local_position/pose");
    setpoint_topic_ = this->declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_position/local");
    dimensions_service_name_ = this->declare_parameter<std::string>(
      "dimensions_service", "/get_local_dimensions");
    arming_service_name_ = this->declare_parameter<std::string>(
      "arming_service", "/mavros/cmd/arming");
    mode_service_name_ = this->declare_parameter<std::string>(
      "mode_service", "/mavros/set_mode");

    // ── Takeoff parameters ──
    takeoff_height_above_floor_ = this->declare_parameter<double>(
      "takeoff_height_above_floor", 0.45);
    ceiling_margin_ = this->declare_parameter<double>(
      "ceiling_margin", 0.6);
    min_room_height_ = this->declare_parameter<double>(
      "min_room_height", 1.8);

    // ── Tolerances ──
    hover_tolerance_z_ = this->declare_parameter<double>(
      "hover_tolerance_z", 0.10);
    hover_tolerance_xy_ = this->declare_parameter<double>(
      "hover_tolerance_xy", 0.15);

    // ── Control ──
    setpoint_rate_hz_ = this->declare_parameter<double>(
      "setpoint_rate_hz", 20.0);
    prestream_count_required_ = this->declare_parameter<int>(
      "prestream_count_required", 40);
    auto_arm_ = this->declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = this->declare_parameter<bool>("auto_offboard", true);

    // ── Hover hold times (seconds) ──
    hover_hold_seconds_ = this->declare_parameter<double>(
      "hover_hold_seconds", 3.0);
    waypoint_hold_seconds_ = this->declare_parameter<double>(
      "waypoint_hold_seconds", 2.0);
    final_hold_seconds_ = this->declare_parameter<double>(
      "final_hold_seconds", 3.0);
    disarm_delay_seconds_ = this->declare_parameter<double>(
      "disarm_delay_seconds", 5.0);

    // ── Flight mode ──
    // "hover"    = takeoff, hold, auto-land
    // "forward"  = takeoff, hover, move +X, hold, auto-land
    // "left"     = takeoff, hover, move +Y, hold, auto-land
    // "right"    = takeoff, hover, move -Y, hold, auto-land
    // "single"   = takeoff, hover, fly to custom dx/dy waypoint, hold, auto-land
    // "square"   = takeoff, hover, fly a square, return to start, auto-land
    // "click"    = takeoff, hover, wait for RViz 2D Goal Pose clicks
    flight_mode_ = this->declare_parameter<std::string>("flight_mode", "hover");

    // ── Move distance for forward/left/right modes (meters) ──
    move_distance_ = this->declare_parameter<double>("move_distance", 0.25);

    // ── Single waypoint offset (used when flight_mode = "single") ──
    waypoint_dx_ = this->declare_parameter<double>("waypoint_dx", 0.0);
    waypoint_dy_ = this->declare_parameter<double>("waypoint_dy", 0.0);

    // ── Square size (used when flight_mode = "square") ──
    square_size_ = this->declare_parameter<double>("square_size", 0.25);

    // ── OctoMap query offset ──
    octomap_z_offset_ = this->declare_parameter<double>("octomap_z_offset", 0.3);

    // ── Subscriptions ──
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4TakeoffManagerNode::poseCallback, this, std::placeholders::_1));

    // Subscribe to RViz 2D Goal Pose (used in "click" mode)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&Px4TakeoffManagerNode::goalCallback, this, std::placeholders::_1));

    // Subscribe to planned path from octomap_planner_node (used in "click" mode)
    planned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&Px4TakeoffManagerNode::plannedPathCallback, this, std::placeholders::_1));

    // ── Publishers ──
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      setpoint_topic_, 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/drone_path", 10);

    // ── Service clients ──
    dimensions_client_ =
      this->create_client<servo_control::srv::GetLocalDimensions>(dimensions_service_name_);
    arming_client_ =
      this->create_client<mavros_msgs::srv::CommandBool>(arming_service_name_);
    mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>(mode_service_name_);

    // ── Timer ──
    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&Px4TakeoffManagerNode::timerCallback, this));
    last_path_publish_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "px4_takeoff_manager_node started");
    RCLCPP_INFO(this->get_logger(), "Flight mode: %s", flight_mode_.c_str());

    if (flight_mode_ == "square") {
      RCLCPP_INFO(this->get_logger(),
        "Square pattern: %.2fm sides | hover_hold=%.1fs | wp_hold=%.1fs",
        square_size_, hover_hold_seconds_, waypoint_hold_seconds_);
    } else if (flight_mode_ == "forward") {
      RCLCPP_INFO(this->get_logger(),
        "Forward mode: %.2fm | hover_hold=%.1fs",
        move_distance_, hover_hold_seconds_);
    } else if (flight_mode_ == "left") {
      RCLCPP_INFO(this->get_logger(),
        "Left mode: %.2fm | hover_hold=%.1fs",
        move_distance_, hover_hold_seconds_);
    } else if (flight_mode_ == "right") {
      RCLCPP_INFO(this->get_logger(),
        "Right mode: %.2fm | hover_hold=%.1fs",
        move_distance_, hover_hold_seconds_);
    } else if (flight_mode_ == "single") {
      RCLCPP_INFO(this->get_logger(),
        "Single waypoint: dx=%.2f dy=%.2f | hover_hold=%.1fs",
        waypoint_dx_, waypoint_dy_, hover_hold_seconds_);
    } else if (flight_mode_ == "click") {
      RCLCPP_INFO(this->get_logger(),
        "Click-to-fly mode: use RViz 2D Goal Pose to send targets");
    } else {
      RCLCPP_INFO(this->get_logger(), "Hover-only mode");
    }
  }

private:
  enum class State
  {
    WAIT_FOR_POSE,
    WAIT_FOR_SERVICES,
    REQUEST_DIMENSIONS,
    WAIT_DIMENSIONS_RESPONSE,
    PRESTREAM_SETPOINTS,
    SET_OFFBOARD,
    WAIT_OFFBOARD_RESPONSE,
    ARM,
    WAIT_ARM_RESPONSE,
    ASCEND,
    HOVER_AT_TAKEOFF,
    MOVE_TO_WAYPOINT,
    HOLD_AT_WAYPOINT,
    HOLD_FINAL,
    WAIT_FOR_GOAL,
    MOVE_TO_GOAL,
    HOVER_AT_GOAL,
    REQUEST_LAND,
    WAIT_LAND_RESPONSE,
    LANDED,
    REQUEST_DISARM,
    WAIT_DISARM_RESPONSE,
    DISARMED,
    FAIL
  };

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    have_pose_ = true;

    // Publish path trail (throttled to ~2Hz to avoid flooding)
    auto now = this->now();
    if ((now - last_path_publish_time_).seconds() >= 0.5) {
      path_trail_.header.stamp = now;
      path_trail_.header.frame_id = "map";
      auto pose_swapped = *msg;
      pose_swapped.pose.position.x = msg->pose.position.y;
      pose_swapped.pose.position.y = msg->pose.position.x;
      pose_swapped.pose.position.z = -msg->pose.position.z;
      path_trail_.poses.push_back(pose_swapped);
      path_pub_->publish(path_trail_);
      last_path_publish_time_ = now;
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (flight_mode_ != "click") {
      return;  // ignore goals if not in click mode
    }

    if (state_ != State::WAIT_FOR_GOAL && state_ != State::HOVER_AT_GOAL) {
      RCLCPP_WARN(this->get_logger(), "Goal received but not ready — ignoring");
      return;
    }

    // In click mode, the octomap_planner_node handles path planning.
    // It subscribes to /goal_pose, runs A*, and publishes /planned_path.
    // We just log that a goal was received and wait for the planned path.
    RCLCPP_INFO(this->get_logger(),
      "Goal click received: map=(%.2f, %.2f) — waiting for planner",
      msg->pose.position.x, msg->pose.position.y);
  }

  void plannedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (flight_mode_ != "click") {
      return;
    }

    if (state_ != State::WAIT_FOR_GOAL && state_ != State::HOVER_AT_GOAL) {
      RCLCPP_WARN(this->get_logger(), "Planned path received but not ready — ignoring");
      return;
    }

    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty planned path — ignoring");
      return;
    }

    // Load planned waypoints (already in MAVROS frame from planner)
    // Skip the first pose (it's the current position)
    waypoints_.clear();
    for (size_t i = 1; i < msg->poses.size(); i++) {
      auto wp = msg->poses[i];
      // Keep the cruise altitude from our own OctoMap query
      wp.pose.position.z = target_z_;
      // Swap XY: map frame -> MAVROS frame
      //double map_x = wp.pose.position.x;
      //double map_y = wp.pose.position.y;
      //wp.pose.position.x = map_y;
      //wp.pose.position.y = map_x;


      wp.pose.orientation = takeoff_pose_.pose.orientation;
      waypoints_.push_back(wp);
    }

    if (waypoints_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Planned path has no waypoints after filtering");
      return;
    }

    current_wp_index_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "Planned path loaded: %zu waypoints", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
        "  WP %zu: (%.2f, %.2f, %.2f)",
        i + 1,
        waypoints_[i].pose.position.x,
        waypoints_[i].pose.position.y,
        waypoints_[i].pose.position.z);
    }

    state_ = State::MOVE_TO_WAYPOINT;
  }

  void timerCallback()
  {
    switch (state_) {
      case State::WAIT_FOR_POSE:            handleWaitForPose();            break;
      case State::WAIT_FOR_SERVICES:        handleWaitForServices();        break;
      case State::REQUEST_DIMENSIONS:       handleRequestDimensions();      break;
      case State::WAIT_DIMENSIONS_RESPONSE: handleWaitDimensionsResponse(); break;
      case State::PRESTREAM_SETPOINTS:      handlePrestreamSetpoints();     break;
      case State::SET_OFFBOARD:             handleSetOffboard();            break;
      case State::WAIT_OFFBOARD_RESPONSE:   handleWaitOffboardResponse();   break;
      case State::ARM:                      handleArm();                    break;
      case State::WAIT_ARM_RESPONSE:        handleWaitArmResponse();        break;
      case State::ASCEND:                   handleAscend();                 break;
      case State::HOVER_AT_TAKEOFF:         handleHoverAtTakeoff();         break;
      case State::MOVE_TO_WAYPOINT:         handleMoveToWaypoint();         break;
      case State::HOLD_AT_WAYPOINT:         handleHoldAtWaypoint();         break;
      case State::HOLD_FINAL:               handleHoldFinal();              break;
      case State::WAIT_FOR_GOAL:            handleWaitForGoal();            break;
      case State::MOVE_TO_GOAL:             handleMoveToGoal();             break;
      case State::HOVER_AT_GOAL:            handleHoverAtGoal();            break;
      case State::REQUEST_LAND:             handleRequestLand();            break;
      case State::WAIT_LAND_RESPONSE:       handleWaitLandResponse();       break;
      case State::LANDED:
        handleLanded();
        break;
      case State::REQUEST_DISARM:
        handleRequestDisarm();
        break;
      case State::WAIT_DISARM_RESPONSE:
        handleWaitDisarmResponse();
        break;
      case State::DISARMED:
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Disarmed — mission complete");
        break;
      case State::FAIL:
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Takeoff manager in FAIL state");
        break;
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  State handlers
  // ═══════════════════════════════════════════════════════════════

  void handleWaitForPose()
  {
    if (!have_pose_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for current pose...");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Got pose: x=%.2f y=%.2f z=%.2f",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y,
      current_pose_.pose.position.z);

    state_ = State::WAIT_FOR_SERVICES;
  }

  void handleWaitForServices()
  {
    bool dims_ok = dimensions_client_->wait_for_service(100ms);
    bool arm_ok  = arming_client_->wait_for_service(100ms);
    bool mode_ok = mode_client_->wait_for_service(100ms);

    if (!(dims_ok && arm_ok && mode_ok)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for services: dims=%d arm=%d mode=%d",
        dims_ok, arm_ok, mode_ok);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "All required services available");
    state_ = State::REQUEST_DIMENSIONS;
  }

  void handleRequestDimensions()
  {
    // TEMPORARY: hardcoded room dimensions (OctoMap query not working)
    double fake_floor = 0.0;
    double fake_ceiling = 2.0;
    double fake_room_height = fake_ceiling - fake_floor;

    RCLCPP_WARN(this->get_logger(),
      "Using HARDCODED dimensions: floor=%.2f ceiling=%.2f height=%.2f",
      fake_floor, fake_ceiling, fake_room_height);

    if (fake_room_height < min_room_height_) {
      fail("Room height too small for safe takeoff");
      return;
    }

    // Compute safe cruise altitude (same logic as before)
    double desired_z = fake_floor + takeoff_height_above_floor_;
    double max_safe_z = fake_ceiling - ceiling_margin_;
    target_z_ = std::min(desired_z, max_safe_z);

    // Safety: reject if target_z is more than 0.55m above current position
    double climb_distance = target_z_ - current_pose_.pose.position.z;
    if (climb_distance > 0.55) {
      RCLCPP_ERROR(this->get_logger(),
        "Target altitude too high: climb=%.2fm (max 0.55m)",
        climb_distance);
      fail("Target altitude too far above current position");
      return;
    }

    if (target_z_ <= current_pose_.pose.position.z + 0.05) {
      fail("Computed target_z is not above current altitude");
      return;
    }

    // Takeoff target: straight up, hold XY
    takeoff_pose_ = current_pose_;
    takeoff_pose_.pose.position.z = target_z_;

    // Build waypoint list based on flight mode
    buildWaypoints();

    // Start by targeting the takeoff position
    active_target_ = takeoff_pose_;
    prestream_counter_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "Dimensions OK: floor=%.2f ceiling=%.2f room=%.2f -> cruise_z=%.2f",
      fake_floor, fake_ceiling, fake_room_height, target_z_);
    RCLCPP_INFO(this->get_logger(),
      "Takeoff target:  (%.2f, %.2f, %.2f)",
      takeoff_pose_.pose.position.x,
      takeoff_pose_.pose.position.y,
      takeoff_pose_.pose.position.z);

    for (size_t i = 0; i < waypoints_.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu: (%.2f, %.2f, %.2f)",
        i + 1,
        waypoints_[i].pose.position.x,
        waypoints_[i].pose.position.y,
        waypoints_[i].pose.position.z);
    }

    // Skip WAIT_DIMENSIONS_RESPONSE — go straight to prestream
    state_ = State::PRESTREAM_SETPOINTS;
  }

  void handleWaitDimensionsResponse()
  {
    // Not used with hardcoded dimensions — should never reach here
    RCLCPP_WARN(this->get_logger(), "handleWaitDimensionsResponse called unexpectedly");
    state_ = State::PRESTREAM_SETPOINTS;
  }

  void handlePrestreamSetpoints()
  {
    publishActivePose();
    prestream_counter_++;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Prestreaming setpoints... %d/%d",
      prestream_counter_, prestream_count_required_);

    if (prestream_counter_ >= prestream_count_required_) {
      state_ = State::SET_OFFBOARD;
    }
  }

  void handleSetOffboard()
  {
    if (!auto_offboard_) {
      RCLCPP_WARN(this->get_logger(), "auto_offboard=false, skipping mode change");
      state_ = auto_arm_ ? State::ARM : State::ASCEND;
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    req->custom_mode = "OFFBOARD";

    mode_future_ = mode_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
    state_ = State::WAIT_OFFBOARD_RESPONSE;
  }

  void handleWaitOffboardResponse()
  {
    publishActivePose();

    if (!mode_future_.valid()) {
      fail("OFFBOARD mode future became invalid");
      return;
    }

    auto status = mode_future_.wait_for(0ms);
    if (status != std::future_status::ready) {
      return;
    }

    auto resp = mode_future_.get();
    if (!resp->mode_sent) {
      fail("PX4 rejected OFFBOARD mode request");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode request accepted");
    state_ = auto_arm_ ? State::ARM : State::ASCEND;
  }

  void handleArm()
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    arm_future_ = arming_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested arm");
    state_ = State::WAIT_ARM_RESPONSE;
  }

  void handleWaitArmResponse()
  {
    publishActivePose();

    if (!arm_future_.valid()) {
      fail("Arm future became invalid");
      return;
    }

    auto status = arm_future_.wait_for(0ms);
    if (status != std::future_status::ready) {
      return;
    }

    auto resp = arm_future_.get();
    if (!resp->success) {
      fail("PX4 arm request rejected");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Arm request accepted — ascending");
    state_ = State::ASCEND;
  }

  // ── Ascend to cruise altitude ──
  void handleAscend()
  {
    active_target_ = takeoff_pose_;
    publishActivePose();

    double dz  = std::fabs(current_pose_.pose.position.z - takeoff_pose_.pose.position.z);
    double dxy = xyDistance(current_pose_, takeoff_pose_);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Ascending... current_z=%.2f target_z=%.2f err_z=%.2f err_xy=%.2f",
      current_pose_.pose.position.z, takeoff_pose_.pose.position.z, dz, dxy);

    if (dz < hover_tolerance_z_ && dxy < hover_tolerance_xy_) {
      RCLCPP_INFO(this->get_logger(), "Takeoff altitude reached — stabilizing");
      hover_start_time_ = this->now();
      state_ = State::HOVER_AT_TAKEOFF;
    }
  }

  // ── Stabilize at takeoff position ──
  void handleHoverAtTakeoff()
  {
    active_target_ = takeoff_pose_;
    publishActivePose();

    double elapsed = (this->now() - hover_start_time_).seconds();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Hover stabilizing... %.1f / %.1f s",
      elapsed, hover_hold_seconds_);

    if (elapsed >= hover_hold_seconds_) {
      if (flight_mode_ == "click") {
        RCLCPP_INFO(this->get_logger(),
          "Hover stable — click mode: waiting for RViz goal");
        state_ = State::WAIT_FOR_GOAL;
      } else if (waypoints_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Hover stable — no waypoints, holding position");
        state_ = State::HOLD_FINAL;
      } else {
        current_wp_index_ = 0;
        RCLCPP_INFO(this->get_logger(),
          "Hover stable — moving to waypoint 1/%zu (%.2f, %.2f, %.2f)",
          waypoints_.size(),
          waypoints_[0].pose.position.x,
          waypoints_[0].pose.position.y,
          waypoints_[0].pose.position.z);
        state_ = State::MOVE_TO_WAYPOINT;
      }
    }
  }

  // ── Fly to current waypoint ──
  void handleMoveToWaypoint()
  {
    auto & wp = waypoints_[current_wp_index_];
    active_target_ = wp;
    publishActivePose();

    double dxy = xyDistance(current_pose_, wp);
    double dz  = std::fabs(current_pose_.pose.position.z - wp.pose.position.z);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Moving to WP %zu/%zu... current=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) err_xy=%.2f err_z=%.2f",
      current_wp_index_ + 1, waypoints_.size(),
      current_pose_.pose.position.x,
      current_pose_.pose.position.y,
      current_pose_.pose.position.z,
      wp.pose.position.x,
      wp.pose.position.y,
      wp.pose.position.z,
      dxy, dz);

    if (dxy < hover_tolerance_xy_ && dz < hover_tolerance_z_) {
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu/%zu reached — holding %.1fs",
        current_wp_index_ + 1, waypoints_.size(), waypoint_hold_seconds_);
      wp_hold_start_time_ = this->now();
      state_ = State::HOLD_AT_WAYPOINT;
    }
  }

  // ── Hold at current waypoint, then advance to next ──
  void handleHoldAtWaypoint()
  {
    auto & wp = waypoints_[current_wp_index_];
    active_target_ = wp;
    publishActivePose();

    double elapsed = (this->now() - wp_hold_start_time_).seconds();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Holding at WP %zu/%zu... %.1f / %.1f s",
      current_wp_index_ + 1, waypoints_.size(),
      elapsed, waypoint_hold_seconds_);

    if (elapsed >= waypoint_hold_seconds_) {
      current_wp_index_++;

      if (current_wp_index_ < waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(),
          "Moving to waypoint %zu/%zu (%.2f, %.2f, %.2f)",
          current_wp_index_ + 1, waypoints_.size(),
          waypoints_[current_wp_index_].pose.position.x,
          waypoints_[current_wp_index_].pose.position.y,
          waypoints_[current_wp_index_].pose.position.z);
        state_ = State::MOVE_TO_WAYPOINT;
      } else {
        if (flight_mode_ == "click") {
          RCLCPP_INFO(this->get_logger(),
            "All waypoints complete — waiting for next click");
          state_ = State::WAIT_FOR_GOAL;
        } else {
          RCLCPP_INFO(this->get_logger(),
            "All waypoints complete — holding final position");
          state_ = State::HOLD_FINAL;
        }
      }
    }
  }

  // ── Wait for RViz goal click (click mode) ──
  void handleWaitForGoal()
  {
    // Keep hovering at current position while waiting
    publishActivePose();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Waiting for RViz goal... click 2D Goal Pose (planner will find path)");

    // Flow: RViz click -> /goal_pose -> octomap_planner_node -> /planned_path -> plannedPathCallback
  }

  // ── Fly to clicked goal ──
  void handleMoveToGoal()
  {
    active_target_ = click_goal_;
    publishActivePose();

    double dxy = xyDistance(current_pose_, click_goal_);
    double dz  = std::fabs(current_pose_.pose.position.z - click_goal_.pose.position.z);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Flying to goal... current=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) err_xy=%.2f err_z=%.2f",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y,
      current_pose_.pose.position.z,
      click_goal_.pose.position.x,
      click_goal_.pose.position.y,
      click_goal_.pose.position.z,
      dxy, dz);

    if (dxy < hover_tolerance_xy_ && dz < hover_tolerance_z_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached — waiting for next click");
      goal_received_ = false;
      state_ = State::HOVER_AT_GOAL;
    }
  }

  // ── Hover at goal, wait for next click ──
  void handleHoverAtGoal()
  {
    active_target_ = click_goal_;
    publishActivePose();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Holding at goal (%.2f, %.2f, %.2f) — click new goal or land via QGC",
      click_goal_.pose.position.x,
      click_goal_.pose.position.y,
      click_goal_.pose.position.z);

    // goalCallback will change state to MOVE_TO_GOAL when a new click arrives
  }

  // ── Hold final position, then auto-land ──
  void handleHoldFinal()
  {
    publishActivePose();

    if (!final_hold_started_) {
      final_hold_start_time_ = this->now();
      final_hold_started_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Holding final position for %.1fs before landing...", final_hold_seconds_);
    }

    double elapsed = (this->now() - final_hold_start_time_).seconds();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Final hold... %.1f / %.1f s at (%.2f, %.2f, %.2f)",
      elapsed, final_hold_seconds_,
      active_target_.pose.position.x,
      active_target_.pose.position.y,
      active_target_.pose.position.z);

    if (elapsed >= final_hold_seconds_) {
      RCLCPP_INFO(this->get_logger(), "Final hold complete — requesting AUTO.LAND");
      state_ = State::REQUEST_LAND;
    }
  }

  // ── Send AUTO.LAND to PX4 ──
  void handleRequestLand()
  {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    req->custom_mode = "AUTO.LAND";

    land_future_ = mode_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested AUTO.LAND mode");
    state_ = State::WAIT_LAND_RESPONSE;
  }

  // ── Wait for land response ──
  void handleWaitLandResponse()
  {
    if (!land_future_.valid()) {
      fail("AUTO.LAND future became invalid");
      return;
    }

    auto status = land_future_.wait_for(0ms);
    if (status != std::future_status::ready) {
      return;
    }

    auto resp = land_future_.get();
    if (!resp->mode_sent) {
      RCLCPP_WARN(this->get_logger(), "AUTO.LAND rejected — holding position");
      state_ = State::HOLD_FINAL;
      final_hold_started_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "AUTO.LAND accepted — descending");
    landed_time_ = this->now();
    state_ = State::LANDED;
  }

  // ── Wait for touchdown then disarm ──
  void handleLanded()
  {
    // Wait a few seconds for PX4 to actually touch down
    double elapsed = (this->now() - landed_time_).seconds();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Landing... %.1fs elapsed (disarm in %.1fs)",
      elapsed, disarm_delay_seconds_);

    if (elapsed >= disarm_delay_seconds_) {
      RCLCPP_INFO(this->get_logger(), "Requesting disarm");
      state_ = State::REQUEST_DISARM;
    }
  }

  // ── Send disarm command ──
  void handleRequestDisarm()
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;

    disarm_future_ = arming_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested disarm");
    state_ = State::WAIT_DISARM_RESPONSE;
  }

  // ── Wait for disarm response ──
  void handleWaitDisarmResponse()
  {
    if (!disarm_future_.valid()) {
      RCLCPP_WARN(this->get_logger(), "Disarm future invalid — may already be disarmed");
      state_ = State::DISARMED;
      return;
    }

    auto status = disarm_future_.wait_for(0ms);
    if (status != std::future_status::ready) {
      return;
    }

    auto resp = disarm_future_.get();
    if (!resp->success) {
      RCLCPP_WARN(this->get_logger(), "Disarm rejected — PX4 may auto-disarm on its own");
    } else {
      RCLCPP_INFO(this->get_logger(), "Disarm confirmed");
    }

    state_ = State::DISARMED;
  }

  // ═══════════════════════════════════════════════════════════════
  //  Waypoint builder
  // ═══════════════════════════════════════════════════════════════

  void buildWaypoints()
  {
    waypoints_.clear();
    current_wp_index_ = 0;

    double start_x = takeoff_pose_.pose.position.x;
    double start_y = takeoff_pose_.pose.position.y;

    if (flight_mode_ == "square") {
      //  Square pattern (viewed from above):
      //
      //   4 ←──── 3
      //   |       |
      //   Start → 1
      //
      //   +X = forward, +Y = left
      //   WP1: forward
      //   WP2: forward + left
      //   WP3: left
      //   WP4: back to start

      double s = square_size_;

      geometry_msgs::msg::PoseStamped wp = takeoff_pose_;

      // WP1: forward
      wp.pose.position.x = start_x + s;
      wp.pose.position.y = start_y;
      waypoints_.push_back(wp);

      // WP2: forward + left
      wp.pose.position.x = start_x + s;
      wp.pose.position.y = start_y + s;
      waypoints_.push_back(wp);

      // WP3: left only
      wp.pose.position.x = start_x;
      wp.pose.position.y = start_y + s;
      waypoints_.push_back(wp);

      // WP4: return to start
      wp.pose.position.x = start_x;
      wp.pose.position.y = start_y;
      waypoints_.push_back(wp);

      RCLCPP_INFO(this->get_logger(),
        "Built square pattern: %.2fm sides, 4 waypoints", s);

    } else if (flight_mode_ == "single") {
      if (std::fabs(waypoint_dx_) > 0.01 || std::fabs(waypoint_dy_) > 0.01) {
        geometry_msgs::msg::PoseStamped wp = takeoff_pose_;
        wp.pose.position.x = start_x + waypoint_dx_;
        wp.pose.position.y = start_y + waypoint_dy_;
        waypoints_.push_back(wp);
      }

    } else if (flight_mode_ == "forward") {
      geometry_msgs::msg::PoseStamped wp = takeoff_pose_;
      wp.pose.position.x = start_x + move_distance_;
      wp.pose.position.y = start_y;
      waypoints_.push_back(wp);
      RCLCPP_INFO(this->get_logger(), "Built forward waypoint: +%.2fm in X", move_distance_);

    } else if (flight_mode_ == "left") {
      geometry_msgs::msg::PoseStamped wp = takeoff_pose_;
      wp.pose.position.x = start_x;
      wp.pose.position.y = start_y + move_distance_;
      waypoints_.push_back(wp);
      RCLCPP_INFO(this->get_logger(), "Built left waypoint: +%.2fm in Y", move_distance_);

    } else if (flight_mode_ == "right") {
      geometry_msgs::msg::PoseStamped wp = takeoff_pose_;
      wp.pose.position.x = start_x;
      wp.pose.position.y = start_y - move_distance_;
      waypoints_.push_back(wp);
      RCLCPP_INFO(this->get_logger(), "Built right waypoint: -%.2fm in Y", move_distance_);

    } else {
      // "hover" mode — no waypoints
      RCLCPP_INFO(this->get_logger(), "Hover mode — no waypoints built");
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Helpers
  // ═══════════════════════════════════════════════════════════════

  void publishActivePose()
  {
    active_target_.header.stamp = this->now();
    setpoint_pub_->publish(active_target_);
  }

  double xyDistance(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const
  {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  void fail(const std::string & msg)
  {
    RCLCPP_ERROR(this->get_logger(), "FAIL: %s", msg.c_str());
    state_ = State::FAIL;
  }

  // ═══════════════════════════════════════════════════════════════
  //  Parameters
  // ═══════════════════════════════════════════════════════════════
  std::string pose_topic_;
  std::string setpoint_topic_;
  std::string dimensions_service_name_;
  std::string arming_service_name_;
  std::string mode_service_name_;

  double takeoff_height_above_floor_;
  double ceiling_margin_;
  double min_room_height_;
  double hover_tolerance_z_;
  double hover_tolerance_xy_;
  double setpoint_rate_hz_;
  int    prestream_count_required_;
  bool   auto_arm_;
  bool   auto_offboard_;

  double hover_hold_seconds_;
  double waypoint_hold_seconds_;
  double final_hold_seconds_;
  double disarm_delay_seconds_;
  std::string flight_mode_;
  double waypoint_dx_;
  double waypoint_dy_;
  double square_size_;
  double move_distance_;
  double octomap_z_offset_;

  // ═══════════════════════════════════════════════════════════════
  //  ROS interfaces
  // ═══════════════════════════════════════════════════════════════
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             planned_path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
  rclcpp::Client<servo_control::srv::GetLocalDimensions>::SharedPtr dimensions_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr          arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr              mode_client_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // ═══════════════════════════════════════════════════════════════
  //  State
  // ═══════════════════════════════════════════════════════════════
  State state_{State::WAIT_FOR_POSE};
  bool  have_pose_{false};
  int   prestream_counter_{0};
  double target_z_{0.0};

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped takeoff_pose_;
  geometry_msgs::msg::PoseStamped active_target_;

  nav_msgs::msg::Path path_trail_;
  rclcpp::Time last_path_publish_time_;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_wp_index_{0};

  geometry_msgs::msg::PoseStamped click_goal_;
  bool goal_received_{false};

  rclcpp::Time hover_start_time_;
  rclcpp::Time wp_hold_start_time_;
  rclcpp::Time final_hold_start_time_;
  rclcpp::Time landed_time_;
  bool final_hold_started_{false};

  rclcpp::Client<servo_control::srv::GetLocalDimensions>::SharedFuture dimensions_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture          arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture          disarm_future_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture              mode_future_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture              land_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TakeoffManagerNode>());
  rclcpp::shutdown();
  return 0;
}
