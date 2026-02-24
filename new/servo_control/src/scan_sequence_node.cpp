#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

class ScanSequenceNode : public rclcpp::Node
{
public:
  ScanSequenceNode()
  : Node("scan_sequence_node"), pan_angle_(0.0), tilt_angle_(-45.0),
    tilt_step_(10.0), pan_step_(10.0),
    state_(State::MOVE), settle_start_(this->get_clock()->now()),
    scan_received_(false)
  {
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angles", 10);

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ScanSequenceNode::scan_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ScanSequenceNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Step-and-scan: full tilt sweep per pan position. Pan ±5°, Tilt -45° to 90°");
  }

private:
  enum class State {
    MOVE,
    SETTLING,
    WAIT_FOR_SCAN
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/)
  {
    if (state_ == State::WAIT_FOR_SCAN) {
      scan_received_ = true;
    }
  }

  void timer_callback()
  {
    switch (state_) {
      case State::MOVE:
      {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.push_back(pan_angle_);
        msg.data.push_back(tilt_angle_);
        angle_publisher_->publish(msg);

        settle_start_ = this->get_clock()->now();
        state_ = State::SETTLING;
        break;
      }

      case State::SETTLING:
      {
        auto elapsed = (this->get_clock()->now() - settle_start_).seconds();
        if (elapsed >= 0.2) {
          scan_received_ = false;
          state_ = State::WAIT_FOR_SCAN;
        }
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.push_back(pan_angle_);
        msg.data.push_back(tilt_angle_);
        angle_publisher_->publish(msg);
        break;
      }

      case State::WAIT_FOR_SCAN:
      {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.push_back(pan_angle_);
        msg.data.push_back(tilt_angle_);
        angle_publisher_->publish(msg);

        if (scan_received_) {
          advance_angles();
          state_ = State::MOVE;
        }
        break;
      }
    }
  }

  void advance_angles()
  {
    // Advance tilt first (full sweep at current pan)
    /*pan_angle_ += pan_step_;

    if (pan_angle_ > 90.0) {
      // Tilt sweep complete, step pan
      pan_angle_ = -90.0;
      tilt_angle_ += tilt_step_;

      if (tilt_angle_ > 90.0) {
        // Full cycle done, reset
        tilt_angle_ = -45.0;
        RCLCPP_INFO(this->get_logger(), "Full scan cycle complete, restarting");
      }
      RCLCPP_INFO(this->get_logger(), "Pan stepped to %.2f°, Tilt reset to %.2f°",
                  pan_angle_, tilt_angle_);
    }*/
    tilt_angle_ += tilt_step_;

    if (tilt_angle_ > 90.0 || tilt_angle_ < -45.0) {
      // Tilt sweep complete, step pan
      //tilt_angle_ = -45.0;, instead we can flip the sign of the tilt_step_
      tilt_step_ *= -1; //flip increment direction
      tilt_angle_ += tilt_step_;
      pan_angle_ += pan_step_;

      if (pan_angle_ > 45.0 || pan_angle_ < -45.0) {
        // Full cycle done, reset
        //pan_angle_ = -45.0;, instead we flip sign of the increment
        pan_step_ *= -1;
        pan_angle_ += pan_step_;
        RCLCPP_INFO(this->get_logger(), "Full scan cycle complete, restarting");
      }
      RCLCPP_INFO(this->get_logger(), "Pan stepped to %.2f°, Tilt reset to %.2f°",
                  pan_angle_, tilt_angle_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pan_angle_;
  double tilt_angle_;
  double tilt_step_;  // Tilt step size in degrees
  double pan_step_;         // Pan step size in degrees

  State state_;
  rclcpp::Time settle_start_;
  bool scan_received_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
