#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>

class TargetTrackerNode : public rclcpp::Node
{
public:
  TargetTrackerNode();

private:
  // Callbacks
  void local_position_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void detected_object_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void timer_callback();

  // Subscribers
  //
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detected_obj_sub_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Fixed offset of the LiDAR relative to parent_frame (meters)
  offset_x_ = this->declare_parameter<double>("offset_x", 0.0);
  offset_y_ = this->declare_parameter<double>("offset_y", 0.0);
  offset_z_ = this->declare_parameter<double>("offset_z", 0.0);
  // State
  double pan_angle_;
  double tilt_angle_;
  double pan_increment_;
  int pan_direction_;
};

private:
  void timer_callback()
  {
  auto msg = std_msgs::msg::Float32MultiArray();
  msg.data.push_back(pan_angle_);
  msg.data.push_back(tilt_angle_);
  publisher_->publish(msg);

  pan_angle_ += pan_increment_ * pan_direction_;
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&DynamicTFBroadcaster::pose_cb, this, std::placeholders::_1));

  px_ = msg->pose.position.x;
  py_ = msg->pose.position.y;
  pz_ = msg->pose.position.z;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double pan_angle_;
  double tilt_angle_;
  double pan_increment_;
  int pan_direction_;
};

TargetTrackerNode::TargetTrackerNode()
: Node("target_tracker_node"),
  pan_angle_(0.0),
  tilt_angle_(150.0),
  pan_increment_(0.75),
  pan_direction_(1)
{
  publisher_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/servo_angles", 10);

  // Subscriber: local position
  local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&TargetTrackerNode::local_position_callback, this, std::placeholders::_1));

  // Subscriber: detected object position
  detected_obj_sub_ =
    this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/detected_object_position",
      10,
      std::bind(
        &TargetTrackerNode::detected_object_callback,
        this,
        std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&TargetTrackerNode::timer_callback, this));
}

void TargetTrackerNode::local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  px_ = msg->pose.position.x;
  py_ = msg->pose.position.y;
  pz_ = msg->pose.position.z;
  last_stamp_ = msg->header.stamp;

  RCLCPP_DEBUG(
    this->get_logger(),
    "Local pos: x=%.2f y=%.2f z=%.2f", px_, py_, pz_);
}

void TargetTrackerNode::detected_object_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) return;

  float obj_x = msg->data[0];
  float obj_y = msg->data[1];

  RCLCPP_DEBUG(
    this->get_logger(),
    "Detected object: x=%.2f y=%.2f", obj_x, obj_y);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
