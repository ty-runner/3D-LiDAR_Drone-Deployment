
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <cmath>

class TargetTrackerNode : public rclcpp::Node
{
public:
    TargetTrackerNode();

private:
    // Callbacks
    void local_position_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void detected_object_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void angle_transform(float* pan_tilt_angles, float* drone_position, float* object_position);

    void timer_callback();

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped::SharedPtr> local_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped::SharedPtr> detected_obj_sub_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Fixed offset of the LiDAR relative to parent_frame (meters)
    double offset_x_;
    double offset_y_;
    double offset_z_;

    // State
    double pan_angle_;
    double tilt_angle_;
    double pan_increment_;
    int pan_direction_;

    float px_, py_, pz_; // drone position variables
    float ox_, oy_, oz_; // object position variables
};

TargetTrackerNode::TargetTrackerNode() // constructor
: Node("target_tracker_node"),
  pan_angle_(0.0),
  tilt_angle_(150.0),
  pan_increment_(0.75),
  pan_direction_(1),
  px_(0.0),
  py_(0.0),
  pz_(0.0),
  ox_(0.0),
  oy_(0.0),
  oz_(0.0)
{
    // Declare/load offsets from parameters
    offset_x_ = this->declare_parameter<double>("offset_x", 0.0);
    offset_y_ = this->declare_parameter<double>("offset_y", 0.0);
    offset_z_ = this->declare_parameter<double>("offset_z", 0.0);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/servo_angles", 10);

    // Subscriber: local position
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
        std::bind(&TargetTrackerNode::local_position_callback, this, std::placeholders::_1));

    // Subscriber: detected object position
    detected_obj_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_detection", rclcpp::SensorDataQoS(),
        std::bind(&TargetTrackerNode::detected_object_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&TargetTrackerNode::timer_callback, this));
}

void TargetTrackerNode::local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    px_ = msg->pose.position.x + offset_x_;
    py_ = msg->pose.position.y + offset_y_;
    pz_ = msg->pose.position.z + offset_z_;

    last_stamp_ = msg->header.stamp;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Local pos: x=%.2f y=%.2f z=%.2f", px_, py_, pz_);
}

void TargetTrackerNode::detected_object_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ox_ = msg->pose.position.x;
    oy_ = msg->pose.position.y;
    oz_ = msg->pose.position.z;

    last_stamp_ = msg->header.stamp;

    RCLCPP_DEBUG(
        this->get_logger(),
        "Detected object pos: x=%.2f y=%.2f z=%.2f", ox_, oy_, oz_);
}

void TargetTrackerNode::angle_transform(float* pan_tilt_angles, float* drone_position, float* object_position)
{
    float xd_ = object_position[0] - drone_position[0]; // diff in x
    float yd_ = object_position[1] - drone_position[1]; // diff in y
    float zd_ = object_position[2] - drone_position[2]; // diff in z
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
