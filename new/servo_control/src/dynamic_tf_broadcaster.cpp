#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
  DynamicTFBroadcaster()
  : Node("dynamic_tf_broadcaster"),
    pan_deg_(0.0), tilt_deg_(0.0)
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "base_link");
    child_frame_  = this->declare_parameter<std::string>("child_frame",  "laser_frame");

    // Offset from pan pivot to tilt pivot (base_link -> pan_link)
    pan_to_tilt_x_ = this->declare_parameter<double>("pan_to_tilt_x", 0.0);
    pan_to_tilt_y_ = this->declare_parameter<double>("pan_to_tilt_y", 0.0);
    pan_to_tilt_z_ = this->declare_parameter<double>("pan_to_tilt_z", 0.0);  //0.5 ~2 inches up

    // Offset from tilt pivot to LiDAR scan center (tilt_link -> laser_frame)
    tilt_to_lidar_x_ = this->declare_parameter<double>("tilt_to_lidar_x", 0.00);   //0.05 ~1 inch forward
    tilt_to_lidar_y_ = this->declare_parameter<double>("tilt_to_lidar_y", 0.0); //0.2
    tilt_to_lidar_z_ = this->declare_parameter<double>("tilt_to_lidar_z", 0.00);  //0.065 ~2.5 inches up
    //0.05,0.04,0.09
    double pub_rate = this->declare_parameter<double>("publish_rate", 50.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/servo_angles", 10,
      std::bind(&DynamicTFBroadcaster::angles_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / pub_rate)),
      std::bind(&DynamicTFBroadcaster::publish_tf, this));

    // Publish static transform: tilt_link -> laser_frame (fixed offset)
    publish_static_tf();

    RCLCPP_INFO(get_logger(),
      "THREE-FRAME TF: %s -> pan_link -> tilt_link -> %s",
      parent_frame_.c_str(), child_frame_.c_str());
    RCLCPP_INFO(get_logger(),
      "Pan-to-tilt offset: (%.3f, %.3f, %.3f)", pan_to_tilt_x_, pan_to_tilt_y_, pan_to_tilt_z_);
    RCLCPP_INFO(get_logger(),
      "Tilt-to-LiDAR offset: (%.3f, %.3f, %.3f)", tilt_to_lidar_x_, tilt_to_lidar_y_, tilt_to_lidar_z_);
  }

private:
  void angles_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "servo_angles has <2 values");
      return;
    }
    pan_deg_  = msg->data[0];
    tilt_deg_ = msg->data[1];
  }

  void publish_static_tf()
  {
    geometry_msgs::msg::TransformStamped t_static;
    t_static.header.stamp = this->get_clock()->now();
    t_static.header.frame_id = "tilt_link";
    t_static.child_frame_id  = child_frame_;

    t_static.transform.translation.x = tilt_to_lidar_x_;
    t_static.transform.translation.y = tilt_to_lidar_y_;
    t_static.transform.translation.z = tilt_to_lidar_z_;

    t_static.transform.rotation.x = 0.0;
    t_static.transform.rotation.y = 0.0;
    t_static.transform.rotation.z = 0.0;
    t_static.transform.rotation.w = 1.0; //1 was og

    static_tf_broadcaster_->sendTransform(t_static);
  }

  void publish_tf()
  {
    auto now = this->get_clock()->now();

    // --- Transform 1: base_link -> pan_link (pure yaw + offset to tilt axis) ---
    geometry_msgs::msg::TransformStamped t_pan;
    t_pan.header.stamp = now;
    t_pan.header.frame_id = parent_frame_;
    t_pan.child_frame_id  = "pan_link";

    t_pan.transform.translation.x = pan_to_tilt_x_;
    t_pan.transform.translation.y = pan_to_tilt_y_;
    t_pan.transform.translation.z = pan_to_tilt_z_;

    const double pan_rad = -pan_deg_ * M_PI / 180.0;
    tf2::Quaternion q_pan;
    q_pan.setRPY(0.0, 0.0, pan_rad);
    q_pan.normalize();

    t_pan.transform.rotation.x = q_pan.x();
    t_pan.transform.rotation.y = q_pan.y();
    t_pan.transform.rotation.z = q_pan.z();
    t_pan.transform.rotation.w = q_pan.w();

    // --- Transform 2: pan_link -> tilt_link (pure pitch at tilt pivot) ---
    geometry_msgs::msg::TransformStamped t_tilt;
    t_tilt.header.stamp = now;
    t_tilt.header.frame_id = "pan_link";
    t_tilt.child_frame_id  = "tilt_link";

    t_tilt.transform.translation.x = 0.0;
    t_tilt.transform.translation.y = 0.0;
    t_tilt.transform.translation.z = 0.0;

    const double tilt_rad = tilt_deg_ * M_PI / 180.0;
    tf2::Quaternion q_tilt;
    q_tilt.setRPY(0.0, tilt_rad, 0.0);
    q_tilt.normalize();

    t_tilt.transform.rotation.x = q_tilt.x();
    t_tilt.transform.rotation.y = q_tilt.y();
    t_tilt.transform.rotation.z = q_tilt.z();
    t_tilt.transform.rotation.w = q_tilt.w();

    tf_broadcaster_->sendTransform(t_pan);
    tf_broadcaster_->sendTransform(t_tilt);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr angles_sub_;
  double pan_deg_, tilt_deg_;
  std::string parent_frame_, child_frame_;
  double pan_to_tilt_x_, pan_to_tilt_y_, pan_to_tilt_z_;
  double tilt_to_lidar_x_, tilt_to_lidar_y_, tilt_to_lidar_z_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
