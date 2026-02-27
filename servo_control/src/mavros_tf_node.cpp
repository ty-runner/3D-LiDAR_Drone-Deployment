#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/qos.hpp"


class MavrosTFNode : public rclcpp::Node
{
public:
  MavrosTFNode()
  : Node("mavros_tf_node")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "map");
    child_frame_  = this->declare_parameter<std::string>("child_frame", "base_link");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&MavrosTFNode::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MAVROS TF: %s -> %s from /mavros/local_position/pose",
                parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_frame_;
    t.child_frame_id  = child_frame_;

    t.transform.translation.x = msg->pose.position.y;
    t.transform.translation.y = msg->pose.position.x;
    t.transform.translation.z = (msg->pose.position.z)*-1;

    t.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::string parent_frame_, child_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavrosTFNode>());
  rclcpp::shutdown();
  return 0;
}
