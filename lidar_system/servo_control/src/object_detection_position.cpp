//just the packing for the geometry msg nothing more
geometry_msgs::msg::PoseStamped msg;

msg.header.stamp = this->get_clock()->now();
msg.header.frame_id = "map";

msg.pose.position.x = x;
msg.pose.position.y = y;
msg.pose.position.z = z;

msg.pose.orientation.x = 0.0;
msg.pose.orientation.y = 0.0;
msg.pose.orientation.z = 0.0;
msg.pose.orientation.w = 1.0;

pose_pub_->publish(msg);
