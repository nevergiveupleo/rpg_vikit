/*
 * output_helper.cpp
 *
 *  Created on: Jan 20, 2013
 *      Modified for ROS 2: 2024
 *      Author: chrigi
 */

#include <vikit/output_helper.h>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace vk {
namespace output_helper {

void
publishTfTransform(const Sophus::SE3& T, const rclcpp::Time& stamp,
                   const std::string& frame_id, const std::string& child_frame_id,
                   std::shared_ptr<tf2_ros::TransformBroadcaster> br)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = frame_id;
  transform_msg.child_frame_id = child_frame_id;

  transform_msg.transform.translation.x = T.translation().x();
  transform_msg.transform.translation.y = T.translation().y();
  transform_msg.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.rotation_matrix());
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  br->sendTransform(transform_msg);
}

void
publishPointMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                   const Eigen::Vector3d& pos,
                   const std::string& ns,
                   const rclcpp::Time& timestamp,
                   int id,
                   int action,
                   double marker_scale,
                   const Eigen::Vector3d& color,
                   rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world"; // Note: No leading '/'
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::CUBE;
  msg.action = action; // 0 = add/modify
  msg.scale.x = marker_scale;
  msg.scale.y = marker_scale;
  msg.scale.z = marker_scale;
  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];
  msg.lifetime = lifetime;
  msg.pose.position.x = pos[0];
  msg.pose.position.y = pos[1];
  msg.pose.position.z = pos[2];
  pub->publish(msg);
}

} // namespace output_helper
} // namespace vk
