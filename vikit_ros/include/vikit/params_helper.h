/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Modified for ROS 2: 2024
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline
bool hasParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  return node->has_parameter(name);
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name, const T& defaultValue)
{
  T v;
  rclcpp::Parameter param;
  if(node->get_parameter(name, param))
  {
    v = param.get_value<T>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    node->declare_parameter(name, defaultValue);
    return defaultValue;
  }
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  T v;
  rclcpp::Parameter param;
  if(node->get_parameter(name, param))
  {
    v = param.get_value<T>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: " << name);
    throw std::runtime_error("Parameter not found: " + name);
  }
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
