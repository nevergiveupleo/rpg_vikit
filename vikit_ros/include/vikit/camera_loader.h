/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Modified for ROS 2: 2024
 *      Author: cforster
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace using a ROS 2 node
bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, std::shared_ptr<vk::AbstractCamera>& cam)
{
  bool res = true;
  std::string cam_model;
  
  // std::vector<std::string> parameter_names = node->list_parameters({}, 0).names;
  // RCLCPP_INFO(node->get_logger(), "List of all parameters:");
  // for (const auto& name : parameter_names) {
  //     // RCLCPP_INFO(node->get_logger(), " - %s", name.c_str());
  //     // get parameter value of each
  //     if (name == "cam_model") {
  //       node->get_parameter(name, cam_model);
  //       if (cam_model != "pinhole") {
  //         RCLCPP_ERROR(node->get_logger(), "Camera model must be pinhole");
  //         return false;
  //       }
  //       cam = std::make_shared<vk::PinholeCamera>;
  //     }
  //     else if (name == "cam_width") {
        
  //     }
  // }

  if(!node->get_parameter("cam_model", cam_model)){
    RCLCPP_ERROR(node->get_logger(), "Camera model must be specified");
  }
  else{
    RCLCPP_INFO(node->get_logger(), "Camera model: %s", cam_model.c_str());
  }

  if(cam_model == "Ocam") {
    std::string calib_file;
    node->get_parameter_or("cam_calib_file", calib_file, std::string(""));
    cam = std::make_shared<vk::OmniCamera>(calib_file);
  } else if(cam_model == "pinhole") {
    cam = std::make_shared<vk::PinholeCamera>(
      node->get_parameter("cam_width").as_int(),
      node->get_parameter("cam_height").as_int(),
      node->get_parameter("cam_fx").as_double(),
      node->get_parameter("cam_fy").as_double(),
      node->get_parameter("cam_cx").as_double(),
      node->get_parameter("cam_cy").as_double(),
      node->get_parameter_or("cam_d0", 0.0),
      node->get_parameter_or("cam_d1", 0.0),
      node->get_parameter_or("cam_d2", 0.0),
      node->get_parameter_or("cam_d3", 0.0)
    );
    RCLCPP_INFO(node->get_logger(), "pinhole camera model loaded");
  } else if(cam_model == "ATAN") {
    cam = std::make_shared<vk::ATANCamera>(
      node->get_parameter("cam_width").as_int(),
      node->get_parameter("cam_height").as_int(),
      node->get_parameter("cam_fx").as_double(),
      node->get_parameter("cam_fy").as_double(),
      node->get_parameter("cam_cx").as_double(),
      node->get_parameter("cam_cy").as_double(),
      node->get_parameter("cam_d0").as_double()
    );
  } else {
    cam = nullptr;
    res = false;
  }
  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
