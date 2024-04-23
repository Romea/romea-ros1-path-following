// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ros
#include <pluginlib/class_list_macros.h>

// romea
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_mobile_base/info/MobileBaseType.hpp>

// local
#include "romea_path_following/path_following_nodelet.hpp"

namespace romea
{
namespace ros1
{

void PathFollowingNodelet::onInit()
try {
  auto & nh = getNodeHandle();
  auto & priv_nh = getPrivateNodeHandle();

  auto mobile_base_type = load_param<std::string>(priv_nh, "base/type");
  auto command_type = core::get_command_type(mobile_base_type);
  if (command_type == "two_axle_steering") {
    following_ = std::make_unique<PathFollowing<core::TwoAxleSteeringCommand>>(nh, priv_nh);
  } else if (command_type == "one_axle_steering") {
    following_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(nh, priv_nh);
  } else if (command_type == "skid_steering") {
    following_ = std::make_unique<PathFollowing<core::SkidSteeringCommand>>(nh, priv_nh);
  } else {
    throw std::runtime_error("Mobile base type " + mobile_base_type + " is not supported");
  }

  if (load_param_or<bool>(priv_nh, "autoconfigure", false)) {
    bool configured = on_configure();

    if (load_param_or<bool>(priv_nh, "autostart", false) && configured) {
      on_activate();
    }
  }
} catch (const std::exception & e) {
  ROS_ERROR_STREAM("node initialization failed: " << e.what());
}

//-----------------------------------------------------------------------------
bool PathFollowingNodelet::on_configure()
try {
  following_->configure();

  ros::NodeHandle & nh = getNodeHandle();
  ros::NodeHandle & priv_nh = getPrivateNodeHandle();
  joy_nh_ = ros::NodeHandle(priv_nh, "joy");
  joystick_ = std::make_unique<Joystick>(nh, joy_nh_);

  joystick_->registerButtonCallback("start", JoystickButton::PRESSED, [this]() {
    ROS_INFO("press start");
    on_activate();
  });

  joystick_->registerButtonCallback("stop", JoystickButton::PRESSED, [this]() {
    ROS_INFO("press stop");
    on_deactivate();
  });

  ROS_INFO("configured");
  return true;

} catch (const std::runtime_error & e) {
  ROS_ERROR_STREAM("configuration failed: " << e.what());
  return false;
}

//-----------------------------------------------------------------------------
bool PathFollowingNodelet::on_activate()
{
  following_->activate();
  ROS_INFO("activated");
  return true;
}

//-----------------------------------------------------------------------------
bool PathFollowingNodelet::on_deactivate()
{
  following_->deactivate();
  ROS_INFO("deactivated");
  return true;
}

}  // namespace ros1
}  // namespace romea

PLUGINLIB_EXPORT_CLASS(romea::ros1::PathFollowingNodelet, nodelet::Nodelet)
