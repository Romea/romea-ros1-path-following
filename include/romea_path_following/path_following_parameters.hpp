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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_

// ros1
#include <ros/ros.h>

// romea
#include <romea_core_path_following/setpoint.hpp>
#include <romea_mobile_base_utils/params/command_limits_parameters.hpp>
#include <romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp>

namespace romea::ros1
{

inline double get_sampling_period(const ros::NodeHandle & nh)
{
  return load_param<double>(nh, "sampling_period");
}

inline bool get_one_steering_equivalence(const ros::NodeHandle & nh)
{
  return load_param_or<bool>(nh, "one_steering_equivalence", false);
}

inline std::string get_base_type(const ros::NodeHandle & nh)
{
  return load_param<std::string>(nh, "base/type");
}

inline double get_wheelbase(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "base/wheelbase", 2.0);
}

inline core::MobileBaseInertia get_inertia(const ros::NodeHandle & nh)
{
  return get_inertia_info(ros::NodeHandle(nh, "base/inertia"));
}

template<typename CommandLimits>
CommandLimits get_base_command_limits(const ros::NodeHandle & nh)
{
  return get_command_limits<CommandLimits>(ros::NodeHandle(nh, "base/command_limits"));
}

inline core::path_following::SetPoint get_setpoint(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "setpoint/desired_linear_speed"),
    load_param_or<double>(nh, "setpoint/desired_lateral_deviation", 0.0),
    load_param_or<double>(nh, "setpoint/desired_course_deviation", 0.0),
  };
}

inline std::string get_selected_lateral_control(const ros::NodeHandle & nh)
{
  return load_param<std::string>(nh, "lateral_control/selected");
}

inline std::string get_selected_sliding_observer(const ros::NodeHandle & nh)
{
  return load_param_or<std::string>(nh, "sliding_observer/selected", "none");
}

inline int get_joystick_start_button_mapping(const ros::NodeHandle & nh)
{
  return load_param<int>(nh, "joystick/start");
}

inline int get_joystick_stop_button_mapping(const ros::NodeHandle & nh)
{
  return load_param<int>(nh, "joystick/stop");
}

inline std::map<std::string, int> get_joystick_mapping(const ros::NodeHandle & nh)
{
  return {
    {"start", get_joystick_start_button_mapping(nh)},
    {"stop", get_joystick_stop_button_mapping(nh)},
  };
}

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_
