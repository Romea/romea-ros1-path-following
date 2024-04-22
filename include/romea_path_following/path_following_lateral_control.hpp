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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LATERAL_CONTROL_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LATERAL_CONTROL_HPP_

// ros
#include <ros/ros.h>

#include "romea_core_path_following/lateral_control/LateralControlBackStepping.hpp"
#include "romea_core_path_following/lateral_control/LateralControlClassic.hpp"
#include "romea_core_path_following/lateral_control/LateralControlPredictive.hpp"
#include "romea_path_following/path_following_parameters.hpp"

namespace romea
{
namespace ros1
{

template<typename LateralControl>
struct PathFollowingLateralControlParameters
{
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>>
{
  using LateralControl = core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {{load_param<double>(nh, "gains/front_kd")}};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>>
{
  using LateralControl = core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {load_param<double>(nh, "gains.front_kd"),
       load_param_or<double>(nh, "gains.rear_kd", std::numeric_limits<double>::quiet_NaN())}};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>>
{
  using LateralControl = core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {load_param<double>(nh, "gains.front_kd")},
      load_param<int>(nh, "prediction.horizon"),
      load_param<double>(nh, "prediction.a0"),
      load_param<double>(nh, "prediction.a1"),
      load_param<double>(nh, "prediction.b1"),
      load_param<double>(nh, "prediction.b2")};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>>
{
  using LateralControl = core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {load_param<double>(nh, "gains.front_kd"),
       load_param_or<double>(nh, "gains.rear_kd", std::numeric_limits<double>::quiet_NaN())},
      load_param<int>(nh, "prediction.horizon"),
      load_param<double>(nh, "prediction.a0"),
      load_param<double>(nh, "prediction.a1"),
      load_param<double>(nh, "prediction.b1"),
      load_param<double>(nh, "prediction.b2")};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>>
{
  using LateralControl = core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {
        load_param<double>(nh, "gains.kp"),
        load_param_or<double>(nh, "gains.ki", 0.0),
        load_param<double>(nh, "gains.kd"),
        load_param_or<double>(nh, "gains.iclamp", 0.0),
      },
      load_param<double>(nh, "maximal_omega_d")};
  }
};

template<typename LateralControl>
typename LateralControl::Parameters get_lateral_control_parameters(const ros::NodeHandle & nh)
{
  return PathFollowingLateralControlParameters<LateralControl>::get(nh);
}

template<typename LateralControl>
std::shared_ptr<LateralControl> make_lateral_control(const ros::NodeHandle & nh)
{
  return std::make_shared<LateralControl>(
    get_sampling_period(nh),
    get_wheelbase(nh),
    get_inertia(nh),
    get_lateral_control_parameters<LateralControl>(nh));
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LATERAL_CONTROL_HPP_
