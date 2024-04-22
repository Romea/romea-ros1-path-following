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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_

// ros
#include <ros/ros.h>

// romea
#include <romea_core_path_following/longitudinal_control/LongitudinalControlClassic.hpp>

namespace romea
{
namespace ros1
{

template<typename LongitudinalControl>
struct PathFollowingLongitudinalControlParameters
{
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>>
{
  using Longitudinal = core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(ros::NodeHandle /*nh*/) { return {}; }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>>
{
  using Longitudinal = core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(ros::NodeHandle /*nh*/) { return {}; }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>>
{
  using Longitudinal = core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(ros::NodeHandle /*nh*/) { return {}; }
};

template<typename LongitudinalControl>
typename LongitudinalControl::Parameters get_longitudinal_control_parameters(
  const ros::NodeHandle & nh)
{
  return PathFollowingLongitudinalControlParameters<LongitudinalControl>::get(nh);
}

template<typename LongitudinalControl>
std::shared_ptr<LongitudinalControl> make_longitudinal_control(
  const ros::NodeHandle & nh)
{
  return std::make_shared<LongitudinalControl>(
    get_longitudinal_control_parameters<LongitudinalControl>(nh));
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_
