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
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_path_following/longitudinal_control/classic.hpp>
#include <romea_core_path_following/longitudinal_control/curvature_transition.hpp>
#include <romea_core_path_following/longitudinal_control/constant.hpp>

namespace romea::ros1
{

template<typename LongitudinalControl>
struct PathFollowingLongitudinalControlParameters
{
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::path_following::LongitudinalControlClassic<core::OneAxleSteeringCommand>>
{
  using Longitudinal =
    core::path_following::LongitudinalControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {load_param_or(nh, "minimal_linear_speed", 0.3)};
  }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::path_following::LongitudinalControlClassic<core::TwoAxleSteeringCommand>>
{
  using Longitudinal =
    core::path_following::LongitudinalControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {load_param_or(nh, "minimal_linear_speed", 0.3)};
  }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::path_following::LongitudinalControlClassic<core::SkidSteeringCommand>>
{
  using Longitudinal = core::path_following::LongitudinalControlClassic<core::SkidSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {load_param_or(nh, "minimal_linear_speed", 0.3)};
  }
};

template<typename Command>
struct PathFollowingLongitudinalControlParameters<
  core::path_following::LongitudinalControlCurvatureTransition<Command>>
{
  using Longitudinal = core::path_following::LongitudinalControlCurvatureTransition<Command>;
  using Parameters = typename Longitudinal::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      load_param_or(nh, "minimal_linear_speed", 0.3),
      load_param<double>(nh, "lateral_error_max"),
      load_param<double>(nh, "settling_time"),
      load_param<double>(nh, "settling_distance"),
      load_param<double>(nh, "convergence_ratio"),
    };
  }
};


template<typename Command>
struct PathFollowingLongitudinalControlParameters<
  core::path_following::LongitudinalControlConstant<Command>>
{
  using Longitudinal = core::path_following::LongitudinalControlConstant<Command>;
  using Parameters = typename Longitudinal::Parameters;

  static Parameters get(const ros::NodeHandle &  /*nh*/)
  {
    return {};
  }
};

template<typename LongitudinalControl>
typename LongitudinalControl::Parameters get_longitudinal_control_parameters(
  const ros::NodeHandle & nh)
{
  return PathFollowingLongitudinalControlParameters<LongitudinalControl>::get(nh);
}

template<typename LongitudinalControl>
std::shared_ptr<LongitudinalControl> make_longitudinal_control(const ros::NodeHandle & nh)
{
  return std::make_shared<LongitudinalControl>(
    get_longitudinal_control_parameters<LongitudinalControl>(nh));
}

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_
