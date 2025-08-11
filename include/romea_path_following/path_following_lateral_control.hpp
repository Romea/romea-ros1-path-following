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

// romea
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_path_following/lateral_control/back_stepping.hpp>
#include <romea_core_path_following/lateral_control/classic.hpp>
#include <romea_core_path_following/lateral_control/front_rear_decoupled.hpp>
#include <romea_core_path_following/lateral_control/predictive.hpp>
#include <romea_core_path_following/lateral_control/skid_backstepping.hpp>
#include <romea_core_path_following/lateral_control/desbos_generic.hpp>
#include <romea_core_path_following/lateral_control/desbos_generic_predictive.hpp>
#include <romea_core_path_following/utils.hpp>

// local
#include "romea_path_following/path_following_parameters.hpp"

namespace romea::ros1
{

template<typename LateralControl>
struct PathFollowingLateralControlParameters
{
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlClassic<core::OneAxleSteeringCommand>>
{
  using LateralControl = core::path_following::LateralControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {{load_param<double>(nh, "gains/front_kd")}};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlClassic<core::TwoAxleSteeringCommand>>
{
  using LateralControl = core::path_following::LateralControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {load_param<double>(nh, "gains/front_kd"),
       load_param_or<double>(nh, "gains/rear_kd", std::numeric_limits<double>::quiet_NaN())}};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlPredictive<core::OneAxleSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlPredictive<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {load_param<double>(nh, "gains/front_kd")},
      load_param<int>(nh, "prediction/horizon"),
      load_param<double>(nh, "prediction/a0"),
      load_param<double>(nh, "prediction/a1"),
      load_param<double>(nh, "prediction/b1"),
      load_param<double>(nh, "prediction/b2")};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlPredictive<core::TwoAxleSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlPredictive<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {
        load_param<double>(nh, "gains/front_kd"),
        load_param_or<double>(nh, "gains/rear_kd", std::numeric_limits<double>::quiet_NaN()),
      },
      load_param<int>(nh, "prediction/horizon"),
      load_param<double>(nh, "prediction/a0"),
      load_param<double>(nh, "prediction/a1"),
      load_param<double>(nh, "prediction/b1"),
      load_param<double>(nh, "prediction/b2")};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlFrontRearDecoupled<core::TwoAxleSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlFrontRearDecoupled<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {
        load_param<double>(nh, "gains/front_kp"),
        load_param<double>(nh, "gains/rear_kp"),
      },
    };
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlBackStepping<core::SkidSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlBackStepping<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {
        load_param<double>(nh, "gains/kp"),
        load_param<double>(nh, "gains/kd"),
      },
      load_param<double>(nh, "maximal_omega_d")};
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlSkidBackstepping<core::SkidSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlSkidBackstepping<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      {
        load_param<double>(nh, "gains/lateral_kp"),
        load_param<double>(nh, "gains/course_kp"),
      },
      load_param<double>(nh, "maximal_target_course_deg") * M_PI / 180.,
    };
  };
};

template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlDesbosGeneric<core::SkidSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlDesbosGeneric<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    constexpr auto nan = std::numeric_limits<double>::quiet_NaN();
    return {
      {
        load_param<double>(nh, "gains/lateral_kp"),
        load_param<double>(nh, "gains/course_kp"),
        load_param_or<double>(nh, "gains/longitudinal_kp", nan),
      },
      load_param<bool>(nh, "use_adaptive_gains"),
    };
  };
};


template<>
struct PathFollowingLateralControlParameters<
  core::path_following::LateralControlDesbosGenericPredictive<core::SkidSteeringCommand>>
{
  using LateralControl =
    core::path_following::LateralControlDesbosGenericPredictive<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    constexpr auto nan = std::numeric_limits<double>::quiet_NaN();
    return {
      {
        load_param<double>(nh, "gains/lateral_kp"),
        load_param<double>(nh, "gains/course_kp"),
        load_param_or<double>(nh, "gains/longitudinal_kp", nan),
      },
      load_param<double>(nh, "alpha"),
      load_param<double>(nh, "prediction/a0"),
      load_param<double>(nh, "prediction/a1"),
      load_param<double>(nh, "prediction/b1"),
      load_param<double>(nh, "prediction/b2"),
      load_param<int>(nh, "prediction/horizon"),
      load_param<bool>(nh, "use_adaptive_gains"),
      load_param<bool>(nh, "use_lmpc"),
      load_param<int>(nh, "lmpc_model_order"),
    };
  }
};

template<typename LateralControl>
typename LateralControl::Parameters get_lateral_control_parameters(const ros::NodeHandle & nh)
{
  return PathFollowingLateralControlParameters<LateralControl>::get(nh);
}

template<typename LateralControl>
std::shared_ptr<LateralControl> make_lateral_control(
  const ros::NodeHandle & control_nh, const ros::NodeHandle & root_nh)
{
  return std::make_shared<LateralControl>(
    get_sampling_period(root_nh),
    get_wheelbase(root_nh),
    get_inertia(root_nh),
    get_lateral_control_parameters<LateralControl>(control_nh));
}

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LATERAL_CONTROL_HPP_
