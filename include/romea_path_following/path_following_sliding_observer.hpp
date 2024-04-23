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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_

// ros
#include <ros/ros.h>

#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLinearTangent.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLyapunov.hpp"
#include "romea_path_following/path_following_parameters.hpp"

namespace romea
{
namespace ros1
{

template<typename Observer>
struct PathFollowingSlidingObserverParameters
{
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::OneAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      load_param<double>(nh, "gains/lateral_deviation"),
      load_param<double>(nh, "gains/course_deviation"),
      load_param<double>(nh, "filter_weights/lateral_deviation"),
      load_param<double>(nh, "filter_weights/course_deviation"),
      load_param<double>(nh, "filter_weights/front_sliding_angle"),
      load_param<double>(nh, "filter_weights/rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::TwoAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::TwoAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      load_param<double>(nh, "gains/lateral_deviation"),
      load_param<double>(nh, "gains/course_deviation"),
      load_param<double>(nh, "filter_weights/lateral_deviation"),
      load_param<double>(nh, "filter_weights/course_deviation"),
      load_param<double>(nh, "filter_weights/front_sliding_angle"),
      load_param<double>(nh, "filter_weights/rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      load_param<double>(nh, "gains/x_deviation"),
      load_param<double>(nh, "gains/y_deviation"),
      load_param<double>(nh, "gains/course_deviation"),
      load_param<double>(nh, "gains/front_sliding_angle"),
      load_param<double>(nh, "gains/rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::TwoAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  static Parameters get(const ros::NodeHandle & nh)
  {
    return {
      load_param<double>(nh, "gains/x_deviation"),
      load_param<double>(nh, "gains/y_deviation"),
      load_param<double>(nh, "gains/course_deviation"),
      load_param<double>(nh, "gains/front_sliding_angle"),
      load_param<double>(nh, "gains/rear_sliding_angle")};
  }
};

template<typename SlidingObserver>
typename SlidingObserver::Parameters get_sliding_observer_parameters(const ros::NodeHandle & nh)
{
  return PathFollowingSlidingObserverParameters<SlidingObserver>::get(nh);
}

template<typename SlidingObserver>
std::shared_ptr<SlidingObserver> make_sliding_observer(
  const ros::NodeHandle & sliding_nh, const ros::NodeHandle & root_nh)
{
  return std::make_shared<SlidingObserver>(
    get_sampling_period(root_nh),
    get_wheelbase(root_nh),
    get_inertia(root_nh),
    get_sliding_observer_parameters<SlidingObserver>(sliding_nh));
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_
