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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_

// romea
#include <romea_core_path_following/lateral_control/back_stepping.hpp>
#include <romea_core_path_following/lateral_control/classic.hpp>
#include <romea_core_path_following/lateral_control/desbos_generic.hpp>
#include <romea_core_path_following/lateral_control/desbos_generic_predictive.hpp>
#include <romea_core_path_following/lateral_control/front_rear_decoupled.hpp>
#include <romea_core_path_following/lateral_control/predictive.hpp>
#include <romea_core_path_following/lateral_control/skid_backstepping.hpp>
#include <romea_core_path_following/longitudinal_control/classic.hpp>
#include <romea_core_path_following/longitudinal_control/curvature_transition.hpp>
#include <romea_core_path_following/path_following.hpp>
#include <romea_core_path_following/sliding_observer/extended/cinematic_linear_tangent.hpp>
#include <romea_core_path_following/sliding_observer/extended/cinematic_lyapunov.hpp>
#include <romea_core_path_following/sliding_observer/skid/picard_backstepping.hpp>
#include <romea_core_path_following/sliding_observer/skid/picard_lyapunov.hpp>

namespace romea::ros1
{

template<typename CommandType>
struct PathFollowingTraits
{
};

template<>
struct PathFollowingTraits<core::OneAxleSteeringCommand>
{
  using PathFollowingBase = core::path_following::PathFollowingBase<core::OneAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::path_following::LongitudinalControlClassic<core::OneAxleSteeringCommand>;
    using CurvatureTransition =
      core::path_following::LongitudinalControlCurvatureTransition<
        core::OneAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic = core::path_following::LateralControlClassic<core::OneAxleSteeringCommand>;
    using Predictive = core::path_following::LateralControlPredictive<core::OneAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = core::path_following::SlidingObserverExtendedCinematicLinearTangent<
      core::OneAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::path_following::SlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::TwoAxleSteeringCommand>
{
  using PathFollowingBase = core::path_following::PathFollowingBase<core::TwoAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::path_following::LongitudinalControlClassic<core::TwoAxleSteeringCommand>;
    using CurvatureTransition =
      core::path_following::LongitudinalControlCurvatureTransition<
        core::TwoAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic = core::path_following::LateralControlClassic<core::TwoAxleSteeringCommand>;
    using Predictive = core::path_following::LateralControlPredictive<core::TwoAxleSteeringCommand>;
    using FrontRearDecoupled =
      core::path_following::LateralControlFrontRearDecoupled<core::TwoAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = core::path_following::SlidingObserverExtendedCinematicLinearTangent<
      core::TwoAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::path_following::SlidingObserverExtendedCinematicLyapunov<core::TwoAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::SkidSteeringCommand>
{
  using PathFollowingBase = core::path_following::PathFollowingBase<core::SkidSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::path_following::LongitudinalControlClassic<core::SkidSteeringCommand>;
    using CurvatureTransition =
      core::path_following::LongitudinalControlCurvatureTransition<
        core::SkidSteeringCommand>;
  };

  struct LateralControl
  {
    using BackStepping =
      core::path_following::LateralControlBackStepping<core::SkidSteeringCommand>;
    using SkidBackstepping =
      core::path_following::LateralControlSkidBackstepping<core::SkidSteeringCommand>;
    using DesbosGeneric =
      core::path_following::LateralControlDesbosGeneric<core::SkidSteeringCommand>;
    using DesbosGenericPredictive =
      core::path_following::LateralControlDesbosGenericPredictive<core::SkidSteeringCommand>;
  };

  struct SlidingObserver
  {
    using PicardSkidBackstepping =
      core::path_following::SlidingObserverPicardSkidBackstepping<core::SkidSteeringCommand>;
    using PicardSkidLyapunov =
      core::path_following::SlidingObserverPicardSkidLyapunov<core::SkidSteeringCommand>;
  };
};

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_
