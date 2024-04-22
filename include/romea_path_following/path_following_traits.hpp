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
#include "romea_core_path_following/PathFollowing.hpp"
#include "romea_core_path_following/lateral_control/LateralControlBackStepping.hpp"
#include "romea_core_path_following/lateral_control/LateralControlClassic.hpp"
#include "romea_core_path_following/lateral_control/LateralControlPredictive.hpp"
#include "romea_core_path_following/longitudinal_control/LongitudinalControlClassic.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLinearTangent.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLyapunov.hpp"

namespace romea
{
namespace ros1
{

template<typename CommandType>
struct PathFollowingTraits
{
};

template<>
struct PathFollowingTraits<core::OneAxleSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::OneAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic = core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>;
    using Predictive = core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
      core::OneAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::TwoAxleSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::TwoAxleSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>;
  };

  struct LateralControl
  {
    using Classic = core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>;
    using Predictive = core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>;
  };

  struct SlidingObserver
  {
    using ExtendedCinematic = core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
      core::TwoAxleSteeringCommand>;
    using ExtendedLyapunov =
      core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::TwoAxleSteeringCommand>;
  };
};

template<>
struct PathFollowingTraits<core::SkidSteeringCommand>
{
  using PathFollowingBase = core::PathFollowingBase<core::SkidSteeringCommand>;

  struct LongitudinalControl
  {
    using Classic = core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>;
  };

  struct LateralControl
  {
    using BackStepping = core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>;
  };

  struct SlidingObserver
  {
  };
};

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_TRAITS_HPP_
