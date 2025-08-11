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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_

// romea
#include "romea_path_following/path_following_lateral_control.hpp"
#include "romea_path_following/path_following_longitudinal_control.hpp"
#include "romea_path_following/path_following_parameters.hpp"
#include "romea_path_following/path_following_sliding_observer.hpp"
#include "romea_path_following/path_following_traits.hpp"

namespace romea::ros1
{

inline std::string full_name(const std::string & ns, const std::string & name)
{
  if (name.empty()) {
    return ns;
  }
  return ns + "." + name;
}

template<typename LatCtrl, typename LonCtrl, typename SlObs>
std::unique_ptr<core::path_following::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>>
make_path_following(
  const ros::NodeHandle & nh,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name,
  const std::string & sliding_observer_name)
{
  ros::NodeHandle lat_nh(nh, "lateral_control/" + lateral_control_name);
  ros::NodeHandle lon_nh(nh, "longitudinal_control/" + longitudinal_control_name);
  ros::NodeHandle sliding_nh(nh, "sliding_observer/" + sliding_observer_name);

  using Pf = core::path_following::PathFollowingWithSlidingObserver<LatCtrl, LonCtrl, SlObs>;
  return std::make_unique<Pf>(
    make_lateral_control<LatCtrl>(lat_nh, nh),
    make_longitudinal_control<LonCtrl>(lon_nh),
    make_sliding_observer<SlObs>(sliding_nh, nh));
}

template<typename LatCtrl, typename LonCtrl>
std::unique_ptr<core::path_following::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>>
make_path_following(
  const ros::NodeHandle & nh,
  const std::string & lateral_control_name,
  const std::string & longitudinal_control_name)
{
  ros::NodeHandle lat_nh(nh, "lateral_control/" + lateral_control_name);
  ros::NodeHandle lon_nh(nh, "longitudinal_control/" + longitudinal_control_name);

  using Pf = core::path_following::PathFollowingWithoutSlidingObserver<LatCtrl, LonCtrl>;
  return std::make_unique<Pf>(
    make_lateral_control<LatCtrl>(lat_nh, nh), make_longitudinal_control<LonCtrl>(lon_nh));
}

template<typename CommandType>
struct PathFollowingFactory
{
};

template<>
struct PathFollowingFactory<core::OneAxleSteeringCommand>
{
  using Base = PathFollowingTraits<core::OneAxleSteeringCommand>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<core::OneAxleSteeringCommand>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<core::OneAxleSteeringCommand>::LateralControl::Classic;
  using LatCtrlPredictive =
    PathFollowingTraits<core::OneAxleSteeringCommand>::LateralControl::Predictive;
  using SlObsExtendedCinematic =
    PathFollowingTraits<core::OneAxleSteeringCommand>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov =
    PathFollowingTraits<core::OneAxleSteeringCommand>::SlidingObserver::ExtendedLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LatCtrlClassic>(nh, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "predictive") {
      return make<LatCtrlPredictive>(nh, lateral_control_name, sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive]");
  }

  template<typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control_name, "");
    }
    if (sliding_observer_name == "extended_cinematic") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::TwoAxleSteeringCommand>
{
  using Command = core::TwoAxleSteeringCommand;
  using Base = PathFollowingTraits<Command>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<Command>::LongitudinalControl::Classic;
  using LatCtrlClassic = PathFollowingTraits<Command>::LateralControl::Classic;
  using LatCtrlPredictive = PathFollowingTraits<Command>::LateralControl::Predictive;
  using LatCtrlDecoupled = PathFollowingTraits<Command>::LateralControl::FrontRearDecoupled;
  using SlObsExtendedCinematic = PathFollowingTraits<Command>::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = PathFollowingTraits<Command>::SlidingObserver::ExtendedLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (lateral_control_name == "classic") {
      return make<LatCtrlClassic>(nh, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "predictive") {
      return make<LatCtrlPredictive>(nh, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "front_rear_decoupled") {
      return make<LatCtrlDecoupled>(nh, lateral_control_name, sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [classic, predictive, front_rear_decoupled]");
  }

  template<typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control_name, "");
    }
    if (sliding_observer_name == "extended_cinematic") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    if (sliding_observer_name == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::SkidSteeringCommand>
{
  using Command = core::SkidSteeringCommand;
  using Base = PathFollowingTraits<Command>::PathFollowingBase;
  using LonCtrl = PathFollowingTraits<Command>::LongitudinalControl::Classic;
  using LatCtrlBackStepping = PathFollowingTraits<Command>::LateralControl::BackStepping;
  using LatCtrlSkidBs = PathFollowingTraits<Command>::LateralControl::SkidBackstepping;
  using LatCtrlDebosGen = PathFollowingTraits<Command>::LateralControl::DesbosGeneric;
  using LatCtrlDebosGenPred = PathFollowingTraits<Command>::LateralControl::DesbosGenericPredictive;
  using SOPSBackstepping = PathFollowingTraits<Command>::SlidingObserver::PicardSkidBackstepping;
  using SOPSLyapunov = PathFollowingTraits<Command>::SlidingObserver::PicardSkidLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name,
    bool one_axle_steering_equivalence = false)
  {
    if (one_axle_steering_equivalence) {
      return std::make_unique<core::path_following::OneAxleSteeringEquivalence>(
        PathFollowingFactory<core::OneAxleSteeringCommand>::make(
          nh, lateral_control_name, sliding_observer_name),
        get_wheelbase(nh));
    }
    if (lateral_control_name == "back_stepping") {
      if (sliding_observer_name == "none") {
        return make_path_following<LatCtrlBackStepping, LonCtrl>(nh, lateral_control_name, "");
      }
      throw std::runtime_error(
        "Unknown sliding_observer algorithm '" + sliding_observer_name + "'");
    }
    if (lateral_control_name == "skid_backstepping") {
      return make<LatCtrlSkidBs>(nh, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "desbos_generic") {
      return make<LatCtrlDebosGen>(nh, lateral_control_name, sliding_observer_name);
    }
    if (lateral_control_name == "desbos_generic_predictive") {
      return make<LatCtrlDebosGenPred>(nh, lateral_control_name, sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control_name +
      "'. Available: [back_stepping, skid_backstepping, desbos_generic, "
      "desbos_generic_predictive]");
  }

  template<typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & lateral_control_name,
    const std::string & sliding_observer_name)
  {
    if (sliding_observer_name == "none") {
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control_name, "");
    }
    if (sliding_observer_name == "picard_skid_backstepping") {
      return make_path_following<LatCtrl, LonCtrl, SOPSBackstepping>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    if (sliding_observer_name == "picard_skid_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SOPSLyapunov>(
        nh, lateral_control_name, "", sliding_observer_name);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer_name +
      "'. Available: [none, picard_skid_backstepping, picard_skid_lyapunov]");
  }
};

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
