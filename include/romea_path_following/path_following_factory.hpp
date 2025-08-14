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
  using Traits = PathFollowingTraits<core::OneAxleSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlConstant = Traits::LongitudinalControl::Constant;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlClassic = Traits::LateralControl::Classic;
  using LatCtrlPredictive = Traits::LateralControl::Predictive;
  using SlObsExtendedCinematic = Traits::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = Traits::SlidingObserver::ExtendedLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (longi_control == "constant") {
      return make<LonCtrlConstant>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "classic") {
      return make<LonCtrlClassic>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longi_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template<typename LonCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (lateral_control == "classic") {
      return make<LonCtrl, LatCtrlClassic>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (lateral_control == "predictive") {
      return make<LonCtrl, LatCtrlPredictive>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control +
      "'. Available: [classic, predictive]");
  }

  template<typename LonCtrl, typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (sliding_observer == "none") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control, "");
    }
    if (sliding_observer == "extended_cinematic") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    if (sliding_observer == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::TwoAxleSteeringCommand>
{
  using Traits = PathFollowingTraits<core::TwoAxleSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlConstant = Traits::LongitudinalControl::Constant;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlClassic = Traits::LateralControl::Classic;
  using LatCtrlPredictive = Traits::LateralControl::Predictive;
  using LatCtrlDecoupled = Traits::LateralControl::FrontRearDecoupled;
  using SlObsExtendedCinematic = Traits::SlidingObserver::ExtendedCinematic;
  using SlObsExtendedLyapunov = Traits::SlidingObserver::ExtendedLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (longi_control == "constant") {
      return make<LonCtrlConstant>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "classic") {
      return make<LonCtrlClassic>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longi_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template<typename LonCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (lateral_control == "classic") {
      return make<LonCtrl, LatCtrlClassic>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (lateral_control == "predictive") {
      return make<LonCtrl, LatCtrlPredictive>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (lateral_control == "front_rear_decoupled") {
      return make<LonCtrl, LatCtrlDecoupled>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control +
      "'. Available: [classic, predictive, front_rear_decoupled]");
  }

  template<typename LonCtrl, typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (sliding_observer == "none") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control, "");
    }
    if (sliding_observer == "extended_cinematic") {
      // return nullptr;
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedCinematic>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    if (sliding_observer == "extended_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SlObsExtendedLyapunov>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer +
      "'. Available: [none, extended_cinematic, extended_lyapunov]");
  }
};

template<>
struct PathFollowingFactory<core::SkidSteeringCommand>
{
  using Traits = PathFollowingTraits<core::SkidSteeringCommand>;
  using Base = Traits::PathFollowingBase;
  using LonCtrlConstant = Traits::LongitudinalControl::Constant;
  using LonCtrlClassic = Traits::LongitudinalControl::Classic;
  using LonCtrlCurvTrans = Traits::LongitudinalControl::CurvatureTransition;
  using LatCtrlBackStepping = Traits::LateralControl::BackStepping;
  using LatCtrlSkidBs = Traits::LateralControl::SkidBackstepping;
  using LatCtrlDebosGen = Traits::LateralControl::DesbosGeneric;
  using LatCtrlDebosGPred = Traits::LateralControl::DesbosGenericPredictive;
  using SOPSBackstepping = Traits::SlidingObserver::PicardSkidBackstepping;
  using SOPSLyapunov = Traits::SlidingObserver::PicardSkidLyapunov;

  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer,
    bool one_axle_steering_equivalence = false)
  {
    if (one_axle_steering_equivalence) {
      return std::make_unique<core::path_following::OneAxleSteeringEquivalence>(
        PathFollowingFactory<core::OneAxleSteeringCommand>::make(
          nh, longi_control, lateral_control, sliding_observer),
        get_wheelbase(nh));
    }
    if (longi_control == "constant") {
      return make<LonCtrlConstant>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "classic") {
      return make<LonCtrlClassic>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (longi_control == "curvature_transition") {
      return make<LonCtrlCurvTrans>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown longitudinal_control '"} + longi_control +
      "'. Available: [constant, classic, curvature_transition]");
  }

  template <typename LonCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (lateral_control == "back_stepping") {
      if (sliding_observer == "none") {
        return make_path_following<LatCtrlBackStepping, LonCtrl>(nh, lateral_control, "");
      }
      throw std::runtime_error(
        "Unknown sliding_observer algorithm '" + sliding_observer + "'");
    }
    if (lateral_control == "skid_backstepping") {
      return make<LonCtrl, LatCtrlSkidBs>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (lateral_control == "desbos_generic") {
      return make<LonCtrl, LatCtrlDebosGen>(nh, longi_control, lateral_control, sliding_observer);
    }
    if (lateral_control == "desbos_generic_predictive") {
      return make<LonCtrl, LatCtrlDebosGPred>(nh, longi_control, lateral_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown lateral_control '"} + lateral_control +
      "'. Available: [back_stepping, skid_backstepping, desbos_generic, "
      "desbos_generic_predictive]");
  }

  template<typename LonCtrl, typename LatCtrl>
  static std::unique_ptr<Base> make(
    const ros::NodeHandle & nh,
    const std::string & longi_control,
    const std::string & lateral_control,
    const std::string & sliding_observer)
  {
    if (sliding_observer == "none") {
      return make_path_following<LatCtrl, LonCtrl>(nh, lateral_control, longi_control);
    }
    if (sliding_observer == "picard_skid_backstepping") {
      return make_path_following<LatCtrl, LonCtrl, SOPSBackstepping>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    if (sliding_observer == "picard_skid_lyapunov") {
      return make_path_following<LatCtrl, LonCtrl, SOPSLyapunov>(
        nh, lateral_control, longi_control, sliding_observer);
    }
    throw std::runtime_error(
      std::string{"Unknown sliding_observer '"} + sliding_observer +
      "'. Available: [none, picard_skid_backstepping, picard_skid_lyapunov]");
  }
};

}  // namespace romea::ros1

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_FACTORY_HPP_
