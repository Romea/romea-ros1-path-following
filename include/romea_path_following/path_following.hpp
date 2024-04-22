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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_

// ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

// romea
#include <romea_common_utils/conversions/twist2d_conversions.hpp>
#include <romea_mobile_base_utils/control/command_interface.hpp>
#include <romea_mobile_base_utils/control/command_traits.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>

// local
#include "romea_path_following/path_following_factory.hpp"

namespace romea
{
namespace ros1
{


class PathFollowingBase
{
public:
  virtual ~PathFollowingBase() = default;
  virtual void configure() = 0;
  virtual void activate() {}
  virtual void deactivate() {}
};

template<class CommandType>
class PathFollowing : public PathFollowingBase
{
public:
  using SetPoint = core::PathFollowingSetPoint;
  using VehiculeInterface = CommandInterface<CommandType>;
  using CommandLimits = typename CommandTraits<CommandType>::CommandLimits;
  using OdometryMeasure = typename CommandTraits<CommandType>::Measure;
  using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

public:
  explicit PathFollowing(ros::NodeHandle & nh);

  void configure() override;
  void activate() override;
  void deactivate() override;

protected:
  void process_matching_info_(const romea_path_msgs::PathMatchingInfo2D & msg);
  void process_odometry_(const OdometryMeasureMsg & msg);

protected:
  ros::NodeHandle nh_;

  std::unique_ptr<VehiculeInterface> cmd_interface_;
  ros::Subscriber matching_sub_;
  ros::Subscriber odometry_sub_;

  core::SharedVariable<SetPoint> setpoint_;
  core::SharedVariable<CommandLimits> command_limits_;
  core::SharedVariable<OdometryMeasure> odometry_measure_;
  std::unique_ptr<core::PathFollowingBase<CommandType>> path_following_;

  std::shared_ptr<core::SimpleFileLogger> logger_;
};

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_HPP_
