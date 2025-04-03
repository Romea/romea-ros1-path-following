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

// romea
#include <romea_common_utils/params/algorithm_parameters.hpp>
#include <romea_mobile_base_utils/params/command_interface_parameters.hpp>

// local
#include "romea_path_following/path_following.hpp"
#include "romea_path_following/path_following_factory.hpp"

namespace romea::ros1
{
//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowing<CommandType>::PathFollowing(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
: nh_(nh), private_nh_(private_nh)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::configure()
{
  setpoint_.store(get_setpoint(private_nh_));
  if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
    path_following_ = PathFollowingFactory<CommandType>::make(
      private_nh_,
      get_selected_lateral_control(private_nh_),
      get_selected_sliding_observer(private_nh_),
      get_one_steering_equivalence(private_nh_));
  } else {
    path_following_ = PathFollowingFactory<CommandType>::make(
      private_nh_,
      get_selected_lateral_control(private_nh_),
      get_selected_sliding_observer(private_nh_));
  }

  if (get_debug(private_nh_)) {
    logger_ = std::make_shared<core::SimpleFileLogger>(get_log_filename(private_nh_));
    path_following_->register_logger(logger_);
  }

  command_limits_.store(get_base_command_limits<CommandLimits>(private_nh_));
  auto interface_config =
    get_command_interface_configuration(ros::NodeHandle(private_nh_, "cmd_output"));
  cmd_interface_ = std::make_unique<VehiculeInterface>(private_nh_, std::move(interface_config));

  matching_sub_ = nh_.subscribe("matching/info", 1, &PathFollowing::process_matching_info_, this);

  odometry_sub_ = nh_.subscribe("odometry", 1, &PathFollowing::process_odometry_, this);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::activate()
{
  path_following_->reset();
  cmd_interface_->start();
}

template<class CommandType>
void PathFollowing<CommandType>::deactivate()
{
  cmd_interface_->stop(true);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::process_odometry_(const OdometryMeasureMsg & msg)
{
  odometry_measure_.store(to_romea(msg.measure));
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_matching_info_(
  const romea_path_msgs::PathMatchingInfo2D & msg)
{
  core::Twist2D filtered_twist = romea::to_romea(msg.twist);
  std::vector<core::PathMatchedPoint2D> matchedPoints = romea::to_romea(msg.matched_points);

  if (cmd_interface_->is_started()) {
    auto command = path_following_->compute_command(
      setpoint_.load(),
      command_limits_.load(),
      matchedPoints,
      odometry_measure_.load(),
      filtered_twist);

    if (command) {
      cmd_interface_->send_command(*command);

      if (logger_) {
        logger_->addEntry("t", ros::Time(msg.header.stamp).toSec());
        logger_->writeRow();
      }
    } else {
      cmd_interface_->send_null_command();
    }
  }
}

template class PathFollowing<core::TwoAxleSteeringCommand>;
template class PathFollowing<core::OneAxleSteeringCommand>;
template class PathFollowing<core::SkidSteeringCommand>;

} // namespace romea::ros1

