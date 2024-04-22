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

// std
#include <string>
#include <memory>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "../test/test_helper.h"

#include "romea_path_following/path_following_parameters.hpp"
#include "romea_path_following/path_following_lateral_control.hpp"

class TestLateralControl : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    std::string config_filename = std::string(TEST_DIR) + "/test_lateral_control.yaml";

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_lateral_control_parameters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestLateralControl, TestLatealControlBackSteppingSkidSteeringParameters)
{
  using LateralControl =
    romea::core::PathFollowingLateralControlBackStepping<romea::core::SkidSteeringCommand>;

  romea::ros2::declare_lateral_control_parameters<LateralControl>(
    node, "lateral_control.back_stepping");

  auto parameters = romea::ros2::get_lateral_control_parameters<LateralControl>(
    node, "lateral_control.back_stepping");

  EXPECT_DOUBLE_EQ(parameters.gains.KP, 30.0);
  EXPECT_DOUBLE_EQ(parameters.gains.KI, 31.0);
  EXPECT_DOUBLE_EQ(parameters.gains.KD, 32.0);
  EXPECT_DOUBLE_EQ(parameters.gains.IClamp, 33.0);
  EXPECT_DOUBLE_EQ(parameters.maximalOmegaD, 37.0);
}

TEST_F(TestLateralControl, TestLateralControlClassicOneAxleSteeingParameters)
{
  using LateralControl =
    romea::core::PathFollowingLateralControlClassic<romea::core::OneAxleSteeringCommand>;

  romea::ros2::declare_lateral_control_parameters<LateralControl>(
    node, "lateral_control.classic");

  auto parameters = romea::ros2::get_lateral_control_parameters<LateralControl>(
    node, "lateral_control.classic");

  EXPECT_DOUBLE_EQ(parameters.gains.frontKD, 10.0);
}

TEST_F(TestLateralControl, TestLateralControlClassicTwoAxleSteeringParameters)
{
  using LateralControl =
    romea::core::PathFollowingLateralControlClassic<romea::core::TwoAxleSteeringCommand>;

  romea::ros2::declare_lateral_control_parameters<LateralControl>(
    node, "lateral_control.classic");

  auto parameters = romea::ros2::get_lateral_control_parameters<LateralControl>(
    node, "lateral_control.classic");

  EXPECT_DOUBLE_EQ(parameters.gains.frontKD, 10.0);
  EXPECT_DOUBLE_EQ(parameters.gains.rearKD, 11.0);
}

TEST_F(TestLateralControl, TestLateralControlPredictiveOneAxleSteeringParameters)
{
  using LateralControl =
    romea::core::PathFollowingLateralControlPredictive<romea::core::OneAxleSteeringCommand>;

  romea::ros2::declare_lateral_control_parameters<LateralControl>(
    node, "lateral_control.predictive");

  auto parameters = romea::ros2::get_lateral_control_parameters<LateralControl>(
    node, "lateral_control.predictive");

  EXPECT_DOUBLE_EQ(parameters.gains.frontKD, 20.0);
  EXPECT_EQ(parameters.horizon, 22);
  EXPECT_DOUBLE_EQ(parameters.a0, 23.0);
  EXPECT_DOUBLE_EQ(parameters.a1, 24.0);
  EXPECT_DOUBLE_EQ(parameters.b1, 25.0);
  EXPECT_DOUBLE_EQ(parameters.b2, 26.0);
}

TEST_F(TestLateralControl, TestLateralControlPredictiveTwoAxleSteeringParameters)
{
  using LateralControl =
    romea::core::PathFollowingLateralControlPredictive<romea::core::TwoAxleSteeringCommand>;

  romea::ros2::declare_lateral_control_parameters<LateralControl>(
    node, "lateral_control.predictive");

  auto parameters = romea::ros2::get_lateral_control_parameters<LateralControl>(
    node, "lateral_control.predictive");

  EXPECT_DOUBLE_EQ(parameters.gains.frontKD, 20.0);
  EXPECT_DOUBLE_EQ(parameters.gains.rearKD, 21.0);
  EXPECT_EQ(parameters.horizon, 22);
  EXPECT_DOUBLE_EQ(parameters.a0, 23.0);
  EXPECT_DOUBLE_EQ(parameters.a1, 24.0);
  EXPECT_DOUBLE_EQ(parameters.b1, 25.0);
  EXPECT_DOUBLE_EQ(parameters.b2, 26.0);
}

TEST_F(TestLateralControl, TestCreateLateralControlBackSteppingSkidSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using LateralControl =
    romea::core::PathFollowingLateralControlBackStepping<romea::core::SkidSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_lateral_control<LateralControl>(node, "lateral_control.back_stepping"));
}

TEST_F(TestLateralControl, TestCreateLateralControlClassicOneAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using LateralControl =
    romea::core::PathFollowingLateralControlClassic<romea::core::OneAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_lateral_control<LateralControl>(node, "lateral_control.classic"));
}

TEST_F(TestLateralControl, TestCreateLateralControlClassicTwoAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using LateralControl =
    romea::core::PathFollowingLateralControlClassic<romea::core::TwoAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_lateral_control<LateralControl>(node, "lateral_control.classic"));
}

TEST_F(TestLateralControl, TestCreateLateralControlPredictiveOneAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using LateralControl =
    romea::core::PathFollowingLateralControlPredictive<romea::core::OneAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_lateral_control<LateralControl>(node, "lateral_control.predictive"));
}

TEST_F(TestLateralControl, TestCreateLateralControlPredictiveTwoAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using LateralControl =
    romea::core::PathFollowingLateralControlPredictive<romea::core::TwoAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_lateral_control<LateralControl>(node, "lateral_control.predictive"));
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
