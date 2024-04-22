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
#include "romea_path_following/path_following_sliding_observer.hpp"

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
    std::string config_filename = std::string(TEST_DIR) + "/test_sliding_observer.yaml";

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_sliding_observer_parameters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestLateralControl, TestLateralSlidingObserverCinematicOneAxleSteering)
{
  using SlidingObserver = romea::core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
    romea::core::OneAxleSteeringCommand>;

  romea::ros2::declare_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.cinematic");

  auto parameters = romea::ros2::get_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.cinematic");

  EXPECT_DOUBLE_EQ(parameters.lateralDeviationGain, 7.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 8.0);
  EXPECT_DOUBLE_EQ(parameters.lateralDeviationFilterWeight, 0.09);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationFilterWeight, 0.10);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleFilterWeight, 0.11);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleFilterWeight, 0.12);
}

TEST_F(TestLateralControl, TestLateralSlidingObserverCinematicTwoAxleSteering)
{
  using SlidingObserver = romea::core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
    romea::core::TwoAxleSteeringCommand>;

  romea::ros2::declare_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.cinematic");

  auto parameters = romea::ros2::get_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.cinematic");

  EXPECT_DOUBLE_EQ(parameters.lateralDeviationGain, 7.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 8.0);
  EXPECT_DOUBLE_EQ(parameters.lateralDeviationFilterWeight, 0.09);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationFilterWeight, 0.10);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleFilterWeight, 0.11);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleFilterWeight, 0.12);
}

TEST_F(TestLateralControl, TestLateralSlidingObserverLyapunovOneAxleSteering)
{
  using SlidingObserver = romea::core::PathFollowingSlidingObserverExtendedCinematicLyapunov<
    romea::core::OneAxleSteeringCommand>;

  romea::ros2::declare_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.lyapunov");

  auto parameters = romea::ros2::get_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.lyapunov");

  EXPECT_DOUBLE_EQ(parameters.xDeviationGain, 13.0);
  EXPECT_DOUBLE_EQ(parameters.yDeviationGain, 14.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 15.0);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleGain, 16.0);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleGain, 17.0);
}

TEST_F(TestLateralControl, TestLateralSlidingObserverLyapunovTwoAxleSteering)
{
  using SlidingObserver = romea::core::PathFollowingSlidingObserverExtendedCinematicLyapunov<
    romea::core::TwoAxleSteeringCommand>;

  romea::ros2::declare_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.lyapunov");

  auto parameters = romea::ros2::get_sliding_observer_parameters<SlidingObserver>(
    node, "sliding_observer.lyapunov");

  EXPECT_DOUBLE_EQ(parameters.xDeviationGain, 13.0);
  EXPECT_DOUBLE_EQ(parameters.yDeviationGain, 14.0);
  EXPECT_DOUBLE_EQ(parameters.courseDeviationGain, 15.0);
  EXPECT_DOUBLE_EQ(parameters.frontSlidingAngleGain, 16.0);
  EXPECT_DOUBLE_EQ(parameters.rearSlidingAngleGain, 17.0);
}

TEST_F(TestLateralControl, TestCreateSlidingObserverCinematicOneAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using SlidingObserver =
    romea::core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
    romea::core::OneAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_sliding_observer<SlidingObserver>(node, "sliding_observer.cinematic"));
}

TEST_F(TestLateralControl, TestCreateSlidingObserverCinematicTwoAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using SlidingObserver =
    romea::core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<
    romea::core::TwoAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_sliding_observer<SlidingObserver>(node, "sliding_observer.cinematic"));
}


TEST_F(TestLateralControl, TestCreateSlidingObserverLyapunovOneAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using SlidingObserver =
    romea::core::PathFollowingSlidingObserverExtendedCinematicLyapunov<
    romea::core::OneAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_sliding_observer<SlidingObserver>(node, "sliding_observer.lyapunov"));
}

TEST_F(TestLateralControl, TestCreateSlidingObserverLyapunovTwoAxleSteering)
{
  romea::ros2::declare_parameter<double>(node, "sampling_period");
  romea::ros2::declare_parameter<double>(node, "base", "wheelbase");
  romea::ros2::declare_inertia_info(node, "base.inertia");

  using SlidingObserver =
    romea::core::PathFollowingSlidingObserverExtendedCinematicLyapunov<
    romea::core::TwoAxleSteeringCommand>;

  EXPECT_NO_THROW(
    romea::ros2::make_sliding_observer<SlidingObserver>(node, "sliding_observer.lyapunov"));
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
