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

#include "romea_path_following/path_following_factory.hpp"

class TestPathFollowingFactory : public ::testing::Test
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
    std::string config_filename = std::string(TEST_DIR) + "/test_path_following_factory.yaml";

    rclcpp::NodeOptions no;
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_path_following_factory", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestPathFollowingFactory, TestFactorySkidSteeringCommandBackSteppingNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::SkidSteeringCommand>::make(
      node, "back_stepping", "none") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactorySkidSteeringCommandBackSteppingNotNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::SkidSteeringCommand>::make(
      node, "back_stepping", "not_none") == nullptr);
}


TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandClassicNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "classic", "none") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandPredictiveNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "predictive", "none") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandClassicCinematic)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "classic", "extended_cinematic") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandClassicLyapunov)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "classic", "extended_lyapunov") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandPredictiveCinematic)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "predictive", "extended_cinematic") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryOneAxleSteeringCommandPredictiveLyapunov)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::OneAxleSteeringCommand>::make(
      node, "predictive", "extended_lyapunov") != nullptr);
}


TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandClassicNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "classic", "none") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandPredictiveNone)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "predictive", "none") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandClassicCinematic)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "classic", "extended_cinematic") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandClassicLyapunov)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "classic", "extended_lyapunov") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandPredictiveCinematic)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "predictive", "extended_cinematic") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactoryTwoAxleSteeringCommandPredictiveLyapunov)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::TwoAxleSteeringCommand>::make(
      node, "predictive", "extended_lyapunov") != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactorySkidSteeringCommandClassicNoneByEquivalence)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::SkidSteeringCommand>::make(
      node, "classic", "none", true) != nullptr);
}

TEST_F(TestPathFollowingFactory, TestFactorySkidSteeringCommandClassicCinematicByEquivalence)
{
  EXPECT_TRUE(
    romea::ros2::PathFollowingFactory<romea::core::SkidSteeringCommand>::make(
      node, "classic", "extended_cinematic", true) != nullptr);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
