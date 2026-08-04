// SPDX-License-Identifier: Apache-2.0

#include "dapier_so101_core/joint_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace core = dapier_so101_core;

namespace
{

core::JointModel make_two_joint_model()
{
  return core::JointModel({
    {"joint_a", 1, -1.0, 1.0, 0.5},
    {"joint_b", 2, -2.0, 2.0, 1.0}});
}

}  // namespace

TEST(JointModel, LoadsTheSixJointContract)
{
  const auto model = core::JointModel::from_yaml(
    std::string(TEST_CONFIG_DIR) + "/so101_joint_contract.yaml");

  ASSERT_EQ(model.size(), 6U);
  EXPECT_EQ(model.names().front(), "shoulder_pan");
  EXPECT_EQ(model.names().back(), "gripper");
  EXPECT_EQ(model.joints().front().motor_id, 1);
  EXPECT_EQ(model.joints().back().motor_id, 6);
}

TEST(JointModel, RejectsDuplicateNamesAndMotorIds)
{
  EXPECT_THROW(
    core::JointModel({
      {"same", 1, -1.0, 1.0, 0.5},
      {"same", 2, -1.0, 1.0, 0.5}}),
    std::invalid_argument);
  EXPECT_THROW(
    core::JointModel({
      {"joint_a", 1, -1.0, 1.0, 0.5},
      {"joint_b", 1, -1.0, 1.0, 0.5}}),
    std::invalid_argument);
}

TEST(JointModel, ReordersNamedJointStates)
{
  const auto model = make_two_joint_model();
  const auto ordered = model.reorder(
    {"unrelated", "joint_b", "joint_a"},
    {99.0, 0.25, -0.5});

  ASSERT_EQ(ordered.size(), 2U);
  EXPECT_DOUBLE_EQ(ordered[0], -0.5);
  EXPECT_DOUBLE_EQ(ordered[1], 0.25);
}

TEST(JointModel, RejectsMissingOrNonFiniteJointStates)
{
  const auto model = make_two_joint_model();
  EXPECT_THROW(model.reorder({"joint_a"}, {0.0}), std::invalid_argument);
  EXPECT_THROW(
    model.reorder({"joint_a", "joint_b"}, {0.0, std::nan("")}),
    std::invalid_argument);
}

TEST(JointModel, AppliesPositionAndVelocityLimits)
{
  const auto model = make_two_joint_model();
  const auto result = model.limit({0.0, 0.0}, {2.0, -1.0}, 0.1);

  ASSERT_EQ(result.command.size(), 2U);
  EXPECT_NEAR(result.command[0], 0.05, 1e-12);
  EXPECT_NEAR(result.command[1], -0.10, 1e-12);
  EXPECT_TRUE(result.position_limited[0]);
  EXPECT_TRUE(result.velocity_limited[0]);
  EXPECT_FALSE(result.position_limited[1]);
  EXPECT_TRUE(result.velocity_limited[1]);
  EXPECT_TRUE(result.was_limited());
}

TEST(CalibrationEntry, MapsBothDirectionsAndClamps)
{
  const core::CalibrationEntry normal{
    "joint", 1, 0, 1000, 3000, -1.0, 1.0, false};
  EXPECT_NEAR(normal.raw_to_position(1000), -1.0, 1e-12);
  EXPECT_NEAR(normal.raw_to_position(2000), 0.0, 1e-12);
  EXPECT_NEAR(normal.raw_to_position(4000), 1.0, 1e-12);
  EXPECT_EQ(normal.position_to_raw(-2.0), 1000);
  EXPECT_EQ(normal.position_to_raw(0.0), 2000);

  const core::CalibrationEntry inverted{
    "joint", 1, 0, 1000, 3000, -1.0, 1.0, true};
  EXPECT_NEAR(inverted.raw_to_position(1000), 1.0, 1e-12);
  EXPECT_NEAR(inverted.raw_to_position(3000), -1.0, 1e-12);
  EXPECT_EQ(inverted.position_to_raw(1.0), 1000);
}

TEST(CalibrationSet, RejectsUnverifiedFilesByDefault)
{
  const std::string path = std::string(TEST_CONFIG_DIR) + "/calibration.example.yaml";
  EXPECT_THROW(core::CalibrationSet::from_yaml(path), std::invalid_argument);

  const auto calibration = core::CalibrationSet::from_yaml(path, false);
  const auto model = core::JointModel::from_yaml(
    std::string(TEST_CONFIG_DIR) + "/so101_joint_contract.yaml");
  EXPECT_FALSE(calibration.verified());
  EXPECT_EQ(calibration.device_id(), "EXAMPLE_DO_NOT_USE");
  EXPECT_NO_THROW(calibration.validate_against(model));
}
