// SPDX-License-Identifier: Apache-2.0

#include "dapier_task_executor/task_executor.hpp"

#include <gtest/gtest.h>

namespace dapier_task_executor
{

TEST(TaskExecutorTest, PregraspIsProposalOnly)
{
  TaskExecutor executor;
  const auto decision = executor.evaluate({Skill::kPregrasp, 1, false}, {});
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.previous_phase, TaskPhase::kManipulation);
  EXPECT_EQ(decision.next_phase, TaskPhase::kManipulation);
  EXPECT_FALSE(decision.command_dispatch_allowed);
  EXPECT_FALSE(decision.hardware_execution);
}

TEST(TaskExecutorTest, LiftThenNavigationUsesExplicitGates)
{
  TaskExecutor executor;
  const auto lift = executor.evaluate({Skill::kLift, 1, false}, {true, true, true, true, 0});
  ASSERT_TRUE(lift.accepted) << lift.reason;
  EXPECT_EQ(executor.phase(), TaskPhase::kCarryReady);

  const auto navigation = executor.evaluate({Skill::kNavigate, 2, false}, {true, true, true, true, 0});
  EXPECT_TRUE(navigation.accepted) << navigation.reason;
  EXPECT_EQ(executor.phase(), TaskPhase::kNavigation);
}

TEST(TaskExecutorTest, RejectsMovementWhenBaseIsNotSettled)
{
  TaskExecutor executor;
  const auto decision = executor.evaluate({Skill::kApproach, 1, false}, {true, false, false, true, 0});
  EXPECT_FALSE(decision.accepted);
  EXPECT_NE(decision.reason.find("base is moving"), std::string::npos);
}

TEST(TaskExecutorTest, RejectsStaleAndHardwareClaimingRequests)
{
  TaskExecutor executor;
  EXPECT_TRUE(executor.evaluate({Skill::kPregrasp, 1, false}, {}).accepted);
  EXPECT_FALSE(executor.evaluate({Skill::kApproach, 1, false}, {}).accepted);
  EXPECT_FALSE(executor.evaluate({Skill::kApproach, 2, true}, {}).accepted);
  EXPECT_FALSE(executor.evaluate({Skill::kApproach, 3, false}, {false, true, false, true, 0}).accepted);
}

TEST(TaskExecutorTest, StopIsAlwaysProposalOnlyAndLatches)
{
  TaskExecutor executor;
  const auto stopped = executor.evaluate({Skill::kStop, 0, true}, {false, false, false, false, 0});
  EXPECT_TRUE(stopped.accepted);
  EXPECT_EQ(executor.phase(), TaskPhase::kStopped);
  EXPECT_FALSE(stopped.command_dispatch_allowed);
  EXPECT_FALSE(executor.evaluate({Skill::kPregrasp, 1, false}, {}).accepted);
}

}  // namespace dapier_task_executor
