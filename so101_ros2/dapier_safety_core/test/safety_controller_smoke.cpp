// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/localization_runtime.hpp"
#include "dapier_safety_core/safety_controller.hpp"
#include "dapier_so101_core/generated/research_realtime_contract.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
int failures = 0;
void expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

std::vector<std::string> names()
{
  return {"left_joint", "right_joint"};
}

dapier_safety_core::SafetyControllerConfig config()
{
  using dapier_safety_core::JointSafetyLimit;
  dapier_safety_core::SafetyControllerConfig value;
  value.joints = {
    JointSafetyLimit{"left_joint", -1.0, 1.0, 0.5},
    JointSafetyLimit{"right_joint", -1.0, 1.0, 0.5},
  };
  value.measured_state_timeout_ns = 100000000;
  value.command_watchdog_ns = 250000000;
  value.command_horizon_s = 0.05;
  return value;
}

dapier_safety_core::MeasuredRobotState measured(std::int64_t timestamp)
{
  dapier_safety_core::MeasuredRobotState value;
  value.received_monotonic_ns = timestamp;
  value.joint_names = names();
  value.joint_position_rad = {0.0, 0.0};
  value.base_settled = true;
  value.collision_clear = true;
  value.carry_pose_clear = true;
  value.grasp_verified = true;
  return value;
}

dapier_so101_core::ResearchControlIntent arm_intent(
  std::uint64_t sequence,
  double left = 0.01,
  double right = -0.01)
{
  dapier_so101_core::ResearchControlIntent value;
  value.schema_version = std::string(
    dapier_so101_core::generated::kResearchRealtimeSchemaVersion);
  value.sequence = sequence;
  value.kind = dapier_so101_core::ResearchIntentKind::kArmJointPosition;
  value.source = "act_replay";
  value.source_monotonic_ns = 10;
  value.ttl_ns = 200000000;
  value.joint_names = names();
  value.joint_position_rad = {left, right};
  value.joint_max_velocity_rad_s = {0.4, 0.4};
  return value;
}

dapier_so101_core::ResearchControlIntent base_intent(std::uint64_t sequence)
{
  dapier_so101_core::ResearchControlIntent value;
  value.schema_version = std::string(
    dapier_so101_core::generated::kResearchRealtimeSchemaVersion);
  value.sequence = sequence;
  value.kind = dapier_so101_core::ResearchIntentKind::kBaseTwist;
  value.source = "nav_replay";
  value.source_monotonic_ns = 10;
  value.ttl_ns = 200000000;
  value.base_linear_x_mps = 0.04;
  value.base_angular_z_rad_s = 0.1;
  return value;
}

dapier_so101_core::ResearchControlIntent hold_intent(std::uint64_t sequence)
{
  dapier_so101_core::ResearchControlIntent value;
  value.schema_version = std::string(
    dapier_so101_core::generated::kResearchRealtimeSchemaVersion);
  value.sequence = sequence;
  value.kind = dapier_so101_core::ResearchIntentKind::kHold;
  value.source = "watchdog";
  value.source_monotonic_ns = 10;
  value.ttl_ns = 200000000;
  return value;
}

dapier_safety_core::SafetyContext arm_context()
{
  return {
    dapier_localization_core::MotionDecision::kProceed,
    true,
    false,
  };
}

}  // namespace

int main()
{
  using dapier_localization_core::MotionDecision;
  using dapier_safety_core::RecordingMockCommandSink;
  using dapier_safety_core::SafetyController;
  using dapier_safety_core::SafetyDecision;

  SafetyController controller(config());
  controller.set_estop_healthy(true);
  auto state = measured(1000000000);

  auto command = controller.evaluate(arm_intent(1), state, arm_context(), 1000000000);
  expect(command.decision == SafetyDecision::kReject,
    "motion must reject while operator enable is false");

  controller.set_operator_enabled(true);
  command = controller.evaluate(arm_intent(1), state, arm_context(), 1000000000);
  expect(command.decision == SafetyDecision::kDispatch, "valid arm intent should dispatch");
  expect(command.dispatch_allowed, "safe command should be dispatchable to a sink");
  expect(!command.hardware_execution, "controller evaluation does not execute hardware");

  RecordingMockCommandSink sink;
  sink.dispatch(command);
  expect(sink.commands().size() == 1, "mock sink should record one command");

  auto moving = measured(1050000000);
  moving.base_settled = false;
  moving.base_linear_mps = 0.02;
  command = controller.evaluate(arm_intent(2), moving, arm_context(), 1050000000);
  expect(command.decision == SafetyDecision::kReject,
    "arm motion must reject while base is moving");

  auto bad_localization = arm_context();
  bad_localization.localization_decision = MotionDecision::kHold;
  command = controller.evaluate(arm_intent(2), measured(1050000000), bad_localization, 1050000000);
  expect(command.decision == SafetyDecision::kReject,
    "localization hold must block arm motion");

  command = controller.evaluate(arm_intent(2, 2.0, 0.0), measured(1050000000), arm_context(), 1050000000);
  expect(command.decision == SafetyDecision::kReject,
    "hard joint limit violation must reject");

  auto base_state = measured(1050000000);
  auto base_context = dapier_safety_core::SafetyContext{MotionDecision::kProceed, false, true};
  command = controller.evaluate(base_intent(2), base_state, base_context, 1050000000);
  expect(command.decision == SafetyDecision::kDispatch,
    "base transport should dispatch with carry and grasp interlocks");

  auto no_grasp = measured(1100000000);
  no_grasp.grasp_verified = false;
  command = controller.evaluate(base_intent(3), no_grasp, base_context, 1100000000);
  expect(command.decision == SafetyDecision::kReject,
    "base transport must reject without a verified grasp");

  auto stale = measured(1000000000);
  command = controller.evaluate(hold_intent(3), stale, arm_context(), 1200000000);
  expect(command.decision == SafetyDecision::kSafeStop,
    "stale measured state must latch a safe stop");
  expect(controller.safe_stop_latched(), "safe stop must remain latched");

  controller.set_estop_healthy(true);
  bool enabled_clear_threw = false;
  controller.set_operator_enabled(true);
  try {
    controller.acknowledge_safe_stop();
  } catch (const std::logic_error &) {
    enabled_clear_threw = true;
  }
  expect(enabled_clear_threw, "safe stop cannot clear while operator enable is true");
  controller.set_operator_enabled(false);
  controller.acknowledge_safe_stop();
  expect(!controller.safe_stop_latched(), "explicit disabled reset should clear safe stop");

  controller.set_operator_enabled(true);
  state = measured(2000000000);
  command = controller.evaluate(arm_intent(3), state, arm_context(), 2000000000);
  expect(command.decision == SafetyDecision::kDispatch, "controller should rearm after explicit reset");
  auto watchdog = controller.tick(measured(2300000001), 2300000001);
  expect(watchdog.has_value(), "command watchdog should emit a safe stop");
  expect(watchdog->decision == SafetyDecision::kSafeStop,
    "watchdog output must be a safe stop");

  auto explicit_hold = controller.evaluate(
    hold_intent(4), measured(2300000001), arm_context(), 2300000001);
  expect(explicit_hold.decision == SafetyDecision::kSafeStop,
    "explicit hold cannot silently clear a latched safe stop");

  dapier_safety_core::SafeCommand forged;
  forged.dispatch_allowed = true;
  forged.hardware_execution = true;
  bool forged_threw = false;
  try {
    sink.dispatch(forged);
  } catch (const std::invalid_argument &) {
    forged_threw = true;
  }
  expect(forged_threw, "mock sink must reject hardware-execution claims");

  if (failures != 0) {
    std::cerr << failures << " safety controller checks failed\n";
    return 1;
  }
  std::cout << "C++ safety controller and mock sink checks passed\n";
  return 0;
}
