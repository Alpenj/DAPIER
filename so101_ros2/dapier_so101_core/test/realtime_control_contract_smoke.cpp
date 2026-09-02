// SPDX-License-Identifier: Apache-2.0

#include "dapier_so101_core/realtime_control.hpp"

#include "dapier_so101_core/generated/research_realtime_contract.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

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

dapier_so101_core::ResearchControlIntent valid_arm_intent()
{
  using dapier_so101_core::ResearchControlIntent;
  using dapier_so101_core::ResearchIntentKind;
  using dapier_so101_core::generated::kResearchRealtimeSchemaVersion;
  return ResearchControlIntent{
    std::string(kResearchRealtimeSchemaVersion),
    2,
    ResearchIntentKind::kArmJointPosition,
    "act_policy_eval",
    100,
    100000000,
    {"left_shoulder_pan", "left_shoulder_lift"},
    {0.1, -0.2},
    {0.4, 0.4},
    0.0,
    0.0,
  };
}

}  // namespace

int main()
{
  using dapier_so101_core::ResearchIntentKind;
  using dapier_so101_core::ResearchIntentRejection;
  using dapier_so101_core::validate_research_control_intent;

  auto arm = valid_arm_intent();
  auto result = validate_research_control_intent(arm, 500, 1);
  expect(result.accepted, "valid arm intent should be accepted");
  expect(result.accepted_until_monotonic_ns == 100000500, "expiry must use receiver-local time");

  result = validate_research_control_intent(arm, 500, 2);
  expect(!result.accepted, "replayed sequence should be rejected");
  expect(result.rejection == ResearchIntentRejection::kSequenceRejected, "replay rejection code");

  arm.ttl_ns = dapier_so101_core::generated::kMaxIntentTtlNs + 1;
  result = validate_research_control_intent(arm, 500);
  expect(result.rejection == ResearchIntentRejection::kInvalidTtl, "oversized TTL should fail");

  arm = valid_arm_intent();
  arm.joint_names[1] = arm.joint_names[0];
  result = validate_research_control_intent(arm, 500);
  expect(result.rejection == ResearchIntentRejection::kDuplicateJoint, "duplicate joints should fail");

  arm = valid_arm_intent();
  arm.base_linear_x_mps = 0.1;
  result = validate_research_control_intent(arm, 500);
  expect(result.rejection == ResearchIntentRejection::kMixedControlDomains, "arm/base mixing should fail");

  auto hold = valid_arm_intent();
  hold.kind = ResearchIntentKind::kHold;
  result = validate_research_control_intent(hold, 500);
  expect(result.rejection == ResearchIntentRejection::kMixedControlDomains, "hold with targets should fail");

  arm = valid_arm_intent();
  arm.source = "   ";
  result = validate_research_control_intent(arm, 500);
  expect(result.rejection == ResearchIntentRejection::kInvalidSource, "blank source should fail");

  arm = valid_arm_intent();
  arm.joint_names[0] = "   ";
  result = validate_research_control_intent(arm, 500);
  expect(result.rejection == ResearchIntentRejection::kInvalidShape, "blank joint name should fail");

  arm = valid_arm_intent();
  arm.ttl_ns = 100;
  result = validate_research_control_intent(
    arm,
    std::numeric_limits<std::int64_t>::max() - 50);
  expect(result.rejection == ResearchIntentRejection::kExpiryOverflow, "expiry overflow should fail");

  expect(!dapier_so101_core::generated::kResearchMayAuthorizeHardware,
    "generated contract must deny Python hardware authorization");

  if (failures != 0) {
    std::cerr << failures << " contract checks failed\n";
    return 1;
  }
  std::cout << "research/realtime C++ contract checks passed\n";
  return 0;
}
