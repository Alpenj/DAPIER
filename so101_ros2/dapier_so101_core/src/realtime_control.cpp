// SPDX-License-Identifier: Apache-2.0

#include "dapier_so101_core/realtime_control.hpp"

#include "dapier_so101_core/generated/research_realtime_contract.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dapier_so101_core
{
namespace
{

static_assert(generated::kAllowedIntentKinds.size() == 3);
static_assert(generated::kAllowedIntentKinds[0] == "hold");
static_assert(generated::kAllowedIntentKinds[1] == "arm_joint_position");
static_assert(generated::kAllowedIntentKinds[2] == "base_twist");

ResearchIntentValidation reject(
  ResearchIntentRejection rejection,
  std::string reason)
{
  return ResearchIntentValidation{false, rejection, std::move(reason), 0};
}

bool is_blank(const std::string & value)
{
  return value.empty() || std::all_of(
    value.begin(), value.end(), [](unsigned char character) {return std::isspace(character) != 0;});
}

bool is_zero_base(const ResearchControlIntent & intent)
{
  return intent.base_linear_x_mps == 0.0 && intent.base_angular_z_rad_s == 0.0;
}

bool joint_fields_are_empty(const ResearchControlIntent & intent)
{
  return intent.joint_names.empty() && intent.joint_position_rad.empty() &&
         intent.joint_max_velocity_rad_s.empty();
}

bool contains_non_finite(const std::vector<double> & values)
{
  return std::any_of(
    values.begin(), values.end(), [](double value) {return !std::isfinite(value);});
}

}  // namespace

ResearchIntentValidation validate_research_control_intent(
  const ResearchControlIntent & intent,
  std::int64_t receiver_monotonic_ns,
  std::optional<std::uint64_t> last_accepted_sequence)
{
  if (intent.schema_version != generated::kResearchRealtimeSchemaVersion) {
    return reject(
      ResearchIntentRejection::kSchemaMismatch,
      "schema version does not match the generated control boundary");
  }
  if (last_accepted_sequence && intent.sequence <= *last_accepted_sequence) {
    return reject(
      ResearchIntentRejection::kSequenceRejected,
      "sequence must be greater than the last accepted sequence");
  }
  if (receiver_monotonic_ns < 0 || intent.source_monotonic_ns < 0) {
    return reject(
      ResearchIntentRejection::kInvalidTimestamp,
      "monotonic timestamps must be non-negative");
  }
  if (intent.ttl_ns <= 0 || intent.ttl_ns > generated::kMaxIntentTtlNs) {
    return reject(
      ResearchIntentRejection::kInvalidTtl,
      "ttl is outside the shared control boundary");
  }
  if (is_blank(intent.source)) {
    return reject(
      ResearchIntentRejection::kInvalidSource,
      "source must be a non-empty identifier");
  }
  if (!std::isfinite(intent.base_linear_x_mps) ||
    !std::isfinite(intent.base_angular_z_rad_s) ||
    contains_non_finite(intent.joint_position_rad) ||
    contains_non_finite(intent.joint_max_velocity_rad_s))
  {
    return reject(
      ResearchIntentRejection::kNonFiniteValue,
      "all numeric command fields must be finite");
  }

  switch (intent.kind) {
    case ResearchIntentKind::kHold:
      if (!joint_fields_are_empty(intent) || !is_zero_base(intent)) {
        return reject(
          ResearchIntentRejection::kMixedControlDomains,
          "hold intent must contain no joint targets and zero base motion");
      }
      break;

    case ResearchIntentKind::kArmJointPosition: {
        const auto joint_count = intent.joint_names.size();
        if (joint_count == 0 || joint_count > generated::kMaxArmJointCount ||
          intent.joint_position_rad.size() != joint_count ||
          intent.joint_max_velocity_rad_s.size() != joint_count)
        {
          return reject(
            ResearchIntentRejection::kInvalidShape,
            "arm joint arrays must be non-empty, bounded, and equal in length");
        }
        if (!is_zero_base(intent)) {
          return reject(
            ResearchIntentRejection::kMixedControlDomains,
            "arm intent must not request base motion");
        }
        std::unordered_set<std::string> unique_names;
        unique_names.reserve(joint_count);
        for (const auto & name : intent.joint_names) {
          if (is_blank(name)) {
            return reject(
              ResearchIntentRejection::kInvalidShape,
              "joint names must be non-empty");
          }
          if (!unique_names.insert(name).second) {
            return reject(
              ResearchIntentRejection::kDuplicateJoint,
              "joint names must be unique");
          }
        }
        if (std::any_of(
            intent.joint_max_velocity_rad_s.begin(),
            intent.joint_max_velocity_rad_s.end(),
            [](double value) {return value <= 0.0;}))
        {
          return reject(
            ResearchIntentRejection::kInvalidVelocity,
            "joint max velocities must be positive");
        }
        break;
      }

    case ResearchIntentKind::kBaseTwist:
      if (!joint_fields_are_empty(intent)) {
        return reject(
          ResearchIntentRejection::kMixedControlDomains,
          "base intent must not contain joint targets");
      }
      break;
  }

  if (intent.ttl_ns > std::numeric_limits<std::int64_t>::max() - receiver_monotonic_ns) {
    return reject(
      ResearchIntentRejection::kExpiryOverflow,
      "receiver-local expiry would overflow int64");
  }

  return ResearchIntentValidation{
    true,
    ResearchIntentRejection::kNone,
    "accepted by the research/realtime boundary; robot-specific gates still apply",
    receiver_monotonic_ns + intent.ttl_ns,
  };
}

}  // namespace dapier_so101_core
