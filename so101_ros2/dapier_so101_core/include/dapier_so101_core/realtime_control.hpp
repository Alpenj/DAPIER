// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dapier_so101_core
{

enum class ResearchIntentKind
{
  kHold,
  kArmJointPosition,
  kBaseTwist,
};

enum class ResearchIntentRejection
{
  kNone,
  kSchemaMismatch,
  kSequenceRejected,
  kInvalidTimestamp,
  kInvalidTtl,
  kInvalidSource,
  kInvalidShape,
  kDuplicateJoint,
  kNonFiniteValue,
  kInvalidVelocity,
  kMixedControlDomains,
  kExpiryOverflow,
};

struct ResearchControlIntent
{
  std::string schema_version;
  std::uint64_t sequence{0};
  ResearchIntentKind kind{ResearchIntentKind::kHold};
  std::string source;
  std::int64_t source_monotonic_ns{0};
  std::int64_t ttl_ns{0};
  std::vector<std::string> joint_names;
  std::vector<double> joint_position_rad;
  std::vector<double> joint_max_velocity_rad_s;
  double base_linear_x_mps{0.0};
  double base_angular_z_rad_s{0.0};
};

struct ResearchIntentValidation
{
  bool accepted{false};
  ResearchIntentRejection rejection{ResearchIntentRejection::kNone};
  std::string reason;
  std::int64_t accepted_until_monotonic_ns{0};
};

[[nodiscard]] ResearchIntentValidation validate_research_control_intent(
  const ResearchControlIntent & intent,
  std::int64_t receiver_monotonic_ns,
  std::optional<std::uint64_t> last_accepted_sequence = std::nullopt);

}  // namespace dapier_so101_core
