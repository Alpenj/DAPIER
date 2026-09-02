// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace dapier_localization_core
{

enum class TrackingState
{
  kInitializing,
  kTracking,
  kRecentlyLost,
  kLost,
  kRelocalized,
};

enum class LocalizationRejection
{
  kNone,
  kSchemaMismatch,
  kSequenceRejected,
  kGenerationMismatch,
  kInvalidTimestamp,
  kInvalidTtl,
  kInvalidIdentifier,
  kInvalidFramePair,
  kUnknownTrackingState,
  kNonFiniteValue,
  kInvalidQuaternion,
  kInvalidCovariance,
  kInvalidQuality,
  kPrivilegedRuntimeState,
  kHardwareAuthorityViolation,
  kExpiryOverflow,
};

enum class MotionDecision
{
  kProceed,
  kHold,
  kReject,
};

struct LocalizationEstimate
{
  std::string schema_version;
  std::uint64_t sequence{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t ttl_ns{0};
  std::string parent_frame;
  std::string child_frame;
  std::string map_id;
  std::string session_id;
  std::uint64_t reset_generation{0};
  TrackingState tracking_state{TrackingState::kInitializing};
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 36> covariance{};
  std::uint32_t tracked_features{0};
  double mean_reprojection_error_px{0.0};
  double confidence{0.0};
  bool simulator_truth_used{false};
  bool control_authorized{false};
};

struct LocalizationValidation
{
  bool accepted{false};
  LocalizationRejection rejection{LocalizationRejection::kNone};
  std::string reason;
  std::int64_t accepted_until_monotonic_ns{0};
};

struct MotionGateConfig
{
  double minimum_confidence{0.5};
  std::uint32_t minimum_tracked_features{20};
  double maximum_mean_reprojection_error_px{3.0};
  double maximum_position_variance_m2{0.01};
  double maximum_rotation_variance_rad2{0.04};
};

struct MotionAssessment
{
  MotionDecision decision{MotionDecision::kReject};
  std::string reason;
};

[[nodiscard]] LocalizationValidation validate_localization_estimate(
  const LocalizationEstimate & estimate,
  std::int64_t receiver_monotonic_ns,
  std::optional<std::uint64_t> last_accepted_sequence = std::nullopt,
  std::optional<std::uint64_t> expected_reset_generation = std::nullopt);

[[nodiscard]] MotionAssessment assess_localization_for_motion(
  const LocalizationEstimate & estimate,
  const LocalizationValidation & validation,
  const MotionGateConfig & config = MotionGateConfig{});

}  // namespace dapier_localization_core
