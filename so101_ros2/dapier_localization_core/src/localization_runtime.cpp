// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/localization_runtime.hpp"

#include "dapier_localization_core/generated/localization_runtime_contract.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dapier_localization_core
{
namespace
{

static_assert(generated::kAllowedTrackingStates.size() == 5);
static_assert(generated::kAllowedTrackingStates[0] == "initializing");
static_assert(generated::kAllowedTrackingStates[1] == "tracking");
static_assert(generated::kAllowedTrackingStates[2] == "recently_lost");
static_assert(generated::kAllowedTrackingStates[3] == "lost");
static_assert(generated::kAllowedTrackingStates[4] == "relocalized");
static_assert(!generated::kLocalizationMayAuthorizeHardware);
static_assert(!generated::kSimulatorTruthAllowedForRuntime);

LocalizationValidation reject(
  LocalizationRejection rejection,
  std::string reason)
{
  return LocalizationValidation{false, rejection, std::move(reason), 0};
}

MotionAssessment motion(MotionDecision decision, std::string reason)
{
  return MotionAssessment{decision, std::move(reason)};
}

bool is_blank(const std::string & value)
{
  return value.empty() || std::all_of(
    value.begin(), value.end(), [](unsigned char character) {
      return std::isspace(character) != 0;
    });
}

bool invalid_identifier(const std::string & value)
{
  return is_blank(value) || value.size() > generated::kMaxIdentifierLength;
}

template<std::size_t Size>
bool contains_non_finite(const std::array<double, Size> & values)
{
  return std::any_of(values.begin(), values.end(), [](double value) {
    return !std::isfinite(value);
  });
}

bool covariance_is_symmetric(const std::array<double, 36> & covariance)
{
  constexpr double tolerance = 1e-8;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = row + 1; column < 6; ++column) {
      const auto forward = covariance[row * 6 + column];
      const auto reverse = covariance[column * 6 + row];
      if (std::abs(forward - reverse) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

bool gate_config_is_valid(const MotionGateConfig & config)
{
  return std::isfinite(config.minimum_confidence) &&
         config.minimum_confidence >= 0.0 &&
         config.minimum_confidence <= 1.0 &&
         config.minimum_tracked_features > 0 &&
         std::isfinite(config.maximum_mean_reprojection_error_px) &&
         config.maximum_mean_reprojection_error_px > 0.0 &&
         std::isfinite(config.maximum_position_variance_m2) &&
         config.maximum_position_variance_m2 > 0.0 &&
         std::isfinite(config.maximum_rotation_variance_rad2) &&
         config.maximum_rotation_variance_rad2 > 0.0;
}

}  // namespace

LocalizationValidation validate_localization_estimate(
  const LocalizationEstimate & estimate,
  std::int64_t receiver_monotonic_ns,
  std::optional<std::uint64_t> last_accepted_sequence,
  std::optional<std::uint64_t> expected_reset_generation)
{
  if (estimate.schema_version != generated::kLocalizationSchemaVersion) {
    return reject(
      LocalizationRejection::kSchemaMismatch,
      "schema version does not match the generated localization boundary");
  }
  if (last_accepted_sequence && estimate.sequence <= *last_accepted_sequence) {
    return reject(
      LocalizationRejection::kSequenceRejected,
      "sequence must be greater than the last accepted sequence");
  }
  if (expected_reset_generation &&
    estimate.reset_generation != *expected_reset_generation)
  {
    return reject(
      LocalizationRejection::kGenerationMismatch,
      "localization reset generation differs from the active plan generation");
  }
  if (receiver_monotonic_ns < 0 || estimate.source_timestamp_ns < 0) {
    return reject(
      LocalizationRejection::kInvalidTimestamp,
      "source and receiver timestamps must be non-negative");
  }
  if (estimate.ttl_ns <= 0 ||
    estimate.ttl_ns > generated::kMaxEstimateTtlNs)
  {
    return reject(
      LocalizationRejection::kInvalidTtl,
      "ttl is outside the localization contract maximum");
  }
  if (invalid_identifier(estimate.parent_frame) ||
    invalid_identifier(estimate.child_frame) ||
    invalid_identifier(estimate.map_id) ||
    invalid_identifier(estimate.session_id))
  {
    return reject(
      LocalizationRejection::kInvalidIdentifier,
      "frame and map/session identifiers must be non-empty and bounded");
  }
  if (estimate.parent_frame == estimate.child_frame) {
    return reject(
      LocalizationRejection::kInvalidFramePair,
      "parent and child frames must differ");
  }
  switch (estimate.tracking_state) {
    case TrackingState::kInitializing:
    case TrackingState::kTracking:
    case TrackingState::kRecentlyLost:
    case TrackingState::kLost:
    case TrackingState::kRelocalized:
      break;
    default:
      return reject(
        LocalizationRejection::kUnknownTrackingState,
        "tracking state is not part of the generated localization boundary");
  }
  if (contains_non_finite(estimate.position_m) ||
    contains_non_finite(estimate.orientation_xyzw) ||
    contains_non_finite(estimate.covariance) ||
    !std::isfinite(estimate.mean_reprojection_error_px) ||
    !std::isfinite(estimate.confidence))
  {
    return reject(
      LocalizationRejection::kNonFiniteValue,
      "pose, covariance, and quality fields must be finite");
  }

  double quaternion_norm_squared = 0.0;
  for (double value : estimate.orientation_xyzw) {
    quaternion_norm_squared += value * value;
  }
  if (!std::isfinite(quaternion_norm_squared) ||
    std::abs(quaternion_norm_squared - 1.0) > 1e-3)
  {
    return reject(
      LocalizationRejection::kInvalidQuaternion,
      "orientation quaternion must be normalized");
  }

  constexpr std::array<std::size_t, 6> diagonal_indices{0, 7, 14, 21, 28, 35};
  for (std::size_t index : diagonal_indices) {
    if (estimate.covariance[index] < 0.0) {
      return reject(
        LocalizationRejection::kInvalidCovariance,
        "covariance diagonal entries must be non-negative");
    }
  }
  if (!covariance_is_symmetric(estimate.covariance)) {
    return reject(
      LocalizationRejection::kInvalidCovariance,
      "covariance must be symmetric");
  }
  if (estimate.mean_reprojection_error_px < 0.0 ||
    estimate.confidence < 0.0 || estimate.confidence > 1.0)
  {
    return reject(
      LocalizationRejection::kInvalidQuality,
      "quality metrics are outside their valid ranges");
  }
  if (estimate.simulator_truth_used) {
    return reject(
      LocalizationRejection::kPrivilegedRuntimeState,
      "runtime localization must not depend on simulator truth");
  }
  if (estimate.control_authorized) {
    return reject(
      LocalizationRejection::kHardwareAuthorityViolation,
      "localization estimates cannot authorize hardware control");
  }
  if (estimate.ttl_ns >
    std::numeric_limits<std::int64_t>::max() - receiver_monotonic_ns)
  {
    return reject(
      LocalizationRejection::kExpiryOverflow,
      "receiver-local expiry would overflow int64");
  }

  return LocalizationValidation{
    true,
    LocalizationRejection::kNone,
    "accepted by the localization boundary; motion-quality gates still apply",
    receiver_monotonic_ns + estimate.ttl_ns,
  };
}

MotionAssessment assess_localization_for_motion(
  const LocalizationEstimate & estimate,
  const LocalizationValidation & validation,
  const MotionGateConfig & config)
{
  if (!validation.accepted) {
    return motion(
      MotionDecision::kReject,
      "invalid localization estimate cannot be used for motion");
  }
  if (!gate_config_is_valid(config)) {
    return motion(
      MotionDecision::kReject,
      "motion gate configuration is invalid");
  }

  switch (estimate.tracking_state) {
    case TrackingState::kTracking:
      break;
    case TrackingState::kRelocalized:
      return motion(
        MotionDecision::kHold,
        "relocalized pose requires target invalidation and replanning");
    case TrackingState::kInitializing:
      return motion(MotionDecision::kHold, "localization is initializing");
    case TrackingState::kRecentlyLost:
      return motion(MotionDecision::kHold, "localization is recently lost");
    case TrackingState::kLost:
      return motion(MotionDecision::kHold, "localization is lost");
    default:
      return motion(MotionDecision::kReject, "tracking state is invalid");
  }

  if (estimate.confidence < config.minimum_confidence) {
    return motion(MotionDecision::kHold, "localization confidence is too low");
  }
  if (estimate.tracked_features < config.minimum_tracked_features) {
    return motion(MotionDecision::kHold, "tracked feature count is too low");
  }
  if (estimate.mean_reprojection_error_px >
    config.maximum_mean_reprojection_error_px)
  {
    return motion(MotionDecision::kHold, "reprojection error is too high");
  }

  for (std::size_t index : {std::size_t{0}, std::size_t{7}, std::size_t{14}}) {
    if (estimate.covariance[index] > config.maximum_position_variance_m2) {
      return motion(MotionDecision::kHold, "position covariance is too high");
    }
  }
  for (std::size_t index : {std::size_t{21}, std::size_t{28}, std::size_t{35}}) {
    if (estimate.covariance[index] > config.maximum_rotation_variance_rad2) {
      return motion(MotionDecision::kHold, "rotation covariance is too high");
    }
  }

  return motion(MotionDecision::kProceed, "localization is suitable for motion");
}

}  // namespace dapier_localization_core
