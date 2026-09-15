// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/localization_runtime.hpp"

#include "dapier_localization_core/generated/localization_runtime_contract.hpp"

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

dapier_localization_core::LocalizationEstimate valid_estimate()
{
  using dapier_localization_core::LocalizationEstimate;
  using dapier_localization_core::TrackingState;
  using dapier_localization_core::generated::kLocalizationSchemaVersion;

  LocalizationEstimate estimate;
  estimate.schema_version = std::string(kLocalizationSchemaVersion);
  estimate.sequence = 7;
  estimate.source_timestamp_ns = 1000;
  estimate.ttl_ns = 100000000;
  estimate.parent_frame = "map";
  estimate.child_frame = "base_link";
  estimate.map_id = "atlas-map-0";
  estimate.session_id = "session-0";
  estimate.reset_generation = 3;
  estimate.tracking_state = TrackingState::kTracking;
  estimate.position_m = {1.0, -0.2, 0.0};
  estimate.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
  estimate.covariance.fill(0.0);
  estimate.covariance[0] = 0.001;
  estimate.covariance[7] = 0.001;
  estimate.covariance[14] = 0.001;
  estimate.covariance[21] = 0.01;
  estimate.covariance[28] = 0.01;
  estimate.covariance[35] = 0.01;
  estimate.tracked_features = 80;
  estimate.mean_reprojection_error_px = 0.8;
  estimate.confidence = 0.9;
  return estimate;
}

}  // namespace

int main()
{
  using dapier_localization_core::LocalizationRejection;
  using dapier_localization_core::MotionDecision;
  using dapier_localization_core::TrackingState;
  using dapier_localization_core::assess_localization_for_motion;
  using dapier_localization_core::validate_localization_estimate;

  auto estimate = valid_estimate();
  auto validation = validate_localization_estimate(estimate, 5000, 6, 3);
  expect(validation.accepted, "valid estimate should be accepted");
  expect(
    validation.accepted_until_monotonic_ns == 100005000,
    "expiry must use receiver-local monotonic time");
  auto assessment = assess_localization_for_motion(estimate, validation);
  expect(
    assessment.decision == MotionDecision::kProceed,
    "high-quality tracking should permit motion");

  validation = validate_localization_estimate(estimate, 5000, 7, 3);
  expect(
    validation.rejection == LocalizationRejection::kSequenceRejected,
    "replayed sequence should fail");

  estimate = valid_estimate();
  validation = validate_localization_estimate(estimate, 5000, 6, 4);
  expect(
    validation.rejection == LocalizationRejection::kGenerationMismatch,
    "stale plan generation should fail");

  estimate = valid_estimate();
  estimate.ttl_ns =
    dapier_localization_core::generated::kMaxEstimateTtlNs + 1;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kInvalidTtl,
    "oversized TTL should fail");

  estimate = valid_estimate();
  estimate.child_frame = estimate.parent_frame;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kInvalidFramePair,
    "self transform should fail");

  estimate = valid_estimate();
  estimate.orientation_xyzw = {0.0, 0.0, 0.0, 2.0};
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kInvalidQuaternion,
    "non-unit quaternion should fail");

  estimate = valid_estimate();
  estimate.covariance[1] = 0.2;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kInvalidCovariance,
    "asymmetric covariance should fail");

  estimate = valid_estimate();
  estimate.covariance[0] = -0.1;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kInvalidCovariance,
    "negative variance should fail");

  estimate = valid_estimate();
  estimate.simulator_truth_used = true;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kPrivilegedRuntimeState,
    "simulator truth should fail at runtime boundary");

  estimate = valid_estimate();
  estimate.control_authorized = true;
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kHardwareAuthorityViolation,
    "localization cannot authorize control");

  estimate = valid_estimate();
  estimate.tracking_state = static_cast<TrackingState>(99);
  validation = validate_localization_estimate(estimate, 5000);
  expect(
    validation.rejection == LocalizationRejection::kUnknownTrackingState,
    "unknown state should fail");

  estimate = valid_estimate();
  estimate.tracking_state = TrackingState::kRecentlyLost;
  validation = validate_localization_estimate(estimate, 5000);
  assessment = assess_localization_for_motion(estimate, validation);
  expect(
    assessment.decision == MotionDecision::kHold,
    "recently lost should hold motion");

  estimate = valid_estimate();
  estimate.tracking_state = TrackingState::kRelocalized;
  validation = validate_localization_estimate(estimate, 5000);
  assessment = assess_localization_for_motion(estimate, validation);
  expect(
    assessment.decision == MotionDecision::kHold,
    "relocalized state should force replan hold");

  estimate = valid_estimate();
  estimate.confidence = 0.1;
  validation = validate_localization_estimate(estimate, 5000);
  assessment = assess_localization_for_motion(estimate, validation);
  expect(
    assessment.decision == MotionDecision::kHold,
    "low confidence should hold motion");

  estimate = valid_estimate();
  estimate.ttl_ns = 100;
  validation = validate_localization_estimate(
    estimate,
    std::numeric_limits<std::int64_t>::max() - 50);
  expect(
    validation.rejection == LocalizationRejection::kExpiryOverflow,
    "expiry overflow should fail");

  expect(
    !dapier_localization_core::generated::kLocalizationMayAuthorizeHardware,
    "generated contract must deny hardware authority");
  expect(
    !dapier_localization_core::generated::kSimulatorTruthAllowedForRuntime,
    "generated contract must deny simulator truth");

  if (failures != 0) {
    std::cerr << failures << " localization contract checks failed\n";
    return 1;
  }
  std::cout << "localization runtime C++ contract checks passed\n";
  return 0;
}
