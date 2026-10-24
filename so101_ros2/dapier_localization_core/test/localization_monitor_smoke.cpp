// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/generated/localization_runtime_contract.hpp"
#include "dapier_localization_core/localization_monitor.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
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

dapier_localization_core::LocalizationEstimate good_estimate(
  std::uint64_t sequence,
  std::int64_t source_timestamp_ns)
{
  using dapier_localization_core::LocalizationEstimate;
  using dapier_localization_core::TrackingState;
  LocalizationEstimate value;
  value.schema_version = std::string(
    dapier_localization_core::generated::kLocalizationSchemaVersion);
  value.sequence = sequence;
  value.source_timestamp_ns = source_timestamp_ns;
  value.ttl_ns = 200000000;
  value.parent_frame = "map";
  value.child_frame = "base_link";
  value.map_id = "dapier-lab-map-v1";
  value.session_id = "replay-session-001";
  value.reset_generation = 0;
  value.tracking_state = TrackingState::kTracking;
  value.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
  for (std::size_t index : {std::size_t{0}, std::size_t{7}, std::size_t{14}}) {
    value.covariance[index] = 0.0025;
  }
  for (std::size_t index : {std::size_t{21}, std::size_t{28}, std::size_t{35}}) {
    value.covariance[index] = 0.01;
  }
  value.tracked_features = 80;
  value.mean_reprojection_error_px = 0.8;
  value.confidence = 0.9;
  return value;
}

}  // namespace

int main()
{
  using dapier_localization_core::LocalizationIdentity;
  using dapier_localization_core::LocalizationMonitor;
  using dapier_localization_core::MotionDecision;
  using dapier_localization_core::TrackingState;

  LocalizationMonitor monitor;
  auto first = good_estimate(1, 0);
  auto result = monitor.observe(first, 1000000000);
  expect(result.decision == MotionDecision::kHold, "first identity must hold");
  expect(result.replan_required, "first identity must require replan");
  expect(result.discard_pending_actions, "first identity discards stale actions");
  expect(result.identity_changed, "first identity is a new identity boundary");
  expect(!result.hardware_execution, "monitor is hardware-free");

  monitor.acknowledge_replan(
    1,
    LocalizationIdentity{"map", "base_link", "dapier-lab-map-v1", "replay-session-001", 0});
  auto second = good_estimate(2, 50000000);
  result = monitor.observe(second, 1050000000);
  expect(result.decision == MotionDecision::kProceed, "healthy tracking should proceed after replan");
  expect(!result.discard_pending_actions, "healthy tracking keeps the active plan");

  auto duplicate = good_estimate(2, 60000000);
  result = monitor.observe(duplicate, 1060000000);
  expect(result.decision == MotionDecision::kReject, "duplicate sequence must reject");

  auto lost = good_estimate(3, 100000000);
  lost.tracking_state = TrackingState::kLost;
  result = monitor.observe(lost, 1100000000);
  expect(result.decision == MotionDecision::kHold, "lost tracking must hold");
  expect(result.replan_required, "lost tracking invalidates the active plan");

  auto recovered = good_estimate(4, 150000000);
  result = monitor.observe(recovered, 1150000000);
  expect(result.decision == MotionDecision::kHold, "recovery cannot resume without a replan ack");

  auto restarted = good_estimate(5, 200000000);
  restarted.map_id = "dapier-lab-map-v2";
  restarted.session_id = "replay-session-002";
  restarted.reset_generation = 1;
  restarted.tracking_state = TrackingState::kRelocalized;
  result = monitor.observe(restarted, 1200000000);
  expect(result.decision == MotionDecision::kHold, "map restart must hold");
  expect(result.identity_changed, "map restart changes identity");

  auto illegal_identity = good_estimate(6, 250000000);
  illegal_identity.map_id = "unexpected-map";
  illegal_identity.session_id = "unexpected-session";
  illegal_identity.reset_generation = 1;
  result = monitor.observe(illegal_identity, 1250000000);
  expect(result.decision == MotionDecision::kReject,
    "identity change without generation increment must reject");

  LocalizationMonitor watchdog_monitor;
  first = good_estimate(1, 0);
  result = watchdog_monitor.observe(first, 2000000000);
  watchdog_monitor.acknowledge_replan(
    1,
    LocalizationIdentity{"map", "base_link", "dapier-lab-map-v1", "replay-session-001", 0});
  second = good_estimate(2, 100000000);
  result = watchdog_monitor.observe(second, 2100000000);
  expect(result.decision == MotionDecision::kHold, "interarrival watchdog must hold");

  bool mismatch_threw = false;
  try {
    watchdog_monitor.acknowledge_replan(
      999,
      LocalizationIdentity{"map", "base_link", "dapier-lab-map-v1", "replay-session-001", 0});
  } catch (const std::invalid_argument &) {
    mismatch_threw = true;
  }
  expect(mismatch_threw, "mismatched replan acknowledgement must throw");

  if (failures != 0) {
    std::cerr << failures << " localization monitor checks failed\n";
    return 1;
  }
  std::cout << "localization recovery monitor checks passed\n";
  return 0;
}
