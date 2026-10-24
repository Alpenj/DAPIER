// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/localization_runtime.hpp"
#include "dapier_localization_core/orbslam3_bridge.hpp"

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

dapier_localization_core::OrbSlam3FrameStatus status(int raw, std::uint64_t sequence)
{
  dapier_localization_core::OrbSlam3FrameStatus value;
  value.raw_tracking_state = raw;
  value.sequence = sequence;
  value.source_timestamp_ns = static_cast<std::int64_t>(sequence) * 50000000;
  value.map_id = "map-v1";
  value.session_id = "session-v1";
  value.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
  value.tracked_features = 80;
  value.mean_reprojection_error_px = 0.8;
  value.confidence = 0.9;
  return value;
}
}  // namespace

int main()
{
  using dapier_localization_core::OrbSlam3StateAdapter;
  using dapier_localization_core::OrbSlam3TrackingState;
  using dapier_localization_core::TrackingState;

  OrbSlam3StateAdapter adapter;
  auto estimate = adapter.convert(status(
      static_cast<int>(OrbSlam3TrackingState::kNotInitialized), 1));
  expect(estimate.tracking_state == TrackingState::kInitializing,
    "ORB-SLAM3 not initialized must map to initializing");

  estimate = adapter.convert(status(static_cast<int>(OrbSlam3TrackingState::kOk), 2));
  expect(estimate.tracking_state == TrackingState::kTracking,
    "ORB-SLAM3 OK must map to tracking");

  estimate = adapter.convert(status(
      static_cast<int>(OrbSlam3TrackingState::kRecentlyLost), 3));
  expect(estimate.tracking_state == TrackingState::kRecentlyLost,
    "recently lost must remain explicit");

  estimate = adapter.convert(status(static_cast<int>(OrbSlam3TrackingState::kOk), 4));
  expect(estimate.tracking_state == TrackingState::kRelocalized,
    "recovery from loss must force downstream replanning");

  auto changed = status(static_cast<int>(OrbSlam3TrackingState::kOkKlt), 5);
  changed.map_changed = true;
  estimate = adapter.convert(changed);
  expect(estimate.tracking_state == TrackingState::kRelocalized,
    "map correction must invalidate stale targets");
  expect(!estimate.simulator_truth_used, "adapter must never mark simulator truth");
  expect(!estimate.control_authorized, "adapter cannot authorize hardware");

  bool unknown_threw = false;
  try {
    static_cast<void>(adapter.convert(status(99, 6)));
  } catch (const std::invalid_argument &) {
    unknown_threw = true;
  }
  expect(unknown_threw, "unknown ORB-SLAM3 state must fail closed");

  if (failures != 0) {
    std::cerr << failures << " ORB-SLAM3 bridge checks failed\n";
    return 1;
  }
  std::cout << "ORB-SLAM3 state bridge checks passed\n";
  return 0;
}
