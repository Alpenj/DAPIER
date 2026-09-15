// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/orbslam3_bridge.hpp"

#include "dapier_localization_core/generated/localization_runtime_contract.hpp"

#include <stdexcept>
#include <string>

namespace dapier_localization_core
{
namespace
{

bool was_lost(int state) noexcept
{
  return state == static_cast<int>(OrbSlam3TrackingState::kRecentlyLost) ||
         state == static_cast<int>(OrbSlam3TrackingState::kLost);
}

TrackingState translate_state(int raw_state)
{
  switch (static_cast<OrbSlam3TrackingState>(raw_state)) {
    case OrbSlam3TrackingState::kSystemNotReady:
    case OrbSlam3TrackingState::kNoImagesYet:
    case OrbSlam3TrackingState::kNotInitialized:
      return TrackingState::kInitializing;
    case OrbSlam3TrackingState::kOk:
    case OrbSlam3TrackingState::kOkKlt:
      return TrackingState::kTracking;
    case OrbSlam3TrackingState::kRecentlyLost:
      return TrackingState::kRecentlyLost;
    case OrbSlam3TrackingState::kLost:
      return TrackingState::kLost;
  }
  throw std::invalid_argument("unknown ORB-SLAM3 tracking state");
}

}  // namespace

LocalizationEstimate OrbSlam3StateAdapter::convert(
  const OrbSlam3FrameStatus & status)
{
  auto tracking_state = translate_state(status.raw_tracking_state);
  const bool recovered =
    tracking_state == TrackingState::kTracking && was_lost(previous_raw_state_);
  if (recovered || status.map_changed) {
    // Relocalization and loop/map correction can invalidate targets expressed
    // against the previous pose estimate. Downstream consumers must replan.
    tracking_state = TrackingState::kRelocalized;
  }
  previous_raw_state_ = status.raw_tracking_state;

  LocalizationEstimate result;
  result.schema_version = std::string(generated::kLocalizationSchemaVersion);
  result.sequence = status.sequence;
  result.source_timestamp_ns = status.source_timestamp_ns;
  result.ttl_ns = status.ttl_ns;
  result.parent_frame = status.parent_frame;
  result.child_frame = status.child_frame;
  result.map_id = status.map_id;
  result.session_id = status.session_id;
  result.reset_generation = status.reset_generation;
  result.tracking_state = tracking_state;
  result.position_m = status.position_m;
  result.orientation_xyzw = status.orientation_xyzw;
  result.covariance = status.covariance;
  result.tracked_features = status.tracked_features;
  result.mean_reprojection_error_px = status.mean_reprojection_error_px;
  result.confidence = status.confidence;
  result.simulator_truth_used = false;
  result.control_authorized = false;
  return result;
}

void OrbSlam3StateAdapter::reset() noexcept
{
  previous_raw_state_ = static_cast<int>(OrbSlam3TrackingState::kSystemNotReady);
}

}  // namespace dapier_localization_core
