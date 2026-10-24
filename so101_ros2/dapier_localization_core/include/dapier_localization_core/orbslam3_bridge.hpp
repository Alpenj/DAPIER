// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "dapier_localization_core/localization_runtime.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace dapier_localization_core
{

// Numeric values are the public ORB-SLAM3 Tracking::eTrackingState contract.
enum class OrbSlam3TrackingState : int
{
  kSystemNotReady = -1,
  kNoImagesYet = 0,
  kNotInitialized = 1,
  kOk = 2,
  kRecentlyLost = 3,
  kLost = 4,
  kOkKlt = 5,
};

struct OrbSlam3FrameStatus
{
  int raw_tracking_state{0};
  bool map_changed{false};
  std::uint64_t sequence{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t ttl_ns{200000000};
  std::string parent_frame{"map"};
  std::string child_frame{"base_link"};
  std::string map_id;
  std::string session_id;
  std::uint64_t reset_generation{0};
  std::array<double, 3> position_m{0.0, 0.0, 0.0};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 36> covariance{};
  std::uint32_t tracked_features{0};
  double mean_reprojection_error_px{0.0};
  double confidence{0.0};
};

class OrbSlam3StateAdapter
{
public:
  [[nodiscard]] LocalizationEstimate convert(const OrbSlam3FrameStatus & status);
  void reset() noexcept;

private:
  int previous_raw_state_{static_cast<int>(OrbSlam3TrackingState::kSystemNotReady)};
};

}  // namespace dapier_localization_core
