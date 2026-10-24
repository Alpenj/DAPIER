// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "dapier_localization_core/localization_runtime.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace dapier_localization_core
{

struct LocalizationMonitorConfig
{
  std::int64_t maximum_interarrival_ns{75000000};
  MotionGateConfig motion_gate{};
};

struct LocalizationIdentity
{
  std::string parent_frame;
  std::string child_frame;
  std::string map_id;
  std::string session_id;
  std::uint64_t reset_generation{0};

  [[nodiscard]] bool operator==(const LocalizationIdentity & other) const noexcept;
  [[nodiscard]] bool same_frames_and_session(
    const LocalizationIdentity & other) const noexcept;
};

struct MonitoredLocalization
{
  MotionDecision decision{MotionDecision::kReject};
  std::string reason;
  std::optional<std::uint64_t> sequence;
  bool discard_pending_actions{true};
  bool replan_required{false};
  bool identity_changed{false};
  bool hardware_execution{false};
};

class LocalizationMonitor
{
public:
  explicit LocalizationMonitor(
    LocalizationMonitorConfig config = LocalizationMonitorConfig{});

  [[nodiscard]] MonitoredLocalization observe(
    const LocalizationEstimate & estimate,
    std::int64_t receiver_monotonic_ns);

  void acknowledge_replan(
    std::uint64_t sequence,
    const LocalizationIdentity & identity);

  [[nodiscard]] bool awaiting_replan() const noexcept;
  [[nodiscard]] const std::optional<LocalizationIdentity> & identity() const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> last_sequence() const noexcept;

private:
  [[nodiscard]] MonitoredLocalization hold(
    const std::string & reason,
    std::uint64_t sequence,
    bool identity_changed = false);

  LocalizationMonitorConfig config_;
  std::optional<std::uint64_t> last_sequence_;
  std::optional<std::int64_t> last_receiver_monotonic_ns_;
  std::optional<LocalizationIdentity> identity_;
  bool awaiting_replan_{true};
  std::optional<std::uint64_t> replan_sequence_;
};

}  // namespace dapier_localization_core
