// SPDX-License-Identifier: Apache-2.0

#include "dapier_localization_core/localization_monitor.hpp"

#include <stdexcept>
#include <utility>

namespace dapier_localization_core
{

bool LocalizationIdentity::operator==(
  const LocalizationIdentity & other) const noexcept
{
  return parent_frame == other.parent_frame &&
         child_frame == other.child_frame &&
         map_id == other.map_id &&
         session_id == other.session_id &&
         reset_generation == other.reset_generation;
}

bool LocalizationIdentity::same_frames_and_session(
  const LocalizationIdentity & other) const noexcept
{
  return parent_frame == other.parent_frame &&
         child_frame == other.child_frame &&
         map_id == other.map_id &&
         session_id == other.session_id;
}

LocalizationMonitor::LocalizationMonitor(LocalizationMonitorConfig config)
: config_(std::move(config))
{
  if (config_.maximum_interarrival_ns <= 0) {
    throw std::invalid_argument("maximum_interarrival_ns must be positive");
  }
}

MonitoredLocalization LocalizationMonitor::hold(
  const std::string & reason,
  std::uint64_t sequence,
  bool identity_changed)
{
  awaiting_replan_ = true;
  replan_sequence_ = sequence;
  return MonitoredLocalization{
    MotionDecision::kHold,
    reason,
    sequence,
    true,
    true,
    identity_changed,
    false,
  };
}

MonitoredLocalization LocalizationMonitor::observe(
  const LocalizationEstimate & estimate,
  std::int64_t receiver_monotonic_ns)
{
  const auto validation = validate_localization_estimate(
    estimate,
    receiver_monotonic_ns,
    last_sequence_);
  if (!validation.accepted) {
    return MonitoredLocalization{
      MotionDecision::kReject,
      validation.reason,
      std::nullopt,
      true,
      false,
      false,
      false,
    };
  }

  std::int64_t gap_ns = 0;
  if (last_receiver_monotonic_ns_) {
    gap_ns = receiver_monotonic_ns - *last_receiver_monotonic_ns_;
    if (gap_ns <= 0) {
      return MonitoredLocalization{
        MotionDecision::kReject,
        "receiver monotonic time must increase",
        estimate.sequence,
        true,
        false,
        false,
        false,
      };
    }
  }

  const LocalizationIdentity current{
    estimate.parent_frame,
    estimate.child_frame,
    estimate.map_id,
    estimate.session_id,
    estimate.reset_generation,
  };
  const auto previous = identity_;
  if (previous) {
    if (current.reset_generation < previous->reset_generation) {
      return MonitoredLocalization{
        MotionDecision::kReject,
        "reset generation moved backwards",
        estimate.sequence,
        true,
        false,
        false,
        false,
      };
    }
    if (!current.same_frames_and_session(*previous) &&
      current.reset_generation <= previous->reset_generation)
    {
      return MonitoredLocalization{
        MotionDecision::kReject,
        "frame/map/session identity changed without a reset generation increment",
        estimate.sequence,
        true,
        false,
        true,
        false,
      };
    }
  }

  last_sequence_ = estimate.sequence;
  last_receiver_monotonic_ns_ = receiver_monotonic_ns;
  identity_ = current;

  if (!previous) {
    return hold(
      "first localization identity requires a fresh plan",
      estimate.sequence,
      true);
  }
  if (!(current == *previous)) {
    return hold(
      "localization identity changed; stale targets were invalidated",
      estimate.sequence,
      true);
  }
  if (gap_ns > config_.maximum_interarrival_ns) {
    return hold(
      "localization interarrival watchdog expired",
      estimate.sequence);
  }

  const auto quality = assess_localization_for_motion(
    estimate,
    validation,
    config_.motion_gate);
  if (quality.decision == MotionDecision::kReject) {
    return MonitoredLocalization{
      MotionDecision::kReject,
      quality.reason,
      estimate.sequence,
      true,
      false,
      false,
      false,
    };
  }
  if (quality.decision == MotionDecision::kHold) {
    return hold(quality.reason, estimate.sequence);
  }
  if (awaiting_replan_) {
    return MonitoredLocalization{
      MotionDecision::kHold,
      "localization recovered but a plan acknowledgement is still required",
      estimate.sequence,
      true,
      true,
      false,
      false,
    };
  }
  return MonitoredLocalization{
    MotionDecision::kProceed,
    quality.reason,
    estimate.sequence,
    false,
    false,
    false,
    false,
  };
}

void LocalizationMonitor::acknowledge_replan(
  std::uint64_t sequence,
  const LocalizationIdentity & identity)
{
  if (!identity_ || !(identity == *identity_)) {
    throw std::invalid_argument(
            "replan identity does not match active localization identity");
  }
  if (!replan_sequence_ || sequence != *replan_sequence_) {
    throw std::invalid_argument(
            "replan sequence does not match localization invalidation boundary");
  }
  awaiting_replan_ = false;
}

bool LocalizationMonitor::awaiting_replan() const noexcept
{
  return awaiting_replan_;
}

const std::optional<LocalizationIdentity> & LocalizationMonitor::identity() const noexcept
{
  return identity_;
}

std::optional<std::uint64_t> LocalizationMonitor::last_sequence() const noexcept
{
  return last_sequence_;
}

}  // namespace dapier_localization_core
