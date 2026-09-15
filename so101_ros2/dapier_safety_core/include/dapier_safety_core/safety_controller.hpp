// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "dapier_localization_core/localization_runtime.hpp"
#include "dapier_so101_core/realtime_control.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dapier_safety_core
{

enum class SafetyDecision
{
  kDispatch,
  kHold,
  kReject,
  kSafeStop,
};

struct JointSafetyLimit
{
  std::string name;
  double minimum_position_rad{0.0};
  double maximum_position_rad{0.0};
  double maximum_velocity_rad_s{0.0};
};

struct SafetyControllerConfig
{
  std::vector<JointSafetyLimit> joints;
  std::int64_t measured_state_timeout_ns{100000000};
  std::int64_t command_watchdog_ns{250000000};
  double command_horizon_s{0.05};
  double maximum_base_linear_mps{0.08};
  double maximum_base_angular_rad_s{0.25};
  double settled_base_linear_mps{0.0025};
  double settled_base_angular_rad_s{0.0021};
  bool require_verified_grasp_for_base_motion{true};
};

struct MeasuredRobotState
{
  std::int64_t received_monotonic_ns{0};
  std::vector<std::string> joint_names;
  std::vector<double> joint_position_rad;
  double base_linear_mps{0.0};
  double base_angular_rad_s{0.0};
  bool base_settled{false};
  bool collision_clear{false};
  bool carry_pose_clear{false};
  bool grasp_verified{false};
};

struct SafetyContext
{
  dapier_localization_core::MotionDecision localization_decision{
    dapier_localization_core::MotionDecision::kReject};
  bool arm_phase_allowed{false};
  bool base_phase_allowed{false};
};

struct SafeCommand
{
  SafetyDecision decision{SafetyDecision::kReject};
  std::string reason;
  dapier_so101_core::ResearchIntentKind kind{
    dapier_so101_core::ResearchIntentKind::kHold};
  std::uint64_t sequence{0};
  std::vector<std::string> joint_names;
  std::vector<double> joint_position_rad;
  std::vector<double> joint_max_velocity_rad_s;
  double base_linear_x_mps{0.0};
  double base_angular_z_rad_s{0.0};
  std::int64_t expires_at_monotonic_ns{0};
  bool dispatch_allowed{false};
  bool hardware_execution{false};
};

class SafetyController
{
public:
  explicit SafetyController(SafetyControllerConfig config);

  void set_operator_enabled(bool enabled) noexcept;
  void set_estop_healthy(bool healthy) noexcept;

  [[nodiscard]] SafeCommand evaluate(
    const dapier_so101_core::ResearchControlIntent & intent,
    const MeasuredRobotState & measured,
    const SafetyContext & context,
    std::int64_t receiver_monotonic_ns);

  [[nodiscard]] std::optional<SafeCommand> tick(
    const MeasuredRobotState & measured,
    std::int64_t receiver_monotonic_ns);

  void acknowledge_safe_stop();

  [[nodiscard]] bool operator_enabled() const noexcept;
  [[nodiscard]] bool estop_healthy() const noexcept;
  [[nodiscard]] bool safe_stop_latched() const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> last_accepted_sequence() const noexcept;

private:
  [[nodiscard]] SafeCommand reject(
    const std::string & reason,
    std::uint64_t sequence = 0) const;
  [[nodiscard]] SafeCommand hold_from_measured(
    const MeasuredRobotState & measured,
    SafetyDecision decision,
    const std::string & reason,
    std::uint64_t sequence = 0) const;
  [[nodiscard]] SafeCommand latch_safe_stop(
    const MeasuredRobotState & measured,
    const std::string & reason);
  [[nodiscard]] std::vector<double> ordered_positions(
    const MeasuredRobotState & measured) const;
  [[nodiscard]] const JointSafetyLimit & limit_for(
    const std::string & name) const;

  SafetyControllerConfig config_;
  bool operator_enabled_{false};
  bool estop_healthy_{false};
  bool safe_stop_latched_{false};
  bool motion_armed_{false};
  std::optional<std::uint64_t> last_accepted_sequence_;
  std::optional<std::int64_t> last_dispatch_monotonic_ns_;
  std::optional<std::int64_t> active_command_expires_at_ns_;
};

class RecordingMockCommandSink
{
public:
  void dispatch(const SafeCommand & command);
  [[nodiscard]] const std::vector<SafeCommand> & commands() const noexcept;
  void clear() noexcept;

private:
  std::vector<SafeCommand> commands_;
};

}  // namespace dapier_safety_core
