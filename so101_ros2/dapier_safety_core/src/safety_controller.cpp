// SPDX-License-Identifier: Apache-2.0

#include "dapier_safety_core/safety_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace dapier_safety_core
{
namespace
{

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

void validate_config(const SafetyControllerConfig & config)
{
  if (config.joints.empty()) {
    throw std::invalid_argument("safety controller requires joint limits");
  }
  if (config.measured_state_timeout_ns <= 0 || config.command_watchdog_ns <= 0) {
    throw std::invalid_argument("state timeout and command watchdog must be positive");
  }
  if (!finite(config.command_horizon_s) || config.command_horizon_s <= 0.0 ||
    !finite(config.maximum_base_linear_mps) || config.maximum_base_linear_mps <= 0.0 ||
    !finite(config.maximum_base_angular_rad_s) || config.maximum_base_angular_rad_s <= 0.0 ||
    !finite(config.settled_base_linear_mps) || config.settled_base_linear_mps < 0.0 ||
    !finite(config.settled_base_angular_rad_s) || config.settled_base_angular_rad_s < 0.0)
  {
    throw std::invalid_argument("safety controller numeric limits are invalid");
  }
  std::unordered_set<std::string> names;
  for (const auto & joint : config.joints) {
    if (joint.name.empty() || !names.insert(joint.name).second) {
      throw std::invalid_argument("joint safety names must be non-empty and unique");
    }
    if (!finite(joint.minimum_position_rad) ||
      !finite(joint.maximum_position_rad) ||
      !finite(joint.maximum_velocity_rad_s) ||
      joint.minimum_position_rad >= joint.maximum_position_rad ||
      joint.maximum_velocity_rad_s <= 0.0)
    {
      throw std::invalid_argument("joint safety limit is invalid");
    }
  }
}

bool is_state_fresh(
  const MeasuredRobotState & measured,
  std::int64_t receiver_monotonic_ns,
  std::int64_t timeout_ns) noexcept
{
  if (receiver_monotonic_ns < 0 || measured.received_monotonic_ns < 0 ||
    measured.received_monotonic_ns > receiver_monotonic_ns)
  {
    return false;
  }
  return receiver_monotonic_ns - measured.received_monotonic_ns <= timeout_ns;
}

}  // namespace

SafetyController::SafetyController(SafetyControllerConfig config)
: config_(std::move(config))
{
  validate_config(config_);
}

void SafetyController::set_operator_enabled(bool enabled) noexcept
{
  operator_enabled_ = enabled;
  if (!enabled) {
    motion_armed_ = false;
  }
}

void SafetyController::set_estop_healthy(bool healthy) noexcept
{
  estop_healthy_ = healthy;
  if (!healthy) {
    safe_stop_latched_ = true;
    operator_enabled_ = false;
    motion_armed_ = false;
  }
}

SafeCommand SafetyController::reject(
  const std::string & reason,
  std::uint64_t sequence) const
{
  SafeCommand command;
  command.decision = SafetyDecision::kReject;
  command.reason = reason;
  command.sequence = sequence;
  return command;
}

std::vector<double> SafetyController::ordered_positions(
  const MeasuredRobotState & measured) const
{
  if (measured.joint_names.size() != measured.joint_position_rad.size()) {
    throw std::invalid_argument("measured joint arrays differ in length");
  }
  std::unordered_map<std::string, double> positions;
  for (std::size_t index = 0; index < measured.joint_names.size(); ++index) {
    const auto & name = measured.joint_names[index];
    const double position = measured.joint_position_rad[index];
    if (name.empty() || !finite(position) || !positions.emplace(name, position).second) {
      throw std::invalid_argument("measured joint state is invalid or duplicated");
    }
  }
  std::vector<double> ordered;
  ordered.reserve(config_.joints.size());
  for (const auto & joint : config_.joints) {
    const auto found = positions.find(joint.name);
    if (found == positions.end()) {
      throw std::invalid_argument("measured joint state is missing a configured joint");
    }
    if (found->second < joint.minimum_position_rad ||
      found->second > joint.maximum_position_rad)
    {
      throw std::invalid_argument("measured joint state is outside a hard limit");
    }
    ordered.push_back(found->second);
  }
  if (positions.size() != config_.joints.size()) {
    throw std::invalid_argument("measured joint state contains unknown joints");
  }
  return ordered;
}

const JointSafetyLimit & SafetyController::limit_for(const std::string & name) const
{
  const auto found = std::find_if(
    config_.joints.begin(), config_.joints.end(),
    [&name](const JointSafetyLimit & limit) {return limit.name == name;});
  if (found == config_.joints.end()) {
    throw std::invalid_argument("intent references an unknown joint");
  }
  return *found;
}

SafeCommand SafetyController::hold_from_measured(
  const MeasuredRobotState & measured,
  SafetyDecision decision,
  const std::string & reason,
  std::uint64_t sequence) const
{
  SafeCommand command;
  command.decision = decision;
  command.reason = reason;
  command.kind = dapier_so101_core::ResearchIntentKind::kHold;
  command.sequence = sequence;
  command.joint_names.reserve(config_.joints.size());
  for (const auto & joint : config_.joints) {
    command.joint_names.push_back(joint.name);
  }
  command.joint_position_rad = ordered_positions(measured);
  command.joint_max_velocity_rad_s.assign(config_.joints.size(), 0.0);
  command.dispatch_allowed = true;
  command.hardware_execution = false;
  return command;
}

SafeCommand SafetyController::latch_safe_stop(
  const MeasuredRobotState & measured,
  const std::string & reason)
{
  safe_stop_latched_ = true;
  operator_enabled_ = false;
  motion_armed_ = false;
  return hold_from_measured(measured, SafetyDecision::kSafeStop, reason);
}

SafeCommand SafetyController::evaluate(
  const dapier_so101_core::ResearchControlIntent & intent,
  const MeasuredRobotState & measured,
  const SafetyContext & context,
  std::int64_t receiver_monotonic_ns)
{
  if (!estop_healthy_) {
    try {
      return latch_safe_stop(measured, "E-stop health is not confirmed");
    } catch (const std::exception & error) {
      return reject(std::string("E-stop unhealthy and measured hold invalid: ") + error.what());
    }
  }

  const auto validation = dapier_so101_core::validate_research_control_intent(
    intent,
    receiver_monotonic_ns,
    last_accepted_sequence_);
  if (!validation.accepted) {
    return reject(validation.reason, intent.sequence);
  }

  try {
    const auto measured_positions = ordered_positions(measured);
    if (!finite(measured.base_linear_mps) || !finite(measured.base_angular_rad_s)) {
      return reject("measured base velocity must be finite", intent.sequence);
    }
    if (!is_state_fresh(
        measured, receiver_monotonic_ns, config_.measured_state_timeout_ns))
    {
      return latch_safe_stop(measured, "measured robot state is stale");
    }
    if (intent.kind == dapier_so101_core::ResearchIntentKind::kHold) {
      motion_armed_ = false;
      active_command_expires_at_ns_.reset();
      last_accepted_sequence_ = intent.sequence;
      last_dispatch_monotonic_ns_ = receiver_monotonic_ns;
      return hold_from_measured(
        measured,
        safe_stop_latched_ ? SafetyDecision::kSafeStop : SafetyDecision::kHold,
        safe_stop_latched_ ? "safe stop remains latched" : "explicit hold accepted",
        intent.sequence);
    }
    if (safe_stop_latched_) {
      return reject("safe stop is latched", intent.sequence);
    }
    if (!operator_enabled_) {
      return reject("operator enable is required", intent.sequence);
    }
    if (!measured.collision_clear) {
      return latch_safe_stop(measured, "collision interlock is not clear");
    }
    if (context.localization_decision !=
      dapier_localization_core::MotionDecision::kProceed)
    {
      return reject("localization gate does not permit motion", intent.sequence);
    }

    SafeCommand command;
    command.decision = SafetyDecision::kDispatch;
    command.reason = "command passed receiver-local safety gates";
    command.kind = intent.kind;
    command.sequence = intent.sequence;
    command.expires_at_monotonic_ns = validation.accepted_until_monotonic_ns;
    command.dispatch_allowed = true;
    command.hardware_execution = false;

    if (intent.kind == dapier_so101_core::ResearchIntentKind::kArmJointPosition) {
      if (!context.arm_phase_allowed) {
        return reject("task phase does not permit arm motion", intent.sequence);
      }
      if (!measured.base_settled ||
        std::abs(measured.base_linear_mps) > config_.settled_base_linear_mps ||
        std::abs(measured.base_angular_rad_s) > config_.settled_base_angular_rad_s)
      {
        return reject("arm motion requires a settled base", intent.sequence);
      }
      std::unordered_map<std::string, double> current;
      for (std::size_t index = 0; index < config_.joints.size(); ++index) {
        current.emplace(config_.joints[index].name, measured_positions[index]);
      }
      for (std::size_t index = 0; index < intent.joint_names.size(); ++index) {
        const auto & name = intent.joint_names[index];
        const auto & limit = limit_for(name);
        const double target = intent.joint_position_rad[index];
        const double requested_velocity = intent.joint_max_velocity_rad_s[index];
        if (target < limit.minimum_position_rad || target > limit.maximum_position_rad) {
          return reject("joint target is outside a hard limit", intent.sequence);
        }
        if (requested_velocity > limit.maximum_velocity_rad_s) {
          return reject("joint velocity exceeds the configured maximum", intent.sequence);
        }
        const double allowed_delta = requested_velocity * config_.command_horizon_s;
        if (std::abs(target - current.at(name)) > allowed_delta + 1e-12) {
          return reject("joint target exceeds the bounded command horizon", intent.sequence);
        }
      }
      command.joint_names = intent.joint_names;
      command.joint_position_rad = intent.joint_position_rad;
      command.joint_max_velocity_rad_s = intent.joint_max_velocity_rad_s;
    } else if (intent.kind == dapier_so101_core::ResearchIntentKind::kBaseTwist) {
      if (!context.base_phase_allowed) {
        return reject("task phase does not permit base motion", intent.sequence);
      }
      if (!measured.carry_pose_clear) {
        return reject("base motion requires both arms inside the carry envelope", intent.sequence);
      }
      if (config_.require_verified_grasp_for_base_motion && !measured.grasp_verified) {
        return reject("base transport requires a verified grasp", intent.sequence);
      }
      if (std::abs(intent.base_linear_x_mps) > config_.maximum_base_linear_mps ||
        std::abs(intent.base_angular_z_rad_s) > config_.maximum_base_angular_rad_s)
      {
        return reject("base command exceeds configured speed limits", intent.sequence);
      }
      command.base_linear_x_mps = intent.base_linear_x_mps;
      command.base_angular_z_rad_s = intent.base_angular_z_rad_s;
    } else {
      return reject("unsupported motion intent kind", intent.sequence);
    }

    last_accepted_sequence_ = intent.sequence;
    last_dispatch_monotonic_ns_ = receiver_monotonic_ns;
    motion_armed_ = true;
    active_command_expires_at_ns_ = command.expires_at_monotonic_ns;
    return command;
  } catch (const std::exception & error) {
    return reject(std::string("invalid measured state or command: ") + error.what(), intent.sequence);
  }
}

std::optional<SafeCommand> SafetyController::tick(
  const MeasuredRobotState & measured,
  std::int64_t receiver_monotonic_ns)
{
  if (!estop_healthy_) {
    try {
      return latch_safe_stop(measured, "E-stop health was lost");
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  if (!motion_armed_ || !last_dispatch_monotonic_ns_) {
    return std::nullopt;
  }
  if (!is_state_fresh(
      measured, receiver_monotonic_ns, config_.measured_state_timeout_ns))
  {
    return latch_safe_stop(measured, "measured-state watchdog expired");
  }
  if (active_command_expires_at_ns_ &&
    receiver_monotonic_ns >= *active_command_expires_at_ns_)
  {
    return latch_safe_stop(measured, "command TTL expired");
  }
  if (receiver_monotonic_ns < *last_dispatch_monotonic_ns_ ||
    receiver_monotonic_ns - *last_dispatch_monotonic_ns_ > config_.command_watchdog_ns)
  {
    return latch_safe_stop(measured, "command watchdog expired");
  }
  return std::nullopt;
}

void SafetyController::acknowledge_safe_stop()
{
  if (operator_enabled_) {
    throw std::logic_error("disable operator motion before clearing safe stop");
  }
  if (!estop_healthy_) {
    throw std::logic_error("E-stop health must be confirmed before clearing safe stop");
  }
  safe_stop_latched_ = false;
  motion_armed_ = false;
  last_dispatch_monotonic_ns_.reset();
  active_command_expires_at_ns_.reset();
}

bool SafetyController::operator_enabled() const noexcept
{
  return operator_enabled_;
}

bool SafetyController::estop_healthy() const noexcept
{
  return estop_healthy_;
}

bool SafetyController::safe_stop_latched() const noexcept
{
  return safe_stop_latched_;
}

std::optional<std::uint64_t> SafetyController::last_accepted_sequence() const noexcept
{
  return last_accepted_sequence_;
}

void RecordingMockCommandSink::dispatch(const SafeCommand & command)
{
  if (!command.dispatch_allowed) {
    throw std::invalid_argument("mock sink refuses a non-dispatchable command");
  }
  if (command.hardware_execution) {
    throw std::invalid_argument("mock sink refuses a command claiming hardware execution");
  }
  commands_.push_back(command);
}

const std::vector<SafeCommand> & RecordingMockCommandSink::commands() const noexcept
{
  return commands_;
}

void RecordingMockCommandSink::clear() noexcept
{
  commands_.clear();
}

}  // namespace dapier_safety_core
