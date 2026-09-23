#pragma once

#include "dapier_so101_core/joint_model.hpp"
#include "dapier_so101_core/generated/research_realtime_contract.hpp"
#include "dapier_safety_core/safety_controller.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dapier_so101_executor {
using dapier_safety_core::MeasuredRobotState;
using dapier_safety_core::SafetyController;
using dapier_safety_core::SafetyControllerConfig;
using dapier_so101_core::JointModel;

struct SentPositions {
  std::vector<double> position_rad;
  std::vector<int> raw_ticks;
};

inline int bounded_raw_tick(const dapier_so101_core::CalibrationEntry& cal,
    double requested, double lower, double upper) {
  if (!std::isfinite(requested) || !std::isfinite(lower) || !std::isfinite(upper) ||
      lower > upper || requested < lower || requested > upper ||
      requested < cal.position_min || requested > cal.position_max)
    throw std::runtime_error("invalid bounded quantization input");
  const int nearest = cal.position_to_raw(requested);
  // A nearest tick can round outside a velocity/joint bound. Its adjacent tick
  // is the only other possible in-bound nearest candidate. Measurements stay raw.
  for (int raw : {nearest, nearest-1, nearest+1}) {
    if (raw < cal.raw_min || raw > cal.raw_max) continue;
    const double value = cal.raw_to_position(raw);
    if (value >= lower && value <= upper) return raw;
  }
  throw std::runtime_error("no motor tick fits bounded command interval");
}

// The executor is identical for hardware and a lagged test plant. Only I/O differs.
struct MotorTransport {
  virtual ~MotorTransport() = default;
  virtual MeasuredRobotState read() = 0;
  virtual void arm_at_measured_position() = 0;
  virtual SentPositions send(const std::vector<double>& position_rad) = 0;
};

struct StepTrace {
  std::int64_t time_ns{};
  std::string phase, reason;
  std::vector<double> requested_rad, limited_rad, measured_rad;
  SentPositions sent;
};

struct PregraspResult {
  bool reached{false};
  std::string phase{"PREFLIGHT"}, reason;
  std::vector<double> final_measured_rad;
  // Reaching joints does not prove TCP, contact, lift, or task success.
  bool task_success{false};
};

inline bool within_path_envelope(const std::vector<double>& q,
    const std::vector<double>& start, const std::vector<double>& goal, double tolerance) {
  if (q.size() != start.size() || q.size() != goal.size() ||
      !std::isfinite(tolerance) || tolerance <= 0) return false;
  double numerator = 0., denominator = 0.;
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i])) return false;
    numerator += (q[i]-start[i])*(goal[i]-start[i]);
    denominator += (goal[i]-start[i])*(goal[i]-start[i]);
  }
  const double progress = denominator > 0 ? std::clamp(numerator/denominator, 0., 1.) : 0.;
  for (std::size_t i = 0; i < q.size(); ++i)
    if (std::abs(q[i] - start[i] - progress*(goal[i]-start[i])) > tolerance) return false;
  return true;
}

inline PregraspResult execute_pregrasp(
    MotorTransport& transport, const JointModel& model, SafetyControllerConfig config,
    const std::vector<double>& expected_start, const std::vector<double>& goal,
    double maximum_duration_s, const std::function<std::int64_t()>& now,
    const std::function<void()>& wait_cycle, const std::function<bool()>& cancelled,
    const std::function<void(const StepTrace&)>& record, double path_tolerance_rad = .01,
    const std::string& phase = "PREGRASP") {
  PregraspResult result;
  if (!model.within_limits(expected_start) || !model.within_limits(goal) ||
      !std::isfinite(maximum_duration_s) || maximum_duration_s <= 0 || maximum_duration_s > 60) {
    throw std::invalid_argument("invalid bounded pregrasp plan");
  }
  SafetyController safety(config);
  // These represent the caller's already verified attended-run permit; not an
  // independent physical E-stop/watchdog verification or permission to open a port.
  safety.set_operator_enabled(true);
  safety.set_estop_healthy(true);
  dapier_safety_core::SafetyContext context;
  context.arm_phase_allowed = true;
  context.localization_decision = dapier_localization_core::MotionDecision::kProceed;
  const auto started_ns = now();
  std::vector<double> initial;
  double duration_s = 2.5;
  std::uint64_t sequence = 0;
  unsigned settled_samples = 0;
  bool armed = false;
  bool arming_attempted = false;
  bool sent_any = false;
  try {
    for (;;) {
      if (cancelled()) throw std::runtime_error("operator interruption; retain last bounded goal");
      if (now() - started_ns > maximum_duration_s * 1e9)
        throw std::runtime_error("pregrasp deadline exceeded");
      const auto measured = transport.read();
      if (cancelled()) throw std::runtime_error("operator interruption after readback");
      const auto current_ns = now();
      if (current_ns < started_ns || current_ns - started_ns > maximum_duration_s * 1e9)
        throw std::runtime_error("readback exceeded approved execution deadline");
      auto positions = model.reorder(measured.joint_names, measured.joint_position_rad);
      result.final_measured_rad = positions;
      if (!model.within_limits(positions)) throw std::runtime_error("measured state outside limits; not clipped");
      if (!within_path_envelope(positions, expected_start, goal, path_tolerance_rad))
        throw std::runtime_error("measured state left checked path envelope");
      if (sent_any) {
        const auto stop = safety.tick(measured, current_ns);
        if (stop) throw std::runtime_error(stop->reason);
      }
      if (initial.empty()) {
        initial = positions;
        for (std::size_t i = 0; i < initial.size(); ++i) {
          if (std::abs(initial[i] - expected_start[i]) > 0.5 * std::acos(-1.) / 180.)
            throw std::runtime_error("measured start differs from checked path start");
          duration_s = std::max(duration_s, 1.875 * std::abs(goal[i] - initial[i]) /
              model.joints()[i].max_velocity * 1.15);
        }
        if (duration_s + 1.0 > maximum_duration_s)
          throw std::runtime_error("plan duration exceeds approved deadline");
      }
      const double elapsed_s = (current_ns - started_ns) * 1e-9;
      const double u = std::clamp(elapsed_s / duration_s, 0., 1.);
      const double blend = u*u*u*(10. + u*(-15. + 6.*u));
      std::vector<double> requested(goal.size());
      for (std::size_t i = 0; i < goal.size(); ++i)
        requested[i] = initial[i] + blend*(goal[i] - initial[i]);
      const auto limited = model.limit(positions, requested, config.command_horizon_s);
      if (!within_path_envelope(limited.command, expected_start, goal, path_tolerance_rad))
        throw std::runtime_error("limited command left checked path envelope");
      dapier_so101_core::ResearchControlIntent intent;
      intent.schema_version = dapier_so101_core::generated::kResearchRealtimeSchemaVersion;
      intent.sequence = ++sequence;
      intent.kind = dapier_so101_core::ResearchIntentKind::kArmJointPosition;
      intent.source = "bounded_pregrasp_executor";
      intent.source_monotonic_ns = current_ns;
      intent.ttl_ns = config.command_watchdog_ns;
      intent.joint_names = model.names();
      intent.joint_position_rad = limited.command;
      for (const auto& joint : model.joints()) intent.joint_max_velocity_rad_s.push_back(joint.max_velocity);
      const auto command = safety.evaluate(intent, measured, context, current_ns);
      StepTrace trace{current_ns, phase+"_TRAVEL", command.reason, requested, limited.command, positions, {}};
      if (!command.dispatch_allowed || command.decision != dapier_safety_core::SafetyDecision::kDispatch) {
        trace.phase = "REJECTED";
        record(trace);
        throw std::runtime_error(command.reason);
      }
      if (!armed) {
        arming_attempted = true;
        transport.arm_at_measured_position();
        armed = true;
        // Torque startup can take time. Re-read and revalidate before the first motion.
        wait_cycle();
        continue;
      }
      if (cancelled()) throw std::runtime_error("operator interruption before dispatch");
      trace.sent = transport.send(command.joint_position_rad);
      sent_any = true;
      bool settled = u == 1.;
      for (std::size_t i = 0; i < goal.size(); ++i)
        settled = settled && std::abs(positions[i] - goal[i]) <= .001;
      settled_samples = settled ? settled_samples + 1 : 0;
      trace.phase = phase + (u < 1. ? "_TRAVEL" : "_SETTLING");
      record(trace);
      if (settled_samples >= 3) {
        result.reached = true;
        result.phase = phase+"_REACHED_HOLDING";
        result.reason = "measured joint endpoint reached; TCP validation and task acceptance remain separate";
        return result;
      }
      wait_cycle();
    }
  } catch (const std::exception& error) {
    result.phase = armed ? phase+"_ABORTED_HOLDING_LAST_GOAL" :
        arming_attempted ? "ARMING_INTERRUPTED_STATE_UNCERTAIN" : "PREFLIGHT_REJECTED";
    result.reason = error.what();
    // Never torque-off an airborne arm as generic cleanup. The bounded last goal
    // remains active; a failed link requires the attending operator's intervention.
    return result;
  }
}
}  // namespace dapier_so101_executor
