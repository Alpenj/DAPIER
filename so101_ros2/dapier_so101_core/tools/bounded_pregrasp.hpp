#pragma once

#include "dapier_so101_core/joint_model.hpp"
#include "dapier_so101_core/generated/research_realtime_contract.hpp"
#include "dapier_safety_core/safety_controller.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <set>
#include <optional>
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
  bool observed_hold_verified{false};
  bool observed_grasp_verified{false};
  bool observed_lift_verified{false};
  std::int64_t hold_entered_ns{};
  double observed_hold_span_s{};
};

struct BlockGraspObservation {
  std::int64_t sequence{}, captured_ns{};
  std::optional<bool> bilateral_grasp_verified;
  std::string frame_sha256;
};

struct ObservedGrasp {
  std::int64_t last_sequence{}, last_capture_ns{};
  std::string last_frame_sha256;
  std::set<std::string> consumed_frames;
  unsigned positive_frames{};
  bool last_bilateral_grasp{};
  void update(const BlockGraspObservation& observation, std::int64_t now_ns) {
    if (!observation.bilateral_grasp_verified.has_value())
      throw std::runtime_error("CLOSE bilateral grasp observation is unknown");
    if (observation.sequence <= 0 || observation.captured_ns <= 0 ||
        observation.captured_ns > now_ns || now_ns-observation.captured_ns > 250000000 ||
        observation.frame_sha256.size() != 64 ||
        observation.frame_sha256.find_first_not_of("0123456789abcdef") != std::string::npos)
      throw std::runtime_error("CLOSE requires fresh frame-bound grasp evidence");
    if (last_sequence && (observation.sequence < last_sequence ||
        observation.captured_ns < last_capture_ns ||
        (observation.sequence == last_sequence && (observation.captured_ns != last_capture_ns ||
         observation.frame_sha256 != last_frame_sha256 ||
         *observation.bilateral_grasp_verified != last_bilateral_grasp)) ||
        (observation.sequence > last_sequence && observation.captured_ns <= last_capture_ns) ||
        now_ns-last_capture_ns > 250000000))
      throw std::runtime_error("CLOSE observation identity/order/continuity changed");
    if (positive_frames && !*observation.bilateral_grasp_verified)
      throw std::runtime_error("GRASP_CONFIRM lost bilateral grasp; no further closure");
    if (observation.sequence == last_sequence) return;
    if (!consumed_frames.insert(observation.frame_sha256).second)
      throw std::runtime_error("CLOSE raw frame reused with a new identity");
    last_sequence = observation.sequence; last_capture_ns = observation.captured_ns;
    last_frame_sha256 = observation.frame_sha256;
    last_bilateral_grasp = *observation.bilateral_grasp_verified;
    if (*observation.bilateral_grasp_verified) ++positive_frames;
  }
};

struct BlockHoldObservation {
  std::int64_t sequence{}, captured_ns{};
  double bottom_clearance_lower_bound_m{};
  bool bilateral_grasp_verified{false}, external_support{true};
  std::string frame_sha256;
};

// Count a 3s span of distinct fresh observations AFTER entering HOLD. Commands
// and joint convergence cannot establish object grasp, lift or continued support.
struct ObservedBlockHold {
  std::int64_t entered_ns{}, first_capture_ns{}, last_capture_ns{}, last_sequence{};
  std::string last_frame_sha256;
  std::set<std::string> consumed_frames;
  bool update(const BlockHoldObservation& observation, std::int64_t now_ns) {
    constexpr std::int64_t maximum_gap_ns = 250000000;
    if (now_ns <= 0 || observation.sequence <= 0 || observation.captured_ns <= 0 ||
        observation.captured_ns > now_ns || now_ns-observation.captured_ns > maximum_gap_ns ||
        !std::isfinite(observation.bottom_clearance_lower_bound_m) ||
        observation.bottom_clearance_lower_bound_m < .030 ||
        !observation.bilateral_grasp_verified || observation.external_support ||
        observation.frame_sha256.size() != 64 ||
        observation.frame_sha256.find_first_not_of("0123456789abcdef") != std::string::npos)
      throw std::runtime_error("HOLD requires fresh observed grasp and >=30mm unsupported lift");
    if (!entered_ns) entered_ns = now_ns;
    if (observation.captured_ns < entered_ns) return false;
    if (last_sequence && (observation.sequence < last_sequence ||
        observation.captured_ns < last_capture_ns ||
        (observation.sequence == last_sequence && (observation.captured_ns != last_capture_ns ||
         observation.frame_sha256 != last_frame_sha256)) ||
        (observation.sequence > last_sequence && observation.captured_ns <= last_capture_ns)))
      throw std::runtime_error("HOLD observation order changed or frame was retimestamped");
    if (last_capture_ns && (now_ns-last_capture_ns > maximum_gap_ns ||
        observation.captured_ns-last_capture_ns > maximum_gap_ns))
      throw std::runtime_error("HOLD observation continuity lost");
    if (observation.sequence == last_sequence) return false;
    if (!consumed_frames.insert(observation.frame_sha256).second)
      throw std::runtime_error("HOLD raw frame reused with a new identity");
    if (!first_capture_ns) first_capture_ns = observation.captured_ns;
    last_sequence = observation.sequence; last_capture_ns = observation.captured_ns;
    last_frame_sha256 = observation.frame_sha256;
    return observation.captured_ns-first_capture_ns >= 3000000000LL &&
           now_ns-entered_ns >= 3000000000LL;
  }
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

inline std::vector<double> rate_limited_path_point(const JointModel& model,
    const std::vector<double>& measured, const std::vector<double>& start,
    const std::vector<double>& goal, double progress, double horizon_s) {
  if (!model.within_limits(measured) || !model.within_limits(start) || !model.within_limits(goal) ||
      !std::isfinite(progress) || progress < 0 || progress > 1 ||
      !std::isfinite(horizon_s) || horizon_s <= 0)
    throw std::invalid_argument("invalid synchronized path command");
  double lower = 0., upper = 1.;
  for (std::size_t i = 0; i < start.size(); ++i) {
    const auto& joint = model.joints()[i];
    const double step = joint.max_velocity * horizon_s;
    const double lo = std::max(joint.lower_limit, measured[i]-step);
    const double hi = std::min(joint.upper_limit, measured[i]+step);
    const double delta = goal[i]-start[i];
    if (delta == 0.) {
      if (start[i] < lo || start[i] > hi)
        throw std::runtime_error("fixed path joint outside bounded command interval");
      continue;
    }
    const double a = (lo-start[i])/delta, b = (hi-start[i])/delta;
    lower = std::max(lower, std::min(a,b));
    upper = std::min(upper, std::max(a,b));
  }
  if (lower > upper)
    throw std::runtime_error("no common path progress satisfies joint velocity limits");
  // Independent clipping rotates a checked joint-space line. Intersect each
  // joint's allowed progress interval before the existing safety limiter instead.
  const double bounded = std::clamp(progress, lower, upper);
  std::vector<double> command(start.size());
  for (std::size_t i = 0; i < start.size(); ++i)
    command[i] = start[i] + bounded*(goal[i]-start[i]);
  return command;
}

inline PregraspResult execute_pregrasp(
    MotorTransport& transport, const JointModel& model, SafetyControllerConfig config,
    const std::vector<double>& expected_start, const std::vector<double>& goal,
    double maximum_duration_s, const std::function<std::int64_t()>& now,
    const std::function<void()>& wait_cycle, const std::function<bool()>& cancelled,
    const std::function<void(const StepTrace&)>& record, double path_tolerance_rad = .01,
    const std::string& phase = "PREGRASP",
    const std::function<bool(std::int64_t)>& observe_hold = {},
    const std::function<BlockGraspObservation()>& observe_grasp = {},
    const std::optional<BlockGraspObservation>& initial_grasp = std::nullopt,
    const std::function<BlockHoldObservation()>& observe_lift = {},
    const std::optional<BlockHoldObservation>& initial_lift = std::nullopt) {
  PregraspResult result;
  if (!model.within_limits(expected_start) || !model.within_limits(goal) ||
      !std::isfinite(maximum_duration_s) || maximum_duration_s <= 0 || maximum_duration_s > 60) {
    throw std::invalid_argument("invalid bounded pregrasp plan");
  }
  if ((phase == "HOLD" && (!observe_hold || goal != expected_start || maximum_duration_s < 3.1)) ||
      (phase != "HOLD" && observe_hold))
    throw std::invalid_argument("observed HOLD requires a stationary checked plan and observation callback");
  if (phase == "CLOSE") {
    if (!observe_grasp || model.names().back() != "gripper" || goal.back() >= expected_start.back() ||
        !std::equal(goal.begin(), goal.end()-1, expected_start.begin()))
      throw std::invalid_argument("CLOSE requires observed gripper-only closure with fixed arm joints");
  } else if (observe_grasp || initial_grasp) throw std::invalid_argument("grasp callback is only valid for CLOSE");
  if (phase == "LIFT") {
    if (!observe_lift || !initial_lift || model.names().back() != "gripper" ||
        goal.back() != expected_start.back() || goal == expected_start || maximum_duration_s < 4.1)
      throw std::invalid_argument("LIFT requires observed grasp, fixed measured aperture and moving arm plan");
  } else if (observe_lift || initial_lift) throw std::invalid_argument("lift callback is only valid for LIFT");
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
  double duration_s = phase == "HOLD" ? 0. : 2.5;
  std::uint64_t sequence = 0;
  unsigned settled_samples = 0;
  bool armed = false;
  bool arming_attempted = false;
  bool sent_any = false;
  bool holding_started = false;
  ObservedGrasp grasp;
  ObservedBlockHold lifted_hold;
  std::optional<BlockHoldObservation> last_lift;
  std::vector<double> contact_stop;
  const auto consume_lift = [&](const BlockHoldObservation& observation, std::int64_t stamp) {
    if (!std::isfinite(observation.bottom_clearance_lower_bound_m))
      throw std::runtime_error("LIFT metric bottom clearance is unknown/nonfinite");
    if (last_lift && observation.sequence == last_lift->sequence &&
        (observation.bottom_clearance_lower_bound_m != last_lift->bottom_clearance_lower_bound_m ||
         observation.external_support != last_lift->external_support))
      throw std::runtime_error("LIFT observation changed within the same frame identity");
    grasp.update({observation.sequence, observation.captured_ns,
                  observation.bilateral_grasp_verified, observation.frame_sha256}, stamp);
    if (!observation.bilateral_grasp_verified)
      throw std::runtime_error("LIFT requires observed bilateral grasp before further motion");
    last_lift = observation;
  };
  try {
    // Keep the preflight evidence history across driver construction. Otherwise
    // a positive->negative transition could restart closure after grasp loss.
    if (initial_grasp) grasp.update(*initial_grasp, now());
    if (initial_lift) consume_lift(*initial_lift, now());
    for (;;) {
      if (cancelled()) throw std::runtime_error("operator interruption; retain last bounded goal");
      if (now() - started_ns > maximum_duration_s * 1e9)
        throw std::runtime_error("pregrasp deadline exceeded");
      const auto measured = transport.read();
      if (cancelled()) throw std::runtime_error("operator interruption after readback");
      auto current_ns = now();
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
        if (phase == "LIFT" && std::abs(initial.back()-expected_start.back()) > .001)
          throw std::runtime_error("LIFT measured grasp aperture changed since planning");
        // Keep the approved aperture constant in the command path even when
        // readback differs by a tick. Measured positions/result stay untouched.
        if (phase == "LIFT") initial.back() = expected_start.back();
        for (std::size_t i = 0; i < initial.size(); ++i) {
          if (std::abs(initial[i] - expected_start[i]) > 0.5 * std::acos(-1.) / 180.)
            throw std::runtime_error("measured start differs from checked path start");
          duration_s = std::max(duration_s, 1.875 * std::abs(goal[i] - initial[i]) /
              model.joints()[i].max_velocity * 1.15);
        }
        if (duration_s + (phase == "LIFT" ? 4.1 : 1.0) > maximum_duration_s)
          throw std::runtime_error("plan duration exceeds approved deadline");
      }
      if (phase == "CLOSE") {
        const auto observation = observe_grasp();
        current_ns = now();
        if (cancelled() || current_ns < started_ns || current_ns-started_ns > maximum_duration_s*1e9)
          throw std::runtime_error("CLOSE observation exceeded deadline or was interrupted");
        grasp.update(observation, current_ns);
        // A contact observation stops closure at measured aperture, not at the
        // fully-closed command. Separate fresh frames must then confirm grasp.
        if (grasp.positive_frames && contact_stop.empty()) contact_stop = positions;
      }
      if (phase == "LIFT") {
        const auto observation = observe_lift();
        current_ns = now();
        if (cancelled() || current_ns < started_ns || current_ns-started_ns > maximum_duration_s*1e9)
          throw std::runtime_error("LIFT observation exceeded deadline or was interrupted");
        consume_lift(observation, current_ns);
      }
      const double elapsed_s = (current_ns - started_ns) * 1e-9;
      const double u = duration_s > 0 ? std::clamp(elapsed_s / duration_s, 0., 1.) : 1.;
      // The analytic blend is in [0,1]; rounding near u=1 can exceed it by ulps.
      const double blend = std::clamp(u*u*u*(10. + u*(-15. + 6.*u)), 0., 1.);
      std::vector<double> requested(goal.size());
      for (std::size_t i = 0; i < goal.size(); ++i)
        requested[i] = initial[i] + blend*(goal[i] - initial[i]);
      const auto synchronized = contact_stop.empty() ?
          rate_limited_path_point(model, positions, initial, goal, blend, config.command_horizon_s) : contact_stop;
      if (!contact_stop.empty()) requested = contact_stop;
      const auto limited = model.limit(positions, synchronized, config.command_horizon_s);
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
      bool settled = !contact_stop.empty() || u == 1.;
      for (std::size_t i = 0; i < goal.size(); ++i)
        settled = settled && std::abs(positions[i] - (contact_stop.empty() ? goal[i] : contact_stop[i])) <= .001;
      settled_samples = settled ? settled_samples + 1 : 0;
      trace.phase = phase + (u < 1. ? "_TRAVEL" : "_SETTLING");
      if (!contact_stop.empty()) trace.phase = "GRASP_CONFIRM";
      if (phase == "LIFT" && holding_started) trace.phase = "HOLD_OBSERVING";
      record(trace);
      if (settled_samples >= 3) {
        if (phase == "CLOSE") {
          if (contact_stop.empty()) throw std::runtime_error("CLOSE endpoint without observed bilateral grasp");
          if (grasp.positive_frames < 2) { wait_cycle(); continue; }
          result.observed_grasp_verified = true;
          result.reached = true;
          result.phase = "GRASP_CONFIRMED_HOLDING";
          result.reason = "closure stopped at measured aperture; distinct frames confirmed bilateral grasp; lift unverified";
          return result;
        }
        if (phase == "LIFT") {
          const auto observed_ns = now();
          if (cancelled() || observed_ns-started_ns > maximum_duration_s*1e9)
            throw std::runtime_error("LIFT/HOLD dispatch exceeded deadline or was interrupted");
          if (const auto stop = safety.tick(measured, observed_ns))
            throw std::runtime_error(stop->reason);
          // Reuse the existing predicate. Travel time does not count as HOLD;
          // endpoint joints alone cannot prove unsupported object lift.
          const bool complete = lifted_hold.update(*last_lift, observed_ns);
          result.observed_lift_verified = true;
          holding_started = true;
          result.hold_entered_ns = lifted_hold.entered_ns;
          result.observed_hold_span_s = lifted_hold.first_capture_ns && lifted_hold.last_capture_ns ?
              (lifted_hold.last_capture_ns-lifted_hold.first_capture_ns)*1e-9 : 0.;
          if (!complete) { wait_cycle(); continue; }
          result.observed_hold_verified = true;
          result.reached = true;
          result.phase = "HOLD_REACHED_HOLDING";
          result.reason = "observed unsupported lift and continuous HOLD; supported ending remains separate";
          return result;
        }
        if (phase == "HOLD") {
          holding_started = true;
          const bool observed = observe_hold(current_ns);
          const auto observed_ns = now();
          if (cancelled() || observed_ns-started_ns > maximum_duration_s*1e9)
            throw std::runtime_error("HOLD observation exceeded deadline or was interrupted");
          if (const auto stop = safety.tick(measured, observed_ns))
            throw std::runtime_error(stop->reason);
          if (!observed) { wait_cycle(); continue; }
          result.observed_hold_verified = true;
        }
        result.reached = true;
        result.phase = phase+"_REACHED_HOLDING";
        result.reason = "measured joint endpoint reached; TCP validation and task acceptance remain separate";
        return result;
      }
      if (holding_started) throw std::runtime_error("joint endpoint lost during observed HOLD");
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
