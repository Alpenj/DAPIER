#include "../tools/bounded_pregrasp.hpp"
#include <iostream>

using namespace dapier_so101_executor;
struct LaggedTransport : MotorTransport {
  std::int64_t& clock;
  std::vector<double> q{0.}, target{0.};
  bool enabled{false}, stalled{false}, stale{false}, fail_arming{false}, read_cancel{false};
  bool cancel{false};
  unsigned writes{};
  explicit LaggedTransport(std::int64_t& value) : clock(value) {}
  MeasuredRobotState read() override {
    if (read_cancel) cancel = true;
    if (enabled && !stalled) q[0] += .25 * (target[0] - q[0]);
    MeasuredRobotState state;
    state.received_monotonic_ns = stale ? 0 : clock;
    state.joint_names = {"joint"}; state.joint_position_rad = q;
    state.base_settled = state.collision_clear = true;
    return state;
  }
  void arm_at_measured_position() override {
    enabled = true;
    if (fail_arming) throw std::runtime_error("simulated missing torque ACK");
  }
  SentPositions send(const std::vector<double>& value) override {
    target = value; ++writes;
    // No assignment to q: subsequent reads advance an independent lagged plant.
    return {value, {static_cast<int>(std::lround(value[0] * 10000))}};
  }
};

int main() {
  for (bool inverted : {false, true}) {
    dapier_so101_core::CalibrationEntry cal{"joint",1,0,0,4095,-std::acos(-1.),std::acos(-1.),inverted};
    const double current = cal.raw_to_position(2048);
    const double target = current+.015;
    const int raw = bounded_raw_tick(cal,target,current-.015,target);
    if (cal.raw_to_position(raw) > target || std::abs(raw-2048) != 9) return 5;
    const double limit = current+.001;
    if (cal.raw_to_position(bounded_raw_tick(cal,limit,current-.015,limit)) > limit) return 6;
  }
  const JointModel model({{"joint", 1, -1., 1., .3}});
  SafetyControllerConfig config;
  config.joints = {{"joint", -1., 1., .3}};
  for (int scenario = 0; scenario < 6; ++scenario) {
    std::int64_t time = 1000000000;
    LaggedTransport transport(time);
    transport.stalled = scenario == 1;
    transport.stale = scenario == 2;
    transport.read_cancel = scenario == 4;
    transport.fail_arming = scenario == 5;
    bool saw_feedback_lag = false, saw_limiting = false;
    const auto result = execute_pregrasp(transport, model, config, {0.}, {.1}, 6.,
      [&] { return time; }, [&] { time += 50000000; }, [&] { return scenario == 3 || transport.cancel; },
      [&](const StepTrace& trace) {
        if (!trace.sent.position_rad.empty()) {
          saw_feedback_lag |= std::abs(trace.measured_rad[0] - trace.sent.position_rad[0]) > .0001;
          saw_limiting |= std::abs(trace.requested_rad[0] - trace.limited_rad[0]) > .0001;
        }
      });
    if (result.task_success || (scenario == 0 && (!result.reached || !saw_feedback_lag)) ||
        (scenario != 0 && result.reached) || (scenario >= 2 && transport.writes != 0) ||
        (scenario == 1 && !saw_limiting)) return 1;
    if (scenario == 4 && transport.enabled) return 2;
    if (scenario == 5 && result.phase != "ARMING_INTERRUPTED_STATE_UNCERTAIN") return 3;
    std::cout << scenario << ' ' << result.phase << ' ' << result.reason << '\n';
  }
  // One axis has progressed while the other stalled: not on the checked line.
  if (within_path_envelope({.08, 0.}, {0., 0.}, {.1, .1}, .01) ||
      !within_path_envelope({.05, .05}, {0., 0.}, {.1, .1}, .01)) return 4;
}
