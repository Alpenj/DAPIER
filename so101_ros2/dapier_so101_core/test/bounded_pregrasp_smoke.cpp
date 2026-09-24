#include "../tools/bounded_pregrasp.hpp"
#include <iostream>

using namespace dapier_so101_executor;
struct LaggedTransport : MotorTransport {
  std::int64_t& clock;
  std::vector<double> q{0.}, target{0.};
  std::vector<std::string> names{"joint"};
  bool enabled{false}, stalled{false}, stale{false}, fail_arming{false}, read_cancel{false};
  bool cancel{false};
  unsigned writes{};
  explicit LaggedTransport(std::int64_t& value) : clock(value) {}
  MeasuredRobotState read() override {
    if (read_cancel) cancel = true;
    if (enabled && !stalled)
      for (std::size_t i=0; i<q.size(); ++i) q[i] += .25 * (target[i] - q[i]);
    MeasuredRobotState state;
    state.received_monotonic_ns = stale ? 0 : clock;
    state.joint_names = names; state.joint_position_rad = q;
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
    std::vector<int> raw;
    for (double v:value) raw.push_back(static_cast<int>(std::lround(v*10000)));
    return {value, raw};
  }
};

int main() {
  // Different joint travel must share a progress fraction after velocity limits.
  // Per-axis clipping of [.2,.4] from [.1,.2] gives [.115,.215], off the line.
  const JointModel pair({{"a",1,-3.,3.,.3},{"b",2,-3.,3.,.3}});
  const auto synchronized = rate_limited_path_point(pair,{.1,.2},{0.,0.},{1.,2.},.2,.05);
  if (!within_path_envelope(synchronized,{0.,0.},{1.,2.},1e-12) ||
      std::abs(synchronized[0]-.1075)>1e-12 || std::abs(synchronized[1]-.215)>1e-12)
    return 7;
  const auto backwards = rate_limited_path_point(pair,{0.,.5},{.3,1.},{-.3,0.},.9,.05);
  if (!within_path_envelope(backwards,{.3,1.},{-.3,0.},1e-12) ||
      std::abs(backwards[0])>.015 || std::abs(backwards[1]-.5)>.015000000001)
    return 8;
  bool no_progress_rejected=false;
  try { rate_limited_path_point(pair,{.1,.5},{0.,0.},{1.,2.},.2,.05); }
  catch (const std::runtime_error&) { no_progress_rejected=true; }
  if (!no_progress_rejected) return 9;
  // Actual executor regression: unequal travel + independent measured lag.
  // Jump near u=1 also exercises quintic roundoff (unclamped blend exceeds1).
  std::int64_t pair_time=1000000000;
  LaggedTransport pair_transport(pair_time);
  pair_transport.names=pair.names(); pair_transport.q=pair_transport.target={0.,0.};
  SafetyControllerConfig pair_config;
  pair_config.joints={{"a",-3.,3.,.3},{"b",-3.,3.,.3}};
  bool limited_pair=false, lagged_pair=false;
  const auto pair_result=execute_pregrasp(pair_transport,pair,pair_config,{0.,0.},{.1,.2},12.,
    [&] { return pair_time; }, [&] {
      pair_time += pair_time==1000000000 ? 2499997500LL : 50000000LL;
    }, [] { return false; }, [&](const StepTrace& trace) {
      if (trace.sent.position_rad.empty()) return;
      if (!within_path_envelope(trace.sent.position_rad,{0.,0.},{.1,.2},1e-12))
        throw std::runtime_error("sent pair command left checked line");
      for (std::size_t i=0; i<2; ++i) {
        if (std::abs(trace.sent.position_rad[i]-trace.measured_rad[i])>.015000000001)
          throw std::runtime_error("pair command exceeds velocity interval");
        limited_pair |= std::abs(trace.requested_rad[i]-trace.limited_rad[i])>1e-4;
        lagged_pair |= std::abs(trace.sent.position_rad[i]-trace.measured_rad[i])>1e-4;
      }
    });
  if (!pair_result.reached || pair_result.task_success || !limited_pair || !lagged_pair) {
    std::cerr<<pair_result.reason<<'\n'; return 10;
  }
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
