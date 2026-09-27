#include "../tools/bounded_pregrasp.hpp"
#include <iostream>

using namespace dapier_so101_executor;
struct LaggedTransport : MotorTransport {
  std::int64_t& clock;
  std::vector<double> q{0.}, target{0.};
  std::vector<std::string> names{"joint"};
  bool enabled{false}, stalled{false}, stale{false}, fail_arming{false}, read_cancel{false};
  bool cancel{false};
  bool slow_send{false};
  std::int64_t send_delay_ns{};
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
    if (slow_send) clock += 150000000;
    clock += send_delay_ns;
    // No assignment to q: subsequent reads advance an independent lagged plant.
    std::vector<int> raw;
    for (double v:value) raw.push_back(static_cast<int>(std::lround(v*10000)));
    return {value, raw};
  }
};

int main() {
  const auto frame_id=[](std::int64_t sequence) {
    const auto digits=std::to_string(sequence);
    return std::string(64-digits.size(),'0')+digits;
  };
  const auto supported_ending = [&](LaggedTransport& plant, const JointModel& model,
      SafetyControllerConfig config, std::int64_t& clock, std::int64_t& sequence, int mode) {
    for (const std::string phase : {"PLACE", "RELEASE"}) {
      const bool release=phase=="RELEASE";
      const auto start=plant.q;
      auto goal=start;
      if (release) goal.back()+=.12; else goal[0]-=.06;
      const auto began=clock;
      const auto before_writes=plant.writes;
      plant.slow_send=mode==7 && release;
      bool support_seen=false;
      BlockSupportObservation initial{{++sequence,clock-1,release ? 0. : .035,true,release,frame_id(sequence)},release,false};
      if (mode==11) initial.block.external_support=true;
      if (mode==10) initial.block.captured_ns=clock-220000001;
      unsigned writes_before_rejected_observation=0;
      const auto result=execute_pregrasp(plant,model,config,start,goal,6.,
        [&]{return clock;},[&]{clock+=mode==10 ? 10000000 : 50000000;},[]{return false;},[&](const StepTrace& step) {
          if (step.sent.position_rad.empty()) return;
          if (!release && std::abs(step.sent.position_rad.back()-start.back())>1e-12)
            throw std::runtime_error("PLACE changed grasp aperture");
          if (release && std::abs(step.sent.position_rad.front()-start.front())>1e-12)
            throw std::runtime_error("RELEASE moved arm");
          support_seen |= step.phase=="SUPPORT_CONFIRM";
        },.01,phase,{}, {},std::nullopt,{},std::nullopt,[&] {
          const auto elapsed=clock-began;
          bool supported=release || elapsed>=800000000;
          bool released=release && elapsed>=1000000000;
          if (mode==1 && !release) supported=false;
          if ((mode==2 && !release && elapsed>=850000000) ||
              (mode==6 && release && elapsed>=500000000)) supported=false;
          if (mode==5 && release) released=false;
          BlockSupportObservation observation{{++sequence,clock,supported ? 0. : .035,
              !released,supported,frame_id(sequence)},supported,released};
          if (mode==3) observation.approved_support_verified=std::nullopt;
          if (mode==4) clock+=150000000;
          if (mode==8) {
            observation=initial;
            observation.approved_support_verified=true; observation.block.external_support=true;
          }
          if (mode==9) observation.block.captured_ns=clock-500000000;
          if (mode==10 && elapsed>=800000000) {
            // Robot feedback remains fresh (80ms); only object evidence expires.
            observation=initial;
            observation.block={++sequence,clock-220000000,0.,true,true,frame_id(sequence)};
            observation.approved_support_verified=true;
            plant.send_delay_ns=80000000;
          } else if (mode==10) {
            observation.block.captured_ns-=220000000;
          }
          if (mode==12 && elapsed>=500000000) {
            observation.block.external_support=true;
            observation.approved_support_verified=false;
            writes_before_rejected_observation=plant.writes;
          }
          return observation;
        },initial);
      const bool should_pass=mode==0 || (!release && (mode==5 || mode==6 || mode==7));
      if (result.reached!=should_pass || result.task_success) return false;
      if (!should_pass) {
        const std::vector<std::string> reasons={"", "without observed approved support", "support lost",
          "unknown", "measured robot state is stale", "without distinct observed object release", "support lost",
          "measured-state watchdog expired", "identity/order", "fresh frame", "object observation expired",
          "unapproved external support", "unapproved external support"};
        if (result.reason.find(reasons[mode])==std::string::npos) {
          std::cerr<<mode<<" "<<result.reason<<'\n'; return false;
        }
        if ((mode==3 || mode==4 || mode==8 || mode==9 || mode==11) && plant.writes!=before_writes) return false;
        if (mode==12 && plant.writes!=writes_before_rejected_observation) return false;
        return true;
      }
      if (!result.observed_support_verified || result.observed_release_verified!=release ||
          (!release && !support_seen) || !plant.enabled) return false;
    }
    return true;
  };
  // Scripted independent observations drive the same CLOSE dispatch/feedback
  // loop. Contact precedes full closure; measured positions still lag commands.
  for (int mode=0; mode<9; ++mode) {
    std::int64_t clock=1000000000, sequence=0;
    if (mode==7) sequence=1;
    LaggedTransport plant(clock);
    plant.names={"gripper"}; plant.q=plant.target={1.};
    const JointModel model({{"gripper",1,0.,2.,.3}});
    SafetyControllerConfig config; config.joints={{"gripper",0.,2.,.3}};
    bool confirm_seen=false, lag_seen=false;
    double frozen=-1.;
    const auto result=execute_pregrasp(plant,model,config,{1.},{.8},8.,
      [&]{return clock;},[&]{clock+=50000000;},[]{return false;},[&](const StepTrace& step) {
        if (step.sent.position_rad.empty()) return;
        lag_seen |= std::abs(step.sent.position_rad[0]-step.measured_rad[0])>1e-5;
        if (std::abs(step.sent.position_rad[0]-step.measured_rad[0])>.01500000001)
          throw std::runtime_error("CLOSE exceeded velocity bound");
        if (step.phase=="GRASP_CONFIRM") {
          confirm_seen=true;
          if (frozen<0.) frozen=step.sent.position_rad[0];
          if (std::abs(step.sent.position_rad[0]-frozen)>1e-12)
            throw std::runtime_error("closure continued during GRASP_CONFIRM");
        }
      },.01,"CLOSE",{},[&] {
        ++sequence;
        const bool contact=clock>=1800000000 && mode!=6;
        BlockGraspObservation observation{sequence,clock,contact,frame_id(sequence)};
        if (mode==1) observation.bilateral_grasp_verified=std::nullopt;
        if (mode==2) observation.captured_ns=clock-500000000;
        if (mode==3 && clock>=1850000000) observation.bilateral_grasp_verified=false;
        if (mode==4 && contact) observation.frame_sha256=frame_id(1);
        if (mode==5 && contact) clock+=150000000; // Fresh object but robot feedback expired.
        if (mode==8) observation={1,1000000000,clock>1000000000,frame_id(1)};
        return observation;
      },mode==7 ? std::optional<BlockGraspObservation>({1,999999999,true,frame_id(0)}) : std::nullopt);
    if (result.reached!=(mode==0) || result.observed_grasp_verified!=(mode==0) || result.task_success)
      return 15;
    if (mode==0 && (!confirm_seen || !lag_seen || result.phase!="GRASP_CONFIRMED_HOLDING" ||
                   result.final_measured_rad[0]<=.81)) return 16;
    if ((mode==1 || mode==2 || mode==7 || mode==8) && plant.writes) return 17;
    const std::vector<std::string> reasons={"", "unknown", "fresh frame", "lost bilateral",
        "raw frame reused", "measured robot state is stale", "endpoint without", "lost bilateral", "identity/order"};
    if (mode && result.reason.find(reasons[mode])==std::string::npos) {
      std::cerr<<mode<<' '<<result.reason<<'\n'; return 18;
    }
    if (mode==3 && !confirm_seen) return 19;
  }
  // Preserve the same independent plant from CLOSE into LIFT/HOLD. No reset of
  // measured positions to the next command, and travel cannot count as HOLD.
  for (int mode=0; mode<9; ++mode) {
    std::int64_t clock=1000000000, sequence=0;
    LaggedTransport plant(clock);
    plant.names={"arm","gripper"}; plant.q=plant.target={0.,1.};
    const JointModel model({{"arm",1,-1.,1.,.3},{"gripper",2,0.,2.,.3}});
    SafetyControllerConfig config; config.joints={{"arm",-1.,1.,.3},{"gripper",0.,2.,.3}};
    const auto closed=execute_pregrasp(plant,model,config,{0.,1.},{0.,.8},8.,
      [&]{return clock;},[&]{clock+=50000000;},[]{return false;},[](const StepTrace&){},
      .01,"CLOSE",{},[&]{return BlockGraspObservation{++sequence,clock,clock>=1800000000,frame_id(sequence)};});
    if (!closed.observed_grasp_verified) return 20;
    const auto start=closed.final_measured_rad;
    const std::vector<double> goal{.06,start.back()};
    const auto lift_started=clock;
    const auto before_writes=plant.writes;
    if (mode==6) plant.q.back()-=.002;
    if (mode==8) plant.q.back()+=.0005;
    std::int64_t hold_seen=0;
    const auto lifted=execute_pregrasp(plant,model,config,start,goal,8.,
      [&]{return clock;},[&]{clock+=50000000;},[]{return false;},[&](const StepTrace& step) {
        if (step.sent.position_rad.empty()) return;
        if (std::abs(step.sent.position_rad.back()-start.back())>1e-12)
          throw std::runtime_error("LIFT changed grasp aperture");
        if (step.phase=="HOLD_OBSERVING" && !hold_seen) hold_seen=clock;
      },.01,"LIFT",{}, {},std::nullopt,[&] {
        const auto elapsed=clock-lift_started;
        BlockHoldObservation sample{++sequence,clock,elapsed>=1000000000 ? .035 : 0.,true,
                                    elapsed<1000000000,frame_id(sequence)};
        if (mode==1 && elapsed>=500000000) sample.bilateral_grasp_verified=false;
        if (mode==2) sample.captured_ns=clock-500000000;
        if (mode==3) sample.external_support=true;
        if (mode==4) sample.bottom_clearance_lower_bound_m=.01;
        if (mode==5) clock+=150000000;
        if (mode==7 && hold_seen && clock-hold_seen>=1000000000) sample.external_support=true;
        return sample;
      },BlockHoldObservation{++sequence,clock-1,0.,true,true,frame_id(sequence)});
    const bool passed=mode==0 || mode==8;
    if (lifted.reached!=passed || lifted.observed_hold_verified!=passed || lifted.task_success)
      return 21;
    if (passed && (!lifted.observed_lift_verified || lifted.phase!="HOLD_REACHED_HOLDING" ||
        lifted.observed_hold_span_s<3. || lifted.hold_entered_ns-lift_started<2500000000LL || !hold_seen))
      return 22;
    if ((mode==2 || mode==5 || mode==6) && plant.writes!=before_writes) return 23;
    if (mode==7 && (!hold_seen || !lifted.observed_lift_verified)) return 24;
    if (mode==0 && !supported_ending(plant,model,config,clock,sequence,0)) return 25;
  }
  for (int mode=1; mode<13; ++mode) {
    std::int64_t clock=1000000000, sequence=0;
    LaggedTransport plant(clock); plant.names={"arm","gripper"}; plant.q=plant.target={.06,.95};
    const JointModel model({{"arm",1,-1.,1.,.3},{"gripper",2,0.,2.,.3}});
    SafetyControllerConfig config; config.joints={{"arm",-1.,1.,.3},{"gripper",0.,2.,.3}};
    if (!supported_ending(plant,model,config,clock,sequence,mode)) {
      std::cerr<<"supported ending mode "<<mode<<'\n'; return 26;
    }
  }
  ObservedBlockHold hold;
  BlockHoldObservation obs{1,1000000000,.030,true,false,frame_id(1)};
  if (hold.update(obs,obs.captured_ns)) return 11;
  for (int i=1; i<=30; ++i) {
    ++obs.sequence; obs.captured_ns+=100000000;
    obs.frame_sha256=frame_id(obs.sequence);
    if (hold.update(obs,obs.captured_ns) != (i==30)) return 12;
  }
  // A static frame, frame retimestamp, support or unknown/nonfinite lift cannot
  // establish continuous object HOLD even when every motor is motionless.
  for (int mode=0; mode<6; ++mode) {
    ObservedBlockHold invalid;
    BlockHoldObservation a{1,1000000000,.030,true,false,frame_id(1)};
    invalid.update(a,a.captured_ns);
    auto now=a.captured_ns+100000000;
    if (mode==0) now+=250000001;
    if (mode==1) a.captured_ns+=100000000;
    if (mode==2) a.external_support=true;
    if (mode==3) a.bottom_clearance_lower_bound_m=std::nan("");
    if (mode==4) a.bilateral_grasp_verified=false;
    if (mode==5) { ++a.sequence; a.captured_ns+=100000000; }
    bool refused=false;
    try { invalid.update(a,now); } catch (const std::runtime_error&) { refused=true; }
    if (!refused) return 13;
  }
  for (int mode=0; mode<4; ++mode) {
    std::int64_t hold_time=1000000000;
    LaggedTransport plant(hold_time);
    const JointModel single({{"joint",1,-1.,1.,.3}});
    SafetyControllerConfig config; config.joints={{"joint",-1.,1.,.3}};
    ObservedBlockHold observed;
    std::int64_t sequence=0, first_frame=0;
    const auto result=execute_pregrasp(plant,single,config,{0.},{0.},4.,
      [&]{return hold_time;},[&]{hold_time+=50000000;},[]{return false;},[](const StepTrace&){},
      .01,"HOLD",[&](std::int64_t now) {
        if (!first_frame) first_frame=now;
        BlockHoldObservation sample{++sequence,mode==2 ? first_frame : now,.030,true,
                                    mode==1 && now-first_frame>=1000000000,frame_id(sequence)};
        const bool completed=observed.update(sample,now);
        if (mode==3 && completed) hold_time+=150000000; // Fresh object, stale robot feedback.
        return completed;
      });
    if (result.reached != (mode==0) || result.observed_hold_verified != (mode==0) ||
        result.task_success || (mode==0 && observed.last_capture_ns-observed.first_capture_ns<3000000000LL))
      return 14;
  }
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
