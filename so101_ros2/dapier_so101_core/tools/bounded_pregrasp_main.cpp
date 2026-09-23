#include "feetech_pregrasp_transport.hpp"
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using json = nlohmann::json;
using namespace dapier_so101_executor;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
std::string bytes(const std::string& path, bool private_file) {
  const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) throw std::runtime_error("cannot open input file");
  struct stat st{};
  if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != getuid() ||
      st.st_size < 1 || st.st_size > 1048576 || (private_file && (st.st_mode & 077))) {
    close(fd); throw std::runtime_error("input ownership, permissions or size invalid");
  }
  std::string result;
  char buffer[4096]; ssize_t count;
  while ((count = ::read(fd, buffer, sizeof(buffer))) > 0) result.append(buffer, count);
  close(fd);
  if (count < 0) throw std::runtime_error("input read failed");
  return result;
}
std::string sha256(const std::string& value) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(value.data()), value.size(), digest);
  std::ostringstream stream;
  for (auto byte : digest) stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return stream.str();
}
double finite_number(const json& value) {
  if (!value.is_number()) throw std::runtime_error("numeric field required");
  const double number = value.get<double>();
  if (!std::isfinite(number)) throw std::runtime_error("nonfinite numeric field");
  return number;
}
void write_line(int fd, const json& value) {
  const auto text = value.dump() + "\n";
  std::size_t offset = 0;
  while (offset < text.size()) {
    const auto n = ::write(fd, text.data()+offset, text.size()-offset);
    if (n <= 0) throw std::runtime_error("trace write failed");
    offset += n;
  }
}

class LaggedMock final : public MotorTransport {
 public:
  LaggedMock(std::vector<std::string> names, std::vector<double> initial, bool stalled)
      : names_(std::move(names)), measured_(initial), target_(initial), stalled_(stalled) {}
  MeasuredRobotState read() override {
    const auto timestamp = monotonic_ns();
    const double dt = (timestamp - previous_) * 1e-9;
    if (armed_ && !stalled_) {
      const double gain = 1.-std::exp(-std::max(0., dt)/.12);
      for (std::size_t i = 0; i < measured_.size(); ++i)
        measured_[i] += gain*(target_[i]-measured_[i]);
    }
    previous_ = timestamp;
    MeasuredRobotState state;
    state.received_monotonic_ns = timestamp;
    state.joint_names = names_; state.joint_position_rad = measured_;
    state.base_settled = state.collision_clear = true;
    return state;
  }
  void arm_at_measured_position() override { armed_ = true; }
  SentPositions send(const std::vector<double>& value) override {
    target_ = value; return {value, {}};  // No assignment to measured_.
  }
 private:
  std::vector<std::string> names_;
  std::vector<double> measured_, target_;
  bool stalled_, armed_{false};
  std::int64_t previous_{monotonic_ns()};
};
}

int main(int argc, char** argv) {
  int output_fd = -1;
  json report{{"schema_version", "dapier.bounded-pregrasp-result.v1"},
      {"hardware_execution", false}, {"task_success", false}, {"motion_started", false}};
  try {
    std::map<std::string, std::string> args;
    bool present = false;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--operator-present") { present = true; continue; }
      if (i+1 >= argc || !args.emplace(key, argv[++i]).second)
        throw std::runtime_error("invalid or duplicate CLI option");
    }
    const bool hardware = args.at("--transport") == "hardware";
    if (!hardware && args.at("--transport") != "mock") throw std::runtime_error("transport must be mock or hardware");
    if (hardware && (!present || args["--confirm"] != "VISIBLE_LEFT_SENSOR_PREGRASP" || !isatty(0) || !isatty(1)))
      throw std::runtime_error("attended hardware confirmation required before any device access");
    output_fd = open(args.at("--output").c_str(), O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW, 0600);
    if (output_fd < 0) throw std::runtime_error("new exclusive output path required");
    const auto profile_raw = bytes(args.at("--profile"), true);
    const auto plan_raw = bytes(args.at("--plan"), false);
    const auto profile = json::parse(profile_raw), plan = json::parse(plan_raw);
    report["profile_sha256"] = sha256(profile_raw); report["plan_sha256"] = sha256(plan_raw);
    if (profile.at("schema_version") != "dapier.left-pregrasp-profile.v1" ||
        profile.at("device_id") != "dapier_dual_follower_left" ||
        plan.at("schema_version") != "dapier.bounded-pregrasp-plan.v1" ||
        plan.at("profile_sha256") != sha256(profile_raw)) throw std::runtime_error("plan/profile identity mismatch");
    if (hardware && (profile.at("physically_verified") != true || plan.at("sensor_target_verified") != true ||
        plan.at("offline_candidate_accepted") != true || plan.at("path_clear") != true))
      throw std::runtime_error("physical mapping, sensor target and checked path are required");
    const double path_tolerance = finite_number(plan.at("path_tracking_tolerance_rad"));
    if (path_tolerance <= 0 || path_tolerance > .01)
      throw std::runtime_error("invalid checked path tracking envelope");
    if (hardware) {
      const double unix_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      for (const char* field : {"observation_unix_s", "measured_state_unix_s"}) {
        const double age = unix_s - finite_number(plan.at(field));
        if (age < 0 || age > 60.) throw std::runtime_error("stale or future physical plan input");
      }
      if (finite_number(plan.at("position_error_m")) < 0 || finite_number(plan.at("position_error_m")) > .0005 ||
          finite_number(plan.at("axis_error_rad")) < 0 || finite_number(plan.at("axis_error_rad")) > 2.*std::acos(-1.)/180. ||
          finite_number(plan.at("path_clearance_m")) < .030)
        throw std::runtime_error("Cartesian/axis/path acceptance failed");
      if (plan.at("path_envelope_verified") != true ||
          finite_number(plan.at("path_envelope_clearance_m")) < .030)
        throw std::runtime_error("tracking envelope lacks conservative clearance verification");
    }
    const auto calibration_raw = bytes(profile.at("calibration_path"), false);
    if (sha256(calibration_raw) != profile.at("calibration_sha256").get<std::string>()) throw std::runtime_error("calibration SHA mismatch");
    const auto calibration = json::parse(calibration_raw);
    std::vector<dapier_so101_core::JointSpec> joints;
    std::vector<dapier_so101_core::CalibrationEntry> entries;
    std::vector<double> velocities;
    SafetyControllerConfig safety;
    for (const auto& spec : profile.at("joints")) {
      const std::string name = spec.at("name");
      const auto& cal = calibration.at(name);
      const double minimum = finite_number(spec.at("minimum_rad"));
      const double maximum = finite_number(spec.at("maximum_rad"));
      const double velocity = finite_number(spec.at("maximum_velocity_rad_s"));
      if (velocity <= 0 || velocity > .3) throw std::runtime_error("velocity exceeds bounded policy cap");
      const int low = cal.at("range_min"), high = cal.at("range_max");
      double pmin, pmax; bool inverted = false;
      if (name == "gripper") {
        pmin = finite_number(profile.at("gripper_rad_limits").at(0));
        pmax = finite_number(profile.at("gripper_rad_limits").at(1));
      } else {
        const double sign = finite_number(spec.at("sign"));
        if (sign != 1 && sign != -1) throw std::runtime_error("joint sign must be +/-1");
        const double offset = finite_number(spec.at("zero_offset_deg"));
        const double half = (high-low)*180./4095.;
        pmin = (offset-half)*std::acos(-1.)/180.; pmax = (offset+half)*std::acos(-1.)/180.;
        inverted = sign < 0;
      }
      if (low < 0 || high > 4095 || low >= high || pmin >= pmax || minimum < pmin || maximum > pmax)
        throw std::runtime_error("joint limits exceed calibration mapping");
      const int id = cal.at("id");
      joints.push_back({name, id, minimum, maximum, velocity});
      entries.push_back({name, id, cal.at("homing_offset"), low, high, pmin, pmax, inverted});
      velocities.push_back(velocity);
      safety.joints.push_back({name, minimum, maximum, velocity});
    }
    const JointModel model(joints);
    if (model.size() != 6 || model.names() != std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"})
      throw std::runtime_error("follower joint order mismatch");
    const auto start = plan.at("start_rad").get<std::vector<double>>();
    const auto goal = plan.at("goal_rad").get<std::vector<double>>();
    if (!model.within_limits(start) || !model.within_limits(goal)) throw std::runtime_error("plan outside model limits");
    const auto& proposal = plan.at("goal_intent");
    if (proposal.at("kind") != "arm_joint_position") throw std::runtime_error("arm goal intent required");
    for (const char* key : {"sequence", "source_monotonic_ns", "ttl_ns"})
      if (!proposal.at(key).is_number_integer()) throw std::runtime_error("intent timestamp/sequence must be integer");
    dapier_so101_core::ResearchControlIntent goal_intent;
    goal_intent.schema_version = proposal.at("schema_version");
    goal_intent.sequence = proposal.at("sequence");
    goal_intent.kind = dapier_so101_core::ResearchIntentKind::kArmJointPosition;
    goal_intent.source = proposal.at("source");
    goal_intent.source_monotonic_ns = proposal.at("source_monotonic_ns");
    goal_intent.ttl_ns = proposal.at("ttl_ns");
    goal_intent.joint_names = proposal.at("joint_names").get<std::vector<std::string>>();
    goal_intent.joint_position_rad = proposal.at("joint_position_rad").get<std::vector<double>>();
    goal_intent.joint_max_velocity_rad_s = proposal.at("joint_max_velocity_rad_s").get<std::vector<double>>();
    goal_intent.base_linear_x_mps = finite_number(proposal.at("base_linear_x_mps"));
    goal_intent.base_angular_z_rad_s = finite_number(proposal.at("base_angular_z_rad_s"));
    const auto checked_goal = dapier_so101_core::validate_research_control_intent(goal_intent, monotonic_ns());
    if (!checked_goal.accepted || goal_intent.joint_names != model.names() || goal_intent.joint_position_rad != goal)
      throw std::runtime_error("goal intent is invalid or differs from checked plan");
    for (std::size_t i = 0; i < velocities.size(); ++i)
      if (goal_intent.joint_max_velocity_rad_s[i] > velocities[i])
        throw std::runtime_error("goal intent exceeds profile velocity");
    report["accepted_goal_intent"] = proposal;
    const double duration = finite_number(plan.at("maximum_duration_s"));
    if (duration <= 0 || duration > 60) throw std::runtime_error("invalid bounded duration");
    const std::string phase = plan.value("phase", "PREGRASP");
    const bool already_holding = plan.value("initial_torque_enabled", false);
    if ((phase != "PREGRASP" && phase != "WRIST_ALIGN") || already_holding != (phase == "WRIST_ALIGN"))
      throw std::runtime_error("phase and expected initial torque state mismatch");
    std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
    std::unique_ptr<MotorTransport> transport;
    if (hardware) {
      report["hardware_access_attempted"] = true;
      const auto deadline_ns = monotonic_ns() + static_cast<std::int64_t>(duration*1e9);
      transport = std::make_unique<FeetechPregraspTransport>(profile.at("port"), profile.at("controller_serial"),
          entries, velocities, args.at("--confirm"), present, [deadline_ns] {
            if (interrupted || monotonic_ns() > deadline_ns)
              throw std::runtime_error("cancel/deadline before native I/O; no further calls permitted");
          }, model, start, goal, path_tolerance, safety.command_horizon_s, already_holding);
      report["hardware_execution"] = true;
    } else transport = std::make_unique<LaggedMock>(model.names(), start, plan.value("mock_stalled", false));
    const auto result = execute_pregrasp(*transport, model, safety, start, goal, duration,
      monotonic_ns, [] { std::this_thread::sleep_for(std::chrono::milliseconds(50)); },
      [] { return interrupted != 0; }, [&](const StepTrace& trace) {
        if (!trace.sent.position_rad.empty()) report["motion_started"] = true;
        write_line(output_fd, {{"event", "step"}, {"time_ns", trace.time_ns}, {"phase", trace.phase},
          {"requested_rad", trace.requested_rad}, {"limited_rad", trace.limited_rad},
          {"sent_rad", trace.sent.position_rad}, {"sent_raw_ticks", trace.sent.raw_ticks},
          {"measured_before_command_rad", trace.measured_rad}, {"reason", trace.reason}});
      }, path_tolerance, phase);
    report["event"] = "result"; report["phase"] = result.phase; report["reason"] = result.reason;
    report["reached_joint_endpoint"] = result.reached; report["final_measured_rad"] = result.final_measured_rad;
    report["cartesian_endpoint_verified"] = false;
    report["ending"] = "retain_torque_and_last_bounded_goal; supported ending is a separate approved phase";
    if (auto* real = dynamic_cast<FeetechPregraspTransport*>(transport.get())) {
      report["motor_writes"] = {{"Goal_Position", real->goal_position_writes}, {"Goal_Velocity", real->goal_velocity_writes},
          {"Torque_Enable_1", real->torque_enable_writes}, {"Torque_Enable_0", 0}, {"protection_or_calibration", 0}};
      report["motor_write_attempts"] = {{"Goal_Position", real->goal_position_attempts},
          {"Goal_Velocity", real->goal_velocity_attempts}, {"Torque_Enable_1", real->torque_enable_attempts}};
      report["write_count_semantics"] = "SDK calls and acknowledged calls; SDK retries may send multiple packets";
      report["motor_state_may_have_changed"] = real->goal_position_attempts || real->goal_velocity_attempts || real->torque_enable_attempts;
      report["io_stop_limit"] = "cancel checked between SDK calls; blocked OS/SDK call needs outer process deadline and attending operator";
    }
    write_line(output_fd, report); close(output_fd);
    std::cout << report.dump() << '\n';
    return result.reached ? 0 : 1;
  } catch (const std::exception& error) {
    report["event"] = "result"; report["phase"] = "REJECTED"; report["reason"] = error.what();
    if (output_fd >= 0) { try { write_line(output_fd, report); } catch (...) {} close(output_fd); }
    std::cerr << report.dump() << '\n'; return 1;
  }
}
