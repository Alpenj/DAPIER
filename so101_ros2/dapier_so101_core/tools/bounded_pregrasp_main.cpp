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
#include <tuple>

using json = nlohmann::json;
using namespace dapier_so101_executor;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
std::string bytes(const std::string& path, bool private_file, std::size_t maximum_bytes = 1048576) {
  const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) throw std::runtime_error("cannot open input file");
  struct stat st{};
  if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != getuid() ||
      st.st_size < 1 || static_cast<std::uint64_t>(st.st_size) > maximum_bytes || (private_file && (st.st_mode & 077))) {
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

json read_bound_block_observation(const json& binding, bool hardware, int output_fd,
    const std::string& schema, const std::string& event, std::set<std::string>& history) {
  const auto raw = bytes(binding.at("path"), false);
  const auto value = json::parse(raw);
  if (value.at("schema_version") != schema ||
      value.at("source_kind") != (hardware ? "hardware" : "mock") ||
      value.at("boot_id") != binding.at("boot_id"))
    throw std::runtime_error("HOLD observation source/clock identity mismatch");
  for (const char* key : {"run_id", "object_id", "calibration_revision", "producer_sha256"}) {
    if (!binding.at(key).is_string() || binding.at(key).get<std::string>().empty() ||
        value.at(key) != binding.at(key))
      throw std::runtime_error("HOLD observation does not belong to the approved task/source");
  }
  const auto frame_raw = bytes(value.at("frame_path"), false, 16777216);
  if (sha256(frame_raw) != value.at("frame_sha256").get<std::string>())
    throw std::runtime_error("HOLD raw frame SHA mismatch");
  history.insert(value.at("frame_sha256").get<std::string>());
  for (const char* key : {"sequence", "captured_monotonic_ns"})
    if (!value.at(key).is_number_integer()) throw std::runtime_error("HOLD integer sequence/time required");
  // Preserve even unknown/negative observations before a phase gate rejects them.
  write_line(output_fd, {{"event", event}, {"received_monotonic_ns", monotonic_ns()},
      {"observation_sha256", sha256(raw)}, {"observation", value}});
  return value;
}

BlockHoldObservation read_hold_observation(const json& binding, bool hardware, int output_fd, std::set<std::string>& history,
    const std::string& event = "hold_observation") {
  const auto value = read_bound_block_observation(binding, hardware, output_fd,
      "dapier.block-hold-observation.v1", event, history);
  for (const char* key : {"bilateral_grasp_verified", "external_support"})
    if (!value.at(key).is_boolean()) throw std::runtime_error("HOLD boolean evidence required");
  BlockHoldObservation observation{value.at("sequence"), value.at("captured_monotonic_ns"),
      finite_number(value.at("bottom_clearance_lower_bound_m")),
      value.at("bilateral_grasp_verified"), value.at("external_support"), value.at("frame_sha256")};
  return observation;
}

BlockGraspObservation read_grasp_observation(const json& binding, bool hardware, int output_fd, std::set<std::string>& history) {
  const auto value = read_bound_block_observation(binding, hardware, output_fd,
      "dapier.block-grasp-observation.v1", "grasp_observation", history);
  const auto& grasp = value.at("bilateral_grasp_verified");
  if (!grasp.is_null() && !grasp.is_boolean())
    throw std::runtime_error("CLOSE grasp evidence must be boolean or unknown/null");
  return {value.at("sequence"), value.at("captured_monotonic_ns"),
      grasp.is_null() ? std::nullopt : std::optional<bool>(grasp.get<bool>()), value.at("frame_sha256")};
}

BlockSupportObservation read_support_observation(const json& binding, bool hardware, int output_fd,
    std::set<std::string>& history) {
  const auto value = read_bound_block_observation(binding, hardware, output_fd,
      "dapier.block-support-observation.v1", "support_observation", history);
  if (!binding.at("support_id").is_string() || binding.at("support_id").get<std::string>().empty() ||
      value.at("support_id") != binding.at("support_id"))
    throw std::runtime_error("observed support differs from approved support destination");
  const auto optional_bool = [&](const char* key) -> std::optional<bool> {
    const auto& field = value.at(key);
    if (field.is_null()) return std::nullopt;
    if (!field.is_boolean()) throw std::runtime_error("support observation boolean or null required");
    return field.get<bool>();
  };
  for (const char* key : {"bilateral_grasp_verified", "external_support"})
    if (!value.at(key).is_boolean()) throw std::runtime_error("known grasp/support observations required");
  return {{value.at("sequence"), value.at("captured_monotonic_ns"),
      finite_number(value.at("bottom_clearance_lower_bound_m")), value.at("bilateral_grasp_verified"),
      value.at("external_support"), value.at("frame_sha256")},
      optional_bool("approved_support_verified"), optional_bool("object_released_verified")};
}

std::tuple<std::int64_t, std::int64_t, std::set<std::string>> bind_completed_phase(const json& source, const json& binding, bool hardware,
    const std::string& profile_sha, const std::vector<double>& start,
    const std::string& expected_phase = "GRASP_CONFIRMED_HOLDING",
    const std::string& verified_flag = "observed_grasp_verified") {
  const auto raw = bytes(source.at("path"), false, 16777216);
  if (sha256(raw) != source.at("sha256").get<std::string>())
    throw std::runtime_error("LIFT grasp-confirmation source SHA mismatch");
  std::istringstream input(raw);
  std::string line;
  json result;
  std::set<std::string> positive_frames;
  std::set<std::string> all_frames;
  std::int64_t last_capture = 0;
  std::int64_t last_sequence = 0;
  while (std::getline(input, line)) {
    result = json::parse(line);
    const auto event = result.value("event", "");
    if (event != "grasp_observation" && event != "lift_observation" &&
        event != "hold_observation" && event != "support_observation") continue;
    const auto& observation = result.at("observation");
    for (const char* key : {"run_id", "object_id", "calibration_revision", "producer_sha256", "boot_id"})
      if (observation.at(key) != binding.at(key))
        throw std::runtime_error("LIFT prior grasp belongs to another task/source");
    if (observation.at("source_kind") != (hardware ? "hardware" : "mock"))
      throw std::runtime_error("LIFT prior grasp transport scope mismatch");
    all_frames.insert(observation.at("frame_sha256").get<std::string>());
    bool positive = observation.at("bilateral_grasp_verified") == true;
    if (expected_phase == "HOLD_REACHED_HOLDING")
      positive = positive && observation.at("external_support") == false &&
          finite_number(observation.at("bottom_clearance_lower_bound_m")) >= .030;
    if (expected_phase == "SUPPORTED_PLACED_HOLDING") {
      if (observation.at("support_id") != binding.at("support_id"))
        throw std::runtime_error("prior placement used another support destination");
      positive = positive && observation.at("approved_support_verified") == true;
    }
    if (positive) {
      positive_frames.insert(observation.at("frame_sha256").get<std::string>());
      last_capture = observation.at("captured_monotonic_ns");
      last_sequence = observation.at("sequence");
    }
  }
  if (result.value("event", "") != "result" || result.value("phase", "") != expected_phase ||
      result.at(verified_flag) != true || result.at("hardware_execution") != hardware ||
      result.at("profile_sha256") != profile_sha || positive_frames.size() < 2)
    throw std::runtime_error("LIFT requires completed observed grasp from the same profile/transport");
  if (expected_phase == "HOLD_REACHED_HOLDING" && finite_number(result.at("observed_hold_span_s")) < 3.)
    throw std::runtime_error("placement requires completed3s observed HOLD");
  if (expected_phase != "GRASP_CONFIRMED_HOLDING" && !result.contains("observed_frame_history"))
    throw std::runtime_error("supported ending requires inherited phase frame history");
  if (result.contains("observed_frame_history")) {
    for (const auto& entry : result.at("observed_frame_history")) {
      const auto hash = entry.get<std::string>();
      if (hash.size()!=64 || hash.find_first_not_of("0123456789abcdef")!=std::string::npos)
        throw std::runtime_error("invalid inherited observation SHA");
      all_frames.insert(hash);
    }
  }
  const auto now = monotonic_ns();
  if (last_capture <= 0 || now < last_capture || now-last_capture > 60000000000LL)
    throw std::runtime_error("LIFT prior grasp confirmation is stale/future");
  const auto measured = result.at("final_measured_rad").get<std::vector<double>>();
  if (measured.size() != start.size()) throw std::runtime_error("LIFT prior measured joint order/size mismatch");
  for (std::size_t i = 0; i < start.size(); ++i)
    if (!std::isfinite(measured[i]) || std::abs(measured[i]-start[i]) > .001)
      throw std::runtime_error("LIFT start differs from confirmed measured grasp state");
  return {last_capture, last_sequence, all_frames};
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
    if (hardware && (!present || (args["--confirm"] != "VISIBLE_LEFT_SENSOR_PREGRASP" &&
        args["--confirm"] != "VISIBLE_LEFT_OBSERVED_HOLD") || !isatty(0) || !isatty(1)))
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
    if ((phase != "PREGRASP" && phase != "WRIST_ALIGN" && phase != "HOLD" && phase != "CLOSE" && phase != "LIFT" &&
         phase != "PLACE" && phase != "RELEASE") ||
        already_holding != (phase != "PREGRASP"))
      throw std::runtime_error("phase and expected initial torque state mismatch");
    // The existing >=30mm pregrasp certificate excludes intended jaw/block
    // contact. MOCK coverage cannot authorize a physical contact policy.
    if (hardware && (phase == "CLOSE" || phase == "LIFT" || phase == "PLACE" || phase == "RELEASE"))
      throw std::runtime_error("contact phase hardware unavailable: physical contact-path policy and observer unverified");
    if (hardware && args["--confirm"] != (phase == "HOLD" ?
        "VISIBLE_LEFT_OBSERVED_HOLD" : "VISIBLE_LEFT_SENSOR_PREGRASP"))
      throw std::runtime_error("hardware confirmation does not authorize this phase");
    ObservedBlockHold hold;
    std::set<std::string> observed_frame_history;
    std::function<bool(std::int64_t)> observe_hold;
    std::function<BlockGraspObservation()> observe_grasp;
    std::optional<BlockGraspObservation> initial_grasp;
    std::function<BlockHoldObservation()> observe_lift;
    std::optional<BlockHoldObservation> initial_lift;
    std::function<BlockSupportObservation()> observe_support;
    std::optional<BlockSupportObservation> initial_support;
    if (phase == "PLACE" || phase == "RELEASE") {
      const auto binding = plan.at("support_observation");
      std::ifstream boot_stream("/proc/sys/kernel/random/boot_id");
      std::string boot_id; std::getline(boot_stream, boot_id);
      if (!boot_stream || boot_id.empty() || binding.at("boot_id") != boot_id)
        throw std::runtime_error("supported ending observation boot mismatch");
      if ((phase == "PLACE" && (start.back()!=goal.back() || start==goal)) ||
          (phase == "RELEASE" && (goal.back()<=start.back() || !std::equal(goal.begin(),goal.end()-1,start.begin()))))
        throw std::runtime_error("supported ending joint plan changes forbidden axes");
      const auto [prior_capture, prior_sequence, prior_frames] = bind_completed_phase(
          plan.at("previous_phase"), binding, hardware, sha256(profile_raw), start,
          phase=="PLACE" ? "HOLD_REACHED_HOLDING" : "SUPPORTED_PLACED_HOLDING",
          phase=="PLACE" ? "observed_hold_verified" : "observed_support_verified");
      observed_frame_history = prior_frames;
      report["previous_phase"] = plan.at("previous_phase");
      observe_support = [&, binding, prior_capture, prior_sequence, prior_frames] {
        const auto observation = read_support_observation(binding, hardware, output_fd, observed_frame_history);
        if (observation.block.captured_ns<=prior_capture || observation.block.sequence<=prior_sequence ||
            prior_frames.contains(observation.block.frame_sha256))
          throw std::runtime_error("supported ending reused a prior phase frame/time");
        return observation;
      };
      initial_support = observe_support();
      ObservedSupport preflight; preflight.update(*initial_support, monotonic_ns());
      if ((phase=="PLACE" && !initial_support->block.bilateral_grasp_verified) ||
          (phase=="RELEASE" && !initial_support->approved_support_verified.value_or(false)))
        throw std::runtime_error("supported ending initial grasp/support is unverified");
    }
    if (phase == "LIFT") {
      if (start.back() != goal.back() || start == goal)
        throw std::runtime_error("LIFT must keep measured grasp aperture and move the arm");
      const auto binding = plan.at("hold_observation");
      std::ifstream boot_stream("/proc/sys/kernel/random/boot_id");
      std::string boot_id; std::getline(boot_stream, boot_id);
      if (!boot_stream || boot_id.empty() || binding.at("boot_id") != boot_id)
        throw std::runtime_error("LIFT observation boot identity mismatch");
      const auto [prior_capture, prior_sequence, prior_frames] = bind_completed_phase(
          plan.at("grasp_confirmation"), binding, hardware, sha256(profile_raw), start);
      report["grasp_confirmation"] = plan.at("grasp_confirmation");
      observed_frame_history = prior_frames;
      observe_lift = [&, binding, prior_capture, prior_sequence, prior_frames] {
        const auto observation = read_hold_observation(binding, hardware, output_fd, observed_frame_history, "lift_observation");
        if (observation.captured_ns <= prior_capture || observation.sequence <= prior_sequence ||
            prior_frames.contains(observation.frame_sha256))
          throw std::runtime_error("LIFT reused a prior grasp frame or capture time");
        return observation;
      };
      initial_lift = observe_lift();
      ObservedGrasp preflight;
      preflight.update({initial_lift->sequence, initial_lift->captured_ns,
          initial_lift->bilateral_grasp_verified, initial_lift->frame_sha256}, monotonic_ns());
      if (!initial_lift->bilateral_grasp_verified)
        throw std::runtime_error("LIFT requires currently observed bilateral grasp");
    }
    if (phase == "CLOSE") {
      if (goal.back() >= start.back() || !std::equal(goal.begin(), goal.end()-1, start.begin()))
        throw std::runtime_error("CLOSE requires gripper-only closure");
      const auto binding = plan.at("grasp_observation");
      std::ifstream boot_stream("/proc/sys/kernel/random/boot_id");
      std::string boot_id; std::getline(boot_stream, boot_id);
      if (!boot_stream || boot_id.empty() || binding.at("boot_id") != boot_id)
        throw std::runtime_error("CLOSE observation boot identity mismatch");
      ObservedGrasp preflight;
      const auto initial_observation = read_grasp_observation(binding, hardware, output_fd, observed_frame_history);
      preflight.update(initial_observation, monotonic_ns());
      initial_grasp = initial_observation;
      observe_grasp = [&, binding] { return read_grasp_observation(binding, hardware, output_fd, observed_frame_history); };
    }
    if (phase == "HOLD") {
      if (start != goal || duration < 3.1)
        throw std::runtime_error("HOLD requires a stationary plan with at least3.1s budget");
      const auto binding = plan.at("hold_observation");
      std::ifstream boot_stream("/proc/sys/kernel/random/boot_id");
      std::string boot_id; std::getline(boot_stream, boot_id);
      if (!boot_stream || boot_id.empty()) throw std::runtime_error("local boot identity unavailable");
      if (binding.at("boot_id") != boot_id ||
          (hardware && binding.at("observer_physically_verified") != true))
        throw std::runtime_error("HOLD requires same-boot observations and a verified physical observer");
      // Fail before opening a port if evidence is absent, stale or not lifted.
      ObservedBlockHold preflight;
      const auto initial_observation = read_hold_observation(binding, hardware, output_fd, observed_frame_history);
      preflight.update(initial_observation, monotonic_ns());
      observe_hold = [&, binding](std::int64_t) {
        const auto observation = read_hold_observation(binding, hardware, output_fd, observed_frame_history);
        return hold.update(observation, monotonic_ns());
      };
    }
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
      }, path_tolerance, phase, observe_hold, observe_grasp, initial_grasp, observe_lift, initial_lift, observe_support, initial_support);
    report["event"] = "result"; report["phase"] = result.phase; report["reason"] = result.reason;
    report["phase_completed"] = result.reached;
    report["reached_joint_endpoint"] = result.reached && !result.observed_grasp_verified && phase!="PLACE";
    report["final_measured_rad"] = result.final_measured_rad;
    report["cartesian_endpoint_verified"] = false;
    report["observed_hold_verified"] = result.observed_hold_verified;
    report["observed_grasp_verified"] = result.observed_grasp_verified;
    report["observed_lift_verified"] = result.observed_lift_verified;
    report["observed_support_verified"] = result.observed_support_verified;
    report["observed_release_verified"] = result.observed_release_verified;
    report["observed_frame_history"] = observed_frame_history;
    if (phase == "LIFT") {
      report["hold_entered_monotonic_ns"] = result.hold_entered_ns;
      report["observed_hold_span_s"] = result.observed_hold_span_s;
    }
    if (phase == "HOLD") {
      report["hold_entered_monotonic_ns"] = hold.entered_ns;
      report["observed_hold_span_s"] = hold.first_capture_ns && hold.last_capture_ns ?
          (hold.last_capture_ns-hold.first_capture_ns)*1e-9 : 0.;
    }
    report["ending"] = result.observed_release_verified ?
        "object supported and released; retain arm torque; arm shutdown not verified" :
        "retain_torque_and_last_bounded_goal; supported ending is a separate approved phase";
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
