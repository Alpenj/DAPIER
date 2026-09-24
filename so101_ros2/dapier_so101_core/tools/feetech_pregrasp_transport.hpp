#pragma once

#include "bounded_pregrasp.hpp"
#include <feetech_driver/communication_protocol.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <unistd.h>

namespace dapier_so101_executor {
inline std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline void verify_controller(const std::string& port, const std::string& serial) {
  namespace fs = std::filesystem;
  if (serial.empty()) throw std::runtime_error("controller identity missing");
  auto node = fs::canonical(fs::path("/sys/class/tty") / fs::canonical(port).filename() / "device");
  for (; node != node.root_path(); node = node.parent_path()) {
    std::ifstream stream(node / "serial");
    if (!stream) continue;
    std::string actual; std::getline(stream, actual);
    if (actual != serial) throw std::runtime_error("controller identity mismatch");
    return;
  }
  throw std::runtime_error("controller identity unavailable");
}

class FeetechPregraspTransport final : public MotorTransport {
 public:
  FeetechPregraspTransport(const std::string& port, const std::string& serial,
      std::vector<dapier_so101_core::CalibrationEntry> calibration,
      const std::vector<double>& maximum_velocity_rad_s,
      const std::string& confirmation, bool operator_present,
      std::function<void()> check_io, JointModel model, std::vector<double> start,
      std::vector<double> goal, double path_tolerance_rad, double command_horizon_s, bool already_holding)
      : calibration_(std::move(calibration)), check_io_(std::move(check_io)), model_(std::move(model)),
        start_(std::move(start)), goal_(std::move(goal)), path_tolerance_rad_(path_tolerance_rad),
        command_horizon_s_(command_horizon_s) {
    armed_ = already_holding;  // Expected state only; every read verifies register 40.
    const bool confirmed = confirmation == "VISIBLE_LEFT_SENSOR_PREGRASP" ||
        (confirmation == "VISIBLE_LEFT_OBSERVED_HOLD" && already_holding && start_ == goal_);
    if (!operator_present || !confirmed || !isatty(0) || !isatty(1))
      throw std::runtime_error("physical transport requires attended TTY and exact confirmation");
    if (calibration_.size() != 6) throw std::runtime_error("six calibrated follower joints required");
    if (maximum_velocity_rad_s.size() != calibration_.size()) throw std::runtime_error("velocity bounds required");
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto& cal = calibration_[i];
      const double v = maximum_velocity_rad_s[i];
      if (!std::isfinite(v) || v <= 0 || v > .3 || cal.raw_min < 0 || cal.raw_max > 4095 ||
          cal.raw_min >= cal.raw_max || !std::isfinite(cal.position_min) || !std::isfinite(cal.position_max) ||
          cal.position_min >= cal.position_max)
        throw std::runtime_error("invalid calibration or velocity cap");
      const int speed = static_cast<int>(std::floor(v * (cal.raw_max-cal.raw_min) / (cal.position_max-cal.position_min)));
      if (speed < 1 || speed > 32767) throw std::runtime_error("velocity cap cannot be represented");
      maximum_speed_tick_s_.push_back(speed);
    }
    check_io_();
    verify_controller(port, serial);
    auto serial_port = std::make_unique<feetech_driver::SerialPort>(port);
    checked(serial_port->configure());
    protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));
    verify_controller(port, serial);
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto& cal = calibration_[i];
      if (cal.motor_id != static_cast<int>(i + 1)) throw std::runtime_error("unexpected motor ID order");
      std::array<std::uint8_t, 40> eeprom{};
      check_io_();
      checked(protocol_->read(cal.motor_id, 0, &eeprom));
      if (word(eeprom, 3) != 777 || eeprom[5] != cal.motor_id || eeprom[33] != 0 ||
          feetech_driver::decode_sign_magnitude(word(eeprom, 31), 11) != cal.homing_offset ||
          word(eeprom, 9) > cal.raw_min || word(eeprom, 11) < cal.raw_max)
        throw std::runtime_error("motor model/mode/calibration differs from approved profile");
      protection_.push_back(eeprom);
    }
  }

  MeasuredRobotState read() override {
    MeasuredRobotState state;
    state.received_monotonic_ns = monotonic_ns();
    last_raw_.clear();
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto& cal = calibration_[i];
      std::array<std::uint8_t, 31> bytes{};
      check_io_();
      checked(protocol_->read(cal.motor_id, SMS_STS_TORQUE_ENABLE, &bytes));
      const int raw = word(bytes, 16);
      if (raw < cal.raw_min || raw > cal.raw_max)
        throw std::runtime_error("measured raw tick outside calibration; not clipped");
      if (bytes[0] != (armed_ ? 1 : 0) || bytes[25] != 0 || word(bytes, 4) != 0 ||
          (armed_ && (word(bytes, 6) != maximum_speed_tick_s_[i] ||
           std::abs(feetech_driver::decode_sign_magnitude(word(bytes, 18), 15)) > maximum_speed_tick_s_[i])) ||
          bytes[22] < protection_[i][15] || bytes[22] > protection_[i][14] ||
          bytes[23] >= protection_[i][13])
        throw std::runtime_error("unexpected torque/status/voltage/temperature");
      state.joint_names.push_back(cal.name);
      state.joint_position_rad.push_back(cal.raw_to_position(raw));
      last_raw_.push_back(raw);
    }
    last_read_ns_ = state.received_monotonic_ns;
    // Only the fixed-base, path-audited entrypoint may construct this transport.
    // These are plan interlocks, not claims of live collision sensing or odometry.
    state.base_settled = state.collision_clear = true;
    return state;
  }

  void arm_at_measured_position() override {
    if (last_raw_.size() != calibration_.size() || monotonic_ns() - last_read_ns_ > 100000000)
      throw std::runtime_error("fresh measured presync required before torque enable");
    check_io_();
    if (armed_) return;  // A checked wrist step inherits the prior holding phase.
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const int speed = maximum_speed_tick_s_[i];
      check_io_();
      ++goal_velocity_attempts;
      checked(protocol_->write(calibration_[i].motor_id, SMS_STS_GOAL_SPEED_L,
          std::array<std::uint8_t, 2>{static_cast<std::uint8_t>(speed & 255), static_cast<std::uint8_t>(speed >> 8)}));
      ++goal_velocity_writes;
      // Re-read this axis immediately before presync: the earlier six-axis sample
      // can age while speed registers are acknowledged. Do not clamp the sample.
      const auto& cal = calibration_[i];
      std::array<std::uint8_t, 18> current{};
      check_io_();
      const auto read_started = monotonic_ns();
      checked(protocol_->read(cal.motor_id, SMS_STS_TORQUE_ENABLE, &current));
      const int raw = word(current, 16);
      if (current[0] != 0 || word(current, 6) != speed || raw < cal.raw_min || raw > cal.raw_max ||
          std::abs(cal.raw_to_position(raw) - cal.raw_to_position(last_raw_[i])) > .5*std::acos(-1.)/180.)
        throw std::runtime_error("axis changed or speed verification failed before presync");
      auto presync_state = last_raw_;
      presync_state[i] = raw;
      std::vector<double> presync_rad;
      for (std::size_t j = 0; j < presync_state.size(); ++j)
        presync_rad.push_back(calibration_[j].raw_to_position(presync_state[j]));
      if (!model_.within_limits(presync_rad) ||
          !within_path_envelope(presync_rad, start_, goal_, path_tolerance_rad_))
        throw std::runtime_error("fresh presync outside model/path envelope");
      write_goal(i, raw);
      std::array<std::uint8_t, 2> presynced{};
      check_io_();
      checked(protocol_->read(cal.motor_id, SMS_STS_GOAL_POSITION_L, &presynced));
      if (word(presynced, 0) != raw || monotonic_ns() - read_started > 100000000)
        throw std::runtime_error("presync verification failed or aged before torque enable");
      // Only register 40; the SDK enable_torque helper also writes EEPROM lock.
      check_io_();
      ++torque_enable_attempts;
      checked(protocol_->write(cal.motor_id, SMS_STS_TORQUE_ENABLE, std::array<std::uint8_t, 1>{1}));
      ++torque_enable_writes;
    }
    armed_ = true;
  }

  SentPositions send(const std::vector<double>& positions) override {
    if (!armed_ || positions.size() != calibration_.size()) throw std::runtime_error("transport not armed or wrong command shape");
    SentPositions sent;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const auto& cal = calibration_[i];
      if (!std::isfinite(positions[i]) || positions[i] < cal.position_min || positions[i] > cal.position_max)
        throw std::runtime_error("command outside calibrated range; not clipped");
      const auto& joint = model_.joints()[i];
      const double measured = cal.raw_to_position(last_raw_.at(i));
      const double delta = joint.max_velocity*command_horizon_s_;
      const int raw = bounded_raw_tick(cal, positions[i], std::max(joint.lower_limit, measured-delta),
          std::min(joint.upper_limit, measured+delta));
      sent.raw_ticks.push_back(raw);
      sent.position_rad.push_back(cal.raw_to_position(raw));
    }
    if (!model_.within_limits(sent.position_rad) ||
        !within_path_envelope(sent.position_rad, start_, goal_, path_tolerance_rad_))
      throw std::runtime_error("quantized command outside model/path envelope");
    std::vector<double> measured;
    for (std::size_t i = 0; i < calibration_.size(); ++i)
      measured.push_back(calibration_[i].raw_to_position(last_raw_.at(i)));
    if (model_.limit(measured, sent.position_rad, command_horizon_s_).was_limited())
      throw std::runtime_error("quantized command exceeds measured velocity horizon");
    for (std::size_t i = 0; i < positions.size(); ++i) write_goal(i, sent.raw_ticks[i]);
    return sent;
  }

  unsigned goal_position_writes{}, goal_velocity_writes{}, torque_enable_writes{};
  // Attempts count SDK calls, not wire packets (the SDK may retry). A missing ACK
  // cannot establish that no motor register changed.
  unsigned goal_position_attempts{}, goal_velocity_attempts{}, torque_enable_attempts{};
  // Destruction closes the port only. It never releases an airborne arm's torque.
 private:
  template<class Result> static void checked(const Result& result) {
    if (!result) throw std::runtime_error(result.error());
  }
  template<std::size_t N> static int word(const std::array<std::uint8_t, N>& data, std::size_t index) {
    return data.at(index) | (static_cast<int>(data.at(index + 1)) << 8);
  }
  void write_goal(std::size_t i, int raw) {
    std::array<std::uint8_t, 2> bytes{static_cast<std::uint8_t>(raw & 255), static_cast<std::uint8_t>(raw >> 8)};
    // Avoid write_position/sync_write_position: they also overwrite acceleration
    // and goal time. Speed is explicitly bounded before arming; protections stay intact.
    check_io_();
    if (armed_ && monotonic_ns() - last_read_ns_ > 100000000)
      throw std::runtime_error("measured state aged before per-axis dispatch");
    ++goal_position_attempts;
    checked(protocol_->write(calibration_[i].motor_id, SMS_STS_GOAL_POSITION_L, bytes));
    ++goal_position_writes;
  }
  std::vector<dapier_so101_core::CalibrationEntry> calibration_;
  std::vector<std::array<std::uint8_t, 40>> protection_;
  std::vector<int> last_raw_;
  std::vector<int> maximum_speed_tick_s_;
  std::int64_t last_read_ns_{};
  bool armed_{false};
  std::function<void()> check_io_;
  JointModel model_;
  std::vector<double> start_, goal_;
  double path_tolerance_rad_, command_horizon_s_;
  std::unique_ptr<feetech_driver::CommunicationProtocol> protocol_;
};
}  // namespace dapier_so101_executor
