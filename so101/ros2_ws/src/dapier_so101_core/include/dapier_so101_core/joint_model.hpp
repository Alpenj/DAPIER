// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace dapier_so101_core
{

struct JointSpec
{
  std::string name;
  int motor_id;
  double lower_limit;
  double upper_limit;
  double max_velocity;
};

struct LimitResult
{
  std::vector<double> command;
  std::vector<bool> position_limited;
  std::vector<bool> velocity_limited;

  [[nodiscard]] bool was_limited() const;
};

class JointModel
{
public:
  explicit JointModel(std::vector<JointSpec> joints);

  [[nodiscard]] static JointModel from_yaml(const std::string & path);
  [[nodiscard]] const std::vector<JointSpec> & joints() const noexcept;
  [[nodiscard]] std::vector<std::string> names() const;
  [[nodiscard]] std::size_t size() const noexcept;

  [[nodiscard]] std::vector<double> reorder(
    const std::vector<std::string> & source_names,
    const std::vector<double> & source_positions) const;

  [[nodiscard]] bool within_limits(
    const std::vector<double> & positions,
    double tolerance = 0.0) const;

  [[nodiscard]] LimitResult limit(
    const std::vector<double> & previous,
    const std::vector<double> & target,
    double dt_seconds) const;

private:
  std::vector<JointSpec> joints_;
  std::unordered_map<std::string, std::size_t> index_by_name_;
};

struct CalibrationEntry
{
  std::string name;
  int motor_id;
  int homing_offset;
  int raw_min;
  int raw_max;
  double position_min;
  double position_max;
  bool inverted;

  [[nodiscard]] double raw_to_position(int raw) const;
  [[nodiscard]] int position_to_raw(double position) const;
};

class CalibrationSet
{
public:
  CalibrationSet(
    std::string device_id,
    bool verified,
    std::vector<CalibrationEntry> entries);

  [[nodiscard]] static CalibrationSet from_yaml(
    const std::string & path,
    bool require_verified = true);

  [[nodiscard]] const std::string & device_id() const noexcept;
  [[nodiscard]] bool verified() const noexcept;
  [[nodiscard]] const CalibrationEntry & at(const std::string & joint_name) const;
  void validate_against(const JointModel & model) const;

private:
  std::string device_id_;
  bool verified_;
  std::vector<CalibrationEntry> entries_;
  std::unordered_map<std::string, std::size_t> index_by_name_;
};

}  // namespace dapier_so101_core
