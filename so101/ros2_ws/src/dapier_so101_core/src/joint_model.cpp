// SPDX-License-Identifier: Apache-2.0

#include "dapier_so101_core/joint_model.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace dapier_so101_core
{
namespace
{

void require_finite(double value, const std::string & field)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(field + " must be finite");
  }
}

void require_vector_size(
  const std::vector<double> & values,
  std::size_t expected,
  const std::string & label)
{
  if (values.size() != expected) {
    throw std::invalid_argument(
            label + " has " + std::to_string(values.size()) +
            " values; expected " + std::to_string(expected));
  }
}

}  // namespace

bool LimitResult::was_limited() const
{
  return std::any_of(position_limited.begin(), position_limited.end(), [](bool value) {
      return value;
    }) || std::any_of(velocity_limited.begin(), velocity_limited.end(), [](bool value) {
      return value;
    });
}

JointModel::JointModel(std::vector<JointSpec> joints)
: joints_(std::move(joints))
{
  if (joints_.empty()) {
    throw std::invalid_argument("joint model must contain at least one joint");
  }

  std::unordered_set<int> motor_ids;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto & joint = joints_[index];
    if (joint.name.empty()) {
      throw std::invalid_argument("joint name must not be empty");
    }
    if (!index_by_name_.emplace(joint.name, index).second) {
      throw std::invalid_argument("duplicate joint name: " + joint.name);
    }
    if (joint.motor_id <= 0 || joint.motor_id >= 254) {
      throw std::invalid_argument("motor id must be in [1, 253] for joint: " + joint.name);
    }
    if (!motor_ids.insert(joint.motor_id).second) {
      throw std::invalid_argument("duplicate motor id: " + std::to_string(joint.motor_id));
    }
    require_finite(joint.lower_limit, joint.name + ".lower_limit");
    require_finite(joint.upper_limit, joint.name + ".upper_limit");
    require_finite(joint.max_velocity, joint.name + ".max_velocity");
    if (joint.lower_limit >= joint.upper_limit) {
      throw std::invalid_argument("lower_limit must be smaller than upper_limit: " + joint.name);
    }
    if (joint.max_velocity <= 0.0) {
      throw std::invalid_argument("max_velocity must be positive: " + joint.name);
    }
  }
}

JointModel JointModel::from_yaml(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("unsupported or missing joint contract schema_version");
  }
  const YAML::Node joints = root["joints"];
  if (!joints || !joints.IsSequence()) {
    throw std::invalid_argument("joint contract must contain a joints sequence");
  }

  std::vector<JointSpec> specs;
  specs.reserve(joints.size());
  for (const auto & node : joints) {
    specs.push_back(JointSpec{
      node["name"].as<std::string>(),
      node["motor_id"].as<int>(),
      node["lower_limit"].as<double>(),
      node["upper_limit"].as<double>(),
      node["max_velocity"].as<double>()});
  }
  return JointModel(std::move(specs));
}

const std::vector<JointSpec> & JointModel::joints() const noexcept
{
  return joints_;
}

std::vector<std::string> JointModel::names() const
{
  std::vector<std::string> result;
  result.reserve(joints_.size());
  for (const auto & joint : joints_) {
    result.push_back(joint.name);
  }
  return result;
}

std::size_t JointModel::size() const noexcept
{
  return joints_.size();
}

std::vector<double> JointModel::reorder(
  const std::vector<std::string> & source_names,
  const std::vector<double> & source_positions) const
{
  if (source_names.size() != source_positions.size()) {
    throw std::invalid_argument("source joint names and positions have different sizes");
  }

  std::unordered_map<std::string, double> source;
  source.reserve(source_names.size());
  for (std::size_t index = 0; index < source_names.size(); ++index) {
    require_finite(source_positions[index], "position for " + source_names[index]);
    if (!source.emplace(source_names[index], source_positions[index]).second) {
      throw std::invalid_argument("duplicate source joint name: " + source_names[index]);
    }
  }

  std::vector<double> ordered;
  ordered.reserve(joints_.size());
  for (const auto & joint : joints_) {
    const auto it = source.find(joint.name);
    if (it == source.end()) {
      throw std::invalid_argument("missing joint in source state: " + joint.name);
    }
    ordered.push_back(it->second);
  }
  return ordered;
}

bool JointModel::within_limits(
  const std::vector<double> & positions,
  double tolerance) const
{
  require_vector_size(positions, joints_.size(), "positions");
  require_finite(tolerance, "tolerance");
  if (tolerance < 0.0) {
    throw std::invalid_argument("tolerance must be non-negative");
  }

  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const double position = positions[index];
    if (!std::isfinite(position)) {
      return false;
    }
    if (position < joints_[index].lower_limit - tolerance ||
      position > joints_[index].upper_limit + tolerance)
    {
      return false;
    }
  }
  return true;
}

LimitResult JointModel::limit(
  const std::vector<double> & previous,
  const std::vector<double> & target,
  double dt_seconds) const
{
  require_vector_size(previous, joints_.size(), "previous command");
  require_vector_size(target, joints_.size(), "target command");
  require_finite(dt_seconds, "dt_seconds");
  if (dt_seconds <= 0.0) {
    throw std::invalid_argument("dt_seconds must be positive");
  }

  LimitResult result;
  result.command.resize(joints_.size());
  result.position_limited.assign(joints_.size(), false);
  result.velocity_limited.assign(joints_.size(), false);

  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto & joint = joints_[index];
    require_finite(previous[index], "previous command for " + joint.name);
    require_finite(target[index], "target command for " + joint.name);

    const double safe_previous = std::clamp(
      previous[index], joint.lower_limit, joint.upper_limit);
    const double bounded_target = std::clamp(
      target[index], joint.lower_limit, joint.upper_limit);
    result.position_limited[index] = bounded_target != target[index];

    const double max_delta = joint.max_velocity * dt_seconds;
    result.command[index] = std::clamp(
      bounded_target,
      safe_previous - max_delta,
      safe_previous + max_delta);
    result.command[index] = std::clamp(
      result.command[index], joint.lower_limit, joint.upper_limit);
    result.velocity_limited[index] = result.command[index] != bounded_target;
  }

  return result;
}

double CalibrationEntry::raw_to_position(int raw) const
{
  if (raw_min >= raw_max || position_min >= position_max) {
    throw std::logic_error("invalid calibration entry: " + name);
  }
  const int bounded_raw = std::clamp(raw, raw_min, raw_max);
  double ratio = static_cast<double>(bounded_raw - raw_min) /
    static_cast<double>(raw_max - raw_min);
  if (inverted) {
    ratio = 1.0 - ratio;
  }
  return position_min + ratio * (position_max - position_min);
}

int CalibrationEntry::position_to_raw(double position) const
{
  if (raw_min >= raw_max || position_min >= position_max) {
    throw std::logic_error("invalid calibration entry: " + name);
  }
  require_finite(position, "position for " + name);
  const double bounded_position = std::clamp(position, position_min, position_max);
  double ratio = (bounded_position - position_min) / (position_max - position_min);
  if (inverted) {
    ratio = 1.0 - ratio;
  }
  return static_cast<int>(std::lround(
      static_cast<double>(raw_min) + ratio * static_cast<double>(raw_max - raw_min)));
}

CalibrationSet::CalibrationSet(
  std::string device_id,
  bool verified,
  std::vector<CalibrationEntry> entries)
: device_id_(std::move(device_id)),
  verified_(verified),
  entries_(std::move(entries))
{
  if (device_id_.empty()) {
    throw std::invalid_argument("calibration device_id must not be empty");
  }
  if (entries_.empty()) {
    throw std::invalid_argument("calibration must contain at least one joint");
  }

  std::unordered_set<int> motor_ids;
  for (std::size_t index = 0; index < entries_.size(); ++index) {
    const auto & entry = entries_[index];
    if (entry.name.empty()) {
      throw std::invalid_argument("calibration joint name must not be empty");
    }
    if (!index_by_name_.emplace(entry.name, index).second) {
      throw std::invalid_argument("duplicate calibration joint: " + entry.name);
    }
    if (entry.motor_id <= 0 || entry.motor_id >= 254) {
      throw std::invalid_argument("invalid calibration motor id for: " + entry.name);
    }
    if (!motor_ids.insert(entry.motor_id).second) {
      throw std::invalid_argument("duplicate calibration motor id: " +
              std::to_string(entry.motor_id));
    }
    if (entry.raw_min >= entry.raw_max) {
      throw std::invalid_argument("raw_min must be smaller than raw_max: " + entry.name);
    }
    require_finite(entry.position_min, entry.name + ".position_min");
    require_finite(entry.position_max, entry.name + ".position_max");
    if (entry.position_min >= entry.position_max) {
      throw std::invalid_argument("position_min must be smaller than position_max: " + entry.name);
    }
  }
}

CalibrationSet CalibrationSet::from_yaml(
  const std::string & path,
  bool require_verified)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("unsupported or missing calibration schema_version");
  }
  const bool verified = root["verified"] && root["verified"].as<bool>();
  if (require_verified && !verified) {
    throw std::invalid_argument("calibration is not marked verified: " + path);
  }

  const YAML::Node joints = root["joints"];
  if (!joints || !joints.IsSequence()) {
    throw std::invalid_argument("calibration must contain a joints sequence");
  }

  std::vector<CalibrationEntry> entries;
  entries.reserve(joints.size());
  for (const auto & node : joints) {
    entries.push_back(CalibrationEntry{
      node["name"].as<std::string>(),
      node["motor_id"].as<int>(),
      node["homing_offset"].as<int>(0),
      node["raw_min"].as<int>(),
      node["raw_max"].as<int>(),
      node["position_min"].as<double>(),
      node["position_max"].as<double>(),
      node["inverted"].as<bool>(false)});
  }

  return CalibrationSet(
    root["device_id"].as<std::string>(), verified, std::move(entries));
}

const std::string & CalibrationSet::device_id() const noexcept
{
  return device_id_;
}

bool CalibrationSet::verified() const noexcept
{
  return verified_;
}

const CalibrationEntry & CalibrationSet::at(const std::string & joint_name) const
{
  const auto it = index_by_name_.find(joint_name);
  if (it == index_by_name_.end()) {
    throw std::out_of_range("joint not present in calibration: " + joint_name);
  }
  return entries_[it->second];
}

void CalibrationSet::validate_against(const JointModel & model) const
{
  if (entries_.size() != model.size()) {
    throw std::invalid_argument("calibration and joint model have different joint counts");
  }
  for (const auto & joint : model.joints()) {
    const auto & calibration = at(joint.name);
    if (calibration.motor_id != joint.motor_id) {
      throw std::invalid_argument("motor id mismatch for joint: " + joint.name);
    }
    constexpr double epsilon = 1e-9;
    if (std::abs(calibration.position_min - joint.lower_limit) > epsilon ||
      std::abs(calibration.position_max - joint.upper_limit) > epsilon)
    {
      throw std::invalid_argument("position limit mismatch for joint: " + joint.name);
    }
  }
}

}  // namespace dapier_so101_core
