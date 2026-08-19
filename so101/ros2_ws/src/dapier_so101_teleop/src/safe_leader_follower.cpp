// SPDX-License-Identifier: Apache-2.0

#include "dapier_so101_core/joint_model.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dapier_so101_teleop
{

class SafeLeaderFollower : public rclcpp::Node
{
public:
  SafeLeaderFollower()
  : Node("safe_leader_follower")
  {
    const std::string joint_config_file = declare_parameter<std::string>(
      "joint_config_file", "");
    if (joint_config_file.empty()) {
      throw std::invalid_argument("joint_config_file is required");
    }
    model_ = std::make_unique<dapier_so101_core::JointModel>(
      dapier_so101_core::JointModel::from_yaml(joint_config_file));

    leader_topic_ = declare_parameter<std::string>(
      "leader_topic", "/leader/joint_states");
    follower_state_topic_ = declare_parameter<std::string>(
      "follower_state_topic", "/follower/joint_states");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/follower/trajectory_controller/joint_trajectory");
    enable_service_name_ = declare_parameter<std::string>(
      "enable_service", "/dapier_so101/teleop/enable");
    enabled_topic_ = declare_parameter<std::string>(
      "enabled_topic", "/dapier_so101/teleop/enabled");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    max_start_error_rad_ = declare_parameter<double>("max_start_error_rad", 0.35);
    hard_limit_tolerance_rad_ = declare_parameter<double>(
      "hard_limit_tolerance_rad", 0.02);
    command_horizon_s_ = declare_parameter<double>("command_horizon_s", 0.04);

    validate_parameters();
    control_period_s_ = 1.0 / publish_rate_hz_;

    // SensorDataQoS is appropriate for rapidly replaced joint samples. The safety
    // decision does not depend on receiving every sample; it depends on freshness.
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SafeLeaderFollower::on_leader_state, this, std::placeholders::_1));
    follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      follower_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SafeLeaderFollower::on_follower_state, this, std::placeholders::_1));

    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      command_topic_, rclcpp::QoS(10).reliable());
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      enabled_topic_, rclcpp::QoS(1).reliable().transient_local());

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      enable_service_name_,
      std::bind(
        &SafeLeaderFollower::on_enable_request, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_period_s_),
      std::bind(&SafeLeaderFollower::control_step, this));

    publish_enabled_state();
    RCLCPP_INFO(
      get_logger(),
      "Safe relay ready but DISABLED. Align both arms, then call %s with data=true.",
      enable_service_name_.c_str());
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  std::unique_ptr<dapier_so101_core::JointModel> model_;

  std::string leader_topic_;
  std::string follower_state_topic_;
  std::string command_topic_;
  std::string enable_service_name_;
  std::string enabled_topic_;
  double publish_rate_hz_{50.0};
  double stale_timeout_s_{0.25};
  double max_start_error_rad_{0.35};
  double hard_limit_tolerance_rad_{0.02};
  double command_horizon_s_{0.04};
  double control_period_s_{0.02};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState::SharedPtr leader_state_;
  sensor_msgs::msg::JointState::SharedPtr follower_state_;
  SteadyClock::time_point leader_received_at_{};
  SteadyClock::time_point follower_received_at_{};
  std::vector<double> last_command_;
  bool enabled_{false};

  void validate_parameters() const
  {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (!std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= 0.0) {
      throw std::invalid_argument("stale_timeout_s must be positive");
    }
    if (!std::isfinite(max_start_error_rad_) || max_start_error_rad_ < 0.0) {
      throw std::invalid_argument("max_start_error_rad must be non-negative");
    }
    if (!std::isfinite(hard_limit_tolerance_rad_) || hard_limit_tolerance_rad_ < 0.0) {
      throw std::invalid_argument("hard_limit_tolerance_rad must be non-negative");
    }
    if (!std::isfinite(command_horizon_s_) || command_horizon_s_ <= 0.0) {
      throw std::invalid_argument("command_horizon_s must be positive");
    }
  }

  void on_leader_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    leader_state_ = message;
    leader_received_at_ = SteadyClock::now();
  }

  void on_follower_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    follower_state_ = message;
    follower_received_at_ = SteadyClock::now();
  }

  [[nodiscard]] bool states_are_fresh() const
  {
    if (!leader_state_ || !follower_state_) {
      return false;
    }
    const auto now = SteadyClock::now();
    const double leader_age = std::chrono::duration<double>(now - leader_received_at_).count();
    const double follower_age = std::chrono::duration<double>(now - follower_received_at_).count();
    return leader_age <= stale_timeout_s_ && follower_age <= stale_timeout_s_;
  }

  [[nodiscard]] std::vector<double> ordered_positions(
    const sensor_msgs::msg::JointState & state) const
  {
    return model_->reorder(state.name, state.position);
  }

  void on_enable_request(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!request->data) {
      disable("operator request");
      response->success = true;
      response->message = "teleoperation disabled";
      return;
    }

    if (!states_are_fresh()) {
      response->success = false;
      response->message = "leader and follower states must both be present and fresh";
      return;
    }

    try {
      const auto leader = ordered_positions(*leader_state_);
      const auto follower = ordered_positions(*follower_state_);

      if (!model_->within_limits(leader, hard_limit_tolerance_rad_)) {
        response->success = false;
        response->message = "leader state is outside the configured hard limits";
        return;
      }
      if (!model_->within_limits(follower, hard_limit_tolerance_rad_)) {
        response->success = false;
        response->message = "follower state is outside the configured hard limits";
        return;
      }

      double largest_error = 0.0;
      std::size_t largest_error_index = 0;
      for (std::size_t index = 0; index < model_->size(); ++index) {
        const double error = std::abs(leader[index] - follower[index]);
        if (error > largest_error) {
          largest_error = error;
          largest_error_index = index;
        }
      }
      if (largest_error > max_start_error_rad_) {
        std::ostringstream message;
        message << "align arms before enabling: "
                << model_->joints()[largest_error_index].name
                << " differs by " << largest_error << " rad; limit is "
                << max_start_error_rad_ << " rad";
        response->success = false;
        response->message = message.str();
        return;
      }

      // Seeding with measured follower state prevents the first command from jumping
      // directly to the leader pose. Later commands may only move by v_max * dt.
      last_command_ = follower;
      enabled_ = true;
      publish_enabled_state();
      response->success = true;
      response->message = "teleoperation enabled";
      RCLCPP_INFO(get_logger(), "Teleoperation ENABLED by explicit request.");
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("invalid joint state: ") + error.what();
    }
  }

  void control_step()
  {
    if (!enabled_) {
      return;
    }
    if (!states_are_fresh()) {
      disable("joint state timeout");
      return;
    }

    try {
      const auto target = ordered_positions(*leader_state_);
      const auto limited = model_->limit(last_command_, target, control_period_s_);
      publish_command(limited.command);
      last_command_ = limited.command;

      if (limited.was_limited()) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Leader target is being constrained by position or velocity limits.");
      }
    } catch (const std::exception & error) {
      disable(std::string("invalid joint state: ") + error.what());
    }
  }

  void publish_command(const std::vector<double> & positions)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = model_->names();

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    const auto seconds = static_cast<int32_t>(command_horizon_s_);
    point.time_from_start.sec = seconds;
    point.time_from_start.nanosec = static_cast<uint32_t>(
      (command_horizon_s_ - static_cast<double>(seconds)) * 1e9);
    trajectory.points.push_back(std::move(point));
    command_pub_->publish(trajectory);
  }

  void disable(const std::string & reason)
  {
    const bool was_enabled = enabled_;
    enabled_ = false;
    last_command_.clear();
    publish_enabled_state();
    if (was_enabled) {
      // Publishing no further setpoints intentionally leaves stop behavior to the
      // controller/hardware layer. A torque-off emergency stop belongs there.
      RCLCPP_WARN(get_logger(), "Teleoperation DISABLED: %s", reason.c_str());
    }
  }

  void publish_enabled_state()
  {
    std_msgs::msg::Bool status;
    status.data = enabled_;
    enabled_pub_->publish(status);
  }
};

}  // namespace dapier_so101_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dapier_so101_teleop::SafeLeaderFollower>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("safe_leader_follower"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
