// SPDX-License-Identifier: Apache-2.0

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dapier_interfaces/msg/interlock_state.hpp"
#include "dapier_interfaces/msg/localization_estimate.hpp"
#include "dapier_interfaces/msg/replan_acknowledgement.hpp"
#include "dapier_interfaces/msg/research_control_intent.hpp"
#include "dapier_interfaces/msg/safe_command.hpp"
#include "dapier_interfaces/msg/safety_status.hpp"
#include "dapier_localization_core/localization_monitor.hpp"
#include "dapier_safety_core/safety_controller.hpp"
#include "dapier_so101_core/joint_model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dapier_safety_bridge
{
namespace
{

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t milliseconds_to_ns(std::int64_t value, const std::string & name)
{
  if (value <= 0 || value > std::numeric_limits<std::int64_t>::max() / 1000000) {
    throw std::invalid_argument(name + " must be a positive bounded duration");
  }
  return value * 1000000;
}

std::string safety_decision_name(dapier_safety_core::SafetyDecision decision)
{
  using dapier_safety_core::SafetyDecision;
  switch (decision) {
    case SafetyDecision::kDispatch:
      return "dispatch";
    case SafetyDecision::kHold:
      return "hold";
    case SafetyDecision::kReject:
      return "reject";
    case SafetyDecision::kSafeStop:
      return "safe_stop";
  }
  return "reject";
}

std::string intent_kind_name(dapier_so101_core::ResearchIntentKind kind)
{
  using dapier_so101_core::ResearchIntentKind;
  switch (kind) {
    case ResearchIntentKind::kHold:
      return "hold";
    case ResearchIntentKind::kArmJointPosition:
      return "arm_joint_position";
    case ResearchIntentKind::kBaseTwist:
      return "base_twist";
  }
  return "hold";
}

dapier_so101_core::ResearchIntentKind intent_kind_from_name(
  const std::string & value)
{
  using dapier_so101_core::ResearchIntentKind;
  if (value == "hold") {
    return ResearchIntentKind::kHold;
  }
  if (value == "arm_joint_position") {
    return ResearchIntentKind::kArmJointPosition;
  }
  if (value == "base_twist") {
    return ResearchIntentKind::kBaseTwist;
  }
  throw std::invalid_argument("unknown research intent kind: " + value);
}

dapier_localization_core::TrackingState tracking_state_from_name(
  const std::string & value)
{
  using dapier_localization_core::TrackingState;
  if (value == "initializing") {
    return TrackingState::kInitializing;
  }
  if (value == "tracking") {
    return TrackingState::kTracking;
  }
  if (value == "recently_lost") {
    return TrackingState::kRecentlyLost;
  }
  if (value == "lost") {
    return TrackingState::kLost;
  }
  if (value == "relocalized") {
    return TrackingState::kRelocalized;
  }
  throw std::invalid_argument("unknown localization tracking state: " + value);
}

}  // namespace

class SafetyBridgeNode final : public rclcpp::Node
{
public:
  SafetyBridgeNode()
  : Node("dapier_safety_bridge"),
    controller_config_(load_controller_config()),
    controller_(controller_config_),
    localization_config_(load_localization_config()),
    localization_monitor_(localization_config_)
  {
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto state_qos = rclcpp::SensorDataQoS();
    const auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    safe_command_pub_ = create_publisher<dapier_interfaces::msg::SafeCommand>(
      "/dapier/safe_command", command_qos);
    status_pub_ = create_publisher<dapier_interfaces::msg::SafetyStatus>(
      "/dapier/safety_status", latch_qos);

    intent_sub_ = create_subscription<dapier_interfaces::msg::ResearchControlIntent>(
      "/dapier/research_intent", command_qos,
      std::bind(&SafetyBridgeNode::on_intent, this, std::placeholders::_1));
    localization_sub_ = create_subscription<dapier_interfaces::msg::LocalizationEstimate>(
      "/dapier/localization_estimate", command_qos,
      std::bind(&SafetyBridgeNode::on_localization, this, std::placeholders::_1));
    replan_sub_ = create_subscription<dapier_interfaces::msg::ReplanAcknowledgement>(
      "/dapier/replan_acknowledgement", command_qos,
      std::bind(&SafetyBridgeNode::on_replan_acknowledgement, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", state_qos,
      std::bind(&SafetyBridgeNode::on_joint_state, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", state_qos,
      std::bind(&SafetyBridgeNode::on_odometry, this, std::placeholders::_1));
    interlock_sub_ = create_subscription<dapier_interfaces::msg::InterlockState>(
      "/dapier/interlocks", command_qos,
      std::bind(&SafetyBridgeNode::on_interlocks, this, std::placeholders::_1));
    operator_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dapier/operator_enable", latch_qos,
      std::bind(&SafetyBridgeNode::on_operator_enable, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dapier/estop_healthy", latch_qos,
      std::bind(&SafetyBridgeNode::on_estop_health, this, std::placeholders::_1));

    safe_stop_reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/dapier/acknowledge_safe_stop",
      std::bind(
        &SafetyBridgeNode::on_safe_stop_reset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SafetyBridgeNode::on_watchdog, this));

    publish_status("waiting", "waiting for localization, measured state and interlocks");
  }

private:
  dapier_safety_core::SafetyControllerConfig load_controller_config()
  {
    const std::string default_contract =
      ament_index_cpp::get_package_share_directory("dapier_safety_core") +
      "/config/dual_so101_joint_contract.yaml";
    const std::string joint_contract_path = declare_parameter<std::string>(
      "joint_contract_path", default_contract);
    const auto joint_model = dapier_so101_core::JointModel::from_yaml(
      joint_contract_path);

    dapier_safety_core::SafetyControllerConfig config;
    config.joints.reserve(joint_model.size());
    for (const auto & joint : joint_model.joints()) {
      config.joints.push_back(dapier_safety_core::JointSafetyLimit{
        joint.name,
        joint.lower_limit,
        joint.upper_limit,
        joint.max_velocity,
      });
    }
    config.measured_state_timeout_ns = milliseconds_to_ns(
      declare_parameter<std::int64_t>("measured_state_timeout_ms", 100),
      "measured_state_timeout_ms");
    config.command_watchdog_ns = milliseconds_to_ns(
      declare_parameter<std::int64_t>("command_watchdog_ms", 250),
      "command_watchdog_ms");
    config.command_horizon_s = declare_parameter<double>(
      "command_horizon_s", 0.05);
    config.maximum_base_linear_mps = declare_parameter<double>(
      "maximum_base_linear_mps", 0.08);
    config.maximum_base_angular_rad_s = declare_parameter<double>(
      "maximum_base_angular_rad_s", 0.25);
    config.settled_base_linear_mps = declare_parameter<double>(
      "settled_base_linear_mps", 0.0025);
    config.settled_base_angular_rad_s = declare_parameter<double>(
      "settled_base_angular_rad_s", 0.0021);
    config.require_verified_grasp_for_base_motion = declare_parameter<bool>(
      "require_verified_grasp_for_base_motion", true);
    return config;
  }

  dapier_localization_core::LocalizationMonitorConfig load_localization_config()
  {
    dapier_localization_core::LocalizationMonitorConfig config;
    config.maximum_interarrival_ns = milliseconds_to_ns(
      declare_parameter<std::int64_t>("localization_interarrival_timeout_ms", 100),
      "localization_interarrival_timeout_ms");
    config.motion_gate.minimum_confidence = declare_parameter<double>(
      "localization_minimum_confidence", 0.5);
    config.motion_gate.minimum_tracked_features = static_cast<std::uint32_t>(
      declare_parameter<std::int64_t>("localization_minimum_tracked_features", 20));
    config.motion_gate.maximum_mean_reprojection_error_px = declare_parameter<double>(
      "localization_maximum_reprojection_error_px", 3.0);
    config.motion_gate.maximum_position_variance_m2 = declare_parameter<double>(
      "localization_maximum_position_variance_m2", 0.01);
    config.motion_gate.maximum_rotation_variance_rad2 = declare_parameter<double>(
      "localization_maximum_rotation_variance_rad2", 0.04);
    return config;
  }

  dapier_localization_core::LocalizationEstimate to_core(
    const dapier_interfaces::msg::LocalizationEstimate & message) const
  {
    dapier_localization_core::LocalizationEstimate estimate;
    estimate.schema_version = message.schema_version;
    estimate.sequence = message.sequence;
    estimate.source_timestamp_ns = message.source_timestamp_ns;
    estimate.ttl_ns = message.ttl_ns;
    estimate.parent_frame = message.parent_frame;
    estimate.child_frame = message.child_frame;
    estimate.map_id = message.map_id;
    estimate.session_id = message.session_id;
    estimate.reset_generation = message.reset_generation;
    estimate.tracking_state = tracking_state_from_name(message.tracking_state);
    estimate.position_m = message.position_m;
    estimate.orientation_xyzw = message.orientation_xyzw;
    estimate.covariance = message.covariance;
    estimate.tracked_features = message.tracked_features;
    estimate.mean_reprojection_error_px = message.mean_reprojection_error_px;
    estimate.confidence = message.confidence;
    estimate.simulator_truth_used = message.simulator_truth_used;
    estimate.control_authorized = message.control_authorized;
    return estimate;
  }

  dapier_so101_core::ResearchControlIntent to_core(
    const dapier_interfaces::msg::ResearchControlIntent & message) const
  {
    if (message.control_authorized) {
      throw std::invalid_argument(
              "research intent message must not authorize hardware");
    }
    dapier_so101_core::ResearchControlIntent intent;
    intent.schema_version = message.schema_version;
    intent.sequence = message.sequence;
    intent.kind = intent_kind_from_name(message.kind);
    intent.source = message.source;
    intent.source_monotonic_ns = message.source_monotonic_ns;
    intent.ttl_ns = message.ttl_ns;
    intent.joint_names = message.joint_names;
    intent.joint_position_rad = message.joint_position_rad;
    intent.joint_max_velocity_rad_s = message.joint_max_velocity_rad_s;
    intent.base_linear_x_mps = message.base_linear_x_mps;
    intent.base_angular_z_rad_s = message.base_angular_z_rad_s;
    return intent;
  }

  std::optional<dapier_safety_core::MeasuredRobotState> measured_state() const
  {
    if (!have_joint_state_ || !have_odometry_ || !have_interlocks_) {
      return std::nullopt;
    }
    dapier_safety_core::MeasuredRobotState measured;
    measured.received_monotonic_ns = std::min(
      {joint_state_received_ns_, odometry_received_ns_, interlock_received_ns_});
    measured.joint_names = joint_names_;
    measured.joint_position_rad = joint_positions_;
    measured.base_linear_mps = base_linear_mps_;
    measured.base_angular_rad_s = base_angular_rad_s_;
    measured.base_settled =
      std::abs(base_linear_mps_) <= controller_config_.settled_base_linear_mps &&
      std::abs(base_angular_rad_s_) <= controller_config_.settled_base_angular_rad_s;
    measured.collision_clear = interlocks_.collision_clear;
    measured.carry_pose_clear = interlocks_.carry_pose_clear;
    measured.grasp_verified = interlocks_.grasp_verified;
    return measured;
  }

  dapier_safety_core::SafetyContext safety_context() const
  {
    return dapier_safety_core::SafetyContext{
      localization_decision_,
      have_interlocks_ && interlocks_.arm_phase_allowed,
      have_interlocks_ && interlocks_.base_phase_allowed,
    };
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    joint_names_ = message->name;
    joint_positions_ = message->position;
    joint_state_received_ns_ = steady_now_ns();
    have_joint_state_ = true;
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    base_linear_mps_ = message->twist.twist.linear.x;
    base_angular_rad_s_ = message->twist.twist.angular.z;
    odometry_received_ns_ = steady_now_ns();
    have_odometry_ = true;
  }

  void on_interlocks(const dapier_interfaces::msg::InterlockState::SharedPtr message)
  {
    if (have_interlocks_ && message->sequence <= interlocks_.sequence) {
      handle_external_stop(
        "interlock sequence is duplicate or out of order",
        message->sequence);
      return;
    }
    interlocks_ = *message;
    interlock_received_ns_ = steady_now_ns();
    have_interlocks_ = true;
    if (!message->collision_clear && controller_.motion_armed()) {
      handle_external_stop("collision interlock became unsafe", message->sequence);
    }
  }

  void on_operator_enable(const std_msgs::msg::Bool::SharedPtr message)
  {
    controller_.set_operator_enabled(message->data);
    publish_status(
      message->data ? "disabled_until_command" : "disabled",
      message->data ? "operator enable received" : "operator disabled motion");
  }

  void on_estop_health(const std_msgs::msg::Bool::SharedPtr message)
  {
    controller_.set_estop_healthy(message->data);
    if (!message->data) {
      handle_external_stop("E-stop health input became false", 0);
    } else {
      publish_status("estop_healthy", "E-stop health input is true");
    }
  }

  void on_localization(
    const dapier_interfaces::msg::LocalizationEstimate::SharedPtr message)
  {
    const auto now = steady_now_ns();
    try {
      const auto estimate = to_core(*message);
      const auto monitored = localization_monitor_.observe(estimate, now);
      localization_decision_ = monitored.decision;
      latest_localization_sequence_ = estimate.sequence;
      localization_received_ns_ = now;
      if (monitored.decision != dapier_localization_core::MotionDecision::kReject &&
        estimate.ttl_ns > 0 && estimate.ttl_ns <=
        std::numeric_limits<std::int64_t>::max() - now)
      {
        localization_expiry_ns_ = now + estimate.ttl_ns;
      } else {
        localization_expiry_ns_.reset();
      }
      publish_status(
        monitored.decision == dapier_localization_core::MotionDecision::kProceed ?
        "localization_ready" : "localization_hold",
        monitored.reason);
      if (controller_.motion_armed() &&
        monitored.decision != dapier_localization_core::MotionDecision::kProceed)
      {
        handle_external_stop(
          "localization invalidated active motion: " + monitored.reason,
          estimate.sequence);
      }
    } catch (const std::exception & error) {
      localization_decision_ = dapier_localization_core::MotionDecision::kReject;
      localization_expiry_ns_.reset();
      handle_external_stop(
        std::string("localization message rejected: ") + error.what(),
        message->sequence);
    }
  }

  void on_replan_acknowledgement(
    const dapier_interfaces::msg::ReplanAcknowledgement::SharedPtr message)
  {
    try {
      localization_monitor_.acknowledge_replan(
        message->localization_sequence,
        dapier_localization_core::LocalizationIdentity{
          message->parent_frame,
          message->child_frame,
          message->map_id,
          message->session_id,
          message->reset_generation,
        });
      publish_status(
        "replan_acknowledged",
        "waiting for the next healthy localization estimate before motion");
    } catch (const std::exception & error) {
      publish_status(
        "replan_rejected",
        std::string("invalid replan acknowledgement: ") + error.what());
    }
  }

  void on_intent(
    const dapier_interfaces::msg::ResearchControlIntent::SharedPtr message)
  {
    latest_intent_sequence_ = message->sequence;
    try {
      const auto measured = measured_state();
      if (!measured) {
        publish_rejection(
          "joint state, odometry and interlocks are required",
          message->sequence);
        return;
      }
      const auto intent = to_core(*message);
      const auto command = controller_.evaluate(
        intent,
        *measured,
        safety_context(),
        steady_now_ns());
      publish_safe_command(command);
      publish_status(safety_decision_name(command.decision), command.reason);
    } catch (const std::exception & error) {
      publish_rejection(error.what(), message->sequence);
    }
  }

  void on_safe_stop_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      controller_.acknowledge_safe_stop();
      response->success = true;
      response->message = "safe stop cleared; operator enable remains required";
      publish_status("safe_stop_cleared", response->message);
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      publish_status("safe_stop_reset_rejected", response->message);
    }
  }

  void on_watchdog()
  {
    const auto now = steady_now_ns();
    if (controller_.motion_armed()) {
      if (!localization_expiry_ns_ || now > *localization_expiry_ns_) {
        handle_external_stop("localization receiver-local TTL expired", 0);
      } else if (
        localization_received_ns_ &&
        now - *localization_received_ns_ > localization_config_.maximum_interarrival_ns)
      {
        handle_external_stop("localization interarrival watchdog expired", 0);
      }
    }

    const auto measured = measured_state();
    if (measured) {
      const auto watchdog = controller_.tick(*measured, now);
      if (watchdog) {
        publish_safe_command(*watchdog);
        publish_status(safety_decision_name(watchdog->decision), watchdog->reason);
      }
    }
  }

  void handle_external_stop(const std::string & reason, std::uint64_t sequence)
  {
    const auto measured = measured_state();
    if (measured && controller_.motion_armed()) {
      try {
        auto command = controller_.request_safe_stop(*measured, reason);
        command.sequence = sequence;
        publish_safe_command(command);
        publish_status("safe_stop", reason);
        return;
      } catch (const std::exception & error) {
        publish_rejection(
          std::string("failed to construct measured safe stop: ") + error.what(),
          sequence);
        return;
      }
    }
    publish_status("hold", reason);
  }

  void publish_rejection(const std::string & reason, std::uint64_t sequence)
  {
    dapier_safety_core::SafeCommand command;
    command.decision = dapier_safety_core::SafetyDecision::kReject;
    command.reason = reason;
    command.sequence = sequence;
    publish_safe_command(command);
    publish_status("reject", reason);
  }

  void publish_safe_command(const dapier_safety_core::SafeCommand & command)
  {
    dapier_interfaces::msg::SafeCommand message;
    message.decision = safety_decision_name(command.decision);
    message.reason = command.reason;
    message.kind = intent_kind_name(command.kind);
    message.sequence = command.sequence;
    message.joint_names = command.joint_names;
    message.joint_position_rad = command.joint_position_rad;
    message.joint_max_velocity_rad_s = command.joint_max_velocity_rad_s;
    message.base_linear_x_mps = command.base_linear_x_mps;
    message.base_angular_z_rad_s = command.base_angular_z_rad_s;
    message.expires_at_monotonic_ns = command.expires_at_monotonic_ns;
    message.dispatch_allowed = command.dispatch_allowed;
    message.safe_stop_latched = controller_.safe_stop_latched();
    message.hardware_execution = false;
    safe_command_pub_->publish(message);
  }

  void publish_status(const std::string & state, const std::string & reason)
  {
    dapier_interfaces::msg::SafetyStatus message;
    message.state = state;
    message.reason = reason;
    message.operator_enabled = controller_.operator_enabled();
    message.estop_healthy = controller_.estop_healthy();
    message.safe_stop_latched = controller_.safe_stop_latched();
    const auto measured = measured_state();
    const auto now = steady_now_ns();
    message.measured_state_fresh =
      measured && measured->received_monotonic_ns <= now &&
      now - measured->received_monotonic_ns <=
      controller_config_.measured_state_timeout_ns;
    message.localization_allows_motion =
      localization_decision_ == dapier_localization_core::MotionDecision::kProceed;
    message.has_last_intent_sequence = latest_intent_sequence_.has_value();
    message.last_intent_sequence = latest_intent_sequence_.value_or(0);
    message.has_last_localization_sequence = latest_localization_sequence_.has_value();
    message.last_localization_sequence = latest_localization_sequence_.value_or(0);
    message.hardware_execution = false;
    status_pub_->publish(message);
  }

  dapier_safety_core::SafetyControllerConfig controller_config_;
  dapier_safety_core::SafetyController controller_;
  dapier_localization_core::LocalizationMonitorConfig localization_config_;
  dapier_localization_core::LocalizationMonitor localization_monitor_;

  bool have_joint_state_{false};
  bool have_odometry_{false};
  bool have_interlocks_{false};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  double base_linear_mps_{0.0};
  double base_angular_rad_s_{0.0};
  std::int64_t joint_state_received_ns_{0};
  std::int64_t odometry_received_ns_{0};
  std::int64_t interlock_received_ns_{0};
  dapier_interfaces::msg::InterlockState interlocks_;

  dapier_localization_core::MotionDecision localization_decision_{
    dapier_localization_core::MotionDecision::kReject};
  std::optional<std::int64_t> localization_received_ns_;
  std::optional<std::int64_t> localization_expiry_ns_;
  std::optional<std::uint64_t> latest_localization_sequence_;
  std::optional<std::uint64_t> latest_intent_sequence_;

  rclcpp::Publisher<dapier_interfaces::msg::SafeCommand>::SharedPtr safe_command_pub_;
  rclcpp::Publisher<dapier_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<dapier_interfaces::msg::ResearchControlIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<dapier_interfaces::msg::LocalizationEstimate>::SharedPtr localization_sub_;
  rclcpp::Subscription<dapier_interfaces::msg::ReplanAcknowledgement>::SharedPtr replan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<dapier_interfaces::msg::InterlockState>::SharedPtr interlock_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr operator_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safe_stop_reset_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace dapier_safety_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dapier_safety_bridge::SafetyBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dapier_safety_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
