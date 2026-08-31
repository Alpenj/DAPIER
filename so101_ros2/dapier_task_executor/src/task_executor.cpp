// SPDX-License-Identifier: Apache-2.0

#include "dapier_task_executor/task_executor.hpp"

namespace dapier_task_executor
{

namespace
{

bool is_arm_skill(Skill skill)
{
  return skill == Skill::kPregrasp || skill == Skill::kApproach ||
         skill == Skill::kCloseGripper || skill == Skill::kLift ||
         skill == Skill::kPlace;
}

}  // namespace

const char * to_string(TaskPhase phase) noexcept
{
  switch (phase) {
    case TaskPhase::kManipulation: return "manipulation";
    case TaskPhase::kCarryReady: return "carry_ready";
    case TaskPhase::kNavigation: return "navigation";
    case TaskPhase::kPlace: return "place";
    case TaskPhase::kStopped: return "stopped";
  }
  return "unknown";
}

const char * to_string(Skill skill) noexcept
{
  switch (skill) {
    case Skill::kPregrasp: return "pregrasp";
    case Skill::kApproach: return "approach";
    case Skill::kCloseGripper: return "close_gripper";
    case Skill::kLift: return "lift";
    case Skill::kNavigate: return "navigate";
    case Skill::kPlace: return "place";
    case Skill::kRecover: return "recover";
    case Skill::kStop: return "stop";
  }
  return "unknown";
}

TaskDecision TaskExecutor::reject(const std::string & reason) const
{
  return TaskDecision{false, phase_, phase_, false, false, reason};
}

TaskDecision TaskExecutor::evaluate(const TaskRequest & request, const WorldState & state)
{
  if (request.skill == Skill::kStop) {
    const TaskPhase previous = phase_;
    phase_ = TaskPhase::kStopped;
    return TaskDecision{true, previous, phase_, false, false, "stop proposal accepted; no command dispatched"};
  }
  if (!state.simulation_only) {
    return reject("hardware execution is outside this simulation-only contract");
  }
  if (request.execution_authorized) {
    return reject("task requests cannot grant command or hardware execution authority");
  }
  if (!state.observation_fresh) {
    return reject("stale observation rejected");
  }
  if (request.observation_sequence <= last_observation_sequence_) {
    return reject("observation sequence must increase monotonically");
  }
  if (state.recovery_attempts > 2U && request.skill == Skill::kRecover) {
    return reject("recovery budget exhausted");
  }
  if (is_arm_skill(request.skill) && !state.base_settled) {
    return reject("arm proposal rejected while base is moving");
  }
  if (phase_ == TaskPhase::kStopped) {
    return reject("stopped executor requires a new explicit session");
  }

  const TaskPhase previous = phase_;
  TaskPhase next = phase_;
  std::string reason;
  switch (request.skill) {
    case Skill::kPregrasp:
    case Skill::kApproach:
    case Skill::kCloseGripper:
      if (phase_ != TaskPhase::kManipulation) {
        return reject("grasp preparation is only valid during manipulation");
      }
      reason = std::string(to_string(request.skill)) + " proposal accepted for simulation review";
      break;
    case Skill::kLift:
      if (phase_ != TaskPhase::kManipulation || !state.grasp_verified) {
        return reject("lift requires a verified grasp during manipulation");
      }
      next = TaskPhase::kCarryReady;
      reason = "lift proposal accepted; carry-ready gate entered";
      break;
    case Skill::kNavigate:
      if (phase_ != TaskPhase::kCarryReady || !state.grasp_verified || !state.base_settled) {
        return reject("navigation requires a settled base and verified carry-ready grasp");
      }
      next = TaskPhase::kNavigation;
      reason = "navigation proposal accepted for simulation review";
      break;
    case Skill::kPlace:
      if (phase_ != TaskPhase::kNavigation || !state.base_settled) {
        return reject("place requires completed navigation and a settled base");
      }
      next = TaskPhase::kPlace;
      reason = "place proposal accepted for simulation review";
      break;
    case Skill::kRecover:
      next = TaskPhase::kManipulation;
      reason = "recovery proposal accepted for simulation review";
      break;
    case Skill::kStop:
      return reject("unreachable stop branch");
  }

  phase_ = next;
  last_observation_sequence_ = request.observation_sequence;
  return TaskDecision{true, previous, next, false, false, reason};
}

TaskPhase TaskExecutor::phase() const noexcept
{
  return phase_;
}

std::uint64_t TaskExecutor::last_observation_sequence() const noexcept
{
  return last_observation_sequence_;
}

}  // namespace dapier_task_executor
