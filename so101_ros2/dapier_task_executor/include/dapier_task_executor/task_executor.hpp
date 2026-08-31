// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>

namespace dapier_task_executor
{

enum class TaskPhase
{
  kManipulation,
  kCarryReady,
  kNavigation,
  kPlace,
  kStopped,
};

enum class Skill
{
  kPregrasp,
  kApproach,
  kCloseGripper,
  kLift,
  kNavigate,
  kPlace,
  kRecover,
  kStop,
};

struct WorldState
{
  bool simulation_only{true};
  bool base_settled{true};
  bool grasp_verified{false};
  bool observation_fresh{true};
  unsigned int recovery_attempts{0};
};

struct TaskRequest
{
  Skill skill{Skill::kStop};
  std::uint64_t observation_sequence{0};
  // This contract deliberately rejects a request trying to claim execution authority.
  bool execution_authorized{false};
};

struct TaskDecision
{
  bool accepted{false};
  TaskPhase previous_phase{TaskPhase::kStopped};
  TaskPhase next_phase{TaskPhase::kStopped};
  bool command_dispatch_allowed{false};
  bool hardware_execution{false};
  std::string reason;
};

class TaskExecutor
{
public:
  [[nodiscard]] TaskDecision evaluate(const TaskRequest & request, const WorldState & state);
  [[nodiscard]] TaskPhase phase() const noexcept;
  [[nodiscard]] std::uint64_t last_observation_sequence() const noexcept;

private:
  [[nodiscard]] TaskDecision reject(const std::string & reason) const;
  TaskPhase phase_{TaskPhase::kManipulation};
  std::uint64_t last_observation_sequence_{0};
};

[[nodiscard]] const char * to_string(TaskPhase phase) noexcept;
[[nodiscard]] const char * to_string(Skill skill) noexcept;

}  // namespace dapier_task_executor
