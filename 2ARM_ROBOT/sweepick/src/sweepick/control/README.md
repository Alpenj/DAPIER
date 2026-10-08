# sweepick control

좌우가 공유하는 tick/q 매핑·유한 trajectory run/grip·Jaw contact/hold·collision preview. 실제 write는 기존 executor 경로에 있으며 새 per-cycle writer를 추가하지 않는다. gate·profile 숫자와 판정은 유지한다.

| 파일 | 기존 대응 |
|---|---|
| [sweepick_collision_preview.py](sweepick_collision_preview.py) | sweepick_collision_cert |
| [sweepick_command_application.py](sweepick_command_application.py) | sweepick_real_exec |
| [sweepick_execution_gates.py](sweepick_execution_gates.py) | sweepick_gates |
| [sweepick_gripper_contact_hold.py](sweepick_gripper_contact_hold.py) | sweepick_grasp |
| [sweepick_gripper_contact_inspection.py](sweepick_gripper_contact_inspection.py) | sweepick_contact01 |
| [sweepick_joint_command_mapping.py](sweepick_joint_command_mapping.py) | sweepick_real_command |
| [sweepick_joint_kinematics.py](sweepick_joint_kinematics.py) | sweepick_kin |
| [sweepick_motion_progress.py](sweepick_motion_progress.py) | sweepick_progress |
| [sweepick_servo_access.py](sweepick_servo_access.py) | sweepick_commission |
| [sweepick_trajectory_executor.py](sweepick_trajectory_executor.py) | sweepick_move_a |
| [sweepick_trajectory_profile.py](sweepick_trajectory_profile.py) | sweepick_trajectory |

공통 장치 writer는 `control/sweepick_trajectory_executor.py`의 기존 `run/grip`을 재사용한다. 이번 정리는 import·자산 경로와 이름만 변경한다. 시험·실행 범위는 [제품 README](../../../README.md)를 따른다.
