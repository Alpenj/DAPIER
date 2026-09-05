# 결과·근거·미검증 범위

[루트](../../README.md) · [검토 안내](README.md)

이 문서는 **기존 저장소 기록과 PR 보고를 분류한 안내**입니다. 이번 문서 편집에서 로봇, 시뮬레이터, 학습 또는 전체 테스트를 새로 실행하지 않았습니다. 날짜·장치·branch·revision이 다른 결과를 합산하지 않습니다.

## 1. 기준 버전

| 구분 | 기준 | 해석 |
|---|---|---|
| 기존 main | `c8e1eb0dc6345c6e38f1169e3e1e1b43a297e116`, 2026-09-01 PR #39 병합 | [당시 README](https://github.com/Alpenj/DAPIER/blob/c8e1eb0dc6345c6e38f1169e3e1e1b43a297e116/README.md)의 실물 요약은 8월 20일 기록 |
| 최신 검토한 개발 PR | [#51](https://github.com/Alpenj/DAPIER/pull/51), `3ff180aff3eb312b91a2e2c6ea8e8d6b734148b3` | 2026-09-05 조회 시 open·Draft·미병합. base는 `pro/mujoco-shoe-followup-01` |

PR #51의 현재 코드는 [고정 revision](https://github.com/Alpenj/DAPIER/tree/3ff180aff3eb312b91a2e2c6ea8e8d6b734148b3)으로 확인합니다. 다른 PR의 결과를 이 PR 또는 main의 결과로 옮겨 적지 않습니다.

## 2. main에 기록된 상태

| 영역 | 기록된 확인 | 한계·남은 작업 |
|---|---|---|
| SO-101 SIM | G0의 6축 순서·단위·frame 계약, 조정된 MuJoCo G1 scripted pick-and-lift | 단위·기구학·SIM 통과와 실물 성공은 다름 |
| SO-101 policy | corrected contact에서 wrist-only v2의 unseen seeds 2100..2119, 11/20 | 80% release gate 미달; 실물 준비 gate도 통과 아님 |
| SO-101 HW, 8/20 | leader/follower serial board와 motor ID 1~6 응답 | follower calibration 실패; 당시 teleoperation 금지 |
| SO-101 ROS 2 | 계약·calibration 변환·제한 계산·합성 JointState safe teleop | 이 기준선의 실제 motor driver는 미구현 |
| 카드 딜러 | planner·episode manifest·one-card 기구학 baseline | 실제 카드·흡착·양팔 동작 미검증 |
| TurtleBot3 | Jazzy/gz-sim SLAM·Nav2 end-to-end 기록 | 그 결과를 실물 자율주행 성공으로 해석하지 않음 |
| 커스텀 ros_dd | 7개 package build·gz-sim·TF·Cartographer 학습 기록 | hexa scale 복원 뒤 world와 저장 map 불일치, Nav2 재검증 필요 |
| 2ARM 초기 HW | JDcobot200 두 팔의 STS3215 12개 read-only telemetry, TurtleBot3 stationary baseline·들린 바퀴 응답 | 당시 팔 동작·depth stream·신발 집기 미검증 |
| 4축 로봇암 | RViz/Gazebo와 Arduino Uno 서보 제어를 별도 보관 | SO-101 또는 이동형 양팔의 결과와 합치지 않음 |

근거 입구: [SO-101](../../so101/README.md), [2ARM](../../2ARM_ROBOT/README.md), [하드웨어 evidence](../../2ARM_ROBOT/docs/evidence/HARDWARE_EVIDENCE.md), [TurtleBot3](../../turtlebot3_ws/README.md), [ros_dd](../../ros_dd_ws/README.md).

## 3. 폐기한 성능과 선택하지 않은 접근

기존 contact model의 held-out `14/20` 두 세트는 finger pad가 cube 안으로 최대 약 8.3 mm 들어간 문제가 측정되어 현재 대표 성능에서 제외했습니다. visible pad/cube contact, bilateral contact 기록과 1 mm penetration gate를 적용한 뒤의 결과가 `11/20`입니다.

corrected IK teacher의 초기 30 episode와 후속 100 episode 수집 gate 통과는 **데이터 수집 단계**의 결과입니다. 새 student와 mixed rehearsal checkpoint는 기준선보다 나빠 선택하지 않았고, 기존 v2 policy와 corrected contact model을 유지했습니다.

| 추가 기능·실험 | 기록된 의미 | 확대 해석하지 않을 것 |
|---|---|---|
| action smoothing | chunk 경계의 target jump 감소 | 같은 seed 성공 수 증가로 쓰지 않음 |
| human intervention | policy/human 교정 evidence 보존 | 곧바로 native LeRobot training dataset이라고 쓰지 않음 |
| parallel rollout | 실패 trace 수집에 활용 | 현재 GPU에서 더 빠르다는 결과가 아님 |

세부 실험 흐름과 미선택 checkpoint는 [SO-101 허브](../../so101/README.md)에서 확인합니다.

## 4. PR #51의 RoboTwin SIM 보고

PR 본문의 보고 범위는 official-arm URDF에서 12 active joints를 구성한 SAPIEN embodiment, 충돌 geometry 정리, 좌측 파지·들기 → 우측 접근 → 접촉 전달 → 좌측 release·후퇴와 canonical episode 변환입니다.

- 성공으로 보고한 SIM seed는 **3 하나**이며, 실패한 seed 0~2는 성공 수에 포함하지 않았습니다. 이 기록으로 안정적인 다회 성공률이나 일반화 성능을 계산하지 않습니다.
- 최종 gate 보고는 우측 두 jaw 접촉, opposing normals -0.96178, inter-arm impulse 0, object z 0.899101 m입니다. 가짜 attachment/equality constraint를 쓰지 않았다고 보고합니다.
- 출력은 H.264 1,256 frames와 HDF5 1,255 state/action steps입니다. measured qpos를 기록하고 canonical 12D state/action·좌우 RGB 320×240·H201 640×460 uint16-mm depth로 변환했다고 보고합니다.

이는 PR 보고의 정리이며 이번 작업의 독립적인 물리 재검증이 아닙니다. 특히 보고에서 실제 파일을 변환했다는 뜻과 **실물 로봇 episode**를 수집했다는 뜻을 혼동하지 않습니다. 이 절의 전달 결과는 SIM입니다.

남은 항목은 multi-seed·실측 범위 randomization, 이 데이터로 수행한 ACT 전체 학습·held-out 평가, 실제 카메라 calibration, STS3215 동역학과 mapping, shadow inference·저속 rollout, 실제 뚜껑 열기·신발 꺼내기입니다. Draft를 완료 제품으로 표시하지 않습니다.

## 5. 실물 단계의 기록 형식

과거 main에서 calibration이 `MIN=POS=MAX=2047`로 실패한 기록을 삭제하지 않습니다. 장치·프로필이 달라진 후속 기록은 별도 날짜와 revision으로 추가해야 합니다. 카메라·관절·dataset smoke, 한 step 학습, 전체 정책 평가와 실물 과제 성공을 분리합니다.

각 결과에는 다음 필드를 남깁니다.

```text
실행일 / 수행자:
branch / commit:
환경: SIM | MOCK | HW | OFFLINE
장치·모델·데이터 버전:
입력·명령·seed·분할:
성공 기준 / 중단 기준:
예상값 / 측정값 / artifact:
실패·제외 항목과 이유:
이번 결과로 말할 수 없는 것:
다음 검증:
```

장치 serial·인증정보·개인 calibration과 원시 데이터는 공개 문서에 붙이지 않습니다. 저장된 측정값과 명령값을 구분하고, 이후 결과가 추가되더라도 과거 보고의 기준 revision은 유지합니다.
