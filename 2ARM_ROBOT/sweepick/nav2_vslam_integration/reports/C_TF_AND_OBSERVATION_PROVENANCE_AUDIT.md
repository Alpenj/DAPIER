# C_TF_AND_OBSERVATION_PROVENANCE_AUDIT — TF2 및 센서 출처 감사 (DAPIER C-2, 2026-10-16)

대상: Argon C-1 `6d24284` (`nav2_vslam_integration/`), Navigation `3416c7c`/`b509838`.
수정 커밋: DAPIER `6621b11`, Navigation `c8cc836`(재export·제안 패치만).

## 1. 하드코딩 변환 `camera_link_in_base_footprint_m=(0.073, 0.0, 0.084)`

| 항목 | 값 | 근거 |
|---|---|---|
| C-1 기본값 | (0.073, 0.000, 0.084) | `navigation_bridge.py` 함수 기본 인자 (출처 표기 없음) |
| Waffle Pi URDF `base_footprint→base_link` | (0, 0, 0.010) | `robot_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf` |
| Waffle Pi URDF `base_link→camera_link` | (0.073, −0.011, 0.084) | 같은 파일 |
| launch 정적 TF `camera_link→camera_color_optical_frame` | 평행이동 0, rpy(−π/2, 0, −π/2) | `turtlebot3_visual_slam/launch/vslam_navigation.launch.py` |
| URDF+launch 체인 합성(`base_footprint←camera_color_optical_frame`) | **(0.073, −0.011, 0.094)** | `tf_provenance.compose_static_chain`, 목 그래프에서 tf2 lookup으로 동일 값 확인 |
| 실물 카메라 마운트 | **미측정** | 실제 로봇은 Astra 카메라를 `stl/cam_mount_*.stl` 치구로 장착 — URDF(Pi camera 위치)가 실물을 대표한다는 증거 없음 |

결론: C-1 값은 URDF 체인과도(y −11 mm, z +10 mm) 다르고 실측도 아니다. **상수는 더 이상 쓰지 않는다.**

### 수정
- `transform_optical_pose_to_base_footprint(..., camera_link_in_base_footprint_m=)` 기본값 제거(필수 인자, 수학 회귀 전용).
- `tf_provenance.VerifiedTransform`: `source == TF2_LOOKUP_AT_MESSAGE_TIME`일 때만 `REAL` 등급. 상수·URDF·launch 값은 `MOCK` 등급으로만 생성 가능하고, `build_post_stop_observation(allow_mock=False)`에서 `TF_SOURCE_NOT_A_LOOKUP`으로 거부.
- `ros2_readonly_adapter.SweepickNavReadOnlyNode.lookup_transform_at(parent, child, stamp_s)`: `tf2_ros.Buffer.lookup_transform(..., Time(stamp), timeout 0.2 s)`. `LookupException/ConnectivityException/ExtrapolationException`이면 `None` → `TF_UNAVAILABLE_AT_MESSAGE_TIME`. "latest" 조회나 기본값 대체 없음.
- 변환 시각 ≠ 센서 시각(±0.05 s)이면 `TF_STAMP_DOES_NOT_MATCH_SENSOR_STAMP`, 프레임 사슬 불일치면 `TF_FRAME_CHAIN_MISMATCH`.
- `base_footprint↔arm_base`: C-1과 같이 `HardwareReadinessState.base_to_arm_tf_calibrated=False`이면 `DIRECT_BASE_TO_ARM_METRIC_TF` 거부 유지(test_11). 생성하지 않음.

목 그래프 dry-run(`C_ROS2_READONLY_DRYRUN.json`)에서 실제 tf2 경로로 조회된 값: translation (0.073, −0.011, 0.094), rotation (−0.5, 0.5, −0.5, 0.5) — MOCK 정적 TF이므로 실물 근거가 아니라 **경로 동작 확인**이다.

## 2. `CameraObservation3D`의 `clock_id`·프레임 시퀀스·Inlier·RMS 출처

### 2.1 C-1 상태
| 필드 | C-1 출처 | 판정 |
|---|---|---|
| `clock_id` | `base_stop.clock_id`를 관측에 복사 | 합성(관측 자체의 클록 아님) |
| `frame_seq` | 호출자 인자 `post_stop_frame_seq` | 합성 |
| `captured_at_s` | 호출자 인자 `post_stop_detected_at_s` | 합성 |
| `inlier_count` | 기본값 85 (초기 관측 60) | **합성 기본값** |
| `rms_m` | 기본값 0.008 (초기 0.012) | **합성 기본값** |
| `is_ema_filtered_from_motion`, `is_persistence_latched` | 기본값 False | **합성 기본값** — 실제 검출기는 항상 EMA |

### 2.2 기존 토픽이 실제로 주는 것 (`object_detector_3d.py`, Jazzy)
- `/docking/target_object_pose` (`geometry_msgs/PoseStamped`, 20 Hz, frame `camera_color_optical_frame`)
  - `header.stamp = self.get_clock().now()` — **발행 시각**. 센서(깊이 프레임) 시각이 아니다. `last_depth_stamp`는 노드 내부에만 있다.
  - position = `smoothed_center` (EMA `ema_alpha=0.25`), 검출 실패 후 0.5 s 동안 이전 값 재발행(persistence latch).
  - ROS 2 헤더에 seq 없음. inlier·RMS 없음.
  - **같은 토픽에 두 번째 발행자**: `precision_docking_controller_node.py`도 `/docking/target_object_pose`를 `base_footprint` 프레임으로 발행한다. 프레임만으로 발행자를 구분해야 하며, 어댑터는 `frame_id`와 발행 노드명을 함께 기록한다.
- `/docking/object_detection_status` (`std_msgs/String`): `"... inliers=N, fps=F"` — inlier만 문자열로 있음. RMS 없음(`fit_plane_geometric_score`가 계산하지 않음).
- `/qwen/target_bbox` latch 180 s, 신선 4 s: 토픽에 latch 여부 없음.
- `/camera/color/camera_info`·image: 센서 프레임 시각의 유일한 출처. 어댑터는 camera_info 헤더 시각으로 수신 순번(`sensor_frame_index_at`)을 만든다.

따라서 **현재 토픽만으로는 "정지 후 새 카메라 검출"임을 증명할 수 없다.** 새 `PoseStamped`는 새 검출이 아니다.

### 2.3 C-2 규칙
- `DetectionProvenance`: `sensor_stamp_s`, `anchor_source`, `is_ema_filtered`, `is_persistence_latched`, `inlier_count`, `rms_m`, `detector_mode`가 모두 있어야 `verified`. 출처가 `POSE_HEADER_ONLY`이면 항상 `POST_STOP_OBSERVATION_UNVERIFIED`.
- `observation_id = topic@sensor:<ns>`: 같은 센서 프레임으로 만든 두 번째 `PoseStamped`는 `SensorFrameLedger`·`PostStopReobserveGate`에서 `REPUBLISHED_OBSERVATION_SAME_SENSOR_FRAME`으로 거부.
- `clock_id`는 어댑터의 ROS 클록(`ros2:ROS_TIME(use_sim_time=…)`)으로 관측·정지 증거·TF에 같은 값이 붙고, 다르면 `CLOCK_DOMAIN_MISMATCH`.
- 초기(이동 전) 관측도 같은 규칙; 없으면 `None`으로 두고 파이프라인은 REOBSERVE에서 멈춘다(팔 요청 0).

### 2.4 최소 변경안 (Navigation, **미적용**)
`robot_ws/src/llm_semantic_docker/proposals/C2_object_detector_3d_provenance_sidecar.patch` (`git apply --check` 통과, 44줄 추가·3줄 변경):
1. `PoseStamped.header.stamp`를 그 주기에 쓴 깊이 프레임의 `header.stamp`로.
2. `/docking/target_object_provenance` (`std_msgs/String` JSON) 사이드카: `pose_stamp_s`, `sensor_stamp_s`, `anchor_source`(qwen_fresh/hsv/yolo_world/qwen_latched), `is_ema_filtered`, `is_persistence_latched`, `inlier_count`, `rms_m`, `detector_mode`, `qwen_bbox_stamp_s`, `ema_alpha`, `persistence_timeout_sec`.
3. `fit_plane_geometric_score`에서 이미 가진 inlier 점으로 SVD 평면 RMS 기록.

토픽 타입·QoS·EMA·latch·TF·도킹 FSM은 바꾸지 않는다. 사이드카가 있을 때만 어댑터가 소비하고, 없으면 거부가 유지된다. **추가로 필요한 운영 결정**: EMA가 항상 켜져 있으므로 정지 후 첫 샘플도 `is_ema_filtered=True`가 된다(목 그래프 재발행 모드에서 REOBSERVE 거부로 재현). 정지 후에는 EMA 상태를 리셋하거나 raw 샘플을 함께 싣는 선택이 Navigation 팀에 남아 있다.

## 3. Nav2 도착·베이스 정지 증거 출처
- 도착: `/navigate_to_pose/_action/status`(`action_msgs/GoalStatusArray`, reliable+transient_local) `STATUS_SUCCEEDED`, `/docking/status` `STATE=DOCKED`. `manipulation_succeeded`는 항상 False(계약 유지).
- 정지: `/odom` twist(≤0.005 m/s, ≤0.01 rad/s) 연속 구간, 마지막 관측 `/cmd_vel`(`TwistStamped`)이 0이거나 settle 시간 이상 조용함, Nav2 goal에 ACCEPTED/EXECUTING/CANCELING 없음, 도킹 상태가 베이스를 구동하는 상태가 아님. **어댑터는 `/cmd_vel`을 발행하지 않는다**(`user_publishers_created=0`).
- `stopped_at_s = max(첫 정지 odom 샘플, 도착 시각)`: 도킹 DOCKED 선언은 정지 뒤에 오므로 정지 확정 시각은 도착 이후로 둔다. settle은 그 뒤 시간만 센다.
- `last_nav_frame_seq`: 정지 확정 시각 이전 camera_info 프레임 수. 정지 후 관측의 `frame_seq`는 사이드카 `sensor_stamp_s`에 해당하는 프레임 순번.
- 최종 자세: `map←base_footprint` tf2 lookup이 되면 `final_pose_frame="map"`, 안 되면 `/odom` 자세를 `"odom"`으로 표기(목 dry-run에서 두 경우 모두 발생).

## 4. 검증
`tests/test_c2_provenance_and_tf.py` 13개, `tests/test_c2_ros2_readonly_adapter.py` 7개, `test_12b`: 상수 변환 거부, TF 누락 거부, 과거 TF 거부, MOCK TF 실모드 거부/목모드 허용, 헤더만 있는 관측 거부, RMS 누락 거부, 같은 센서 프레임 재발행 거부, EMA/latch 거부, 게이트 소비, 파이프라인에서 재발행 시 REOBSERVE 정지·팔 요청 0. 전체 57/57 통과.
