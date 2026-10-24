# Dual SO-101 handover 검증 기록 — 2026-09-05

`record_id: DAPIER-2026-09-05-robotwin-handover`

오늘은 공식 RoboTwin의 `handover_block`을 그대로 성공 처리하지 않고, 우리 Dual SO-101 크기와
5축 제약에 맞춘 `dapier_handover_block` 한 과제를 끝까지 파고들었다. 아래 결과는 모두 SIM이다.

## 직접 확인한 성공 조건

성공 seed는 3이었다. seed 0~2는 물체가 떨어져 실패했고 성공으로 세지 않았다. 최종 물리 gate는
다음 값을 냈다.

```text
success=True
right_gripper_link impulse=0.0205790363
right_moving_jaw_so101_v1_link impulse=0.0202880371
normal_dot=-0.9617785162
inter_arm_impulse=0.000000
object_z=0.899101 m
```

두 jaw 모두 물체와 접촉하고 접촉 법선이 서로 반대이며, 왼팔이 빠진 뒤에도 물체가 유지됐다.
양팔 사이 충돌 impulse는 0이었다. 가짜 attach/equality는 사용하지 않았다.

## 생성 artifact

원시 파일은 Git에 올리지 않고 `~/RoboTwin/data/dapier_so101_smoke/dapier_handover_block/`
아래에 보관했다.

| artifact | 결과 |
|---|---|
| trajectory | `episode0.pkl`, 625,545 bytes, SHA-256 `d5428115…dd6c56` |
| HDF5 | `episode_0000000.hdf5`, 4,592,025,676 bytes, SHA-256 `52eab065…9a45fc` |
| video | H.264 640×460, 30 fps, 1,256 frames, SHA-256 `d5906672…f87516` |
| canonical NPZ | 148,544,808 bytes, SHA-256 `fe1031d7…357e1b` |

canonical 결과는 1,255 step, 15 Hz였다.

```text
observation_state float32 (1255, 12)
action            float32 (1255, 12)
left_wrist_rgb    uint8   (1255, 3, 240, 320)
top_h201_depth_mm uint16  (1255, 1, 460, 640)
right_wrist_rgb   uint8   (1255, 3, 240, 320)
```

중간 frame의 세 RGB를 직접 열어 노이즈가 사라졌는지 확인했다. H201 depth는 모든 값이 finite이고
양수였으며 중간 frame 범위는 약 155.6~783.7 mm였다.

## 실패에서 배운 점

첫 결과는 물리적으로 성공했지만 OIDN denoiser handle 오류 때문에 RGB가 학습용으로 부적합했다.
ray tracing sample 수를 늘리는 대신 DAPIER task만 raster shader로 바꿨다. 두 번째 결과는 영상이
정상이었지만 drive target을 state/action으로 저장해 왼팔에 0.25246 rad 불연속이 있었다.

세 번째 수집부터 실제 SAPIEN qpos를 저장했다. arm joint의 15 Hz 최대 step은 0.033236 rad이고,
`state[t+1] == action[t]` 오차는 0이었다. gripper 최대 step은 정규화 값 0.058591이었다.
문제가 있던 dataset은 `rejected_rt_oidn`, `rejected_action_gap`,
`rejected_drive_target_state` 아래에 분리해 재사용하지 않게 했다.

## 아직 확인하지 못한 부분

- 한 seed 성공은 강건성 증거가 아니다. 여러 물체 pose와 마찰에서 성공률을 다시 측정해야 한다.
- wrist camera extrinsic/FOV는 CAD 기반 임시값이며 실물 calibration이 필요하다.
- SAPIEN dynamics가 STS3215의 backlash, current limit, 내부 velocity/acceleration profile과 같지 않다.
- canonical NPZ를 ACT로 실제 학습하고 held-out SIM 평가하는 단계는 아직 남았다.
- 실제 follower calibration과 안전 controller를 통한 저속 rollout은 수행하지 않았다.

따라서 이번 완료 판정은 `RoboTwin 물리 handover + 학습 입력 변환`까지다. sim-to-real 완료 판정은
실물 shadow와 저속 반복 성공 뒤에만 내린다.
