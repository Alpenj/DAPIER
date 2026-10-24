# 2026-08-20 실물 검증 evidence

이 폴더는 집에서도 실측 결과를 재분석할 수 있도록 Git에 포함한 비식별 evidence다. 원본
`output/`은 빌드·실행 산출물 보호를 위해 계속 Git에서 제외한다.

## 포함 파일

| 파일 | 내용 | SHA-256 |
|---|---|---|
| `dual_arm_read_only_summary_20260820.json` | USB 고유 시리얼을 제거한 양팔 STS3215 설정/telemetry 요약 | 아래 검증 명령으로 계산 |
| `dual_so101_symmetric_smoke_20260903.json` | 동일 SO-101 좌·우 팔의 비식별 ±3도 과거 로그 요약(사용자 확인 아님) | `d15fbe5726846a732edbb56f711f293677ccf22247b8f896c1f6d3046c53306f` |
| `dual_so101_sim_real_comparison_20260903.json` | 과거 ±3도 trace의 실측 주기로 재생한 MuJoCo 비교 | `6a2a23238bc834c5b5ad3477c9fdb5a0a8e0db4d23f976a7e2259ddafeed1f00` |
| `turtlebot3_stationary_baseline_report_20260820.json` | 30초 stationary odom 통계와 원본 samples hash | `51f358c1e37a5f413e9ff99bab4c45ce0e2db517a6aae5bed438e3ece589ad4c` |
| `turtlebot3_stationary_odom_samples_20260820.jsonl` | 603개 stationary odom samples | `ebfd65966836533f36da3f757b27443651570558bcaa0b059c0a6a0cf31b037f` |
| `turtlebot3_wheels_off_ground_20260820_v02.json` | 바퀴가 들린 상태의 24개 속도 command stage 원본 | `a7435357a4ae4f37b9b99fa147b7d59f47d7840fa9e56c058287dca0ac21ff6f` |
| `turtlebot3_wheels_off_ground_analysis_20260820.json` | deadband, 양쪽 바퀴 최소값, 추종 상한 분석 | `af5735a8e657673d47e9c72567fe5fadfd803930eacef16fc79cebf4c140ffa3` |

검증:

```bash
cd ~/DAPIER/2ARM_ROBOT
sha256sum docs/evidence/*
```

## 해석 주의

- 바퀴 시험은 사용자가 바퀴를 지면에서 든 것을 확인한 뒤 수행했다.
- 0.26m/s와 1.82rad/s는 시험한 제조사 command ceiling이지 권장 운용 상한이 아니다.
- 5% 추종 오차와 5% 좌우 비대칭 기준의 권장 상한은 0.20m/s와 1.20rad/s다.
- `/battery_state.current`는 전 stage에서 0이라 실제 전류 측정값으로 사용할 수 없다.
- 과거 코드의 leader/follower 표기는 LeRobot 보정 파일 저장 경로를 가리킬 뿐 실물 역할이 아니다.
  좌·우 모두 동일한 SO-101이며 controller serial 기반 stable alias와 각 팔의 고유 보정으로 구분한다.
- 2026-09-03 로그에는 두 팔 shoulder-pan의 +3도/-3도 왕복 응답과 종료 후 토크 해제가 남아 있다.
  사용자가 화면으로 확인한 commissioning은 아니며, 제한된 과거 joint smoke 이상의 근거로
  사용하지 않는다.
- 2026-08-20 요약의 RGB-D 근거는 USB 인식까지다. 이후 Astra stream 복구 결과는 별도 실행
  원장에 기록했으며 intrinsics/extrinsics 실측은 여전히 남아 있다.
- 2026-09-03 센서 역할은 Astra S=`front_rgbd`(Visual SLAM),
  HP-ASC-H201=`workspace_rgbd`(top-view)로 분리했다. H201의 USB 인식과 UVC frame
  capture는 확인했지만 SDK depth unit·intrinsics·동시 4카메라 부하는 아직 실측 근거가 아니다.
