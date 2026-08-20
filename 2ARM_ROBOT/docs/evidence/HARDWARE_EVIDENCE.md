# 2026-08-20 실물 검증 evidence

이 폴더는 집에서도 실측 결과를 재분석할 수 있도록 Git에 포함한 비식별 evidence다. 원본
`output/`은 빌드·실행 산출물 보호를 위해 계속 Git에서 제외한다.

## 포함 파일

| 파일 | 내용 | SHA-256 |
|---|---|---|
| `dual_arm_read_only_summary_20260820.json` | USB 고유 시리얼을 제거한 양팔 STS3215 설정/telemetry 요약 | 아래 검증 명령으로 계산 |
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
- 양팔 요약의 역할 이름은 참고 코드 기반 hypothesis이며 실제 motion validation 전에는 제어 권한을
  부여하지 않는다.
- RGB-D는 USB 인식만 확인했고 depth stream이나 calibration data는 없다.
