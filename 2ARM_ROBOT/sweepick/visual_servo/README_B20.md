# B20 Right Reference Readiness

record_id: `DAPIER-2026-10-15-right-reference-readiness`

B19 `3be52c8`을 기준으로 첫 REAL 비접촉 X 정렬의 관측 계약, 잘림 회귀, 빠른 계산 경로와 A 저장 관측용 어댑터를 정리했다. 현재 저장 자료에는 물리 Pad 기준과 확인된 비접촉 목표 자세가 없어 REAL correction은 생성하지 않는다. 현장 측정과 B21은 시작하지 않았다.

광학 보정의 변환 방향은 solve/runtime에서 일치한다. 기존 K/D/T를 유지하고 저장 board points로 재계산했을 때 holdout의 per-frame PnP RMS는 0.1903/0.2493 px지만, 고정 board projection RMS는 25.2902/32.8706 px이고 3D 자세 일관성 잔차는 23.3569/33.0110 mm다. 8개 recorded FK와 기존 모델 FK는 수치상 일치해도 실제 mapping·mount·optical 정확도는 입증되지 않는다. 원인을 하나로 확정하지 않고 `FIELD_VALIDATION_REQUIRED`로 남겼다. 작은 PnP RMS를 TCP 정확도로 해석하지 않았다.

잘린 centroid는 raw diagnostic으로만 남기고 보정에서 제외했다. 정상 X-only 합성 fixture는 Y target과 graph 없이 계산된다. 부분 edge/hidden face용 제어를 새로 만들지 않았다. 기존 RGB surface를 재사용하고 graph는 `include_geometry=true`에서만 계산한다. source cache는 path와 SHA에 바인딩되어 있다.

CPU 결과는 [benchmark JSON](reports/B20_FAST_PATH_BENCHMARK.json)의 동일 영상별 p50/p95를 사용한다. B19 원본 전체 계산, 빠른 feature, 빠른 feature+servo, 전체 A-record 어댑터, 상세 geometry를 같은 프로세스와 OpenCV thread 1개에서 측정했다. 모듈 초기화 비용과 함수별 workload를 분리했으며 서로 합산하지 않는다. motor 이동·정착·camera 수집·물리 cycle 단축은 측정하지 않았다. A의 gain·motor 속도·STOP·collision margin·reach tolerance는 그대로다.

기존 B19 23개를 현재 코드에서 다시 통과했고 새 단위검사 6개와 실제 저장 근거 검사 2개가 통과했다. B17/B18 59개는 관련 실행 의존성 67개와 입력 SHA 217개가 그대로라 기존 PASS 근거를 재사용했다. 총 90개 근거를 보존했으며, 이번 실행은 31개다. 실제 저장 관측 14개, 잘림 negative 2개 모두 correction null/실행 권한 false를 유지한다. 장치 open·motor command·executor·학습·MuJoCo step은 0회다.

```bash
cd 2ARM_ROBOT/sweepick
python -B -m unittest visual_servo.test_b19 visual_servo.test_b20 -v
python -B visual_servo/run_b20.py --inputs "$B20_PRIVATE_INPUTS" --output "$B20_NEW_OUTPUT"
python -B tests/check_public_snapshot.py
```

기존 Python 3.12.3, NumPy 2.2.6, OpenCV 4.13.0, MuJoCo 3.3.7 환경을 재사용했다. 새 의존성을 설치하지 않았다. private manifest는 두 세션의 `session/a_source`, 원본 B19 `b19_right_source`, `original_calibration`, `route_bound_calibration`, `optical_source`, `optical_dataset`, `b16_root`를 지정한다. 출력 폴더는 새 경로여야 한다. 원본 REAL 자료를 합성하거나 calibration을 다시 solve하지 않는다.

요청한 산출물:

- [물리 기준 A/B 판정 및 최소 현장 근거](B20_RIGHT_REFERENCE_DECISION.md)
- [기존 optical holdout 감사](reports/B20_OPTICAL_HOLDOUT_AUDIT.json)
- [첫 REAL X 측정 계약](B20_RIGHT_X_MEASUREMENT_CONTRACT.md)
- [특징 유효성 회귀](reports/B20_FEATURE_VALIDITY_REGRESSION.json)
- [빠른 경로 CPU 비교](reports/B20_FAST_PATH_BENCHMARK.json)
- [A Shadow 인터페이스](B20_A_SHADOW_INTERFACE.md)

A Native·Console·calibration·ACT·main·통합 브랜치는 변경하지 않았다. 목표 영상/실제 방향 근거와 정확한 현장 실행 승인이 확보되기 전에는 실물 시험을 시작하지 않는다.

동일 CPU·동일 영상 비교 결과(ms, 각 셀 p50 / p95):

| 입력 | B19 전체 | 빠른 특징 | 빠른 특징+servo | 전체 A 어댑터 | 상세 geometry 포함 |
|---|---:|---:|---:|---:|---:|
| 이전 RECEIVE | 201.12 / 227.64 | 11.91 / 13.09 | 12.26 / 29.24 | 22.81 / 55.31 | 163.31 / 165.10 |
| 추가 RECEIVE | 191.22 / 195.29 | 11.31 / 12.12 | 11.54 / 12.53 | 23.58 / 57.85 | 153.11 / 157.65 |
| 추가 관찰 후보 | 189.69 / 209.44 | 9.22 / 10.39 | 8.78 / 9.62 | 17.39 / 35.10 | 148.35 / 150.77 |

이 표는 순수 계산만 비교한다. 전체 A 어댑터에는 packet/영상 binding과 provenance 구성이 추가된다. 위 시간은 모터나 실물 정렬 cycle이 아니다.
