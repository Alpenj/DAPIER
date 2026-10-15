# B19 오른팔 상대 Visual Servo와 성능 검수

record_id: `DAPIER-2026-10-15-b19-right-relative-servo`

저장 REAL 기록에서 오른손목 특징과 실제 관절 피드백을 대조하고, 실행 권한이 없는 `RightRelativeServoProposal`을 구현했다. 기준은 B18 `c44dd1d83083ac0496181c8f97690b2d717ba4f6`이다. A Native·Console·Owner·STOP·motor writer는 변경하지 않았다.

현재 오른손목에서 계산되는 색상 영역·윤곽·면 그래프와, 실제 수신 Pad에 대한 상대 제어 오차를 구분한다. REAL 7개 기록 중 5개에 색상 영역이 보이며 이 중 1개는 복귀 관측이다. 물리적으로 대응이 확인된 Pad 기준과 REAL 상대 Jacobian이 없어, REAL 제안 7개 모두 제어 차원 0·보정량 null·`NEEDS_EVIDENCE`다. X/Y·깊이·회전을 모델 응답으로 채우지 않았다.

광학 K/D/T 보정 파일은 존재하며 한 관측에서 calibrated PnP 후보가 생성됐다. 해당 fit의 재투영 RMS 약 0.615 px는 물체 corner·Pad의 물리적 대응이나 TCP 정확도를 입증하지 않는다. 보정은 offline solved/not field verified 상태다. 기존 wrist surface의 고정 registration 문자열을 광학 보정 파일 부재로 해석하지 않았다.

추가 오른팔 관측 이동 두 개는 원래 `TARGET_NOT_REACHED_HOLDING` 결과를 유지한다. 이후 RGB에 물체 후보가 보이거나 정상 복귀가 확인된 사실과 수신 위치 정렬 성공은 구분했다. Native의 coarse `success` 분류도 `PARTIAL_OR_DIAGNOSTIC` 결과를 상대 정렬 성공으로 바꾸지 않는다.

## 구현과 인터페이스

| 코드 | 역할 |
|---|---|
| [features.py](features.py) | B18 camera/state/source/hash 결합을 좌·우 wrist에 재사용; 기본값은 기존 left |
| [right_relative.py](right_relative.py) | 오른팔 RECEIVE 특징·Pad/held 근거와 순수 계산 제안; left PICK 특징과 분리 |
| [servo_proposal.py](servo_proposal.py) | 기존 Jacobian 검증 유지, 제안에 사용한 축의 방향과 정의된 전체 task 오차 확인 |
| [performance.py](performance.py) | CPU benchmark, global scan을 유지하는 ROI 후보, 명시적 scalar 응답 모형 비교 |
| [run_b19.py](run_b19.py) | 저장 원본/SHA·명령/FK 이동·시간 범위 대조와 무장치 검증 |
| [test_b19.py](test_b19.py) | 오른팔 근거·REAL source·관측 차원·응답 소실/역방향·ROI 회귀 |

물체 영역 중심을 영상 중앙에 맞추지 않는다. 상대 오차는 관측된 물체 특징과 물리 대응이 확인된 Pad/task 기준 사이의 차이다. 관측되지 않은 축은 null이다. CAD 투영과 기존 RGB-D 모델 association은 별도 diagnostic으로 남는다. 반환값은 항상 `executable=false`, `motor_write_authorized=false`다.

응답 검사는 양의 전체 dot product가 한 축의 역응답을 숨기지 못하게 했다. X만 제어하더라도 정의된 Y task 오차가 커지면 전체 norm에서 확인한다. 필요한 특징이 사라지면 재식별을 요구하고, 처음부터 관측되지 않은 Y는 새로 요구하지 않는다. 이는 계산 유효성 판정이며 A의 물리 STOP이나 numeric hard gate를 추가하지 않는다.

## CPU 측정과 왼팔 비교

Python 3.12.3, NumPy 2.2.6, OpenCV 4.13.0, MuJoCo 3.3.7의 기존 환경을 사용했다. 한 benchmark 프로세스에서 OpenCV thread 1개와 `perf_counter_ns`로 측정했다. 별도 함수 workload이므로 아래 중앙값을 합산하지 않는다.

| CPU workload | 중앙값 |
|---|---:|
| 왼팔 object/Jaw 추출, 저장 영상 6개 | 2.353–2.576 ms |
| 동일 영상의 global scan + ROI 후보 | 2.496–2.826 ms |
| 오른손목 RGB surface | 1.197 ms |
| 오른손목 face graph, 고정 RNG timing fixture | 156.201 ms |
| 오른팔 미완성 shadow 요청 전체 | 190.934 ms |
| 기존 A 순수 IK, 모델 timing fixture | 2.084 ms |
| 기존 A 순수 경로 충돌검사, 모델 timing fixture | 13.652 ms |

ROI는 이 6개 저장 영상에서 기존보다 약 5.3–9.7% 느렸다. 18개 encoded burst 영상에서는 중심·폭이 모두 기존 결과와 같았다. 이 후보에서 CPU 속도나 특징 정확도 개선은 확인하지 못했다. 모듈 초기화 비용은 별도 기록하며 Native 실물 지연시간으로 해석하지 않는다.

아래 보정 횟수는 **저장된 후반응을 fit한 scalar 응답 모형**에서 비교했다. 명령 이동과 실제 피드백 기반 candidate FK 이동의 비율을 분리했다. 같은 pair의 후반응이 plant를 정의하고 local-gain 후보는 다른 context의 응답을 사용하므로 독립 REAL 예측이 아니다. 비교용 0.2 px/20회 설정은 수치 fixture이며 실제 task tolerance나 이동 제한이 아니다.

| 비교용 scalar 모형 | A 고정 prior | B 다른 context 국소 gain | C damping | D 오차별 gain | E ROI + A |
|---|---:|---:|---:|---:|---:|
| 저장 A | 2 | 3 | 7 | 4 | 2 |
| 저장 B | 3 | 5 | 10 | 5 | 3 |
| 이번 세션 | 5 | 7 | 12 | 7 | 5 |

A의 실제 계산은 `g=0.8/mm_per_px`, `move=-error/g`이며 이후 같은 높이에서 측정한 gain으로 갱신할 수 있다. 표의 A는 초기 prior를 유지한 비교 convention이다. 이 작업은 Native 계산을 변경하지 않았다. fit된 trace에서는 과보정/진동이 나타나지 않았지만, 실제 응답 변화·지연·새 영상·IK 가능성을 모델링하지 않아 실물 위험이나 시간 절감을 보장하지 않는다.

Jaw 기준은 두 종류로 세었다. 영상의 matching tip만 보면 18개 중 3개에서 기준을 확인하지 못했다. 기존 A 계약은 동일한 measured opening을 이용한 확인까지 포함해 18개 모두 confirmed였다. 영상 가시성과 모터 피드백 기반 확인을 같은 성공률로 표현하지 않았다. encoded burst 중심 표준편차는 최대 약 0.289 px였고 원래 in-memory 특징과 JPEG 재추출 간 최대 0.5 px 차이도 보존했다. 이는 ground truth 검출 오차·sensor noise bound·물리 TCP 정확도가 아니다. 높이·회전·scale·목표 변화도 별도 기록했다.

기록된 camera receipt/frame wait, producer 시간, planner 호출은 원래 범위와 시계를 유지했다. 실제 acquisition, PICK feature/error 계산, isolated settling/actuator command/collision certification 소요 시간은 기록이 없어 UNKNOWN이다. 약 6.6초의 보정 전후 영상 간격을 CPU나 모터 시간으로 귀속하지 않았다. 인증 파일의 trajectory duration도 인증 계산 시간이 아니다.

## 검증과 재현

기존 B17 29개와 B18 30개, 추가 B19 23개로 총 82개 검사가 통과했다. 독립 검토가 찾은 축 누락/전체 norm 문제와 X-only 후속 퇴행을 회귀로 고쳤고 재검수에서 남은 actionable finding은 없었다. 모터 명령·실행기 호출·학습 업데이트·MuJoCo step·장치 open은 모두 0회였다. 기존 원본/모델/입력 SHA 변화도 없었다.

일반 단위검사:

```bash
cd 2ARM_ROBOT/sweepick
python -B -m unittest visual_servo.test_b19 -v
```

전체 검증은 기존 비공개 근거 경로를 지정하고 각 출력에 새 폴더를 사용한다. raw 데이터나 모델을 다운로드/생성하지 않는다.

```bash
python -B goal_bridge/run_b17.py --b16-root "$B16_EVIDENCE" --replay-manifest "$B17_REPLAY_INPUTS" --output "$B19_NEW_B17_OUTPUT"
python -B visual_servo/run_b18.py --inputs "$B18_PRIVATE_INPUTS" --output "$B19_NEW_B18_OUTPUT"
python -B visual_servo/run_b19.py --inputs "$B19_PRIVATE_INPUTS" --output "$B19_NEW_OUTPUT"
```

B19 manifest는 `session`, frozen `a_source`/`a_source_manifest`, `b16_root`, `b18_root`, `optical_evidence`, `source_analysis`, A/B 및 이번 세션의 `left_sessions`를 지정한다. 출력은 원본 provenance를 포함하므로 공개 폴더 밖에 둔다. 첫 실행의 환경 정보 수집 subprocess 차단과 수정 전 revision도 비공개로 보존했다. 차단을 해제하거나 장치 실행을 허용하지 않았다.

## 검수 산출물

- [오른손목 특징](reports/B19_RIGHT_FEATURE_EVIDENCE.json)
- [REAL 상대 Jacobian 식별 가능성](reports/B19_RIGHT_JACOBIAN_IDENTIFIABILITY.json)
- [RightRelativeServoProposal](reports/B19_RIGHT_SERVO_PROPOSAL.json)
- [기록 시간과 CPU benchmark](reports/B19_LATENCY_PROFILE.json)
- [속도·정확도 비교](reports/B19_SPEED_ACCURACY_COMPARISON.json)
- [A Shadow 연동 계약](B19_A_INTEGRATION_CONTRACT.md)
- [회귀 검사](reports/B19_REGRESSION.json)

현재 A 알고리즘을 유지한다. 확인되지 않은 실물 개선이나 성공으로 바꾸지 않는다. 다음 승인 전에는 추가 실행을 시작하지 않으며, Native/Console/ACT/Calibration/main/A 통합 브랜치를 변경하지 않는다. 원본 영상·q12/ticks·camera parameters·보정·장치 식별정보·전체 로그는 공개하지 않는다.
