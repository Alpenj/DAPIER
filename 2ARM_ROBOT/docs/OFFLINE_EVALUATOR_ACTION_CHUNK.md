# Offline evaluator · action chunk/padding 계약

확인일: 2026-08-21
단계: 4/5
범위: held-out ACT prediction chunk의 split·padding·joint/group error·failure inspection 검증

## 왜 Stage 3 다음에 진행하는가

Stage 3는 Dataset reader와 ACT가 같은 `action_is_pad` 계약을 사용한다는 사실을 증명했다. Stage 4는 그 mask를 evaluator에도 그대로 적용해 episode tail의 가짜 action을 metric에서 제외한다. 이 작업 없이 validation error를 계산하면 모델이 예측하지 않은 padding 값이 수치를 왜곡한다.

또한 offline imitation error는 실제 신발을 잡고 정리한 성공률이 아니다. 이 문서는 다음 세 증거를 분리한다.

1. held-out expert action과의 padding-excluded error
2. Stage 5 real rollout의 task success
3. 독립 supervisor의 reject·intervention·staleness·near-limit 기록

## 연구 선정과 채택 판단

상세 조사: [`research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md`](research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md)

| 자료 | 직접 관련성 | DAPIER 결정 |
|---|---|---|
| LeRobot ACTConfig·ACT model | chunk horizon과 `action_is_pad` loss mask의 현재 구현 정본 | 즉시 반영 |
| LeRobot Dataset factory·reader·streaming tests | FPS delta, episode 경계, padding 값/마스크 검증 정본 | 즉시 반영 |
| LeRobot train/eval scripts | offline eval loss와 closed-loop success/reward가 별도 경로임을 확인 | 즉시 반영 |
| Dong et al., PiL-World(2026) | chunk-wise policy-in-the-loop 평가의 최신 참고 | compute·모델 규모가 달라 참고만, 미도입 |
| Chen et al., Robo-DM(2025) | multimodal storage/provenance의 참고 | evaluator metric 근거가 아니므로 참고만 |

최신 VLA·world-model 논문은 JDcobot ACT padding metric을 직접 개선하지 않으므로 추가하지 않았다. world-model evaluator도 6주·RTX 5050·무예산 조건에서 ACT 기준선을 흐리므로 보류했다.

## 입력 manifest

`evaluation_manifest.json`은 metric을 보기 전에 고정한다.

- evaluation source split은 `validation` 또는 `test`
- train/evaluation episode ID overlap 0건
- object set, scene, session, calibration group overlap 명시
- `generalization_claim=true`이면 위 group overlap 0건 강제
- normalization stats source는 train split만 허용
- policy checkpoint, train manifest, normalization stats, hardware profile, code SHA-256 기록
- action names/order, unit, absolute/relative convention, FPS, chunk size, delta timestamps 고정
- prediction JSONL SHA-256 검증

frame random split은 near-duplicate RGB/action 누수를 만들 수 있어 사용하지 않는다.

## metric 계약

`valid = ~action_is_pad`인 timestep만 계산한다.

- horizon별 `valid_step_count`, `coverage`
- horizon×joint `MAE`, `RMSE`, `max_abs`
- `left_arm`, `left_gripper`, `right_arm`, `right_gripper`, `all_arm`, `all_gripper` 그룹 metric
- masked timestep/scalar count
- group별 high-error top-K inspection reference

12차원 global MAE는 출력하지 않는다. arm은 radian, gripper는 normalized position이므로 한 평균으로 합치면 물리적 의미가 없기 때문이다. action name을 metadata에서 읽어 group을 만들며 위치 index만 가정하지 않는다.

## failure taxonomy

report는 다음 조사 경로를 고정한다.

- `data_contract`
- `split_or_calibration_leakage`
- `action_tail_padding`
- `single_joint_or_gripper`
- `bimanual_coordination`
- `perception_or_calibration`
- `off_distribution`
- `safety_or_intervention`

inspection packet은 기존 RGB raw reference, depth availability, episode/frame/horizon, scene/object/calibration ID와 group error만 가리킨다. raw pixel이나 operator 개인정보를 복제하지 않는다.

## 실행

```bash
python -m shoe_sorting_data.cli offline-eval-fixture \
  --root /tmp/dapier_stage4/fixture \
  --padded-prediction 1000000

python -m shoe_sorting_data.cli offline-eval \
  --manifest /tmp/dapier_stage4/fixture/evaluation_manifest.json \
  --output /tmp/dapier_stage4/offline_evaluation_report.json \
  --inspection-top-k 3
```

fixture는 2 episode×3 frame, `H=3`이다. synthetic 결과는 evaluator 계약 검증용이며 model 성능이 아니다.

## 2026-08-21 실제 검증 결과

artifact: `C:\Users\hjjeon\Documents\DAPIER\tmp\stage4-offline-evaluator-20260821-v2`

| 항목 | 결과 |
|---|---|
| records | 6 |
| valid / masked timesteps | 12 / 6 |
| masked scalar count | 72 |
| horizon coverage | 1.000 / 0.667 / 0.333 |
| `all_arm` MAE | 0.1 radian |
| `all_gripper` MAE | 0.2 normalized position |
| padded prediction mutation | `1,000,000`으로 변경해도 metric 동일 |
| cross-episode target ID mutation | FAIL closed |
| mixed-unit global metric | `null` |
| generalization claim | false |
| closed-loop task success | `NOT_MEASURED` |

unit tests는 padded-only mutation invariance, valid-joint mutation sensitivity, split overlap, shape/mask mismatch, cross-episode leak, output no-overwrite를 검증한다.

## 다음 단계와 현장 지표

Stage 5 독립 supervisor가 생긴 뒤 다음을 별도 report로 연결한다.

- task success rate와 시도 수
- joint/rate/workspace/base-moving/stale/watchdog/E-stop reject rate
- human pause·teleop takeover·replan intervention rate
- time-to-intervention/abort
- near-limit margin min/p05
- policy query count, executed action count, chunk index, observation/action staleness

supervisor reject는 성공으로 처리하지 않고 `success=false`, `safety_intervention=true`를 함께 기록한다. 다만 sensor/driver failure와 policy failure는 taxonomy로 분리한다.
