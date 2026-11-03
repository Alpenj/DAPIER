# SO-101 VLA 병렬 MuJoCo rollout 실습

- record_id: `DAPIER-2026-08-11-so101-parallel-rollout`
- 범위: wrist-only SmolVLA simulation evaluation과 학습 후보 분류
- 실제 하드웨어 명령: 없음

## 왜 이 구조를 선택했나

오늘은 하나의 정책을 여러 프로세스에 복제하는 대신, 정책은 GPU에 한 번만 올리고
여러 MuJoCo worker의 관측을 하나의 batch로 추론하는 구조를 직접 붙였다. 현재
노트북은 RTX 5050 Laptop GPU 8 GB, CPU 16 thread라서 이 방식이 정책 복제보다
메모리를 덜 쓴다.

`teleoperate.py --no-viewer`에 `--parallel-envs`를 추가했다. 값이 4라면 LeRobot
evaluator의 `batch_size=4`와 Gymnasium `AsyncVectorEnv`를 사용한다. 각 worker는
`action_traces/env_0.jsonl`처럼 별도 trace를 써서 동시 파일 쓰기 충돌을 피한다.
평가 뒤에는 `parallel_rollout_manifest.json`이 seed별 reward, success와 후속 분류를
기록한다.

## 실제 비교

선택 checkpoint와 seed `1800..1803`, 700 step, smoothing on 조건을 고정했다.

| 실행 | 성공 | eval 시간 | episode당 시간 | trace frame |
|---|---:|---:|---:|---:|
| 1 env, 4회 순차 | 4/4 | 30.726 s | 7.682 s | 1,505 |
| 4 env, 1 batch | 3/4 | 31.763 s | 7.941 s | 2,800 |

직접 실행해 보니 현재 checkpoint와 GPU에서는 4-way가 더 빠르지 않았다. 병렬 batch
안의 실패 worker 한 개가 700 step까지 실행되는 동안 먼저 성공한 worker도 batch를
바로 끝내지 못하는 tail effect가 있었다. 정책의 확률적 action 생성도 batch 모양의
영향을 받아 같은 environment seed라도 순차 실행과 성공 결과가 완전히 같지 않았다.
따라서 이 결과를 성공률 개선이나 4배 가속으로 기록하지 않는다.

반면 같은 약 31초 동안 trace된 simulation frame은 1,505에서 2,800으로 늘었다.
병렬 환경은 현재 장비에서 최종 성능 판정보다 다양한 실패를 수집하는 용도로 먼저
쓰는 편이 맞다. 안정적인 비교 평가는 계속 `--parallel-envs 1`을 기준으로 둔다.

## 발견한 오류와 보완

첫 4-way 실행 종료 때 headless renderer가 X11 `BadWindow` 경고를 냈다. unattended
subprocess의 `MUJOCO_GL` 기본값을 `egl`로 고친 뒤 2 worker, 5-step 실제 checkpoint
smoke를 다시 실행했고 같은 경고 없이 완료했다.

successful worker는 vector environment의 autoreset 뒤에도 다른 worker를 기다리며
trace를 계속 쓸 수 있다. 그래서 trace row에 `reward`, `is_success`, `terminated`,
`truncated`, `episode_done`과 reset seed를 추가했다. manifest는 seed로 worker 파일과
local `trace_episode_index`를 찾아 첫 `episode_done=true`까지 선택하도록 명시한다.
`2 workers × 2 batches`, seed `1820..1823`, 5-step 실제 checkpoint smoke에서 뒤쪽
batch가 각각 `env_0/episode_2`, `env_1/episode_2`로 정확히 연결되고, 각 대상 episode의
마지막 row가 `truncated=true`, `episode_done=true`인 것을 확인했다.

## 학습과 경험 생성을 구분한다

이번 단계에서 optimizer update는 0회다. 병렬 rollout은 나루토식 “분신 경험 수집”에
해당하지만, 그 경험이 자동으로 본체 모델에 합쳐진 것은 아니다. 성공 rollout은
self-imitation 후보이고 실패 rollout은 human intervention 또는 수정한 IK teacher의
교정이 필요하다. 특히 기존 27% gripper target의 cube 관통 문제를 고치기 전에는
실패 action을 그대로 imitation dataset에 넣지 않는다.

다음에는 수정한 IK grasp target으로 새 demonstration을 수집하고, intervention
evidence를 LeRobot dataset으로 변환한 뒤 하나의 student dataset으로 합친다. 그때
batch-4 SmolVLA fine-tuning과 미사용 seed의 1-env/parallel-env 평가를 분리해 비교한다.
