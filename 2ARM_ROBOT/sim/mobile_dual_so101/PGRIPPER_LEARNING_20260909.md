# PGripper 병렬 시연 수집·ACT·PPO 시작 기록

record_id: DAPIER-2026-09-09-pgripper-parallel-learning

나는 양손 PGripper의 MuJoCo 시연을 4개 독립 프로세스에서 수집하고,
ACT 모방학습과 PPO 강화학습을 각각 60분 예산으로 동시에 시작했다.
이 기록은 시작과 짧은 실행 검증의 근거이며, 60분 학습 완료나 실물 성공 보고가 아니다.
물리 계산은 CPU다. Isaac Sim/MJX처럼 GPU에서 수천 환경을 계산하는 구성은 아니다.
ACT는 RTX 5050 Laptop GPU, 작은 PPO MLP와 4개 MuJoCo 환경은 CPU를 사용한다.

## 지금 학습하는 것

- ACT: 기존 LeRobot ACT 구현을 사용한다. 두 손목 RGB 240×320, 측정 관절 12개를
  입력으로 50-step joint action chunk를 학습한다. 매 제어 시점은 25Hz이고
  그리퍼 경계는 0..1, 팔은 rad다. 영상은 ImageNet 평균/표준편차로 정규화한다.
- PPO: 시연의 기준 명령에 작은 residual을 더하는 4초 궤적·접촉 유지 curriculum이다.
  기준 명령은 scripted expert이고 **ACT 출력은 아직 연결하지 않았다**.
  관절·물체 추종 보상, 구간 성공 +20, 실패 -20, 미성공 시간 종료 -10을 사용한다.
  팔 충돌, 1mm 초과 침투, 10N 초과 pad force, grip loss 등은 종료한다.
  물체 정답과 접촉은 reward/검증 전용이며 PPO 관측은 관절·기준 관절·오차·진행률이다.
- PPO reset은 시연의 물리 snapshot에서 시작하고 damping을 0.9..1.1로 바꾼다.
  구간 성공률은 전체 집기→인계→내려놓기 성공률이 아니다.
  두 학습의 `task_success`/`full_task_success`는 미검증인 `null`로 남긴다.

## 직접 확인한 초기 결과

초기 팔 자세를 ±0.5° 바꾼 시연 8회 중 6회는 전체 작업을 통과했다.
2회는 인계 후 마지막 table hold 검사에서 실패했다. 실패 보고와 이미지는 보존했다.
첫 collector 버전은 실패 시 numeric archive 저장 전에 중단했으므로 이 2회의
NPZ는 없다. 수정한 collector는 이후 실패 NPZ도 저장하되 ACT에서 제외한다.

완전한 성공 시연의 train episode는 `[1,2,3,4]`, holdout은 `[6,7]`이다.
정규화 통계는 train만 사용한다. 물체 위치·fixture는 고정이므로 이 split은
새 물체 위치 일반화를 검증하지 않는다. 이전 stock 그리퍼의 실기 데이터와 섞지 않는다.
action label은 `mj_step` 직전 명령이며 측정 state나 다음 state를 복사한 label이 아니다.

- ACT 짧은 검사: optimizer 19 step, 고정 holdout subset의 normalized L1
  `0.927746 → 0.462589`. 이것은 모방 오차 감소이지 물리 작업 성공이 아니다.
- PPO 짧은 검사: 4개 환경, 5,092 transitions, 20 optimizer updates.
- 새 unittest 2개 통과: 단위 왕복·padding·train/holdout 분리·실패 제외,
  실제 MuJoCo transition과 잘못된 action이 물리 step 전에 거부되는지 검사한다.
- 이전 재생 수정 검사 4개 통과. 창의 `R`은 초기화부터 다시 실행하며
  `-replay-001` 등 새 출력 경로를 사용한다. 두 번째 실제 GUI 실행도 전체 작업을 통과했다.

## 재현

기존 MuJoCo 3.8.1, LeRobot/PyTorch 환경에 Stable-Baselines3 2.7.1을 추가했다.
[PPO 공식 문서](https://stable-baselines3.readthedocs.io/en/v2.7.0/modules/ppo.html)의
PPO와 SubprocVecEnv를 사용하며 PPO를 새로 구현하지 않았다.
아래 `python`은 해당 환경 실행 파일을 사용하고 출력 경로는 매번 새 것으로 지정한다.

```bash
export DAPIER_SO101_MJCF=/absolute/path/to/so101_new_calib.xml
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python \
  2ARM_ROBOT/sim/mobile_dual_so101/pgripper_learning.py collect \
  --output /private/new-sim-dataset --workers 4 --episodes 8

# 아래 두 명령은 별도 터미널에서 동시에 실행한다.
HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 python \
  2ARM_ROBOT/sim/mobile_dual_so101/pgripper_learning.py train \
  --dataset /private/new-sim-dataset --output /private/new-act-run --minutes 60

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python \
  2ARM_ROBOT/sim/mobile_dual_so101/pgripper_rl.py \
  --dataset /private/new-sim-dataset --output /private/new-ppo-run --workers 4 --minutes 60

python -m unittest discover -s 2ARM_ROBOT/sim/mobile_dual_so101/test \
  -p test_pgripper_learning.py -v
```

ACT는 `metrics.jsonl`, `normalization.npz`, initial/중간/final checkpoint와
optimizer 상태를 저장한다. 이 checkpoint에는 별도 정규화 계약이 필요하다.
기존 실기 ACT 래퍼로 바로 실행할 수 있는 dataset/checkpoint라고 보지 않는다.
PPO는 `progress.csv`, initial/중간/final 모델을 저장한다.
각각 종료 시 `run.json`에 초기/최종 holdout 비교를 기록한다.
실행 시간 제한은 학습 루프 기준이며 초기화·검증·저장 시간은 추가된다.

전체 ACT closed-loop 작업 평가, ACT 출력에 PPO를 연결하는 통합 정책,
실측 마찰·모터·카메라 보정과 sim-to-real은 아직 확인하지 않았다.
실물 장치 접속·모터 동작·원시 데이터 업로드는 실행하지 않았다.

## 60분 학습 종료와 전체 작업 평가

두 학습이 실제로 종료되고 final checkpoint가 저장된 것을 확인했다.
ACT는 52,163 step, 3,600.75초를 학습했다. holdout normalized L1은
0.927746에서 0.012747로 감소했다. PPO는 1,347,024 transitions,
6,133 optimizer updates, 3,600.01초를 기록했다. 16개 holdout 구간의
성공률은 초기/최종 모두 100%였지만 평균 보상은 219.997에서 197.727로
감소했다. 따라서 PPO 개선을 확인했다고 쓰지 않는다.

이후 `evaluate_pgripper.py`로 모델을 다시 학습하지 않고 전체 작업을 평가했다.
ACT는 매번 새 손목 RGB와 측정 관절만 입력받고, 시연 action이나 물체 정답은
입력받지 않는다. PPO와 비교군은 공개 기준 시연의 joint trajectory를 사용한다.
PPO를 ACT와 연결한 결과는 아니다. 시작부터 110초 SIM 제한, 25Hz 정책 입력,
500Hz 물리 검사, 같은 제어 한계와 안전 중단 조건을 적용했다.

| 평가 대상 | 전체 성공 | 확인한 실패 |
| --- | --- | --- |
| ACT final | 0/2 | 두 번 모두 시작 자세 근처에서 정체, 파지 없이 110초 timeout |
| 기준 시연 + PPO final | 0/2 | 집기·인계·테이블 지지는 확인, 최종 위치 오차 4.61/4.63mm로 3mm 기준 초과 |
| 기준 시연, residual=0 | 1/2 | 한 번은 성공, 다른 한 번은 연속 3초 안착 미달 |

ACT는 nominal home과 미학습 seed의 ±0.5° 초기 자세를 사용했다.
PPO/비교군은 holdout 시연 6·7의 frame zero에서 시작했다. 성공 구간 중간에서
reset하지 않았다. 2회씩의 고정 장면 평가이므로 일반적인 성공 확률로 확대하지 않는다.

판정기는 정책의 단계 주장이나 경과 시간이 아니라 raw contact·높이·속도로
왼손 파지/들림 → 오른손 파지 → 왼손 접촉 없이 오른손 3초 유지 → 테이블 지지
→ 양손 접촉 없이 목표 3mm 이내에서 3초 안정 상태를 순서대로 확인한다.
기존 전체 expert는 통과하고, recipient pad contact 제거 사례는 실패하는지
같은 판정기로 검증했다. 추가 unittest 2개도 통과했다.

첫 평가에서는 테이블 지지가 확인된 뒤에도 파지를 요구하는 판정기 오류를 발견했다.
지지 이후에는 안착 검사를 적용하도록 수정하고 calibration과 모든 본 평가를 다시
실행했다. `*-01`은 보존한 이전 시도이며 최종 수치는 `*-02`에 해당한다.
기준 시연 실패 사례도 500Hz 샘플을 추가 관찰했다. table support 이후 17,276개
샘플 중 8,427개에서 물체 속도가 0.01m/s를 넘었고, 연속 안착은 최대 0.006초였다.
25Hz 저장 화면만으로는 이 진동을 놓칠 수 있어 화면상 놓였다는 이유로 성공 처리하지 않았다.

최종 세 평가의 checkpoint/source hash 불변을 확인했고, 모든 본 시도에서
MuJoCo warning은 0이었다. 다만 정책과 판정기 사이 OS 수준 파일 접근 격리는
구성하지 않았다. 결과는 보고 전용이며 자동 실물 실행·배포 gate가 아니다.

재현은 위 환경에서 다음처럼 실행한다. 각 명령은 새로운 출력 폴더를 사용한다.

```bash
MUJOCO_GL=egl python 2ARM_ROBOT/sim/mobile_dual_so101/evaluate_pgripper.py calibrate \
  --output /private/new-verifier-calibration
MUJOCO_GL=egl HF_HUB_OFFLINE=1 python 2ARM_ROBOT/sim/mobile_dual_so101/evaluate_pgripper.py act \
  --checkpoint /private/act-run/final --output /private/new-act-eval --episodes 2
MUJOCO_GL=egl python 2ARM_ROBOT/sim/mobile_dual_so101/evaluate_pgripper.py ppo \
  --checkpoint /private/ppo-run/final.zip --dataset /private/sim-dataset \
  --output /private/new-ppo-eval --episodes 2
MUJOCO_GL=egl python 2ARM_ROBOT/sim/mobile_dual_so101/evaluate_pgripper.py reference \
  --dataset /private/sim-dataset --output /private/new-reference-eval --episodes 2
python -m unittest discover -s 2ARM_ROBOT/sim/mobile_dual_so101/test -p test_evaluate_pgripper.py -v
```

다음에 확인할 것은 ACT 시작 구간의 행동/관측 시간 정렬과 실제 closed-loop 진행,
PPO의 4초·15mm 구간 목적과 전체 작업·3mm 안착 목적의 차이, 접촉 진동의 영향이다.
이번 평가 중 policy 가중치나 성공 허용 오차를 바꾸어 성공 점수를 만들지는 않았다.
실물 카메라 보정은 SIM 학습 뒤에도 가능하지만, 실물 평가 전에는 내부 파라미터·왜곡과
좌우 손목 camera-to-gripper 관계를 확인한다. 보정이 SIM 가정과 다르면 장면을 수정하고
정책을 다시 평가하거나 추가 학습해야 한다. 현재 결과로 실물 성공을 보장하지 않는다.
