# 실제 녹화 4개로 ACT 연결 시험

record_id: DAPIER-2026-09-07-act-baseline

나는 기존 LeRobot ACT 학습기를 재사용해 양손목 RGB + 현재 관절 상태에서 실제 전송
action을 예측하는 작은 기준 모델을 학습한다. 자체 ACT 런타임이나 실물 성공을
완성했다는 뜻은 아니다. H201 metric depth는 원본에 보존하고 geometry/IK용으로
분리한다. depth 컬러맵을 RGB 입력으로 넣지 않는다.

## 확인한 데이터 경계

- 15Hz, 12축(left 6 → right 6), 팔 degree / 그리퍼 0~100이다.
- 에피소드 0·1·2의 3,382프레임만 학습, 에피소드 3의 1,518프레임은 검증한다.
- 설치된 학습기의 episode split을 재사용하되 state/action 정규화 통계는 학습
  프레임에서만 다시 계산해 메모리에 적용한다. 원본의 전체 통계 파일은 수정하지 않는다.
- 양손목 RGB에는 고정 ImageNet 통계를 쓴다. 저장된 leader→follower 보정을 다시
  더하지 않는다. 로컬 학습만 허용하며 장치 연결·Hub 업로드는 하지 않는다.

## 실제 실행한 조건

기존 LeRobot 환경의 Python에서 프로젝트 루트를 작업 디렉터리로 사용한다.
`DATASET`과 `OUTPUT`은 각각 기존 데이터와 **새로운** 로컬 출력 경로로 지정한다.
출력 폴더가 이미 있으면 기본적으로 거부한다. 아래 환경 변수는 사용자가 지정하는
작업 경로이며 비밀값이나 모터 설정이 아니다.

```bash
HF_HUB_OFFLINE=1 WANDB_MODE=disabled OMP_NUM_THREADS=4 python \
  2ARM_ROBOT/scripts/train_act_baseline.py \
  --dataset.repo_id=dapier-local/dual-so101-handover-20260907 \
  --dataset.root="$DATASET" --dataset.eval_split=0.25 --output_dir="$OUTPUT" \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --policy.dim_model=128 --policy.n_heads=4 --policy.dim_feedforward=512 \
  --policy.n_encoder_layers=2 --policy.n_decoder_layers=1 \
  --policy.n_vae_encoder_layers=2 --policy.latent_dim=16 \
  --policy.chunk_size=16 --policy.n_action_steps=1 \
  --policy.optimizer_lr=0.0001 --policy.optimizer_lr_backbone=0.00001 \
  '--policy.input_features={"observation.state":{"type":"STATE","shape":[12]},"observation.images.left_wrist":{"type":"VISUAL","shape":[3,240,320]},"observation.images.right_wrist":{"type":"VISUAL","shape":[3,240,320]}}' \
  --num_workers=2 --batch_size=8 --steps=2000 --save_freq=500 --log_freq=25 \
  --eval_steps=500 --max_eval_samples=0 --env_eval_freq=0 \
  --seed=1000 --cudnn_deterministic=true
```

ResNet18의 기존 로컬 ImageNet 가중치를 재사용한다. 작은 transformer를 포함해
학습 가능한 파라미터는 12,305,772개다. 새로운 의존성을 설치하지 않았다.

## 행동 예측 검증

```bash
python 2ARM_ROBOT/scripts/train_act_baseline.py --self-test
python 2ARM_ROBOT/scripts/evaluate_act_baseline.py --self-test

HF_HUB_OFFLINE=1 OMP_NUM_THREADS=2 python 2ARM_ROBOT/scripts/evaluate_act_baseline.py \
  --dataset "$DATASET" --checkpoint "$OUTPUT/checkpoints/002000/pretrained_model" \
  --contract "$OUTPUT/split_contract.json" --output "$OUTPUT/eval-002000"
```

체크포인트와 함께 저장된 전·후처리기를 다시 읽고, 정책에는 관절 상태와 양손목
영상만 전달한다. 정답 action은 예측이 끝난 뒤 오차 계산에만 사용한다. 첫 action과
16-step chunk를 각각 측정하며 에피소드 끝의 padding은 제외한다. 현재 자세를 그대로
유지하는 기준과 학습 action 평균 기준도 함께 기록한다.

같은 관절 상태에서 왼손목 영상만, 오른손목 영상만 반 에피소드만큼 이동시키고,
양쪽 영상을 검게 만든 입력도 검사한다. 이는 카메라 입력이 행동에 영향을 주는지
확인하는 진단이지, 올바른 물체 인식이나 일반화 능력을 증명하는 검사가 아니다.

## 2,000-step 실행 결과

2026-09-07 RTX 5050 Laptop에서 약 12분 26초 동안 실행하고 정상 종료했다.
설치된 LeRobot fork commit은 `30da8e687a6dfc617fcd94afc367ac7071c376ce`이다.
학습 로그의 25-step 평균 loss는 초반 11.510에서 마지막 0.108로 내려갔다.
검증 loss는 500/1,000/1,500/2,000 step에서 각각 0.5923/0.5399/0.5507/0.5494였다.
학습 loss에는 VAE KL 항이 포함되므로 검증 loss와 직접 같은 양으로 비교하지 않는다.

최종 체크포인트를 다시 읽어 1,518개 보류 관측 전부로 검사했다. 아래 팔 오차는
좌우 팔의 10개 관절별 MAE를 평균한 degree이며 그리퍼와 단위를 섞지 않는다.

| 입력/기준 | 첫 action 팔 MAE (°) | 16-step chunk 팔 MAE (°) |
| --- | ---: | ---: |
| 정상 양손목 RGB + state | 12.059 | 11.999 |
| 현재 관절 상태 유지 | 0.868 | 1.966 |
| 학습 action 평균 | 22.900 | 22.935 |
| 왼손목 영상만 시간 이동 | 19.992 | 20.190 |
| 오른손목 영상만 시간 이동 | 26.721 | 26.349 |
| 양손목 영상 검게 처리 | 27.050 | 27.178 |

정상 입력의 첫 action 그리퍼 MAE는 좌 11.999, 우 5.455 percentage point였다.
영상 변경이 예측에 영향을 주지만, 정상 입력도 현재 상태 유지 기준보다 오차가
크다. **학습 연결은 확인했지만 쓸 수 있는 조작 정책으로 검증한 것은 아니다.**
이 결과만으로 4개 에피소드의 일반화나 성공률을 주장하지 않는다. 상태 유지 기준의
오차가 작다는 사실도 물체를 집는 정책이 좋다는 뜻은 아니다.

500→2,000 step 체크포인트의 186개 저장 tensor 중 105개가 달라졌고 최대 절대
변화는 0.0635271이었다. 최종 normalizer의 state/action 통계도 원본의 학습 3개
에피소드에서 재계산한 값과 일치했다. 원본 데이터·모터 보정·모델의 기존 SHA-256
검사가 통과했다. 두 self-test와 전체 보류 관측 평가를 실행했으며, 평가 중 정답
action을 정책에 전달하지 않았다. 최초 실행에서 이미지 라이브러리 DEBUG 로그가
과도해 후속 실행의 파일 로그는 INFO 이상만 남기도록 수정했다.

## 폐루프 평가에 남은 조건

녹화 관측을 넣은 오프라인 오차는 정책이 움직인 결과를 다시 관측하는 폐루프
성공률이 아니다. 실제 목표는 매 주기 새 카메라 관측 → ACT 관절 action → 단위·한계·
접촉·시간 유효성 검사 → 실행 → 재관측이다. `n_action_steps=1`로 매 주기 관측을
소비하도록 한다. 관절 action에 IK를 다시 적용하지 않으며 시뮬레이터 정답 물체
좌표를 정책 입력으로 넣지 않는다.

[책상형 재생](../sim/mobile_dual_so101/TABLETOP_REPLAY.md)의 초기 schema v1에서는
첫 자세 테이블 관통 약 11.12mm를 확인했고 블록도 정적 참고물이었다. 현재 v2는
4cm·약 20g 자유 강체와 접촉을 사용한다. 관절 영점·장착 방향과 카메라 보정은
여전히 미검증이다. 이 조건을 해결하기 전에는 CPU 병렬 작업 성공률을 발표하지 않는다.
GPU 병렬 RL도 보상·리셋·물리 환경과 CPU 기준이 확인된 뒤 필요성을 판단한다.

## 2026-09-08 · 실제 CPU 폐루프 연결 시험

`sim/mobile_dual_so101/parallel_tabletop_act.py`에서 DAPIER가 매 주기 새 양손목
렌더 RGB와 측정 관절 상태를 구성해 기존 LeRobot ACT 체크포인트에 전달한다.
정책은 새 관측으로 action을 계산하고, 범위와 명령 변화율을 제한한 뒤 `mj_step()`으로
실행한다. 저장된 action이나 미래 state, 물체 정답 좌표는 정책에 넘기지 않는다.
여기서 native ACT는 기존 체크포인트 형식을 직접 읽는다는 뜻이며 ACT 자체를 새로
구현했다는 뜻이 아니다. DAPIER 소유 범위는 관측·제어 연결과 실행 로그다.

```bash
HF_HUB_OFFLINE=1 MUJOCO_GL=egl OMP_NUM_THREADS=2 python \
  2ARM_ROBOT/sim/mobile_dual_so101/parallel_tabletop_act.py \
  --dataset "$DATASET" --checkpoint "$OUTPUT/checkpoints/002000/pretrained_model" \
  --model /home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml \
  --output /home/dapier-jhj/DAPIER/2ARM_ROBOT/recording/simulation/20260908/act-closed-loop-new \
  --workers 2 --episodes 2 --steps 30
```

MuJoCo 3.8.1의 현재 elliptic 장면에서 CPU 2개 프로세스가 각각 30 transition을
완료했다. 각 worker의 정책 질의 30회, 양손목 이미지 각각 30종, 시뮬레이션 시간
2초를 확인했다. 총 60 transition이고 시작을 포함한 wall time은 약 9.41초였다.
명령은 모든 step에서 변화율 제한을 거쳤고 시뮬레이터 경고는 없었다.
원본 dataset·체크포인트·scene 해시도 유지했다. 실제 15Hz 실시간 추론 성능을
검증한 것은 아니며 EGL 렌더링은 GPU를 사용할 수 있다.

`task_success_rate=null`, `task_evaluation_valid=false`, `physical_mapping_verified=false`다.
기록 첫 자세에서 시작한 관통 17.08mm도 그대로 보고한다. **60 transition은 집기 성공
60회가 아니다.** 별도 영상+IK 집기 baseline의 통과도 ACT 점수로 합치지 않는다.
로컬 근거: `recording/simulation/20260908/act-closed-loop-elliptic/report.json`과
각 episode의 관측·제안·제한 후 명령·다음 상태 trace다. 이 진입점은 optimizer/RL을 실행하지 않는다.

원본 영상·사진·개인 calibration·체크포인트는 공개 저장소에 올리지 않는다.
