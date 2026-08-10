# SO-101 로컬 작업물 통합 기록

`record_id: DAPIER-2026-08-07-so101-consolidation`

## 오늘 확인하려던 것

파일 관리자에서 `so101`, `so101_ros2_ws`, `DAPIER/so101_ros2`,
`DAPIER-lerobot-ros2-lab`가 비슷하게 보여 어떤 폴더가 정본인지 헷갈렸다.
오늘은 각 폴더의 Git revision, dirty file, symlink, 고유 파일과 용량을 직접
확인하고 DAPIER가 소유해야 할 작업만 한곳에서 찾을 수 있게 정리했다.

## 정리 전 조사 결과

| 로컬 위치 | 크기 | 확인한 상태 | 판정 |
|---|---:|---|---|
| `$HOME/so101` | 7.9GB | LeRobot v0.6.0 checkout, 7.7GB venv, MuJoCo custom env, hardware script, dataset, calibration 증거가 혼재 | 실행 작업장; upstream 전체를 DAPIER에 넣지 않음 |
| `$HOME/so101_ros2_ws` | 172MB | colcon `build/install/log`, 외부 `so101-ros-physical-ai`, DAPIER symlink가 혼재 | 빌드 작업장; 생성물은 Git에 넣지 않음 |
| `$HOME/DAPIER/so101_ros2` | 172KB | DAPIER가 소유하는 core/teleop source | ROS 2 정본 유지 |
| `$HOME/DAPIER/dapier_sim_first` | 296KB | G0/G1 source와 test | sim 정본 유지 |
| `$HOME/DAPIER-lerobot-ros2-lab` | 9.5MB | clean Git worktree, main에 없던 계획 문서 1개 | 고유 commit을 main에 합침 |
| `$HOME/dapier-runs/so101-foundation` | 5.4MB | G0/G1 receipt와 preview | 로컬 실행 증거; Git 밖 유지 |

`$HOME/so101_ros2_ws/src/dapier-so101-ros2`가
`$HOME/DAPIER/so101_ros2`를 가리키는 symlink인 것도 다시 확인했다. 따라서
두 폴더에 같은 ROS 2 source가 따로 있는 것이 아니다.

## DAPIER로 가져온 것

1. 별도 worktree의 고유 계획 문서 commit `e73f3ae`를 main의 `dd9f840`으로
   cherry-pick했다. source tree끼리 `diff -qr`했을 때 다른 파일은 없었다.
2. LeRobot checkout의 custom SO-101 MuJoCo 파일 13개를 overlay로 옮겼다.
   upstream tracked 변경 4개 파일은 patch로 만들었다.
3. SO-ARM100 원본 asset 14개의 SHA-256 manifest를 만들었다. 16MB binary는
   DAPIER에 중복 커밋하지 않았다.
4. 외부 Feetech submodule의 dirty 변경 5개 파일을 patch로 보존했다.
5. 하드웨어 보조 script를 `read_only` 4개와 `writes_hardware` 1개로 나눴다.
   개인 홈 경로와 특정 USB serial ID 기본값을 제거하고 Python formatter를
   적용했다.
6. 외부 ROS stack launcher는 그 integration 아래에 두었다. accidental real
   launch를 막기 위해 기본 backend를 `mock`으로 바꾸고 real port를 필수로 했다.

보존 patch의 SHA-256은 다음과 같다.

| artifact | SHA-256 |
|---|---|
| `lerobot-v0.6.0-tracked.patch` | `02d21c2c125d15c4339c2fb00e18ff28d4d4ad91945c0d08b43a0b7c55b110a8` |
| `feetech-driver-local.patch` | `d3585f48c77d04fb1d23176a71361e2aca2dfdf5adbc3d8f462d45b24fd29219` |

## 중복이라 가져오지 않은 Markdown과 생성물

`$HOME/so101`의 `START_HERE.md`, `README.md`, `SETUP_STATUS.md`,
`IMITATION_PIPELINE.md`, `PARTS_AND_IMITATION.md`는 경로가
`$HOME/so101-weekend`로 남아 있거나 아직 실행하지 않은 training/hardware
절차와 이미 DAPIER에 있는 runbook 내용이 섞여 있었다. 그대로 공개하지 않고
정본 runbook의 경로만 이번 구조에 맞게 수정했다. 원본은 삭제하지 않아 다음
검토 때 비교할 수 있다.

`tools/extract_chatgpt_share.py`는 SO-101 runtime 코드가 아니고 외부 대화 원문을
가져오는 도구라서 DAPIER 공개 기록 원칙과 맞지 않아 옮기지 않았다.

다음 생성물도 로컬에 그대로 두었다.

- LeRobot `.venv`, upstream tracked source와 `uv.lock`
- colcon `build/install/log`
- calibration JSON/JSONL, camera image와 zip backup
- Dataset v3 Parquet와 G1 preview video

## 직접 실행해 본 검증

실기체 목표값, torque, calibration write, ROS 2 real launch는 실행하지 않았다.

| 검증 | 결과 |
|---|---|
| `dapier_sim_first` unit test | 13/13 PASS |
| clean LeRobot base + DAPIER patch + overlay 재구성 | patch apply PASS, asset SHA-256 14/14 PASS |
| 재구성한 MuJoCo env test | 10/10 PASS, Gymnasium action-space warning 1건 |
| LeRobot custom source lint | Ruff check PASS; 보존본 `ros2_control.py`는 기존 formatter 차이 1건 유지 |
| hardware tool | Bash syntax PASS, Python `--help` PASS |
| 기존 follower/leader calibration JSON | schema validator PASS |
| Feetech local patch | clean base에 apply PASS, ROS 2 build PASS |
| Feetech test | test target 0개, error/failure 0개; 자동 시험이 있다고 해석하지 않음 |
| DAPIER 소유 Python | Ruff check PASS, format 13 files PASS |

read-only preflight도 실행했다. Ubuntu kernel `7.0.0-28`, LeRobot `0.6.0`,
PyTorch `2.11.0+cu128`, CUDA 사용 가능, RTX 5050 Laptop GPU, camera와 serial
장치 목록을 확인했다. 이 명령은 serial port를 열거나 모터에 쓰지 않았다.

## 외부 실행 증거의 인덱스

### 2026-08-06 joint sweep

- 위치: `$HOME/so101/sim_dataset/so101_mujoco_joint_sweep`
- LeRobot Dataset v3, 30Hz, 5 episode, 450 frame
- `next.success`: 0/450
- `meta/info.json`: `a27b62cc2a1567e8db6be3001a1ec6fe6d4666a65f0483ff5d4f5956309727b9`
- `meta/stats.json`: `5b03292b7cb1a229d41b543f5f8e8b4f9bd2f82c582160169776b7154f1a227e`
- Parquet data: `8ba0b4c58aa661c2909b9f9205f69cea91400540e7bd5409b177ff0ef98c507d`

이 데이터는 관절 sweep 기록 성공이지 cube pick 성공이 아니다.

### 2026-08-07 G1 scripted pick-and-lift

- 위치: `$HOME/dapier-runs/so101-foundation/20260807T004658Z-g1`
- receipt: PASS, 300/300 accepted frame, measured/action 300/300
- 최대 lift: 47.1535mm, 마지막 hold 최소 lift: 42.1169mm
- bilateral pad contact: 마지막 30/30, support contact: 0/30
- receipt SHA-256: `cee7c1e4e1e579d0940f88247303c99f2e612b1f5304805a0e5bf540b9606daa`
- preview SHA-256: `60000d0ad77d8a2b169253993c0ef12548ed28199788e2040faab13731f630ae`

이 PASS는 padded gripper와 높인 지지대를 manifest에 고정한 scripted sim
결과다. 기본 PickCube, 학습 policy, 사람 demonstration, place, 실기체 성공으로
확대해서 적지 않는다.

## 아직 확인하지 못한 부분

- Feetech patch에는 자체 test target이 없어 serial protocol 동작을 자동 검증하지 못했다.
- imported hardware script의 실제 장비 동작은 오늘 확인하지 않았다.
- 별도 worktree와 외부 실행 작업장은 rollback을 위해 아직 삭제하지 않았다.
- joint sweep의 0/5 실패 원인 분석은 G1 task-adapted 성공과 별도 연구로 남아 있다.

다음에는 DAPIER 정본 경로에서만 새 source를 수정하고, 외부 checkout은 upstream
재현 또는 실행 환경으로만 사용한다. 삭제 정리는 새 구조를 한 번 더 사용해 본 뒤
원본이 정말 중복인지 확인하고 진행한다.
