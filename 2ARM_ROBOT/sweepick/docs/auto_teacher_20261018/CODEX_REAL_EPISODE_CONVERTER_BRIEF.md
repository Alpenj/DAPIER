# [Codex 지시서] sweepick 실물 녹화 → ACT 학습 episode 변환기 (work6)

## 목적
실물 자동 집기(교사) 녹화를 Isaac SIM ACT 학습과 **같은 episode 형식**으로 변환한다.
SIM으로 학습한 ACT를 실물 데이터로 추가 학습하는 데 쓴다(추천 순서 3단계 준비).
실물 장치(모터·카메라)는 절대 열지 않는다. 오프라인 파일 변환만 한다.

## 작업 위치
- **서버:** work6. 신규 작업은 work6에서 하고, 노트북에서 하지 않는다.
- **입력(실물 녹화, 읽기 전용):** `/home/robot/sweepick_laptop_archive/sweepick_task_runs/<RUN>/`
  - 첫 검증 대상 성공 run: `20261018_200250_auto`(왼팔 집기·들기·내려놓기), `20261018_203018_auto`(왼팔, 운반 포함 전체 성공)
  - 실패 run도 같은 폴더에 있다. 각 run의 `ANALYSIS.json`에 결과 요약이 있다.
- **SIM 학습 코드·형식 기준:** `/home/robot/sweepick-sim/sweepick_261016_mission/act/`
  - `sweepick_act.py`: 학습. `load_episode(run_dir)`가 `episode.npz`를 읽는다.
  - `sweepick_act_real.py`: 실물 전처리가 이미 있다. `real_points`, `WristPre`, `real_q12_to_source`를 재사용하고 새로 만들지 않는다.
  - `act/episode.npz`: SIM episode 예시. 키는 `t, phase, q(12), action(12), base(5), obj(7), T_world_cam(4x4), jaw, jaw_target, pads, depth(120x160 f16), jpg_os30a, jpg_left_wrist, jpg_right_wrist`
- **출력:** `/home/robot/sweepick-sim/sweepick_261016_mission/real_episodes/<RUN>/episode.npz` + `CONVERT.json`(아래)
- **코드:** 새 파일 `act/sweepick_261018_real_episode_convert.py`. 파일 이름에 sweepick을 넣는 규칙을 지킨다.

## 실물 녹화 구조 (실측 확인한 것)
- `episode/EPISODE.json`: session_id, task_outcome, stage_outcomes, data_quality, cameras, `act_view`(내용 확인할 것), files, clocks
- `episode/events.jsonl`: `t_host_s`, `kind` = episode_start / trail(단계 진행) / phase 등
- `episode/frames.jsonl`: 카메라별 프레임
  - 공통: `camera`, `n`, `recv_host_s`, `file`
  - top 전용: `sdk_color_stamp_s`, `sdk_depth_stamp_s`, `kept_every`, `depth_file`
- `episode/top/top_*.jpg`(헤드 RGB), `top/depth_*.png`(uint16 mm, 460x640)
  - 이전 run은 RGB가 1280x920·모든 FrameSet, 이후 run은 640 px·3 FrameSet마다. `kept_every`로 구분한다.
- `episode/left_wrist/*.jpg`, `episode/right_wrist/*.jpg`: 320x240
- `episode/joints.jsonl`: 녹화기가 읽은 관절 tick(`t_host_s`, side, ticks)
  - **주의:** 실행기가 포트를 쓰는 단계 중에는 녹화기가 그 팔을 못 읽을 수 있다(`source` 필드 확인).
- `<stage>/run*/move.json`: 각 단계 실행기 기록. `rows[]`에 `t`, `goal`(그 시점 Goal_Position = **명령값**), `present`, `velocity`, `load`, `current`, `phase`
  - 단계: pregrasp, align*, descend_a, descend, close, lift, carry, carry_back, lower, open, retreat, home 등
  - **명령값(action)의 원천은 여기다.**
- `STATE.json`: plan, stages(controller_result, end ticks), evidence, object_updates, side(어느 팔)
- `SESSION.json`: 단계 trail
- `looks/<tag>/SCENE.json`: `K`, `T_world_camera`, `q12`, ticks(헤드 외부 파라미터는 여기서 읽는다)

## 해야 할 일
1. **시간축 정합:** 모든 관절·명령값을 `t_host_s` 기준 한 축에 정렬한다.
   - 관절 측정값 = joints.jsonl + move.json `present`를 합쳐 중복을 제거한다.
   - 명령값 = move.json `goal`. 단계 밖 구간은 직전 goal을 유지한다(토크 off 구간이면 표시).
   - 기준 주기는 10 Hz(top RGB 저장 주기). 각 샘플에 가장 가까운 top, 양 손목 프레임과 그 시간차(ms)를 기록한다.
2. **변환:** `sweepick_act_real.py`의 함수를 그대로 쓴다.
   - tick → 모델 q12: 노트북 native의 `sweepick_real_command`(CANDIDATE_Q12_CONFIG + 보정 파일)와 같은 매핑이어야 한다.
     - work6에 사본이 없으면 노트북 `~/sweepick_261016_floor_native/revision/` 해당 파일만 복사하고 sha256을 기록한다.
   - 그다음 `real_q12_to_source`로 SIM 그리퍼 규약에 맞춘다.
   - depth: `DEPTH_SCALE` 적용 → 로봇 좌표 → SIM 형식 depth(120x160)로 줄이거나, 학습이 점군을 바로 받으면 그 경로를 맞춘다.
   - 손목: `WristPre`(undistort → RGB 320x240) 결과를 JPEG로 저장한다.
   - `base` = 0(정지), `obj` = STATE object(관측값; 모르면 NaN + 마스크), `phase` = 단계 이름
3. **품질 검사 (`CONVERT.json`에 기록)**
   - 프레임 누락: data_quality 그대로 + 정합 후 최대 시간차
   - 명령값이 없는 구간 비율, 관절 측정 공백(포트 소유로 못 읽은 구간)
   - 성공 여부: `ANALYSIS.json` verdict + `task_outcome`
   - 사용한 파일 sha256(매핑, 보정, 변환기)
4. **검증 (필수)**
   - 변환된 `episode.npz`가 수정 없는 `sweepick_act.load_episode`로 로드되는지 확인한다.
   - 기존 SIM 정책(예: `act_runs/dagger_C` 체크포인트)으로 이 episode의 action을 예측했을 때 L1 오차를 낸다. 학습이 아니라 형식 확인용이다.
   - 첫 두 성공 run에 대해, 변환한 q 궤적과 move.json `present`가 tick 단위로 같은지 왕복 확인한다.
5. **일괄 실행:** 폴더의 모든 run을 변환하고, run별 성공/실패 + 품질 표를 `real_episodes/INDEX.json`으로 만든다.
   - 성공 run만 학습용(train)으로 표시한다. 실패 run은 별도 표시(DAgger 등 나중 용도).

## 금지·규칙
- 실물 장치를 열거나 명령하지 않는다. 노트북 실행 파일도 수정하지 않는다(읽기·복사만).
- 입력 녹화 폴더는 수정하지 않는다.
- 값을 맞추려고 모델·보정값을 임의로 바꾸지 않는다. 불일치는 잔차로 보고한다.
- 막히면 추측하지 말고 무엇이 없는지 `CONVERT.json`의 `unknown`에 적는다.
- GitHub에 올릴 때는 `Alpenj/DAPIER` 작업 브랜치에 커밋하고 SHA를 기록한다. main merge는 하지 않는다. 민감 자료(녹화 원본)는 올리지 않는다.

## 완료 보고 형식 (한국어)
1. 변환한 run 수, 성공·실패 수
2. run별 정합 품질(최대 시간차, 명령값 공백 %)
3. `load_episode` 로드 결과와 SIM 정책 L1 오차
4. 실물 → SIM 규약 차이에서 발견한 문제(그리퍼 규약, 카메라 외부 파라미터, depth 스케일 등)
5. 3단계 학습을 바로 시작하는 명령어
