**STATIC14/14R1 checkpoint 및 STATIC15 elbow 기준 준비**

record_id: DAPIER-2026-09-30-static14-checkpoint-elbow-reference

나는 같은 관측값의 관절 변환이 달라진 이유와 저장 mono 영상의 보드 좌표 관계를 파일로 계산했다. 이번 checkpoint는 그 소스와 재현 조건을 보존한다. 실물 elbow 후보 채택이나 로봇 구동 성공은 아직 아니다. 원본 카메라/장치 기록, 개인 calibration/profile, dataset/weights와 공유 WIP는 게시하지 않는다.

**Problem → Evidence → Decision:** 이전 explicit mapping은 오른팔 elbow offset −6.263736263736263°, 왼팔 −6.087912087912088°를 적용했고 generic tabletop 진단은 0°를 썼다. 같은 저장 readback으로 오른팔 1.6663099495963445 / 1.556987128317579 rad를 재현했다. 중복 homing이나 임의 raw 조정으로 설명할 차이가 아니다. 두 profile 모두 physically_verified=false이고, 과거 긴 모서리 평행의 육안 근거만으로 승자를 고르지 않았다. servo homing이 적용된 Present_Position을 calibration midpoint 기준 degree로 바꾼 뒤 명시적인 mapping을 한 번 적용했다. model qpos0와 ACT 좌표는 그대로 보존했다.

**Validation → Result:** 14에서 동일 compiled model의 팔10축 FK, 과거 값 재현/관절 제한/TCP 의존성 검사를 완료했다. native mono의 현재 50 코너 PnP는 RMS 0.558148 px / 최대 1.186791 px, 조건부 camera↔board 후보였다. IPPE 대안은 보존했다. 새 보드-only MuJoCo 렌더는 54 코너 검출, RMS 0.099379 px / 최대 0.204122 px를 확인했다. 이 수치는 이전 완료 계산을 인용한 것이며 이 checkpoint에서 PnP/FK/보드 렌더/FREEZE를 다시 실행하지 않았다. native rectification/K/D, datum→robot world와 depth metric 단위는 여전히 미검증이다. 원본 capture incomplete 및 SDK cleanup timeout을 성공으로 바꾸지 않았다. 영상과 q0 시각도 동일 시점으로 승격하지 않았다.

**Source 보존 범위:** `analyze_alignment.py`, `render_board_candidate.py`는 실제 14 소스의 개인 절대 경로만 환경변수로 바꾼 snapshot이다. 원래 실행 소스 SHA, 공개 snapshot SHA와 변경 범위는 [source_identity.json](source_identity.json)에 있다. 지금 두 snapshot의 검사는 문법 확인까지다. **독립 실행 가능한 공개 완성본은 아니다.** 분석 시 사용한 dependency commit `cbe05158159b71fd9f837afccc7ec1778e7cc4fe` 외에도 로컬 변경 source와 비공개 helper/보드 설정/측정값/MJCF·mesh가 필요하다. 직접 호출하는 source SHA를 기록했고 다른 writer의 WIP를 흡수하지 않았다. 과거 source snapshot/patch/FREEZE는 로컬에서 그대로 보존한다.

파일 기반 재현을 준비할 때 기존 Python 3.12 / NumPy / OpenCV 4.13.0 / MuJoCo 3.3.7 환경을 쓴다. 아래 변수는 실제로 확보된 기존 입력을 가리켜야 한다. 이 checkout의 main 기준 source를 analysis 의존성과 같다고 가정하지 않는다.

```sh
# 기존 로컬 분석 worktree, support 폴더, STATIC13 폴더, pinned MJCF를 지정한다.
export DAPIER_ALIGNMENT_WORKTREE='<bound analysis worktree>'
export DAPIER_ALIGNMENT_SUPPORT='<private support directory>'
export DAPIER_ALIGNMENT_STATIC13='<private STATIC13 directory>'
export DAPIER_ALIGNMENT_MJCF='<bound pinned MJCF file>'
export DAPIER_PYTHON='<existing virtualenv Python>'

# 현재 15의 장치 없는 수치/구간 판정 검사만 실행한다.
"$DAPIER_PYTHON" -B 2ARM_ROBOT/research/tools/static_alignment/elbow_reference.py --self-check

# 14를 재계산하지 않고 저장된 model q에서 모서리 예측만 구한다.
"$DAPIER_PYTHON" -B 2ARM_ROBOT/research/tools/static_alignment/elbow_reference.py --comparison '<saved mapping-comparison.json>'
```

원래 14 재현 CLI는 `analyze_alignment.py --output-dir <새 폴더>` 및 `render_board_candidate.py --run <분석 폴더>`다. 출력은 기존 결과를 덮어쓰지 않는다. 이번 15에서는 이 CLI를 실행하지 않았다. 보드-only visual world는 실제 robot-world 정합을 대신하지 않으며 serial/ROS 장치 접근과 독립이다. 렌더의 optical→MuJoCo 축/주점 변환과 GL half-width 의미를 보존했다. [OpenCV pose 방향](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html), [MuJoCo 3.3.7 renderer](https://github.com/google-deepmind/mujoco/blob/3.3.7/src/render/render_gl3.c#L655-L695).

**Lesson / Next:** 두 파일의 산술 차이를 밝히는 것과 실물 영점이 맞는지 확인하는 것은 별개다. [한 세트 elbow 측정 안내](../../../docs/ELBOW_REFERENCE_STATIC15_KO.md)는 실제 STL 외곽 직선 경계를 특정하고, 위팔/아래팔의 상대 모서리 각도를 예측한다. lower body +90° 고정 회전과 mesh/body X 관계를 원본 모델에서 확인했다. 이 새 기하/구간 helper는 finite 입력, folded 기준, 한 후보/둘 다/둘 다 아닌 경우의 runnable assert 검사로 확인했다. 실제 측정값·오차와 같은 자세의 complete 양팔 readback이 없으므로 아직 후보를 채택하지 않는다. 제조/조립/측정 평면 오차를 0으로 가정하지 않는다.

상위 목표는 TJJ의 취미 생활(가제), 저비용 모바일 매니퓰레이터로 물체 인식·이동·정리 후 청소를 연결하는 가정용 서비스 로봇이다. 기존 handover/ACT/C25는 재사용 실험 자산이며 모든 청소 기능의 필수 선행조건이 아니다. 이번 기술 범위는 elbow 기준과 첫 bounded 비접촉 joint-space 시험에 직접 필요한 항목까지다. 카메라 추가 최적화/depth·SDK 수리/바닥 도달성/주행·청소·도킹/학습은 시작하지 않았다. absolute action/B9000/C25/r2/current-pose start와 기존 모드를 보존했다. 장치 open 및 모터 상태 변경/구동은 0회다.

부품 이름만으로 이해하기 어려운 점을 보완해 [전체 SO101 식별용 HTML/SVG](../../../docs/SO101_ELBOW_REFERENCE_STATIC15.html)를 추가했다. pinned CAD/STL의 부품 배치와 기준 모서리를 사용하고 외곽을 단순화했다. 각도 판정용 그림이 아니며, 부품 식별을 위해 펼친 예시 자세다. 사용자 육안의 “거의 수평”은 오차와 대응 자세가 확인되지 않은 정성 관찰로 남긴다.

main merge/release는 하지 않는다. 이 checkpoint와 관련 결과는 별도 writer branch와 draft PR로만 남기며, Git/Notion 반영 상태는 실제 원격 응답과 다시 읽은 내용으로 확인한다.
