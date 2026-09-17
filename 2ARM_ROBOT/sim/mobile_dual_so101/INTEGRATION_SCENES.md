# 확정한 이동형·책상 모델 배치

record_id: DAPIER-2026-09-15-integration-scenes

사용자와 MuJoCo 화면을 대조해 확정한 배치다. 실물 검증 또는 teacher 성공을 뜻하지 않는다.
기존 tower/tabletop baseline과 별도 진입점으로 유지한다.

| 항목 | 값 / 근거 |
| --- | --- |
| 양팔 mounting 중심 간격 | 300 mm, 사용자 화면 피드백 기반 배치 |
| 공통 팔 X 보정 | -32.13529 mm |
| 뒤쪽 홀 | base local X=-13.86471 mm, Y=±31.75 mm; 원본 mesh 원호 fit |
| 뒤 프로파일 중심 | assembly X=-46 mm |
| 이동형 레일 | 폭400 mm(배치 추정), 앞뒤112 mm, 두께20 mm |
| 책상 앞 끝단 | X=-54.5 mm, SO-101 base 받침 뒤끝과 일치 |
| 스탠드 높이 | 설치면→판 중심380 mm, 사용자 측정 |
| 판 기울기 | CAD 원본 약25°, 이전165° 설명보다 CAD 우선 |
| desk 받침판 | 앞뒤112→109 mm; 외곽만 축소, 기둥/중앙부 불변 |
| block | 기존 tabletop의40 mm /20 g 자유물체 |
| 최종 gripper | 사용자 2026-09-16 확인: 좌우 양쪽 NORMA PGripper + RGB 손목 카메라 스탠드 |

## 실행

기존 MuJoCo 환경과 SO-101 asset을 사용한다. 새 의존성 설치나 hardware 연결은 없다.

```bash
export DAPIER_SO101_MJCF=/path/to/so101_new_calib.xml
cd 2ARM_ROBOT/sim/mobile_dual_so101
python integration_scenes.py --scene mobile --viewer
python integration_scenes.py --scene desk --viewer
python check_integration_mounts.py
```

viewer는 정적 검토 전용이며 physics를 진행하지 않는다.
렌더 저장: `MUJOCO_GL=egl python integration_scenes.py --scene desk --render desk.png --report desk.json`.

## CAD와 검사

사용자 제공 `assets/camera_stand/MOUNTBOTTM2.3mf` 및 `cam_mount_top2.3mf`를
원본 보존한다. importer는 millimeter 단위, identity build, 유효한 mesh 인덱스를 확인한다.
상부 print Y→assembly Z 강체 변환 후 기둥–판 상대 형상을 유지한다.
desk의 하단 외곽 띠만 축소하고 원본 파일을 수정하지 않는다.

검사: 실제 compiled mesh의 홀 원호 fit, 홀–rail 중심선, base/stand–desk 끝단,
양 모델 공통 팔 보정,380 mm 높이,25° 각도,12 actuator,초기 qvel0/ctrl=qpos.
이는 나사 체결, T너트, 수직 안착, 구조 강도 또는 전체 collision/path 검증이 아니다.
상·하부 CAD 사이 끼움 깊이는 파일에 assembly transform이 없어 미확정이다.
비볼록 CAD collision은 보수적 convex hull이며 고정 body끼리 contact가 없다는 사실은 무간섭 증거가 아니다.
카메라 enclosure/optical extrinsic, 실제 HOME, 책상 실측 크기는 미확정이다.

## 보존한 이전 SIM 인프라의 범위

collision certificate는 mesh-mesh narrowphase≤0일 때만 독립 축 분리 lower bound를 보충한다.
기존 clearance·pair·joint limit·CCD 설정을 유지한다. 저장 qpos false-zero 및 충돌 regression을 보존한다.

`MOBILE_BLOCK_CONFIG`와 이전 tower/tabletop은 baseline으로 보존한다.
이전 HOME·workspace·IK·성공률은 새 장면 검증 근거로 재사용하지 않는다.

## 실제 teacher physics viewer — 2026-09-16

`center_block_teacher.py --scene desk`와 `grasp_debug.py --scene desk`는
`integration_scenes.task_env()` → 동일한 `build_scene("desk")`를 사용한다.
target은 `red_block`, 지지면은 `table`, finger는 PGripper의
`left_pgripper_pad_1/2` distal CAD convex hull이다.
IK site는 양쪽 내측 면 중점인 `left_cube_grasp`다.
팔·스탠드·책상·CAD 배치를 변경하지 않았다. mobile은 block/작업면 및 desk와의
상대 transform이 없어 task 실행을 명시적으로 거부한다.

두 integration builder의 기본은 `grippers="both"`다. 이전 stock 화면은
`integration_scenes.py --grippers stock`으로 별도 보존한다.
기존 tower/tabletop builder의 stock 기본값은 바꾸지 않았다.
기존 `pgripper.replace_gripper()`가 고정된 NORMA CAD/장착 transform,
housing·양 jaw collision, RGB 카메라 브래킷과 카메라를 함께 적용한다.
중앙 OS30A와 arm mount/desk/profile 배치는 그대로다.
PGripper 명령은 opening-positive rad:0=닫힘,2.2028=열림이다.
passive jaw는 초기화/격리 preview만 kinematic helper를 사용하고 physics에서는
기존 joint equality로 움직인다. stock의 -.1745 rad 닫힘값을 재사용하지 않는다.
실물 카메라 스탠드 장착은 사용자 확인이며, RGB optics/hand-eye와 mount mass/collision은
여전히 기존 모델의 미검증 항목이다.

기존 Python 환경에서 다음을 실행한다. 출력 파일은 새 이름을 지정해야 한다.

```bash
python center_block_teacher.py --scene desk --viewer --output /tmp/desk-teacher-run-01.json
```

- headless는 `--viewer`만 생략한다.
- viewer는 같은 teacher의 **동일 model/data**를 관찰한다. 별도 환경·trajectory가 없다.
- RESET에서만 초기 qpos를 배치하며, 이후 제어는 기존 `ctrl + mj_step()` 경로다.
- 콘솔의 `SIM PHYSICS`와 phase/time으로 실행 상태를 표시한다.
- 종료/실패 결과를 JSON에 먼저 저장하고 physics를 멈춘 채 창을 유지한다.
  창을 닫으면 종료된다. 실행 중 창을 닫으면 teacher도 실패로 정지한다.
- `grasp_debug.py --viewer`는 별도 **격리된 IK pose preview**다. teacher motion이 아니다.
- 런타임 perception이 아니라 명시적인 offline SIM teacher 검증이다. 실물 경로는 없다.

### 현재 최초 실패: HOME

선택한 초기 후보는 확정 viewer의 model-default pose다.
팔은 zero, 양쪽 PGripper는2.2028 rad(열림)이다.
joint limits와 qvel0/ctrl=qpos는 만족하지만 HOME collision gate는 통과하지 못했다.
PGripper 모델의 첫 거부 pair는 right shoulder geom48 ↔ table geom0
(보존한 stock 모델에서는45 ↔ 0):
native distance0 m, required0.030 m이다. arm–table 실제 contact는 없지만
compiled mesh 최저점은 상판 위 약16.2 mm여서30 mm를 만족하지 않는다.
별도로 조사한 HUMANOID_HOME_ACTION도 같은 pair에서16.2 mm로 거부된다.

따라서 현재 viewer의 실제 결과는 **HOME → FAILURE, time0, physics0 step**이다.
RESET/SETTLE/PREGRASP 및 이후 phase를 실행했다고 보고하지 않는다.
mesh-box zero의 추가 진단과 clearance/장착 구조의 불일치를 분리해서 다뤄야 한다.
확정 배치나30 mm gate를 임의로 변경하지 않았다.

공통 거리 함수에서 무한 plane의 finite bounding-sphere 보정을 제외했다.
plane 위/접촉/관통 및 기존 mesh certificate 회귀는 유지한다.
예전 zero pose에서 physics 진행을 기대했던 양성 테스트는 유효한 baseline HOME을
명시하도록 수정했고, zero pose 자체는 physics 이전 거부 regression으로 보존한다.

GLX 항목은 별도다: 한 프로세스에서 viewer 두 개를 연속 종료할 때의
`GLXBadContext`는 scene/physics 통과와 구분한다. 이 실행기는 한 프로세스에
viewer 하나만 연다.

### 2026-09-16 shoulder/table swept-clearance audit

shoulder_clearance_audit.py는 같은 integration_desk builder의 격리 MjData에서
좌우 shoulder-pan의 물리 joint 전체 범위(±110°,0.5° 간격,각441개)를 검사한다.
화면에 KINEMATIC SWEEP / NOT PHYSICS EXECUTION을 표시하고 physics를 진행하지 않는다.
기존 teacher viewer의 실행 상태는 바꾸지 않는다. 큰 회전은 block task 궤적이 아니다.

기존 환경에서 다음과 같이 실행한다(새 output 이름 사용).

    python shoulder_clearance_audit.py --viewer --output /tmp/shoulder-sweep-new.json

좌우 해당 geom은 각각 shoulder_pan 하나에만 의존한다.
compiled full joint range와 소수점 반올림된 actuator ctrl range를 별도 기록하며,
preview는 더 넓은 full joint range를 사용한다. 실제 command 제한은 변경하지 않는다.
이번 실행은 양쪽441개 모두 약16.199998mm 양의 간격,접촉/관통0을 확인했다.
표면 간격의 연속구간 bound도 포함하지만 전체 로봇 sweep의 안전 보증은 아니다.

MESH–BOX native exact-zero fixture를 저장했고, 정확한 BOX support와 전체 compiled
mesh vertices로 box face-normal 분리를 증명하는 별도 lower bound를 추가했다.
native<=0의 MESH/BOX일 때만 평가하고, 양의 증명이 없으면0을 반환한다.
기존 mesh-mesh certificate,plane 예외,30mm 기준과 pair 목록은 유지했다.
HOME pair48/0은 이제 native0 -> guard약16.2mm로 보충되지만30mm 미달로 계속 거부된다.

모델상 near-support(B후보)와 실제 조립의 intended 관계를 구분한다.
실물 base/table 기준면·기울기·안착공차·하중변형 근거가 없어 새 pair-specific
threshold는 UNVERIFIED이며 미구현이다. 이 두 관계만 국소 정책 후보로 검토하고,
다른 external pair의30mm를 낮추거나 해당 pair를 삭제하지 않는다.


## 2026-09-16 approved SIM-only near-support checkpoint

Exact shoulder mesh/table pairs13/0 and48/0 now use structural positive-separation/contact/compiled-geometry invariants; all other general pairs retain30mm. Scope SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED. This supersedes the earlier unimplemented policy note, not its historical evidence.
HOME and RESET passed; actual ctrl+mj_step stopped during SETTLE at0.010s on geom29 left_pgripper_housing BOX versus geom0 table BOX native/final0. Near-support both remained16.2mm without contact. No PREGRASP or later phase reached. No model/limits/threshold tuning.
Detailed report: /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/NEAR_SUPPORT_TASK_REPORT.md


## 2026-09-16 BOX–BOX correction checkpoint

SAT certified lower bound added only for BOX–BOX native<=0; 31 regression tests passed. Saved0.010s false-zero now has195.811mm positive bound. Actual teacher HOME/RESET PASS; SETTLE stopped at0.058s/29steps because both measured gripper qpos exceed actuator upper2.2028rad by~1.21e-9rad. Limits/tolerances unchanged; first blocker preserved. PREGRASP not reached. Viewer holds real physics failure.
OS30A read-only digital-twin alignment is a separate deferred task: plane/transforms/difference report/precision clearance confirmation. No camera connection or hardware motion now.
Report: /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/BOX_BOX_TASK_REPORT.md


## 2026-09-16 command/measured semantics checkpoint

ctrlrange remains strict for commands; linked physical joint range/finite/unit transmission checked separately. Integration desk gripper measured numerical allowance1e-8rad from observed peak8.9633e-9 over1000step2s hold; raw state preserved, no model/limit changes. SETTLE nowPASS100steps/0.2s; next blocker PREGRASP IK nonconvergence(position8.825mm). Candidate path49samples safe but no actual PREGRASP motion. Viewer holds failure. Report: /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/MEASURED_STATE_TASK_REPORT.md
