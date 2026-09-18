# PGripper PREGRASP IK 분해 기록

record_id: DAPIER-2026-09-16-pgripper-pregrasp-ik-decomposition

## Problem

나는 integration_desk에서 HOME/RESET/SETTLE을 통과했지만 PREGRASP IK가 위치8.825mm/접근축4.827도 오차로 실패한 원인을 분리한다. 실제 PREGRASP motion은 실행하지 않았다.
기존 ENGINEERING_LEARNING_RECORD_KO.md는 저장소 및 홈 한정 검색에서 찾지 못해, 요청한 Problem → Evidence → Decision → Validation → Result → Lesson / Next 구조로 이 기록을 새로 남긴다.

## Evidence

ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch: pro/dual-so101-codex-20260907; SHA534c4a6f25ab3eea6dd703526627e89dd9ba66d7; 기존 dirty 변경 보존.
동일 실제 SETTLE0.200s 상태를 격리된 진단 data로 복원했다. 원래 teacher physics viewer는 유지한다.
모델 hash: ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368

### TCP / geometry

left_cube_grasp, parent left_gripper; left_gripperframe과 같은 pose.
local XYZ(m): [-1.6515859643809194e-05, -0.00027482591940811325, -0.09095891892995724]
local quaternion(wxyz): [0.7071067811865475, -0.0, 0.7071067811865475, -2.377879264167753e-17]
settled HOME world XYZ(m): [0.3521115742506746, 0.1482702510762598, 0.2334103896509431]
world rotation(matrix): [[0.9997999975665894, -0.009904061634312452, 0.01737453392151427], [-0.01668048924607291, 0.06630816592949955, 0.9976597558334083], [-0.011032957190062684, -0.99775003718076, 0.06612969954137511]]
양쪽 pad는 left_pgripper_pad_1/2 MESH, parent left_pgripper_jaw_1/2이며 두 jaw 모두 slide로 움직인다. stock fixed/moving 구조가 아니다.
TCP와 compiled inner-face midpoint 차이(world m): [2.3651167302407572e-08, 1.358091854308796e-06, 9.00208810550307e-08]
norm = 1.3612775695943111 micrometers. 이 수치만큼 TCP를 임의 조정하지 않는다.
PGripper CAD tip bounds에서 midpoint를 이미 도출한다. stock quaternion은 이어받지만 실제축은 site+X=parent-Z(접근), site+Z=parent+X(jaw travel)로 현재 geometry와 일치한다.
desk targets()는 stock20mm+20mm offset을 사용하지 않는다. grasp는 settled cube center, pregrasp는 해당점에서 approach 반대60mm다.
TCP는 distal 끝점 자체가 아니라 pinch depth 중심이다. pad의 TCP-frame bounds와 좌우 축은 pregrasp-compiled-supplement.json에 기록했다. 실제 CAD tip convex proxy/실물 정렬 정확성은 별도 미검증이다.

### 동일 target / bounded seed 비교

targetXYZ=[0.19999999999999762, 3.2640378686770537e-18, 0.0799979773084367]; desired approach=[2.5540045787461297e-19, 5.741945423532055e-18, -1.0]
기존 tolerance0.5mm/2deg,300iterations, damping0.02, axis weight0.05m를 유지했다. 5개 deterministic seed ×2objective; random search 없음.
|Seed|Objective|Converged|Iterations|Position mm|Approach deg|First limit iteration|
|---|---|---:|---:|---:|---:|---:|
|settled_HOME|position-only|True|14|0.246598|38.915760|-|
|settled_HOME|position+approach|False|300|8.825210|4.827200|34|
|previous_failed|position-only|True|11|0.407743|3.096861|0|
|previous_failed|position+approach|False|300|8.343639|4.529481|0|
|HOME_elbow_plus|position-only|True|14|0.422874|45.582975|-|
|HOME_elbow_plus|position+approach|False|300|8.475355|4.612700|37|
|HOME_elbow_minus|position-only|True|14|0.307004|31.972906|-|
|HOME_elbow_minus|position+approach|False|300|9.103853|4.992300|31|
|elbow_alternative|position-only|True|6|0.019260|43.927591|-|
|elbow_alternative|position+approach|False|300|8.721210|4.764340|27|

모든 final q,5개 solved joint의 lower/upper margins,매 iteration의 Jacobian singular values/rank는 pregrasp-ik-diagnostic-v2.json에 보존했다. 오른팔과 gripper는 seed에서 고정한다.
position-only Jacobian은 rank3; position+approach 현재 solver Jacobian은 rank5. 대표 HOME constrained singular values=[0.30657117001961076, 0.25852067178303606, 0.15362749220752117, 0.04917256588479906, 0.00960596719591296]
constraint 후보 모두 wrist-flex control upper1.65806rad에 도달한다. physical joint upper와의 잔여2.7293e-6rad는 모델 값 정밀도 차이이며 관절 범위를 변경하지 않았다.
현재 solver는 접근축 cross-product residual에 전체 angular Jacobian3행을 쓴다. 축 주위 spin까지 영각속도로 억제할 수 있으므로 두 rotational DOF task의 projected Jacobian도 별도로 기록했다. 이번에 solver algorithm은 바꾸지 않았다.

## Decision

B+C로 분류: 위치는 도달 가능하지만 현재 approach 제약을 함께 걸면 모든 bounded seed가 wrist-flex 상한에 걸려 실패한다. A(TCP 위치 mismatch) 근거는 없다.
D: 현재 solver Jacobian은 rank loss가 없으므로 주원인으로 확정하지 않는다. E: HOME 한 개만의 문제는 아니지만 bounded seed만으로 모든 local minimum을 배제하지 못한다.
F:60mm top-down pregrasp 구성의 orientation/joint-range 적합성을 후속 확인할 필요가 있다. 전역적 infeasibility 증명이나 arbitrary target 변경은 하지 않는다.
기존 closing criterion15deg도 보존한다. 모든 실행 gate를 만족하는 후보0개이므로 실제 PREGRASP command를 실행하지 않는다.

## Validation

기존 test_physics_ik12개PASS(6.667s). 신규 observer 불변성 test는 최초 필수 builder인자 누락을 수정한 뒤PASS(3.678s). 기존 solver 결과와 callback 관찰 결과 동일, 외부 data qpos불변.
진단 viewer 첫 실행은 존재하지 않는 mj_copyData API로 실패; 기존 mj_getState/mj_setState로 수정했다. v2는10개 결과와 viewer 생성 완료, 예외 없이 유지 중.
viewer: KINEMATIC IK DIAGNOSTIC / NOT PHYSICS. N/B키로 candidate 전환, target/FK/오차vector/desired·actual axis/index/error/margin 표시. 기존 실제 physics viewer와 분리.
git diff --check PASS. iteration/tolerance/limits/geometry/target block 위치 변경 없음.

## Result

Center SUCCESS 아님. 실제 physics의 마지막 유효 상태는 SETTLE 완료0.200s, teacher는 PREGRASP IK 실패에서 정지한 채 유지된다.
진단 후보의 경로 검사 결과는 각 row에 기록되며, 경로만 통과해도 IK/orientation 실패를 무시하지 않는다.

## Lesson / Next

나는 TCP의 위치가 맞는지와 목표방향이 가능한지를 나누어 봐야 한다. 위치IK 성공은 방향까지 만족한다는 뜻이 아니다.
다음에는 현재 axis-task Jacobian의 spin 억제 의미를 검토하고, 정확히 같은 constraint에서 wrist-flex boundary 원인을 검증한다. 구현 변경이나 tilt 후보는 별도 근거를 남기며 기존 실패를 보존한다.
실물/OS30A/multi-seed teacher/ACT 실행 없음. digital-twin 실측은 별도 후속 작업이다.


# Axis-direction Jacobian A/B — 2026-09-16

record_id: DAPIER-2026-09-16-tool-axis-jacobian-ab

## Problem

나는 position-only IK 성공/approach IK 실패를 raw rotation Jacobian의 roll 억제와 실제 kinematic boundary로 분리한다.

## Evidence

ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch pro/dual-so101-codex-20260907; SHA534c4a6f25ab3eea6dd703526627e89dd9ba66d7; dirty 보존.
동일 stored settled0.200s, target,5seeds,damping0.02,axis weight0.05m,tolerance0.5mm/2deg,300iterations,모델 joint/control ranges.
OLD Jw와 residual u×d는 pure spin omega=lambda*u에도 비용lambda²를 준다(u×d는 u에 수직). 따라서 single-axis가 허용해야 하는 roll을 불필요하게 억제한다.
NEW J_axis=-skew(u)@Jw, residual=d-(u·d)u. pure spin의 axis differential은0이며 residual도 tangent plane에 있다. full orientation 목표는 추가하지 않았다.
Antiparallel axis는 이 tangent residual의 stationary case다. 실제 convergence는 acos(u·d)로 별도 검사하므로 거짓 성공 처리하지 않는다. 이번 seeds는 해당 경우가 아니다.
기존 solver default raw_rotation은 유지했다. axis_direction은 명시 선택 후보이며 teacher에 자동 채택하지 않았다.

## Decision

NEW formulation의 수학적 의미를 검증하되, 현재 PREGRASP 비수렴이 formulation 하나 때문이었다고 결론내리지 않는다.
아래 NEW5개 모두 wrist-flex actuator upper1.65806rad에 도달. joint upper margin2.7293335e-6rad는 ctrl/joint 숫자 정밀도 차이다. 어느 range도 변경하지 않는다.

## Validation

test_axis_direction + test_physics_ik:15testsPASS16.516s.
실제 compiled PGripper2poses ×5joints, central perturbation1e-6rad의 axis finite difference가 analytic J_axis와 일치(atol1e-9,rtol1e-7).
roll invariance: pure-roll joint의0rad/1.2rad 두 configuration에서 동일 position/axis; NEW Jacobian/residual동일,roll column0. Axis Jacobian rank2,Ju=0.
OLD5개 IKResult는 이전 저장 baseline과 정확히 동일함을 assert했다.
모든 후보 full interpolated collision/path guardPASS. q,모든 solved-joint margin,rank,svd,iteration traces는 axis-direction-ab.json에 저장.
viewer: KINEMATIC IK DIAGNOSTIC / NOT PHYSICS, N/B로 각 seed의 OLD raw_rotation / NEW axis_direction을 교대로 전환. 기존 physics/diagnostic windows 유지.

| Seed | Formulation | Position mm | Approach deg | Iterations | Rank | min singular value | Converged |
|---|---|---:|---:|---:|---:|---:|---|
|settled_HOME|raw_rotation|8.825210|4.827200|300|5|0.0096059672|False|
|settled_HOME|axis_direction|7.971782|4.284099|300|5|0.00091980049|False|
|previous_failed|raw_rotation|8.343639|4.529481|300|5|0.0096245047|False|
|previous_failed|axis_direction|8.051200|4.338504|300|5|0.00087874041|False|
|HOME_elbow_plus|raw_rotation|8.475355|4.612700|300|5|0.0096201265|False|
|HOME_elbow_plus|axis_direction|7.965921|4.279968|300|5|0.00092317702|False|
|HOME_elbow_minus|raw_rotation|9.103853|4.992300|300|5|0.0095930087|False|
|HOME_elbow_minus|axis_direction|7.979094|4.289215|300|5|0.00091571406|False|
|elbow_alternative|raw_rotation|8.721210|4.764340|300|5|0.0096104697|False|
|elbow_alternative|axis_direction|8.007211|4.308632|300|5|0.0009007852|False|

## Result

NEW位置error7.966〜8.051mm、approach約4.280〜4.339degで既存基準未達。実行可能candidate0。
NEW combined Jacobian rank5だが最小特異値約0.000879〜0.000923でOLDより小さい。軸方向だけの正しいtaskで残るconditioningとactive joint boundaryを区別する必要がある。
実際のtask physicsは新たに進めていない。前回SETTLE100steps/0.200sPASS→PREGRASP IKfailure状態を維持。診断はisolated qpospreviewのみ。CenterSUCCESSなし。
joint/command limits,tolerance,iterations,target block/table/mount geometry,clearance,success metricは変更なし。hardware/multi-seed teacher/ACTなし。

## Lesson / Next

私は不要なroll抑制を除く数学的修正と、目標poseが現行5DoF/関節範囲で実現できることを別の検証と扱う。
今回のbounded searchは全局的infeasibilityの証明ではない。次は実際の5DoF kinematic boundary/current target constructionの検証が必要。iteration増加やtolerance緩和で成功扱いしない。
PREGRASPのposition/approach/limits/pathすべてPASSのcandidateが得られるまで実際motionは送らない。

## Viewer command

DAPIER_SO101_MJCF=/home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml PYTHONPATH=/home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101:/home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101/test /home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101/pregrasp_ik_diagnostic.py --source /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/teacher-measured-state-physics.json --output /tmp/tool-axis-ab-review.json --axis-ab --viewer
既存outputは上書きしないため、再実行時は新しいfilenameを指定する。


# Center block bounded tilt feasibility

record_id: DAPIER-2026-09-16-paired-pregrasp-grasp-tilt

## Problem

나는 수직 접근의 wrist-flex 한계 이후, 현재5DoF limits 안에서 PREGRASP와 GRASP를 함께 만족하는 방향을 찾는다. 단순 위치IK 성공만으로 grasp 가능을 판단하지 않는다.

## Evidence

ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
SHA534c4a6f25ab3eea6dd703526627e89dd9ba66d7, branch pro/dual-so101-codex-20260907, dirty 보존.
동일 SETTLE0.2s 상태/모델hash와 PGripper TCP. PREGRASP와 GRASP XYZ는 이전값을 고정했다. tilt에 따라 PREGRASP 위치를 이동하지 않았다. 따라서 검사한 접근은 두 고정점 사이 joint interpolation이며 tilted straight Cartesian ray가 아니다.
corrected axis_direction,300iterations,0.5mm/2deg,closing15deg 그대로. roll 목표를 새로 넣지 않았다.
Open pad compiled vertex envelope: axial=0.012508602194283317m, radial=0.03150617970312863m, block center height=0.01999797730843671m.
axial*cos(theta)+radial*sin(theta)<=height의 첫 경계로 max tilt=14.49850581657233deg를 계산했다. 모든 roll/azimuth에 대한 보수적 pad nonpenetration 범위이며, 전체 feasible cone의 최대각 또는30mm safety 허가가 아니다.
Coarse:0/4.832835/9.665671/14.498506deg,nonzero tilt에서8azimuth(45deg간격). Refine:best residual 인접 tilt2.416418deg/azimuth22.5deg. 총33directions×2bounded seeds, PREGRASP와GRASP 모두평가.
Seed는settledHOME와기존failedcandidate. GRASP는각PREGRASP result에서초기화. 대규모random search없음.

## Decision

모든 safety gate를 통과한 candidate0개. 실제motion은 실행하지 않는다. IK/convergence/closing과 safety policy 실패를 분리한다.
기존 일반 finger/table30mm와nonfinger/block30mm를 임의 수정하거나 pair를 삭제하지 않았다.

## Validation

실제 structured diagnostic run33directions 완료. target고정/unitaxis/geometry-bound root/iteration cap/eligibility conjunction assertion PASS(tilt-validation.json).
기존axis-direction finite-difference/roll invariance15tests 근거유지. solver수정없음. git diff --check PASS.
각endpoint q/모든joint margin/Jacobian rank/singularvalues/closing axis/face/error/endpoint clearance와HOME 및approach path는tilt-feasibility.json에기록.
Guard가최초위반sample에서정지하면그이후를검사완료로주장하지않는다. block-path 추가검사는general path가이미실패한경우not checked로기록한다.

## Result

6개direction의bestbranch에서PREGRASP와GRASP IK모두PASS. 그중closing15deg까지통과하는direction은tilt7.249253deg/azimuth337.5deg였지만safetyFAIL이다.
가장작은관측IK양끝점성공tilt2.416418deg도closing과safetyFAIL. 최소실행가능tilt를찾았다고주장하지않는다.
주황색reviewcandidate desired axis=[0.11658069425291079, -0.04828930467042675, -0.9920065951302723]

|Metric|PREGRASP|GRASP|
|---|---:|---:|
|position mm|0.043438417|0.0722092814|
|approach deg|1.99697044|1.96650691|
|closing deg|7.86196494|9.78423265|
|wrist flex rad|1.50249479|1.08841543|
|wrist upper margin rad|0.155567943|0.569647296|
|table gap mm|64.0646367|4.16654677|
|nonfinger/block gap mm|71.2851383|11.864144|

HOME→PREGRASP45samplesPASS. PREGRASP→GRASP는fraction0.583333에서geom36 left_pgripper_pad_1 ↔ geom0table가28.858367mm로30mm미달;8samples에서거부.
GRASPendpoint同pair4.166547mm:positive separation이지만30mm미달. nonfinger/block도['left_pgripper_housing', 'red_block_geom'] = 11.864144007639709mm로30mm미달이다.
따라서이번에는tilt로wrist-flex한계를벗어난기구학적후보가존재하지만현재task의접근safety정책까지충족하지못한다. globally unreachable이라기록하지않는다.
실제physics마지막상태는기존SETTLEPASS0.200s/PREGRASP IKfailure. 이번에는kinematicpreview만실행,새physicsPREGRASP없음.

## Lesson / Next

나는5DoF에서position3+axis direction2를사용하고tool-axis roll은axis task에서자유로남겨야한다. 다만closing face기준은별도검증이필요하다.
다음은table위40mmblock을잡는목적과finger/table30mm 및housing/block30mm 정책의정확한물리적의미를검토하는것이다. 모델이나threshold를자동변경하지않는다.
현재bounds는보수적subset이며search resolution/2seeds/roll초기화한계가있다. 물리grasp성공이나전체cone최소각을증명하지않았다.
기존physicsviewer와diagnosticviewer보존. feasibilitymap은green=all gatesPASS,orange=IK+closingPASS이나safetyFAIL,red=기타FAIL. N/B로각candidate의PREGRASP/GRASP preview전환.
CenterSUCCESS없음;hardware/OS30A/multi-seedteacher/ACT없음.

## Viewer command

DAPIER_SO101_MJCF=/home/dapier-jhj/DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml PYTHONPATH=/home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101:/home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101/test /home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907/2ARM_ROBOT/sim/mobile_dual_so101/tilt_feasibility.py --source /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/teacher-measured-state-physics.json --output /tmp/center-tilt-review.json --viewer


### 잔차 요약 (전역 최솟값 아님)

{
  "best_normalized_residual_candidate": {
    "tilt": 4.832835272190777,
    "azimuth": 337.5,
    "seed_id": 1,
    "pregrasp_position_mm": 0.029043121817389505,
    "grasp_position_mm": 0.04166122522063342,
    "pregrasp_approach_deg": 1.9992482587262814,
    "grasp_approach_deg": 1.992116853082558
  },
  "minimum_observed_axis_error_deg": {
    "pregrasp": 1.0713749222851894,
    "grasp": 1.1066295822513286
  }
}

최소 approach error는 해당 endpoint에서 관측한 값이며 다른 모든 gate를 통과한다는 뜻이 아니다.


# Center block task-aware policy / waypoint 결과 (2026-09-16)

## Problem
일반 external-obstacle 30 mm 정책으로는 의도된 pad/support 및 gripper/target 근접을 표현할 수 없었다. 전역 threshold, 승인된 geometry, TCP, HOME, joint limits를 유지하면서 exact pair의 phase 의미를 분리했다.

## Evidence
- ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
- branch: pro/dual-so101-codex-20260907
- HEAD: 534c4a6f25ab3eea6dd703526627e89dd9ba66d7; dirty / 기존 미커밋 보존.
- scene_id: integration_desk; model SHA256: ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368
- gripper: 좌우 NORMA PGripper, 기존 RGB camera stand. 모델/asset hash는 실행 JSON provenance에 기록.
- compiled 40 mm block half-size: [0.02,0.02,0.02] m.
- 저장된 tilted GRASP의 pad 36/38 최저 world Z 및 table gap: 4.1665468 / 9.6526852 mm.
- housing 29 ↔ block 74 gap: 11.8641440 mm.
- pad 36/38 ↔ block 74 gap: 2.5818574 / 1.5612816 mm.
- 모든 exact pair contact 없음. 이 값은 fixture 측정값이며 safety threshold가 아니다.
- TCP frame에서 pad inner closing faces는 약 -25.516639 / +25.516639 mm, pad approach 방향 extent는 약 ±12.508602 mm이다. 전체 compiled projection min/max는 manipulation-envelope.json에 기록.
- 기존 tilt geometry envelope: axial 12.508602 mm, radial 31.506180 mm, available height 19.997977 mm. all-roll 충분조건 subset이지 전역 feasible cone이 아니다.

## Decision
정책 범위는 SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED.
정확히 다음 5개 관계를 별도 검사하고 삭제하지 않는다.

| Pair IDs | 관계 | policy |
|---|---|---|
| 36–0, 38–0 | left pads / table | SUPPORT_NEAR_APPROACH |
| 29–74 | left housing / block | TARGET_NEAR_APPROACH |
| 36–74, 38–74 | left pads / block | 접근 중 TARGET_NEAR_APPROACH, CLOSE 이후 INTENDED_FINGER_CONTACT |

근접 허용은 finite evidence, compiled geometry hash 일치, positive separation, contact 없음이 모두 필요하다.
numerical margin은 64 × float64 epsilon × max(1 m, world position magnitude)이며 현 장면에서 1.42108547e-14 m이다. 조립 공차나 실물 안전거리가 아니다. 기존 거리 certificate의 conservative bound를 유지한다.
의도된 finger contact는 CLOSE/GRASP_CONFIRM 및 접촉을 유지해야 하는 LIFT/HOLD에서만 기존 1 mm penetration 한도와 함께 허용한다. housing/support 접촉은 해당 phase에서도 거부한다.
HOME, RESET, SETTLE, REACH_HIGH, ALIGN_TOOL에는 새 근접 예외가 없다. unrelated arm/table 및 나머지 arm/target 관계는 30 mm를 유지한다.
기존 shoulder-table structural 관계는 별도로 유지한다.

기존 teacher CLOSE/GRASP_CONFIRM/LIFT/HOLD와 controller를 재사용했다.
새 순서는 REACH_HIGH(기존 60 mm stand-off 위치 이동) → ALIGN_TOOL(동일 XYZ axis 정렬) → PREGRASP_NEAR(상태 확인 후 proximity 진입) → APPROACH_COARSE(compiled pad 최저점 기준 일반 30 mm 경계) → APPROACH_FINE → CLOSE → GRASP_CONFIRM → LIFT_5MM → LIFT_15MM → LIFT_30MM → HOLD이다.
LIFT_30MM의 target은 기존 마지막 35 mm target을 재사용하며 성공 기준은 settle bottom 대비 실제 30 mm 이상 상승/양 finger positive contact/연속 3 s 유지이다.
각 solve는 이전 arm IK branch를 seed로 사용하되 현재 gripper command를 유지한다. 각 endpoint와 전체 joint-interpolated segment, 각 실제 physics step에 정책을 검사한다. timed trajectory 종료만으로 통과하지 않고 measured position 0.5 mm, alignment 이후 approach 2°/closing 15°를 검사한다.
격리된 planning data의 qpos만 직접 배치한다. 실제 실행 data는 RESET 외 ctrl + mj_step만 사용한다.

## Validation
- 정책 + 기존 near-support, BOX–BOX, MESH–BOX, collision guard, center teacher: 29 tests PASS (manipulation-policy-tests.log).
- 최종 controller 연결, integration reset/settle, measured state, 기존 teacher: 18 tests PASS (waypoint-final-tests.log).
- 최종 manipulation policy suite: 7 tests PASS (manipulation-final-tests.log).
- negative: support/housing touching 및 penetration contact evidence 거부, finger wrong-phase contact 거부, 기존 1 mm 초과 penetration 거부, compiled geometry 변경 거부.
- unrelated shoulder geom 9/table는 APPROACH_FINE에서도 30 mm 미달 시 거부.
- 기존 tilted PREGRASP→GRASP: 새 phase 정책으로 13-sample full segment PASS. 이것은 kinematic regression이며 실제 접근 실행 성공이 아니다.
- 기존 regression의 실제 penetration/certificate checks 보존. 새 정책의 contact negative tests는 mj_addContact로 명시적 contact evidence를 주입하는 검사이며 실제 dynamics grasp로 과장하지 않는다.
- Viewer actual physics run과 headless 동일 경로에서 같은 첫 blocker 재현.

## Result
HOME PASS → RESET PASS → SETTLE PASS (100 steps, 0.200 s) → REACH_HIGH planning FAIL.
실제 REACH_HIGH 팔 이동은 시작하지 않았다. viewer는 0.200 s 마지막 실제 physics state를 유지한다.

REACH_HIGH XYZ: [0.200000, ~0, 0.0799979773] m.
position-only IK: 14 iterations, converged, position error 0.246598 mm.
left q target(rad): [0.6529965744, -0.3543922979, 0.6483683679, 0.5870686964, -0.0517589783, 2.2028].
measured left q(rad): [1.0490879e-9, 0.0005674787773, 0.0004791017926, 0.0001332200550, -4.0854784e-7, 2.202799995253].
first blocked pair: geom36 left_pgripper_pad_1 / geom74 red_block_geom.
full path sample 19, fraction 0.9473684211: 29.4835821 mm < general 30 mm.
endpoint: same pair 22.7009709 mm < 30 mm.
REACH_HIGH는 tool 정렬 전이므로 near-approach 예외가 적용되지 않는다. 실제 접촉/관통을 일으킨 것이 아니라 실행 전 safety gate에서 거부됐다.
이 실패는 이전 tilted aligned 후보와 다른 position-only intermediate orientation에서 발생했다.
APPROACH/CLOSE/GRASP_CONFIRM/LIFT/HOLD 미실행, Center SUCCESS 없음, HOLD 0 s.
최종 전체 q, joint margin, target/measured state, contact, model/asset/source hashes: waypoint-task-physics-final.json.
viewer actual report: waypoint-task-physics.json (표시 필드 보완 전 실행), 동일 failure를 재현한 최종 headless report와 분리 보존.

## Lesson / Next
최종 grasp orientation이 가능하고 기존 접근 구간이 정책상 안전해도, 정렬 전 position-only REACH_HIGH branch가 target에 먼저 접근할 수 있다. 다음 검토는 REACH_HIGH의 compiled tool envelope에 근거한 stand-off 또는 더 이른 정렬 순서다. 30 mm/geometry/IK tolerance를 변경해서 통과시키지 않았다.
실제 최초 blocker에서 중단하라는 지시에 따라 추가 trajectory tuning은 하지 않는다.
실물 OS30A 측정/digital-twin alignment는 별도 후속이며 이번에 장치 연결/robot motion/ACT/multi-seed를 실행하지 않았다.

## 변경 범위
DAPIER source: collision_guard.py, shoe_task.py, center_block_teacher.py, 새 waypoint_block_teacher.py, integration_manipulation_pairs.json, manipulation fixture/tests, 이 학습 기록.
KIT: local-validation 아래 실행 JSON/log/report만 추가. README/patch/의존성/실물 설정 변경 없음.
기존 preexisting dirty 파일은 보존. git reset/revert/clean/commit/push 없음.

## Viewer 재실행

```bash
bash /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/run-waypoint-viewer.sh
```

동일 실제 physics 경로를 실행하고 첫 실패 상태를 유지한다. 새 결과는 /tmp/dapier-waypoint-review.*/report.json에 보존한다.


# DAPIER-2026-09-17-geometry-staging-physics

## Problem
나는 TCP-only REACH_HIGH가 정렬 전 block에 먼저 접근하는 문제를 geometry/ordering 문제로 분리했다. General 30 mm와 기존 manipulation policy는 그대로 유지했다.

## Evidence
ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch: pro/dual-so101-codex-20260907
HEAD: 534c4a6f25ab3eea6dd703526627e89dd9ba66d7 (dirty / 기존 미커밋 보존)
scene_id: integration_desk
model SHA256: ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368
gripper: NORMA PGripper both; pinned CAD + RGB camera stand
모델·asset·source SHA256 전체와 target/measured q는 staging-physics.json에 보존한다.

Retreat direction = [-0.11658069425291079, 0.04828930467042675, 0.9920065951302723]
Compiled gripper TCP sphere envelope radius = 120.241927 mm; block radius = 34.641016 mm.
검색 상한은 두 radius + 기존 general clearance에서 산출한 184.882943 mm다. 이 상한 자체가 안전성을 보장하지 않으며 모든 실제 candidate의 전체 arm/table/block 경로를 검사한다.
선택 retreat = 9.629320 mm; 바로 아래 거부 offset = 9.147854 mm.
25개 coarse grid 설정에서 첫 feasible bracket을 찾고 기존 position tolerance와 같은 0.5 mm search resolution까지 refine했다. 실제로 검사한 offset은 7개다. 선택값을 상수 offset으로 hard-code하지 않았고, bounded sampled 1D 결과를 전역 최소라고 주장하지 않는다.

## Decision
HOME → SAFE_STAGE → ALIGN_HIGH → PREGRASP_NEAR → APPROACH_COARSE → APPROACH_FINE → CLOSE → GRASP_CONFIRM → LIFT/HOLD 구조로 변경했다.
SAFE_STAGE는 position-only task, ALIGN_HIGH부터 기존 position 0.5 mm / approach 2° / closing 15°를 모두 적용한다. Tool roll을 새 IK 목표로 강제하지 않는다.
SAFE_STAGE와 ALIGN_HIGH에는 task-near exemption이 없다. PREGRASP_NEAR부터만 기존 exact-pair 정책을 적용한다. 각 성공 q를 다음 seed로 사용한다.
모든 사전 waypoint/segment가 통과해야 실제 task motion을 시작한다. IK/preview는 private MjData, 실제 진행은 동일 teacher의 data/controller에서 ctrl + mj_step이다.

## Validation
최종 18 tests PASS / 10.536 s: staging_chain, manipulation_policy, axis_direction, center_block_teacher, measured_state.
실제 state가 planning 전후 bitwise 동일함을 확인했다. Wrong-phase exemption 금지와 unrelated 30 mm, 접촉/관통 거부를 보존했다.
경로 샘플 간 최대 joint 변화는 기존 teacher 추가 경로 검사와 같은 0.025 rad다. 아래 path min은 30 mm로 cap된 guard 요약값이 아니라 해당 samples의 general distance를 별도로 기록한 값이다.

| Phase | XYZ m | IK error mm | Approach deg | Closing deg | Endpoint general mm | Path general min mm | Samples |
|---|---|---:|---:|---:|---:|---:|---:|
| SAFE_STAGE | 0.19887741, 0.00046499, 0.08955033 | 0.135285 | 33.865342 | 46.890356 | 32.176758 | 32.176758 | 28 |
| ALIGN_HIGH | 0.19887741, 0.00046499, 0.08955033 | 0.043652 | 1.997872 | 8.241644 | 34.154427 | 30.135136 | 46 |
| PREGRASP_NEAR | 0.20000000, 0.00000000, 0.07999798 | 0.094133 | 1.997700 | 8.740712 | 72.400006 | 72.400006 | 4 |
| APPROACH_COARSE | 0.20000000, 0.00000000, 0.04580337 | 0.083957 | 1.961004 | 10.330879 | 72.400006 | 72.400006 | 12 |
| APPROACH_FINE | 0.20000000, 0.00000000, 0.01999798 | 0.098924 | 1.956600 | 11.304659 | 48.828859 | 48.828859 | 8 |

SAFE_STAGE의 approach/closing 값은 진단값이며 이 단계에서는 정렬 목표를 적용하지 않는다. ALIGN_HIGH에서는 두 기준 모두 충족한다. 모든 joint margin과 각 segment의 predecessor seed/target q는 실행 JSON의 staging_search.selected.segments에 기록했다.

## Result
실제 HOME PASS → RESET PASS → SETTLE PASS (0.200 s) → SAFE_STAGE PASS (3.064 s) → ALIGN_HIGH 도중 FAIL (5.046 s). 총 2,523 physics steps를 수행했다.
SAFE_STAGE measured position error = 0.254537 mm; measured general gap = 31.998541 mm.
정확한 failure pair: geom 36 left_pgripper_pad_1 / geom 74 red_block_geom. 이 단계는 GENERAL_OBSTACLE / 30 mm다.
Raw measured native = final guard = 29.999496143 mm. Contact/penetration 없음. 30 mm보다 약 0.503857 µm 작아서 fail-closed했다.
같은 시점 command FK gap = 30.168056910 mm; maximum actuator tracking error = 0.001074397258 rad. Command FK는 이상적인 linked-jaw pose의 counterfactual이며 실제 measured state와 구분한다.
이 pair에서는 native distance가 정상 positive라 sphere/certificate가 결과를 올리지 않았다. False-zero correction 문제가 아니다.
저장된 전체 raw qpos를 그대로 mj_forward하여 동일 실패를 재현했다. Actuator-only apply_control_as_pose는 passive jaw coordinates를 ideal linkage로 재배치하므로 약 30.000170 mm로 바뀌어 이 작은 실패를 잘못 PASS로 보일 수 있다. 따라서 actual failure fixture는 passive jaw 포함 전체 qpos/qvel/ctrl/time을 보존한다.
PREGRASP_NEAR 이후 실제 motion은 실행하지 않았다. Bilateral finger force 0 N, HOLD 0 s, Center SUCCESS 없음. Block에는 기존 table 지지 접촉만 있다.
마지막 actual physics state와 별도 kinematic diagnostic viewer는 유지했다. 중복 plan-only viewer만 종료했다. 화면 잠금 상태가 확인돼 사용자가 직접 보고 있었는지는 확인할 수 없으며 잠금 설정은 변경하지 않았다.

## Lesson / Next
기구학적 최소 staging은 dynamic tracking reserve를 보장하지 않는다. 이번 ALIGN_HIGH sampled path minimum 30.135136 mm는 실제 command/measured geometry 차이를 흡수하지 못했다.
다음 검토는 threshold/geometry/IK tolerance를 낮추는 것이 아니라, 측정된 arm·passive jaw 추종오차 envelope를 포함하는 staging/path 검증이다. 이번 요청의 최초 physics blocker에서 멈췄으며 추가 offset 튜닝을 하지 않았다.
긴 wall time의 원인과 viewer/guard 성능은 별도 profiling 대상으로 남긴다. 느린 실행을 timeout으로 잘라 결과를 성공 처리하지 않았다.
실제 hardware/OS30A/multi-seed teacher/ACT 실행 없음. 정책은 SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED다.

## Files / Reproduction
이번 source 수정: waypoint_block_teacher.py, center_block_teacher.py의 marker hook, test_staging_chain.py, staging_chain.json, staging_physics_blocker.json. 기존 task-aware policy/limits/models는 변경하지 않았다.
KIT 변경은 local-validation의 JSON/log/report/재실행 helper뿐이다. GitHub push/commit 또는 calibration 변경은 하지 않았다.
회귀 로그: staging-final-regression.log. 실제 실패 상세: staging-first-physics-blocker.json. 이전 REACH_HIGH 코드는 waypoint-before-staging.py로 보존했다.

```bash
bash /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/run-waypoint-viewer.sh
```
기존 helper는 현재 staging teacher를 동일 환경으로 실행하며 새 결과를 /tmp/dapier-waypoint-review.*/report.json에 저장한다. --plan-only --viewer는 실제 RESET/SETTLE 이후 격리된 계획 검사까지만 수행한다.


# DAPIER-2026-09-17-dynamic-staging-preflight

## Problem
기존 9.629320 mm retreat는 kinematic path PASS지만 actuator tracking과 passive jaw motion 때문에 실제 ALIGN_HIGH에서 30 mm를 소진했다. 나는 general clearance나 joint tolerance를 낮추지 않고 실행 dynamics를 포함한 staging을 확인했다.

## Evidence
ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch: pro/dual-so101-codex-20260907; SHA: 534c4a6f25ab3eea6dd703526627e89dd9ba66d7; dirty 기존 변경 보존.
scene_id: integration_desk; MuJoCo 3.3.7; 양팔 NORMA PGripper + RGB camera stand.
Model SHA256: ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368.
모든 asset/source hash 및 joint mapping은 dynamic-single-viewer.json의 provenance에 기록했다.

| Retreat | Kinematic | Copied physics | 최소 measured general clearance |
|---:|---|---|---:|
| 9.629320 mm | PASS | FAIL: ALIGN_HIGH / 5.046 s | 29.999496 mm |
| 10.110786 mm | PASS | PASS | 30.378087 mm |

기존 실패 pair는 36 left_pgripper_pad_1 ↔ 74 red_block_geom이다. 기존 command/FK 값은 30.168057 mm였지만 raw full-state gap은 29.999496 mm였다. Full-state fixture로 동일 실패가 재현된다. Actuator-only preview는 안전 근거가 아니다.

## Decision
copy.copy(MjData)로 qpos/qvel/ctrl/time/passive linkage/warmstart 등 full integration state를 복사한다. 동일 teacher.move, septic trajectory, actuator, timestep, phase policy와 mj_step으로 SAFE_STAGE→ALIGN_HIGH를 끝까지 실행한다. 매 preflight 전후 live integration state의 bitwise 불변을 검사한다.
기존 kinematic guard 위에 이 SIM-only gate를 추가했다. 기존 approach 반대 방향 1D coarse→refine 검색을 사용한다. 마지막 실패 9.629320 mm와 선택 10.110786 mm의 간격은 0.481466 mm이며 검색 resolution은 0.5 mm다. 전역 최소를 증명한 값은 아니다.
HOME/geometry/30 mm/general pair/phase exemption/limits/controller/timing 설정은 유지했다. Duration A/B는 필요하지 않아 하지 않았다. 이 dynamic preflight의 범위는 staging 두 phase이며, 이후 grasp 전체에 대한 dynamic PASS로 확대하지 않는다.

## Validation
- dynamic, manipulation policy, staging, center teacher, measured-state 회귀: 19 tests PASS / 348.106 s (dynamic-regression.log).
- 단일 viewer 변경 후 center teacher: 2 tests PASS / 3.574 s.
- 새 PREGRASP boundary fixture와 measured-state/center 회귀: 6 tests PASS / 9.197 s (dynamic-final-targeted-regression.log). 일부 기존 검사는 중복이며 합계를 고유 test 수로 주장하지 않는다.
- fresh 검색 후 실제 staging 3,852 step telemetry가 copied preflight와 모두 동일하다. ALIGN_HIGH 종료 full integration state도 bitwise equal, max difference 0.
- 매 physics step(2 ms)에 target/measured q, raw qpos/qvel, passive jaw, command-FK/actual clearance, tracking error, closest general pair를 기록했다.
- 기존 wrong-phase exemption 금지 및 touching/penetration 거부 회귀를 유지했다.

raw-coordinate finite difference(1e-5/5e-6): shoulder_lift -0.207461 m/rad, elbow_flex -0.200922 m/rad, wrist_flex -0.066373 m/rad, wrist_roll +0.012293 m/rad, jaw_1_slide +0.184097 m/m. 이전 실패의 raw error로 선형 예측한 변화 -0.168537 mm는 관측 -0.168561 mm와 근접한다. 다른 좌표를 고정한 국소 진단이며, linkage를 통한 영향이나 일반 안전 margin으로 사용하지 않는다.

## Result
| 실제 phase | 결과 | SIM time |
|---|---|---:|
| HOME / RESET | PASS | 0 s |
| SETTLE | PASS, 100 steps | 0.200 s |
| SAFE_STAGE | PASS | 3.064 s |
| ALIGN_HIGH | PASS | 7.904 s |
| PREGRASP_NEAR | 실제 이동 31 steps 후 measured-state FAIL | 7.966 s |

ALIGN_HIGH 최소 measured general gap 30.378086984 mm (5.292 s), 같은 시점 command-FK 30.545885488 mm. 차이 0.167798503 mm. Staging 최대 actuator tracking error 0.001119969354 rad. 종료 position error 0.190450 mm, approach 약 1.9981°. Endpoint general gap 34.490640 mm.

첫 blocker: left_gripper raw measured 2.2028000148296067 rad, upper 2.2028 rad, excess 1.48296068758e-8 rad > 기존 measured numerical tolerance 1e-8 rad. Command는 2.2027999930692252 rad로 strict ctrlrange 안이다. Right measured excess는 2.697614e-9 rad다. Raw state는 clamp하지 않았고 tolerance/limit 변경도 하지 않았다.
실패 시 general clearance 72.400006 mm, pad1/block 34.361178 mm, pad1/table 73.802163 mm, housing/block 81.133088 mm로 collision policy는 PASS다. Protected contact 없음, 양 finger force 0 N. Block의 기존 table 지지 접촉은 존재한다. CLOSE/GRASP_CONFIRM/LIFT/HOLD는 실행하지 않았다. Center SUCCESS false, HOLD 0 s. 이는 boundary overshoot 관측이며 A/B/C dynamics 원인 분류는 다음 조사로 남긴다.
새 실패 raw qpos/qvel/ctrl/time은 test/fixtures/dynamic_pregrasp_boundary.json에 저장하고 기존 measured validation이 실제로 거부하는 테스트를 추가했다.

## Viewer / 실행 기록
과거 불필요한 MuJoCo 프로세스는 모두 종료 확인했다. 이전 live는 X connection broken으로 ALIGN_HIGH 시작에 종료됐고, 재시도는 두 번째 passive viewer 초기화 시 exit 139였다. 이 둘은 physics failure와 분리한다.
이후 단일 passive viewer로 통합해 정상 실행했다. 화면 전용 MjData는 현재 live 또는 copied preflight state의 snapshot을 관찰하며 별도 trajectory를 실행하지 않는다. Kinematic diagnostic / copied dynamic preflight / LIVE TASK 라벨을 구분한다. 현재 viewer 하나를 실제 실패 상태에서 physics 정지한 채 유지한다. GLX 내부 root cause가 해결됐다고 주장하지 않는다.

재실행:
```bash
bash /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/run-waypoint-viewer.sh
```

## Lesson / Next
기구학적 staging margin에 임의 상수를 더하는 대신 full-state dynamic preflight로 실행 가능한 후보를 검증할 수 있었다. 단, staging 통과가 이후 모든 phase의 유효성을 보장하지 않는다. 다음은 PREGRASP_NEAR 초기 gripper boundary transient와 command/measured/passive dynamics 분석이다. Tiny overshoot라는 이유만으로 tolerance를 넓히지 않는다.
사용자가 사진(/tmp/codex-clipboard-N8fY3G.png)의 접힌 자세를 원하는 시작 자세로 제시했다. 우선 현재 HOME으로 파이프라인을 확인하고 나중에 변경하기로 했다. 현재 결과는 그 새 자세에 적용되지 않는다. 정확한 joint mapping/readback을 기반으로 별도 HOME 후보의 reset qpos/qvel/ctrl contract와 경로를 재검증해야 한다.
SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED. 실제 hardware/OS30A/multi-seed teacher/ACT 실행 없음. RGB-D digital-twin alignment는 후속 read-only calibration 작업으로 유지한다.

## 변경 구분
Source: dynamic_preflight.py, waypoint_block_teacher.py, center_block_teacher.py, test_dynamic_preflight.py, test_measured_state.py, 두 dynamic fixture 및 이 학습 기록. 기존 safety policy/geometry를 이번 작업에서 수정하지 않았다.
KIT: local-validation 아래 JSON/log/Markdown. 머신 설정/보정 및 package/dependency 버전 변경 없음. Commit/push 없음.


# DAPIER-2026-09-17-task-open-reference

## Problem
PREGRASP_NEAR에서 left_gripper measured qpos가 joint upper를 1.48296e-8 rad 초과해 기존 SIM tolerance 1e-8 rad로 거부됐다. 나는 tolerance를 실패마다 키우지 않고 command 출처와 작업용 opening을 먼저 분리했다. 기존 거리 함수, 모델 배치, retreat 검색을 다시 시작하지 않았다.

## Evidence
ACTIVE_WORKTREE: /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch: pro/dual-so101-codex-20260907; HEAD: 534c4a6f25ab3eea6dd703526627e89dd9ba66d7, dirty 보존.
scene_id: integration_desk. Model SHA256 ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368. MuJoCo 3.3.7, 양팔 NORMA PGripper. 정확한 source/asset hash는 task-open-live.json provenance.

명령 추적: HOME은 pgripper.home_action의 2.2028 rad 완전 열림이다. position IK는 팔 5축을 풀며 비조작 gripper 채널을 seed에서 보존해 target 2.2028 rad를 전달했다. CenterBlockTeacher.move는 각 이동 시작에서 전체 measured actuator q를 trajectory start로 사용했다. 따라서 실패 시 2.2027999930692252 rad는 명시적인 새 open 목표가 아니라, 직전 measured 2.2027999930619915 rad에서 기존 upper target으로 septic 보간하던 command다. Mapping은 joint transmission / gear 1을 그대로 사용하며 양의 motor q가 opening 증가 방향이다.

별도 isolated diagnostic은 직전 ALIGN_HIGH full integration state에서 동일 기존 PREGRASP trajectory를 mj_step으로 재생하되 measured safety gate 없이 수치 응답만 관찰했다. 이는 task PASS 근거가 아니다. 왼쪽 upper excess peak 1.98394176643e-8 rad, 오른쪽 5.10069453341e-9 rad였고 이후 범위 안으로 수렴했다. 마지막 왼쪽 tracking error -6.684923e-9 rad, 오른쪽 -4.496371e-9 rad, 모든 warning counter 0. 이 제한된 시간 구간은 bounded solver/coupling transient와 부합하며 지속 발산 증거는 없었다. Joint-limit/equality constraint position/force와 raw qpos/qvel/ctrl는 gripper-old-boundary-diagnostic.json에 보존했다. 전역 수치 오차 상한을 입증한 것으로 확대하지 않는다.

## Decision
Measured state는 IK/경로 시작과 runtime 검사에 그대로 사용한다. Gripper 명령 궤적 시작만 직전 data.ctrl에서 가져오고, 비조작 gripper target은 명시적인 task hold reference로 분리했다. CLOSE는 명시적인 closing command를 사용하고, LIFT/HOLD는 해당 command를 유지한다. Raw qpos clamp나 기록 치환은 없다.

기존 선택 retreat 10.110785931572185 mm를 입력 report에서 재사용한다. 고정 후보 1회만 기구학적 경로와 copied physics로 재검증했다(search_repeated=false). 과거 baseline 생성 경로와 fixture는 보존했다.

Opening은 기존 compiled inner-face datum 및 equality mapping을 사용한다. ALIGN_HIGH→PREGRASP→APPROACH의 기존 q interpolation(최대 0.025 rad/sample)에서 block의 closing-axis projection을 계산했다. 최대 width에 기존 TCP position tolerance 0.5 mm를 양쪽에 예약하고, 남는 opening 폭을 fit reserve와 command upper reserve에 절반씩 배분했다. 임의의 각도를 고른 것이 아니며 실물 tolerance를 새로 선언한 것도 아니다.

| 값 | 결과 |
|---|---:|
| 최대 jaw opening | 51.033277997 mm |
| 기존 접근 최대 block projection | 50.022911342 mm |
| 양쪽 TCP 위치 허용오차 포함 required opening | 51.022911342 mm |
| 선택 task jaw opening | 51.028094670 mm |
| 명시적 좌/우 open reference | 2.2025746379457667 rad |
| command upper margin | 0.000225362054233 rad |
| 추가 fit reserve | 0.005183327 mm = 5.183327 µm |

새 실제 계획의 APPROACH_FINE projection도 다시 계산했다: 50.022914660 mm, TCP 여유 이후 reserve 5.180009 µm로 여전히 양수다. 이는 매우 좁은 sampled SIM envelope다. 외란/다중 초기조건/실물 조립 공차를 만족하는 여유라고 주장하지 않는다. 실제 접근 중 contact와 geometry는 기존 runtime guard가 계속 검사한다.

HOME/reset qpos, joint/ctrl range, gains, timestep, general 30 mm, phase policy를 변경하지 않았다. Numerical tolerance는 양쪽 모두 기존 1e-8 rad를 유지했다. Open reference로의 이동은 SAFE_STAGE의 기존 controller trajectory에서 연속 수행했다.

## Validation
실행 명령은 기존 venv와 DAPIER_SO101_MJCF/PYTHONPATH 환경을 재사용했다.
python -m unittest test_task_open test_measured_state test_dynamic_preflight test_manipulation_policy test_near_support test_collision_guard -q
34 tests PASS / 477.220 s. 범위 밖 command의 사전 거부, finite 검사, 허용된 raw overshoot 보존, 범위 초과 거부, 양쪽 계약, 기존 boundary failure fixture, opening geometry, command-start와 measured path-start 분리, near-support/mesh certificate/실제 penetration 및 wrong-phase 거부를 보존했다.

실제 수정 경로의 preflight와 live를 다시 비교했다. SAFE_STAGE→ALIGN_HIGH telemetry 3852 rows 전체 동일, 종료 mjSTATE_INTEGRATION bitwise equal / max difference 0. 새 staging minimum measured clearance 30.378585882 mm. 원래 30.378086984 mm PASS 결과도 보존하며, 두 값 모두 30 mm를 약 0.378 mm 넘는 현재 조건의 결과이지 다중 초기조건이나 실물 강건성 증거가 아니다.

## Result
| Phase | 결과 | SIM time | measured position error | approach error |
|---|---|---:|---:|---:|
| HOME / RESET | PASS | 0 s | — | — |
| SETTLE | PASS, 100 step | 0.200 s | — | — |
| SAFE_STAGE | PASS | 3.064 s | 0.253628 mm | 정렬 비요구 |
| ALIGN_HIGH | PASS | 7.904 s | 0.190450 mm | 1.998109° |
| PREGRASP_NEAR | 이동 0.8 s 완료 후 orientation gate FAIL | 8.704 s | 0.133659 mm | 2.009502° > 2° |

PREGRASP closing error 8.745122° < 15°. 이전 gripper boundary blocker는 재발하지 않았다. SETTLE의 기존 최대 overshoot 8.963255e-9 rad는 기존 tolerance 내에서 raw 그대로 보존됐다. SAFE_STAGE 전환 초반 0.266 s에서 left 8.185774e-9 / right 8.034717e-9 rad의 tiny overshoot가 있었고 기존 1e-8 tolerance 안에서 raw 그대로 보존했다. ALIGN_HIGH와 PREGRASP_NEAR에서는 양쪽 upper excess가 모두 0이었으며 마지막 양쪽 command는 동일 reference다. 최종 measured left 2.202574637959145 rad / right 2.202574637954053 rad, tracking errors 각각 1.337819e-11 / 8.286261e-12 rad. 최종 jaw opening left 51.028095133 mm / right 51.028094670 mm. Warning 모두 0.

실패 시 general clearance 72.400006 mm, pad1-table 64.100552 mm, pad2-table 69.404155 mm, housing-block 71.382899 mm, pad1-block 24.552272 mm, pad2-block 30.351141 mm. Target-near phase 정책하에 모두 PASS이며 task-specific pair contact는 없다. 양 finger force 0 N, CLOSE/GRASP_CONFIRM/LIFT/HOLD 미실행. Center SUCCESS false. 원래 2° gate를 완화하거나 새로운 blocker를 숨기지 않았다.

Viewer 하나를 이 failure state에서 유지했다. Live/copy diagnostic을 구분하며 command/measured, command margin, upper excess, tolerance, jaw opening 및 clearance를 표시한다. Source: center_block_teacher.py, waypoint_block_teacher.py, dynamic_preflight.py, test_task_open.py, task_open_reference.json. KIT에는 실행 JSON/log와 기존 run-waypoint-viewer.sh의 --staging-report 인자만 추가했다. 별도 학습 원장을 만들지 않고 이 파일에 기록한다.

재실행:
bash /home/dapier-jhj/Downloads/DAPIER_DDS_MuJoCo_ACT_Sim2Real_Kit_20260915/dapier_sim2real_kit/local-validation/integration-task-20260916-aeg3vtmi/run-waypoint-viewer.sh
실제 결과: 같은 local-validation의 task-open-live.json / task-open-live.log. 이전 dynamic-single-viewer.json, dynamic_pregrasp_boundary.json 및 모든 regression fixture 보존.

## Lesson / Next
Gripper command reference와 measured solver state는 다른 역할이다. Command를 measured 상태에 매번 재고정하지 않고, geometry로 가능한 내부 opening을 사용하니 기존 수치 tolerance를 유지한 채 boundary blocker를 넘었다. 다음은 PREGRASP endpoint의 command/actual axis error와 tracking bias를 분리할 차례다. 이번 요청의 첫 새 blocker에서 정지했고 axis tolerance/IK iteration/geometry를 추가로 조정하지 않았다.
사진의 접힌 시작 자세는 이후 별도 HOME 검증 대상으로 유지한다. 현재 HOME 결과를 그대로 전용하지 않는다. 실제 hardware/OS30A/multi-seed teacher/ACT 실행 없음. 정책과 결과는 SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED다.

최종 전체 telemetry 대조: 모든 command가 ctrlrange 안, raw qpos/qvel finite, 모든 measured upper excess가 기존 tolerance 이내, ALIGN_HIGH/PREGRASP_NEAR의 hold command 일정, retreat trial 정확히 1개를 확인했다. 초기 SAFE_STAGE까지 upper excess가 전부 0이라는 더 강한 가정은 실제 기록과 달라 폐기했고, 위 phase별 수치로 바로잡았다.
추가 certificate 회귀: python -m unittest test_mesh_box_certificate test_box_box_certificate -q — 4 tests PASS / 4.586 s (task-open-distance-regression.log). 이번 관련 회귀는 총 38 tests PASS. git diff --check PASS.


# DAPIER-2026-09-17-approach-plan-execution-decomposition

## Problem
나는 PREGRASP_NEAR 0.8 s 종료의 measured approach 2.009502° 실패를 계획/실행 오차로 분리했다. Acceptance 2°, task-open reference, retreat, HOME, geometry, gains, timestep, trajectory timing을 우선 그대로 두었다. 기존 report와 fixture는 삭제하지 않았다.

## Evidence
ACTIVE_WORKTREE /home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch pro/dual-so101-codex-20260907; HEAD 534c4a6f25ab3eea6dd703526627e89dd9ba66d7, dirty 보존.
scene_id integration_desk; MuJoCo 3.3.7; Model SHA256 ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368. Exact asset/source hash는 approach-reserve-live.json provenance에 기록했다.

같은 desired world axis [0.1165806943, -0.0482893047, -0.9920065951]를 사용했다.
기존 planned axis [0.0909652770, -0.0719611056, -0.9932506822].
기존 executed axis [0.0902329577, -0.0713959751, -0.9933582577].
Planned는 IK q_target의 private FK, executed는 저장된 full raw qpos의 FK다. 실제 상태를 command로 대체하지 않았다.

| 기존 endpoint | Planned | Executed |
|---|---:|---:|
| approach error | 1.999758284° | 2.009501538° |
| position error | 0.098380 mm | 0.133659 mm |
| closing error | 8.772502° | 8.745122° |

계획 acceptance 여유는 0.000241716°뿐이었다. Planned→executed approach-error 증가는 0.009743254°지만 두 axis 사이 실제 angular deviation은 0.053357006°다. 두 값을 혼동하지 않는다.

마지막 100 ms (8.604→8.704 s, 51 samples):
- command/FK approach 1.999620574→1.999758284°.
- executed approach 2.009130844→2.009501538°.
- command/executed axis deviation 0.051359322→0.053357006°.
- 모든 actuator saturation false. 기존 trace의 saturation/force는 저장 qpos/qvel/ctrl를 mj_forward한 재계산 증거이며, 새 실행부터 실제 step actuator_force와 saturation도 직접 저장했다.
- shoulder_lift 추종오차는 0.000519898→0.000561894 rad, elbow_flex는 0.000299604→0.000346210 rad로 남았다. 단순히 시간 종료 순간의 큰 속도만으로 설명하지 않는다.

| Joint | 끝점 measured−command rad | 끝점 qvel rad/s |
|---|---:|---:|
| left_shoulder_pan | 0.00000185337 | -0.0000669438 |
| left_shoulder_lift | 0.00056189397 | 0.0002712649 |
| left_elbow_flex | 0.00034621017 | 0.0006053419 |
| left_wrist_flex | 0.00002315363 | -0.0005621360 |
| left_wrist_roll | 0.00000294451 | -0.0002344861 |

작은 ±1e-5 / ±5e-6 rad perturbation의 approach-error sensitivity가 일치했다: shoulder_pan 약 6.48053 deg/rad, shoulder_lift/elbow_flex/wrist_flex 각각 11.18112 deg/rad, wrist_roll 1.07086 deg/rad. Weighted combined Jacobian [Jp; 0.05*J_axis] rank 5, singular values [0.31173486, 0.24475220, 0.14888258, 0.01317364, 0.00092776], condition 약 336.0. 이 condition은 task scaling에 의존한다. Rank loss나 wrist limit saturation이 관찰되지 않았으며 wrist-flex upper margin은 0.143542 rad였다. 전역 singularity-free를 증명한 것은 아니다.

원인 분류는 B: 계획은 2° 이내지만 실행 추종 편차가 작은 계획 여유를 소진했다. C의 sensitivity도 진단했으나 rank loss/limit-active singularity가 주원인이라는 근거는 없다. 단순 iteration 부족이나 planned > 2°인 A로 분류하지 않는다.

## Decision
기존 ALIGN_HIGH 종료 full integration state로 PREGRASP를 복사 재생해 raw endpoint가 기존 실행과 정확히 일치함을 확인했다. 이어 기존 SETTLE의 100-step 길이인 0.2 s만 diagnostic hold했다. 이는 실제 task에 sleep/hold를 넣은 것이 아니다.
- endpoint 2.009501538°, max |qvel| 0.00060534.
- +0.1 s 2.009625301°, max |qvel| 8.4432e-6.
- +0.2 s 2.009628083°, max |qvel| 1.9521e-7.
속도는 줄어도 방향 오차는 해소되지 않아 실제 hold/duration 수정은 채택하지 않았다. 이 결과는 현재 controller의 지속 tracking bias와 부합한다.

마지막 100 ms의 관측된 최대 axis deviation 0.053357006°를 planner 목표에만 예약했다:
internal IK tolerance = 2° − 0.053357006° = 1.946642994°.
Acceptance는 계속 2°다. Weight 0.05, damping 0.02, max iterations 300을 유지했고 측정된 현재 q를 seed로 기존 axis_direction solver를 사용했다. 임의 roll/full-orientation 목표는 추가하지 않았다.

새 candidate는 10 iterations에 수렴했다. Full interpolated segment 및 기존 position/closing/joint gates를 검사한 뒤 PREGRASP 단일 segment를 copied full-state physics로 끝까지 실행한다. 이 preflight가 통과한 경우에만 live state에서 동일 controller/trajectory를 실행한다. 나머지 기존 waypoint와 guard는 유지한다. 관측 reserve는 증명된 일반 upper bound가 아니므로 후보마다 preflight를 생략할 수 없다.

## Validation
- test_approach_tracking, test_task_open, test_measured_state: 7 tests PASS / 36.043 s. Observed reserve, 2° acceptance 유지, candidate full-state physics, live state 비오염, gripper 계약/legacy boundary를 검증했다.
- 직접 force/axis telemetry 추가 후 test_task_open, test_measured_state: 6 tests PASS / 3.483 s (기존 검사의 재실행, 고유 test 수 합산 안 함).
- 실제 fresh 실행에서 staging full state와 기존 preflight가 bitwise equal.
- 새 PREGRASP candidate의 400 physics-step telemetry 전체 및 종료 integration state가 copied preflight와 bitwise equal, maximum state difference 0.
- qpos teleport는 실제 실행에서 사용하지 않았다. RESET/격리 FK/화면 snapshot만 별도이며 controller는 ctrl + mj_step 경로 그대로다.

## Result
| PREGRASP endpoint | 기존 Planned | 기존 Executed | 새 Planned | 새 Executed |
|---|---:|---:|---:|---:|
| approach error deg | 1.999758 | 2.009502 FAIL | 1.945875 | 1.947677 PASS |
| position error mm | 0.098380 | 0.133659 | 0.043158 | 0.186613 |
| closing error deg | 8.772502 | 8.745122 | 10.055035 | 10.029209 |

새 executed position/closing 오차는 기존 수치보다 커졌지만 원래 0.5 mm / 15° acceptance를 모두 만족한다. 모든 오차가 개선됐다고 표현하지 않는다.
새 planned axis [0.0953662000, -0.0748008229, -0.9926278884].
새 executed axis [0.0946269428, -0.0742302121, -0.9927414655].
새 axis deviation은 0.053900616°로 과거 관측 reserve보다 약간 크다. 따라서 reserve 자체를 guaranteed bound로 주장하지 않으며 실제 preflight PASS로 실행을 승인했다. 새 Jacobian rank 5 / condition 337.174로 conditioning 개선은 주장하지 않는다. Wrist-flex upper margin은 0.154207 rad로 증가했다.

| 실제 phase | 결과 | SIM time | position error mm | approach error deg |
|---|---|---:|---:|---:|
| HOME / RESET / SETTLE | PASS | 0.200 | — | — |
| SAFE_STAGE | PASS | 3.064 | 0.253628 | 정렬 비요구 |
| ALIGN_HIGH | PASS | 7.904 | 0.190450 | 1.998109 |
| PREGRASP_NEAR | PASS | 8.704 | 0.186613 | 1.947677 |
| APPROACH_COARSE | PASS | 9.874 | 0.152920 | 1.969632 |
| APPROACH_FINE | PASS | 10.908 | 0.149982 | 1.967136 |
| CLOSE 첫 단계 | PASS | 11.312 | 0.355308 | 1.980111 |
| CLOSE 두 번째 단계 | FAIL | 11.716 | 0.567060 > 0.5 | 1.994438 |

새 첫 blocker는 CLOSE measured waypoint position tolerance다. Acceptance나 geometry를 바꾸지 않고 정지했다.
실패 시 general clearance 48.466535 mm, pad1-table 3.800602 mm, pad2-table 9.109806 mm, housing-block 11.532083 mm, pad1-block 2.253101 mm, pad2-block 0.495536 mm. 승인된 phase-specific pair 정책하에 모두 PASS이며 finger/block contact는 아직 없다. 양 finger normal force 0 N. Block의 table 지지는 유지되고 attachment는 없다. GRASP_CONFIRM/LIFT/HOLD 미실행, Center SUCCESS false.

전체 q vector, 양쪽 joint margin, 세 axis, 마지막 100 ms tracking/qvel/force/saturation은 아래 원본 증거에 저장했다.
- approach-endpoint-decomposition.json: 기존 planned/executed 및 finite-difference sensitivity.
- approach-terminal-hold-diagnostic.json: task 실행과 분리된 hold 진단.
- approach-reserve-endpoint-evidence.json: 새 planned/executed full qpos 및 joint margin, 직접 기록된 100 ms telemetry.
- approach-reserve-live.json / .log: 실제 phase/target/measured/contact/clearance 및 preflight 비교.
모두 기존 KIT local-validation/integration-task-20260916-aeg3vtmi 아래에 있으며 별도 Markdown 원장을 만들지 않는다.

## Lesson / Next
계획상 acceptance 경계에 거의 붙은 해는 작은 지속 tracking bias만으로 실제 gate를 넘을 수 있었다. 임의 sleep, controller gain, acceptance 완화 대신 관측된 execution deviation으로 내부 IK 목표를 정하고 full-state preflight로 후보를 검증했다. 현재 조건의 성공을 강건성 증거로 확대하지 않는다.
다음 조사점은 CLOSE의 arm hold reference다. 현재 finish_task가 매 closing 단계에서 measured arm q를 target으로 복사하며, 로그에서도 shoulder/elbow target이 단계마다 이동한다. 첫·둘째 CLOSE의 position error가 0.355→0.567 mm로 증가했고 finger force는 0이므로 contact force 때문이라고 단정할 수 없다. 이 호출 경로와 누적 tracking bias를 다음 단계에서 분리해야 하며, 이번에는 새 blocker에서 멈춰 추가 수정하지 않았다.

Scope: SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED. 기존 task-open 2.2025746379457667 rad, retreat 10.110785931572185 mm, HOME, geometry, controller, timing, 30 mm 및 모든 acceptance 유지. Staging 30.378586 mm는 여전히 현재 조건의 약 0.378 mm 여유이며 다중 초기조건/실물 강건성이 아니다.
실제 hardware/OS30A/ACT 실행 없음. 현재 단일 viewer를 실제 CLOSE 실패 상태에서 physics 정지한 채 유지한다. 기존 helper에 --approach-reference task-open-live.json을 추가하여 재현할 수 있게 했다.

# DAPIER-2026-09-17-close-arm-reference

## Problem
APPROACH_FINE 실제 physics PASS 이후 CLOSE 두 번째 단계 TCP 오차가 0.567060 mm로 0.5 mm gate를 넘었다. 기존 통과 경로, controller/timing 및 모든 수치 기준을 유지하고 command 생성 경로를 확인했다.

## Evidence
저장된 approach-reserve-live.json을 같은 compiled model로 FK했다. APPROACH_FINE planned/measured TCP 오차는 0.098970/0.149982 mm, CLOSE 1은 0.149982/0.355308 mm, CLOSE 2는 0.355308/0.567060 mm다. 각 CLOSE target의 양팔 arm 채널은 직전 measured 채널과 bitwise 동일하다. 두 번째 단계 command FK 자체는 0.5 mm 안이지만 measured는 밖이다. 따라서 C(reference reseeding)가 누적 drift를 만들고 마지막 endpoint는 B(dynamic tracking) 형태로 실패한 것이다. 접촉 전 force 0 N인 기존 증거를 보존했다. 원본 qpos/qvel, command와 FK는 close-reference-baseline.json 및 test/fixtures/close_reference.json에 남겼다.

## Decision
접촉 전 CLOSE의 arm reference를 APPROACH_FINE terminal ctrl로 고정한다. 실제 measured q는 collision interpolation 시작점과 telemetry에 그대로 사용한다. Controller의 trajectory 시작 arm 채널도 이 explicit reference를 사용해 매 단계 reference가 measured 오차를 따라가지 않게 한다. 첫 접촉이 관측된 뒤 별도 contact 제어를 새로 추가하지 않는다. 양 finger contact 확인과 기존 GRASP_CONFIRM/LIFT/HOLD 조건은 그대로다.
기존 full_state_preflight를 재사용하여 각 CLOSE substage를 동일 duration/controller로 복사 상태에서 먼저 실행하고, PASS인 substage만 live 실행한다. Full integration state 비교로 일치를 확인한다. Bound, numerical tolerance, acceptance, geometry 및 retreat는 변경하지 않는다.

## Validation
기본 CLOSE reference / measured-state / task-open 회귀 7 tests PASS (6.145 s). 추가 raw-state/path 및 원래 실패 fixture 회귀와 단일 viewer 실제 실행 결과는 아래에 이어 기록한다.

## Result
단일 viewer 실제 재실행은 HOME→RESET→SETTLE→SAFE_STAGE→ALIGN_HIGH→PREGRASP_NEAR→APPROACH_COARSE→APPROACH_FINE를 모두 통과했다. Staging full-state preflight/live bitwise 동일, general minimum 30.378585882 mm다. 이전 30.378087 mm 및 현재 약 0.378 mm 여유는 현재 조건의 결과일 뿐 다중 초기조건·실물 강건성 근거가 아니다.

| CLOSE 단계 | planned TCP mm | measured TCP mm | APPROACH_FINE 대비 measured drift mm | arm reference drift | 실제 실행 |
|---|---:|---:|---:|---:|---|
| 1 | 0.098970 | 0.154294 | 0.004583 | 0 rad | PASS |
| 2 | 0.098970 | 0.154294 | 0.004583 | 0 rad | PASS |
| 3–8 | 0.098970 | 0.154294 | 0.004583 | 0 rad | PASS |
| 9 (첫 한쪽 접촉 발생) | 0.098970 | 0.151377 | 0.002463 | 0 rad | PASS |
| 10 (기존 접촉 후 경로) | 0.151377 | 0.352325 | 0.210269 | 0.000570890 rad | PASS |
| 11 (copied-state만 실행) | 0.352325 | 0.559745 | 0.420032 | 0.001141639 rad | FAIL, live 차단 |

접촉 전 1–8단계 approach 약 1.967343°, closing 약 11.328565°, 양 finger force 0 N, endpoint general clearance 약 48.708098 mm. 최대 arm tracking error는 약 0.000576053 rad로 남았지만 이를 command로 재설정하지 않아 TCP 오차가 누적되지 않았다. 기존 2단계 actual 0.567060 mm가 0.154294 mm로 감소했다. Raw measured는 변경하지 않았다.

첫 실제 접촉은 SIM 14.220 s의 jaw_2 force 0.003551 N이다. 9단계 끝 14.544 s에서는 jaw_1=0 N, jaw_2=0.040491 N. 이번 reference 분리는 접촉 전 범위이므로 다음 substage부터 기존 post-contact measured-target 생성 동작을 보존했다. 그 결과 10단계 target가 measured 방향으로 다시 이동하고, 11단계 copied-state endpoint에서 0.559745 mm > 0.5 mm로 실패했다. 이 실패를 contact force만의 탓이라고 결론내리지 않는다. 접촉 이후에도 reference reseeding이 관측되며 별도 분석할 다음 blocker다.

실제 task는 10단계 종료 SIM 14.948 s에서 정지·보존됐다. 실제 TCP error 0.352325 mm, jaw_1=0 N / jaw_2=0.041170 N, general clearance 48.588615 mm. 실패한 복사 상태는 SIM 15.352 s, approach 1.993503°, closing 11.109836°, force 0 / 0.041097 N, general clearance 48.467797 mm다. 실제로 실행된 10개 CLOSE substage는 모두 copied preflight와 full integration state가 bitwise 동일했다. Live state는 실패한 preflight로 오염되지 않았다.

CENTER SUCCESS=false. 양쪽 contact 미성립, GRASP_CONFIRM/LIFT/HOLD 미실행, HOLD 0 s. 약 0.002023 mm의 수치상 maximum bottom 변화는 lift 성공이 아니다. Attachment/weld/mocap support 추가 없음.

새 증거는 기존 KIT local-validation/integration-task-20260916-aeg3vtmi의 close-reference-live.json/.log 및 close-reference-summary.json이다. 단계별 desired/command/measured TCP·axis, q/qvel·joint margin, tracking, gripper/jaw, pair clearance와 force를 기록했다. test/fixtures/close_physics.json은 APPROACH_FINE 종료 full integration state와 접촉 없는 CLOSE 1·2단계 regression을 보존한다. 기존 실패 fixture와 이전 기록은 유지했다.

## Lesson / Next
Measured state는 물리 상태와 안전 검사의 근거다. 이를 매 substage의 hold command로 복사하면 작은 지속 tracking bias가 목표 자체의 이동으로 누적된다. 이번에는 접촉 전 gripper-only 구간의 reference만 분리했고, 접촉 후 제어는 변경하지 않았다. 다음 단계는 한쪽 접촉 이후의 명시적 arm reference와 contact 응답을 따로 분해하는 것이다. 이번 결과를 양쪽 grasp나 lift 성공으로 기록하지 않는다.

ACTIVE_WORKTREE=/home/dapier-jhj/DAPIER/.local-workspaces/pro/dual-so101-codex-20260907
branch=pro/dual-so101-codex-20260907; SHA=534c4a6f25ab3eea6dd703526627e89dd9ba66d7; 기존 dirty 상태 보존.
scene_id=integration_desk; model_sha256=ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368.
Scope=SIM_ONLY / INTEGRATION_DESK / HARDWARE_UNVERIFIED. Hardware/OS30A/multi-seed/ACT 미실행.
기존 run-waypoint-viewer.sh로 동일 경로 재실행 가능하며, 현재 단일 viewer는 실제 14.948 s 정지 상태를 유지한다.

### Validation 완료
- `python -m unittest test_close_reference test_collision_guard test_near_support test_manipulation_policy test_dynamic_preflight test_measured_state test_task_open test_box_box_certificate test_mesh_box_certificate`: 42 tests PASS / 770.817 s. close-reference-final-regression.log.
- `python -m unittest test_center_block_teacher`: 2 tests PASS / 4.854 s. Legacy settled-input 및 path 거부 시 physics 미실행 보존. close-reference-legacy-teacher-regression.log.
- 새 CLOSE full-state 회귀는 실제 APPROACH_FINE 종료 상태에서 1·2단계를 copied/live로 실행해 bitwise 일치, live state 미오염, raw state 보존, reference drift 0, 양 finger force 0 및 success=false를 확인했다.
- `git diff --check` PASS. 기존 dirty 변경/fixtures 보존. 이번 수정 후 최종 회귀 44 tests 모두 PASS이며, 남은 실패는 위에 기록한 contact 이후 CLOSE 11단계 copied-state task gate다.
- Python interpreter: /home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python. 기존 DAPIER_SO101_MJCF/PYTHONPATH 설정 재사용, 의존성 변경 없음.
- Source 수정: center_block_teacher.py, dynamic_preflight.py, waypoint_block_teacher.py. 추가 회귀: test_close_reference.py 및 close_reference.json/close_physics.json. KIT에는 local-validation 증거만 추가, package/config/model 변경 없음.

# DAPIER-2026-09-17-post-contact-reference

## Problem
CLOSE 10 실제 PASS / 14.948 s 상태를 보존하고, CLOSE 11 copied-state의 0.559745 mm 위치 실패가 reference reseeding인지 contact dynamics인지 분리했다. Acceptance 0.5 mm / 2° / 15°, controller, timing, geometry, task-open, retreat, joint/state limits 및 safety policy는 유지한다.

## Evidence
CLOSE 9 종료 full integration state(14.544 s)를 두 정책의 공통 출발점으로 사용했다. Baseline의 CLOSE 10/11 arm command는 직전 measured arm q와 bitwise 동일했고, 재실행 final integration state도 기존 로그와 bitwise 동일했다. Planned/measured TCP 오차는 10단계 0.151377/0.352325 mm, 11단계 0.352325/0.559745 mm다.
Fixed explicit reference는 10–16단계를 통과했다. Arm reference drift 0 rad, 최대 measured TCP error 0.154260 mm. 반대 jaw_1 gap은 공통 시작 약 1.186 mm에서 15단계 0.092046 mm로 줄었고, 16단계 SIM 17.184 s에 bilateral contact가 처음 형성됐다. 16단계 끝 force는 jaw_1=0.097001 N, jaw_2=0.113431 N이다.
Block은 최초 접촉 대비 약 [-0.651825, +0.110135, +0.003552] mm 이동했다. 마지막 yaw는 -0.7779° 부근이며, 보호 대상 비의도 접촉은 0건이다. 진단 전체 maximum contact depth는 baseline 0.013758 mm / fixed 0.003415 mm로 기존 penetration gate 안이다. Contact position/normal/depth/force, world linear/angular velocity, closing error, raw qpos/qvel, command source와 clearance를 매 step 저장했다.
첫 진단 시 validated-reset Python 상태를 초기화하지 않아 physics 전에 거부됐다. 기존 reset을 정상 호출한 뒤 full-state를 복원하여 재실행했고, 이 초기화 오류를 실제 contact 실패로 해석하지 않았다. 첫 로그도 삭제하지 않았다.

## Decision
분류 A: 현재 조건에서 reference reseeding이 주 blocker다. Measured contact response를 다음 command로 자동 채택하지 않고 APPROACH_FINE explicit arm reference를 CLOSE 전체에 유지한다. Actual physics/collision start와 telemetry는 raw measured state다. Gripper schedule만 기존 방식으로 바뀐다. 새 compliance 제어, gate 완화, model/threshold 변경을 하지 않는다.
기존 full_state_preflight 및 controller를 재사용하고, 진단에서는 실패 gate나 bilateral 접촉 종료점까지만 실행한다. 실패 이후 강제로 physics를 진행하지 않는다. 이 결과는 DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS이며 실제 task PASS와 구분한다.

## Validation
새 post-contact fixture에서 unilateral→bilateral 물리 접촉, reference drift 0, full-state copy 미오염, 기존 pose/clearance/penetration gate를 검사했다.
`python -m unittest test_post_contact_close test_close_reference test_measured_state test_manipulation_policy test_near_support`: 20 tests PASS / 266.030 s.
기존 14.948 s live state와 원본 close-reference-live.json을 보존한 뒤 새 후보를 HOME부터 실제 재실행했다. 확정 결과는 아래와 같다.

## Result / Lesson / Next
복사 A/B는 contact가 생겼다는 사실만으로 measured arm q를 hold target으로 다시 삼을 근거가 되지 않음을 보였다. 물리 접촉은 유지하면서 explicit reference를 고정하니 block이 반대 finger 쪽으로 이동해 양쪽 접촉을 만들었다. 이는 SIM center 조건의 증거이며 실물·다중 초기조건 검증이 아니다.
실제 재실행은 동일 controller/scene으로 첫 새 blocker 또는 CENTER SUCCESS에서 종료한다. Hardware/OS30A/multi-seed/ACT는 실행하지 않는다.


### 실제 Result
HOME→RESET→SETTLE→SAFE_STAGE→ALIGN_HIGH→PREGRASP_NEAR→APPROACH_COARSE→APPROACH_FINE→CLOSE 16단계→GRASP_CONFIRM 실제 physics PASS. CLOSE 16개 모두 copied preflight/live full integration state가 bitwise 동일했다.
CLOSE 끝 SIM 17.372 s: planned TCP 0.098970 mm / measured 0.154260 mm, finger force 0.097001/0.113431 N, general clearance 48.707895 mm.
GRASP_CONFIRM 끝 SIM 17.472 s: measured TCP 0.154982 mm, 양 force 0.070893/0.072187 N 유지.
LIFT_5MM 첫 physics step인 SIM 17.474 s에서 양 finger force가 0 N이 되어 기존 bilateral-contact gate가 거부했다. 실패 이유는 `bilateral finger contact lost during lift/hold`다. General clearance는 48.701514 mm였다. 5 mm lift 목표까지 아직 이동하지 않은 첫 step의 position error를 endpoint 실패로 해석하지 않는다.
최종 phase=FAILURE at LIFT_5MM. CENTER SUCCESS=false, HOLD=0 s. Viewer를 실제 17.474 s 상태에서 정지·보존했다. 다음 물리 blocker는 CLOSE hold에서 LIFT로 전환할 때 command continuity와 contact response를 분리하는 것이다. 이번 턴에서는 이를 해결하려고 추가 threshold/control 변경을 하지 않았다.
원본 증거: post-contact-diagnostic-v2.json/.log, post-contact-regression.log, post-contact-live.json/.log (기존 KIT local-validation). 실행 시작 provenance는 534c4a6 dirty이며 실행한 코드 내용은 후속 source commit 5fbb836에 보존했다. Model SHA ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368 유지.

### GitHub handoff와 CI 제한
Source 및 필요한 기존 검증 SIM 의존성을 commit 5fbb836으로 normal push하고 main 대상 PR https://github.com/Alpenj/DAPIER/pull/62 를 생성했다. 별도의 REAL_SCENE_GEOMETRY_AUDIT.md는 기존 untracked 상태로 유지했고 KIT raw 로그는 커밋하지 않았다.
Hardware-free CI 2개는 PASS. MuJoCo CI https://github.com/Alpenj/DAPIER/actions/runs/35181539006 는 340 tests / 486.592 s, failures=3 / errors=15로 FAIL했다. 시간 초과가 아니다.
- 일부 새 테스트의 standalone discovery import 경로 누락.
- viewer 문자열 assertion이 이전 문구 NOT IK preview를 요구함.
- CI compiled model SHA 228e9754026cbbe769296798938bc593ba8cc7ac67e18497b9be738ea034716c 와 로컬 fixture SHA 불일치.
- manipulation compiled geometry identity gate도 CI에서 거부. 단순 플랫폼 차이인지 실제 geometry 차이인지는 미확정.
- legacy sim_episode recording 3개가 protected clearance gate에서 거부.

따라서 regression-clean 병합 조건을 충족하지 않으며 PR을 draft/open으로 보존한다. Hash를 CI 값으로 바꾸거나 geometry gate를 무력화하지 않는다. Main에 병합하지 않았다. 확인한 main SHA는 d75aa89b690d721065cc1ca55f0c170455fc14ba이며 이 턴의 merge SHA가 아니다.
다음 통합 blocker는 CI/local compiled geometry 및 legacy fixture 재현성 조사다. Local 20 PASS와 실제 center progress를 전체 CI PASS로 확대하지 않는다. `git diff --check`는 PASS했다.
Notion 동기화는 기존 비공개 DAPIER 학습 원장에 PR/미병합 상태, 실제 LIFT 첫 실패 및 이 CI 제한을 추가한다. 비공개 URL/ID는 저장소에 기록하지 않는다.

## 2026-09-17 — LIFT transition copied diagnostic / PR #62 CI triage

record_id: DAPIER-2026-09-17-lift-transition-ci

### Problem

나는 실제 GRASP_CONFIRM PASS 뒤 LIFT_5MM 첫 step에서 양 finger force가 0이 된
17.474 s failure state를 보존하고, copied full-state 진단과 전체 CI 18건을 분리해 조사했다.
ACTIVE_WORKTREE는 기존 pro/dual-so101-codex-20260907, 시작 HEAD ffa3f2ce다.
별도 REAL_SCENE_GEOMETRY_AUDIT.md는 수정/추적하지 않았다.

### Evidence

**SIM copied diagnostic / NOT LIVE TASK SUCCESS.** 실제 CLOSE 마지막 integration state에서
50-step GRASP_CONFIRM을 복원했다. 실제 확인 종료 qpos와 첫 LIFT qpos 차이는 모두 0이다.
A=마지막 CLOSE, B=확인 종료, C=첫 LIFT 직후의 full integration state를 보존했다.
50 step / 0.100 s 비교 결과:

| case | 최초 contact loss | 50-step 결과 | 첫 command 최대 delta |
| --- | --- | --- | --- |
| terminal command HOLD | 없음 | bilateral 유지 | 0 rad |
| 기존 LIFT | 1 | 50 step 모두 bilateral gate 미충족 | 0.000578399857 rad |
| 직전 command에서 연속 시작한 진단 LIFT | 36 | 11 step bilateral gate 미충족 | 1.58098e-10 rad |

CLOSE terminal과 GRASP_CONFIRM command는 정확히 같다. LIFT의 gripper command도
2.043112220718924 rad로 변하지 않았다. 팔은 measured q에서 trajectory를 시작하므로
첫 command가 직전 reference와 달라졌다. 새 IK target으로 즉시 점프하는 방식은 아니지만
trajectory 시작 reference에 불연속이 있다. Diagnostic 연속화 후보는 이 즉시 손실만 제거했다.

확인 종료 jaw normal force는 0.070893238 / 0.072187220 N, table 접촉 4개와
normal force 0.195740710 N이다. block center - TCP =
[-0.644197, 0.055773, 0.145552] mm, block yaw=-0.014286132 rad.
따라서 양쪽 positive force만으로 table 없이 운반할 안정 grasp가 입증된 것은 아니다.
HOLD 마지막 force도 0.047872245 / 0.047301963 N까지 감소했다.

매 step에 command/previous command/raw measured qpos/qvel/FK, contact frame의
normal·tangential wrench, contact 위치·normal·depth, block pose·velocity·table contact,
lift와 gate 결과를 저장한다. 실행 순서는 ctrl 설정 → mj_step → integrated-state
mj_forward → geometry/state gate → contact 관측 → teacher contact gate다.
현재 gate가 이전 state의 force를 읽는 D의 증거는 없다.
[mj_forward / mj_contactForce 공식 API](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html)
의 계산 시점 및 contact-frame force 의미와 대조했다.

**CI 340 tests / 3 failures / 15 errors**:
[원본 run](https://github.com/Alpenj/DAPIER/actions/runs/35181539006).
각 traceback의 마지막 프로젝트 지점과 분류는 다음과 같다.

| test | kind / source | 분류·원인 |
| --- | --- | --- |
| test_approach_tracking (import) | error, test_approach_tracking.py:6 | B: standalone discover parent import 누락 |
| test_axis_direction (import) | error, test_axis_direction.py:4 | B: 위와 같음 |
| test_box_box_certificate (import) | error, test_box_box_certificate.py:7 | B: 위와 같음 |
| CloseReferenceTest.test_close_path_uses_raw_state_trajectory_uses_reference | error, collision_guard.py:604 | C: ancestor joint range identity 불일치 |
| DynamicPreflightTest.setUpClass | error, test_dynamic_preflight.py:15 | C: 원본 MJB hash 불일치 |
| ManipulationPolicyTest.test_finger_contact_phase_and_depth | error, collision_guard.py:604 | C: geometry identity 거부 |
| ManipulationPolicyTest.test_geometry_and_positive_near | error, collision_guard.py:604 | C: 위와 같음 |
| ManipulationPolicyTest.test_saved_tilted_approach_full_segment | error, collision_guard.py:604 | C: 위와 같음 |
| ManipulationPolicyTest.test_support_and_housing_contact_rejected | error, collision_guard.py:604 | C: 위와 같음 |
| ManipulationPolicyTest.test_unrelated_arm_table_still_requires_30mm_in_approach | error, collision_guard.py:604 | C: 위와 같음 |
| ManipulationPolicyTest.test_wrong_phase_and_general_pair | error, collision_guard.py:604 | C: 위와 같음 |
| StagingChainTest.setUpClass | error, collision_guard.py:604 | C: 위와 같음 |
| SimEpisodeTest.test_interrupted_recording_can_retry_same_output | error, shoe_task.py:830 | E/A: valid plane gate가 기존 zero-pose 침범을 노출 |
| SimEpisodeTest.test_records_four_synchronized_cameras_and_12_axis_executed_action | error, shoe_task.py:830 | E/A: 위와 같음 |
| SimEpisodeTest.test_terminal_success_without_prior_carry_is_not_accepted | error, shoe_task.py:830 | E/A: 위와 같음 |
| CloseReferenceTest.test_full_state_preflight_matches_live_without_fake_contact | failure, test_close_reference.py:69 | C: 원본 MJB hash 불일치 |
| IntegrationTaskTest.test_viewer_observes_same_physics_state_as_headless | failure, test_integration_task.py:55 | D: 옛 viewer 문자열 assertion |
| PostContactCloseTest.test_fixed_reference_forms_physical_bilateral_contact | failure, test_post_contact_close.py:13 | C: 원본 MJB hash 불일치 |

main d75aa89의 별도 clean /tmp worktree에서 같은 interpreter와 pinned asset으로
test_sim_episode.py 6개 PASS를 확인했다. 따라서 recorder 3건을 기존 main failure(F)라고
부르지 않는다. PR에서 정확해진 plane guard가 드러낸 invalid initialization이며,
synthetic recorder만 기존 valid HOME을 쓰도록 수정했다. 전역 reset 기본값은 보존했다.

### Decision

물리 최초 blocker는 B(phase-transition command discontinuity)로 분류한다.
연속 command 진단에서도 36 step부터 접촉 손실이 있으므로 grasp/contact response 문제가
추가로 남는다. 이 후보는 copied gate를 완전히 통과하지 못했다. 따라서 runtime LIFT,
controller, timing, grip force, friction, acceptance, live state를 변경하지 않았다.
접촉 손실 뒤의 copied 관측은 bounded diagnostic일 뿐 task PASS가 아니다.
그 외 geometry/contact penetration/finite-state gate는 계속 적용했다.

CI canonical XML은 pinned SO-ARM100 7629d2ad9853d10fb903093a33ef6114099d97e5,
SHA d75253eb568e8a7214db9c631ab7bed4217f608a26f7276ebe9a7636cac82580이다.
기존 live XML SHA는 78f7f43fceece8303dc60e58d831d5a6e5114847ad659c6cb5e37bf288db8703이다.
13개 mesh는 byte-identical이며 XML diff는 shoulder_lift joint 하한(-100°/-110°)과
actuator 하한 두 줄뿐이다. 이를 숨기지 않고 integration_desk_source.json에 기존
desk SIM source contract를 명시한다. Canonical 입력에서는 약 10° 확대되지만,
현재 live 모델의 범위를 새로 넓히는 변경이 아니라 기존 모델 재현이다.
stock/shared tabletop/mobile source와 hardware limit에는 적용하지 않는다.

기존 local compiled MJB ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368
및 fixture state·pair identity는 보존한다. MJB의 절대 asset path 저장 때문에
raw binary hash는 실행 위치에 종속된다. 별도 portable compiled identity는
path storage/offset/allocation size만 제외하고 모든 공개 compiled 숫자·배열·bytes·문자열,
opt/stat/vis와 MuJoCo 버전/schema를 포함한다. 알 수 없는 필드는 fail-closed한다.
원본 MJB hash를 새 hash로 덮어쓰지 않는다.

### Validation

- 기존 local source: LIFT transition full-state regression 1 PASS / 29.271 s.
- synthetic recorder 관련 6 PASS / 7.583 s.
- clean main / pinned source: 같은 recorder 6 PASS / 7.066 s.
- joint range, mesh vertex, actuator gain, friction, body/geom, timestep mutation은 identity mismatch여야 한다.
- 전체 headless 및 최종 GitHub 결과는 아래 완료 기록에 추가한다.

### Result

현재 실제 최종 phase는 이전과 같은 LIFT_5MM failure / SIM 17.474 s이며,
GRASP_CONFIRM까지만 실제 PASS다. CENTER SUCCESS false, HOLD 0.
기존 live viewer/state는 보존했고 새 live 실행은 하지 않았다.

### Lesson / Next

양쪽 finger force > 0과 안정적인 운반 grasp는 다르다. 먼저 command 연속성과
table 지지를 받는 grasp의 contact response를 분리해야 한다.
다음 물리 blocker는 연속 command 후보에서 step 36부터 나타나는 contact loss다.
30.378087 mm staging 통과는 현재 center 조건의 약 0.378 mm 여유이며,
다중 초기조건·실물 강건성 증거가 아니다. Hardware/OS30A/ACT/multi-seed는 실행하지 않았다.

#### Compile-order root cause and verification

추가 source 회귀에서 stock 선행 compile → desk compile 순서가 nmeshpoly를
255573에서 201563으로 바꾸는 현상을 찾았다. mesh_poly* 배열을 fingerprint에서
제외하지 않았다. MuJoCo 3.3.7 cached mesh 로딩은 이전 collision hull의 polygon을
visual-only mesh에도 복사할 수 있다.
[공식 LoadCachedMesh / MakePolygons 구현](https://github.com/google-deepmind/mujoco/blob/3.3.7/src/user/user_mesh.cc).
desk build 때 native asset cache만 clear하면 기존 255573과 기존 모델 identity가 복원된다.
현재 호출자는 build_scene(...).compile()을 즉시 직렬 실행한다. 향후 지연/동시 compile
호출자가 생기면 cache 정리와 compile을 함께 묶어야 하며 지금 병렬 compile을 보장하지 않는다.
이것은 cache contents 정리이며 CCD/solver/timestep 설정 변경이 아니다.

- 기존 local 원본 raw MJB: ed4977e7b9b35f9c0fba6d1f91c56ca75238ec71720cdcaf5367486b77221368 (불변).
- 별도 /tmp canonical source의 raw MJB: b63af5b41a2ed004a5f2b80628d511b4be900e0b3e64941320bbd5a89be1ff79.
- 두 입력의 portable identity: b6dafd26e8e6bc34e9e5ecced05e2bb500b36c779a21f2e132efdc32f91811a4 (동일).
- source/profile/relocated-path/physics-mutation 음성 회귀: **3 PASS / 10.397 s**.
- source fixture는 원래 binary hash와 raw state를 보존하고 portable identity만 추가했다.
- 로컬 전체 검증 환경: Python 3.12.3, MuJoCo 3.3.7, NumPy 2.2.6, Pillow 12.3.0.
  CI Python patch version 차이는 원격 CI로 별도 확인한다. 의존성은 바꾸지 않았다.

#### 완료 검증 / handoff

- Repository command `scripts/verify-mujoco-headless --artifact-dir <temporary-output>`
  를 CI pinned asset과 기존 venv에서 실행: **exit 0**.
- Research **33 PASS / 0.019 s**, MuJoCo **353 PASS / 1095.946 s**.
  Canonical multi-view render와 vision artifact 생성도 완료했다.
- 추가한 inner-face midpoint / contact friction telemetry를 포함한 최종 LIFT 회귀:
  **1 PASS / 22.396 s**. 입력·controller·물리 궤적은 같고 metadata만 추가했다.
- Confirm 시 실제 inner-face closing center:
  [0.1999906731, 0.0000530874, 0.0198522245] m.
  Block center - closing center = [-0.642194, 0.055428, 0.145758] mm.
  Normal-force 차이 jaw_1 - jaw_2 = -0.001293982 N.
- 원본 first LIFT는 양쪽 geometry contact entry도 없다. 연속 command candidate의
  first loss(step 36)는 jaw_1 contact가 없고 jaw_2는 남는다. Stale force 읽기의 증거가 아니다.
- 원래 live failure viewer/process는 유지했다. 새 live task를 실행하지 않았다.
- Source 수정은 f2dc027 커밋으로 normal push했고 PR #62(base=main)를 갱신했다.
  최종 read-only telemetry와 이 handoff도 같은 writer 브랜치로 올린다.
- 원격 전체 CI는 이 기록 작성 시 진행 중이다. 최신 head checks가 모두 PASS인 경우에만
  draft를 해제하고 merge한다. 최종 CI/merge SHA는 PR #62 및 비공개 학습 기록에 연결한다.
- 기존 unrelated REAL_SCENE_GEOMETRY_AUDIT.md는 untracked로 보존한다.

### 최종 원격 CI / handoff 정정 (2026-09-17)

- **Problem:** 검증 head db60fd6f6ba04b9767d531a90d81daf14471c14a의 원격 run https://github.com/Alpenj/DAPIER/actions/runs/35196873279 은 **346 tests / 625.486 s, 5 failures / 10 errors**로 실패했다.
- **Evidence:** MuJoCo 3.3.7 / NumPy 2.2.6 / Pillow 12.3.0은 local과 동일. Python CI 3.12.14 / local 3.12.3. CI portable hash 7bd67b162cc6928afe820db830e108f0c5614bc9be832a0d024858397dab1778와 local b6dafd26e8e6bc34e9e5ecced05e2bb500b36c779a21f2e132efdc32f91811a4가 다르다. BOX–BOX fixture native distance는 CI +0.19581108263492744 m / local 0이다.
- **Decision:** hash gate / native 재현 assertion을 삭제하거나 fixture hash를 교체하지 않는다. 원격 compiled field 차이의 근본 원인은 미확정이다. PR #62 draft / not safe to merge 유지, main merge 없음. 로컬 source/path/cache 수정만으로 원격 문제가 모두 해결됐다는 결론을 내리지 않는다.
- **Validation:** local research 33 + MuJoCo 353 PASS, 최종 telemetry regression 1 PASS. 원격 import 3건 / stale viewer / recorder 3건은 통과했지만 잔여 실패는 아래 표처럼 분리된다. 전체 remote FAIL을 local PASS로 대체하지 않는다.
- **Result:** live 재실행 없음. 실제 마지막 PASS GRASP_CONFIRM; LIFT_5MM FAIL / SIM 17.474 s. Handoff 확인 시 viewer PID 163390이 종료됐고 기존 실행 세션 exit 0을 확인했다. 종료 원인은 미확정이며 화면 유지 성공으로 보고하지 않는다. 저장 full-state / diagnostic JSON은 보존됐다.
- **Lesson / Next:** CI compiled geom/body/joint/mesh/options 및 asset hash를 field별로 local과 비교해야 한다. 아직 platform rounding으로 단정하지 않는다. 물리 다음 blocker는 command-continuous copied LIFT step 36 contact loss.

최신 원격 실패 test/traceback 직접 근거:

| Test | 근거 / 분류 |
|---|---|
| test_approach_tracking.test_observed_reserve_and_copied_endpoint | portable hash mismatch / C |
| test_close_reference.test_full_state_preflight_matches_live_without_fake_contact | portable hash mismatch / C |
| test_integration_source.test_approved_desk_contract_and_stock_source_preserved | portable hash mismatch / C |
| test_post_contact_close.test_fixed_reference_forms_physical_bilateral_contact | portable hash mismatch / C |
| test_box_box_certificate.test_saved_failure | native +0.19581108263492744 != 0 / 재현 차이, 근본 원인 미확정 |
| test_close_reference.test_close_path_uses_raw_state_trajectory_uses_reference | manipulation geometry changed / C |
| test_dynamic_preflight.setUpClass | dynamic fixture model changed / C |
| test_lift_transition.test_actual_failure_and_bounded_contact_diagnostics | diagnostic model differs from actual failure model / C |
| test_manipulation_policy.test_finger_contact_phase_and_depth | manipulation geometry changed / C |
| test_manipulation_policy.test_geometry_and_positive_near | manipulation geometry changed / C |
| test_manipulation_policy.test_saved_tilted_approach_full_segment | manipulation geometry changed / C |
| test_manipulation_policy.test_support_and_housing_contact_rejected | manipulation geometry changed / C |
| test_manipulation_policy.test_unrelated_arm_table_still_requires_30mm_in_approach | manipulation geometry changed / C |
| test_manipulation_policy.test_wrong_phase_and_general_pair | manipulation geometry changed / C |
| test_staging_chain.setUpClass | manipulation geometry changed / C |


## 2026-09-17 · 병렬 read-only 조사: LIFT 파지 안정성과 CI 수치 재현성

record_id: DAPIER-2026-09-17-lift-stability-blas

### Problem

나는 기존 GRASP_CONFIRM PASS / LIFT_5MM 첫 step FAIL 상태를 유지한 채 두 분석을 분리했다. A는 저장 full-state 접촉 chronology, B는 clean temporary worktree의 CI/local 환경 비교를 담당했다. Source 수정은 coordinator 한 명만 했다. 시작 HEAD는 acf47cf이며 기존 untracked REAL_SCENE_GEOMETRY_AUDIT.md는 제외했다.

### Evidence

- A는 기존 copied run의 command/raw qpos/qvel/force/contact 및 B_confirm 상태를 bit-exact 재현했다. 추가 계측은 점 Jacobian×raw qvel이며 실제 제어를 바꾸지 않는다. Force는 contact frame의 mj_contactForce 결과다([MuJoCo 3.3.7 API](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-contactforce)).
- 세 case 각각 50 physics steps / 100 ms: HOLD는 bilateral 유지; 기존 LIFT는 첫 step 양쪽 접촉 소멸; continuous-command LIFT는 step36 / +72 ms 한쪽 지지 소실, step42 / +84 ms 양쪽 Fn=0. Step42에도 한쪽 geometric contact는 남는다. **양의 지지력 소실과 geometric contact 소멸을 구분한다.**
- 모든 case에서 table support loss T1은 50step 내 미발생이다. 따라서 남은 실패는 공중에서 떨어지는 slip이 아니라 table 이탈 전에 파지가 무너지는 경우 B다. Continuous의 양쪽 geometry 완전 분리도 관측 구간 내 미발생이다.
- B_confirm Fn=0.070893/0.072187 N, table support=0.195741 N (20g 무게의 약99.77%), 침투1.472/1.501 µm. Contact point 높이차31.058 mm, closing-axis error11.328°, block yaw -0.818535°. Center offset [-0.642194,+0.055428,+0.145758] mm.
- Continuous step35의 |Ft|/(μFn)=0.99399/0.98547, tangent relative speed1.068/0.595 mm/s. Step36 pad1 gap +0.586 µm, table support0.125422 N. 손은 상승하지만 물체는 테이블에 남는다. HOLD 종료 Fn0.047872/0.047302 N으로 감소해 이상적인 μΣFn 상한도 물체 무게에 미달한다. 이 상한은 실제 force/moment 여유의 보장이 아니다.
- B의 clean worktree에서 canonical XML, repo overlay, ROS/CUDA 환경변수 제거를 각각 비교해 동일 local 모델을 확인했다. Remote artifact의 실제 libmujoco SHA, source XML·mesh·task assets·모델 코드와 옵션은 local과 같다.
- Remote와 local의 포함된 public compiled fields20개 차이는 pose/axis/size ≤2.22e-16, inverse weights/actuator_acc0 ≤2.27e-13의 수치 차이다. OPENBLAS_CORETYPE=Zen만 선택한 local 실행이 remote portable hash7bd67b… 및 native BOX–BOX +0.19581108263492744 m를 정확히 재현했다. Default local SkylakeX는 b6dafd… 및 native0이다. Certificate는 두 경우 모두0.19581108263491326 m다.

### Decision

나는 성공을 만들기 위해 friction·force·geometry·controller·timing·threshold를 조정하지 않았다. Copy에서 전체 gate를 통과한 LIFT 후보가 없으므로 live 재시작도 하지 않는다. 작은 preload/엇갈린 contact/relaxation이 유력하지만 각 controller/linkage/solver 기여는 별도 분리가 필요하다.

CI에는 실패 전 compiled field NPZ/MJB 및 허용된 환경 manifest를 업로드하도록 진단만 추가했다. Native false-zero 재현은 BLAS kernel에 따른 sub-ULP frame 차이에 민감하다. 지원되지 않는 SkylakeX를 AVX512 없는 CI CPU에 강제하지 않으며, hash 반올림이나 geometry assertion 삭제로 통과시키지 않는다.

### Validation / Result / Lesson · 진행 상태

확장한 LIFT chronology regression 1 PASS / 21.514 s. 저장 진단 replay helper에 LIFT case와 table support 표시를 연결했다. Actual physics는 이전 LIFT_5MM FAIL / SIM17.474 s이며 CENTER SUCCESS 아님. Hardware/OS30A/multi-seed/ACT는 실행하지 않았다. 수치 프로필의 이식성과 전체 회귀 결과는 아래 최종 handoff에서 구분한다.


### CI 원인 확정 및 최소 수정

직접 확인한 초기 차이는 PGripper CAD z축 norm의 0.9999997374604657(SkylakeX) 대 0.9999997374604656(Haswell)이다. 같은 local에서 Haswell로 계산하면 CI의 portable 포함 필드가 모두 byte-identical해진다(경로4개 필드만 제외). 실제 library/version 변경이나 source/asset 차이가 아니다. 지원되는 다른 kernel과 stdlib norm 가설은 기존 b6 모델을 재현하지 못해 채택하지 않았다.

나는 실제 portable SHA를 바꾸어 보고하지 않고, 기존 source profile에 감사된 b6/7bd 두 exact identity의 비교만 추가했다. 제3의 digest는 거부한다. Manipulation pair의5개 원본 hash는 보존하고 각 pair의 exact Haswell hash만 추가했다. JSON key인7bd는 감사 출처이며, 매 step 전체 model을 캐시해 승인하는 장치가 아니다. 기존과 같이 호출마다 pair+ancestor 해시를 확인하고, 저장-state 경계에서는 전체 fresh model identity를 확인한다. Structural near-support 해시는 변경하지 않았다. 반올림 해시, geometry 수치, clearance/접촉 policy 변경은 없다.

Haswell의 동일 saved-state diagnostic은 GRASP_CONFIRM qpos 최대차7.806255641895632e-17, 첫 LIFT3.079921910499779e-16을 보였다. 같은 identity의 fixture replay는 여전히 exact0을 요구한다. 다른 감사 identity의 과거 fixture 비교에만 float64 4epsilon(8.881784197001252e-16)을 사용하고 사건 step1/36/42와 접촉/clearance gate도 함께 검증한다. 이 값은 runtime measured tolerance·joint range와 무관하다. 동일 프로세스 copied/live 및 원본 state 불변의 array_equal 검사는 유지했다.

BOX–BOX 회귀는 기존 승인 모델의 native0을 계속 확인한다. 두 번째 모델은 관측된 native 양수0.19581108263492744를 유지하며, 두 모델 모두 native0을 mock한 별도 분기로 certificate fallback을 실행한다. Exact touching/shallow/deep penetration은 계속 거부한다. 미감사 identity 및1e-12 geometry 변형 거부 회귀도 추가했다. 독립 검토자는 이 최소 경계에서 새 blocking issue를 찾지 않았으며, 전체 regression 결과는 아래에 기록한다.

집중 검증: Haswell LIFT chronology 1 PASS /20.219s. 실제 LIVE 수정·재실행 없음.

### 전체 검증 및 CI wall-clock 제한

- Source commit389735edcc7133ab0c0b791890cbdd288d623147 normal push 완료. Local canonical `scripts/verify-mujoco-headless`: research33 PASS(0.018s), MuJoCo354 PASS(1091.182s), canonical render·vision artifacts PASS, process exit0. Haswell 집중 회귀: certificate4/source4/manipulation7/LIFT1 PASS. Repository-root에서 manipulation 단독 discovery는 parent import가 없어 실패했지만 해당 module directory의 표준 unittest discovery로7 PASS; 전체 repository invocation에는 임시 PYTHONPATH를 추가하지 않았다.
- [Remote run35201807198](https://github.com/Alpenj/DAPIER/actions/runs/35201807198)는 assertion failure가 아니라 annotation `The job has exceeded the maximum execution time of 20m0s`로 CANCELLED다. Post-contact CLOSE copied replay 도중 중단됐다. 전체 PASS로 취급하지 않는다.
- 이번 hosted runner는 Intel Xeon8573C/AVX512이며 b6dafd…/native0을 재현했다. 이전 EPYC7763는7bd67b…/native+195.811083mm였다. 고정된 ubuntu-latest 라벨이 동일 CPU 수치 경로를 보장하지 않는다. 최신 model artifact10488418168도 보존했다.
- 전체 테스트를 삭제/단축하지 않고 CI job wall-clock cap만20→45min으로 변경했다. Local 전체18.2min와 hosted20min timeout이 근거다. Physics timestep/trajectory duration/runtime timeout/acceptance/controller에는 변화가 없다.
- PR62 base main/draft는 새 전체CI 결과가 확정되기 전 유지한다. 최종 결과와 commit/merge SHA는 PR 및 Notion handoff에 연결한다. 이 문서/CI cap 변경 후 SIM source는389735e와 동일하므로 local 전체를 불필요하게 반복하지 않는다.
- KIT의 기존 local-validation 아래 `lift-stability-20260917`에150-step raw/history/report를, `ci-numeric-audit-20260917`에 local/remote/Haswell manifest와 field diff를 보존했다. 별도 새 학습 원장을 만들지 않았다.
- Diagnostic viewer는 saved physics replay로 표시한다. 기존 실제 failure state는 진행하지 않았다. Actual 마지막 PASS=GRASP_CONFIRM, LIFT_5MM FAIL/SIM17.474s, CENTER SUCCESS=false/HOLD0s. 다음 blocker는 table 이탈 전 불안정한 edge grasp/preload이며, contact-centered placement/closing 검토가 필요하다. 현재 staging30.378087mm는 기준30mm 대비0.378087mm 여유의 단일 조건 결과일 뿐 실물/다중초기조건 강건성 근거가 아니다.


## 2026-09-17 — 접촉 geometry·하중·상대 운동의 병렬 분리

record_id: DAPIER-2026-09-17-grasp-load-evidence

### Problem

나는 GRASP_CONFIRM의 bilateral contact PASS가 실제 하중 지지의 충분조건인지 확인했다. 기존 LIFT command jump를 제거한 copied 진단도 step36에서 한쪽 힘을 잃고 step42에서 양쪽 정상력이0이 된다. 실제 마지막 PASS는 GRASP_CONFIRM이고 LIFT_5MM/CENTER SUCCESS는 아직 아니다.

### Evidence

A/B/C를 read-only로 분담했다. 동시 작업자는 최대2명으로 A/B 뒤 C를 실행했고, coordinator만 소스를 수정했다. 동일 저장 B_confirm 및 continuous50-step 기록을 사용했다. 새 full-range sweep이나 retreat 검색은 하지 않았다.

- Geometry: block COM-local contact는 pad1(-19.999264,+12.013009,-15.459300)mm, pad2(+19.999259,-19.999879,+15.595676)mm다. World Z 높이차31.057826mm, unsigned normal–closing angle10.632049°/5.784915°다. 비슷한 정상력이 대칭 face pinch를 뜻하지 않는다. Pad2는 block x+/y− edge에 있고 compiled mesh bounding-box y 경계 inset0.127230µm/z0.115269mm다. 이는 tapered pad face의 실제 edge 최단거리가 아니다. 최초 소실은 여유가 더 큰 pad1이므로 pad2 edge 이탈을 단독 원인으로 확정하지 않았다.
- Frame 정정: 과거(-19.825,+12.297,-15.461)mm 등의 값은 world COM lever arm이다. 엄밀한 block-local 좌표는 위 값이다. Closing axis는 TCP rotation column2=(.980517309,-.168651884,.100709227), block yaw−.818535°다.
- Force: B_confirm Fn=.070893/.072187N, 실제 finger vertical=.000459441N, table=.195740710N, block weight=.1962N이다. Table이99.77% 지지한다. Finger COM torque world=(−.000518596,−.002185022,−.001634548)Nm다.
- HOLD50: jaw opening은8.603µm 줄지만 Fn 합은33.48% 감소한다. Motor는 벌어지지 않으며 포화도 아니다. Soft jaw-coupling preload relaxation과 contact pose 변화가 함께 관측되지만 각 기여를 단독 확정하지 않는다.
- Elliptic contact의 접선 friction은1.6이다. 중력 방향의 낙관적 상한은 B=.228929N, HOLD50=.152279N이다. HOLD와 continuous 모두 step19부터 weight보다 작다. Contact normal 방향을 포함했으며 torque equilibrium/공유 torsional friction budget을 무시한 상한이므로 상한이 weight보다 커도 lift 가능을 증명하지 않는다.
- Motion: step35 하향 tangent slip은1.06752/.580816mm/s, friction utilization=.993987/.985469다. 첫 finger force loss=36/72ms, 양쪽0=42/84ms. Step42에도 기하 접촉은 남는다. Table support loss는50step 창에서 없다. Step50 TCP는105.561µm 상승하지만 block COM은9.092µm 하강한다. 최대 block COM 상승3.676µm도 완전 table 이탈이 아니다.
- T0에 이미 작은 block velocity/yaw motion이 있다. Step1은 첫 기록 변화이며 실제 운동 개시 시점이라고 단정하지 않는다.

### Decision

주 원인은 지속 가능한 하중 지지가 없는 약한 비대칭 파지다. Command jump는 확인된 별도 기여지만 제거만으로 해결되지 않는다. Gripper opening failure나 contact gate의 stale force만으로 설명할 근거는 없다. Geometry/friction/controller/timing/acceptance/force threshold는 변경하지 않았다.

나는 기존 CLOSE schedule의 다음 한 increment만 copied state에서 시험했다. Arm은 explicit command를 유지하고 left gripper2.043112220719→2.033145819642rad(CLOSE17)로 변경했다. 새 각도/force 기준을 발명하지 않고 기존 schedule·controller·trajectory generator·runtime gate를 재사용했다. 추가 CLOSE의 minimum duration 입력은 기존.1s이고 실제 generator duration은.404s다.

### Validation

- CLOSE17 copied preflight PASS: TCP.155276mm, approach1.9675°/closing11.3286°, Fn=.098751/.101273N, general clearance48.707389mm. 원본 integration state array_equal 보존.
- 기존50step confirmation 후 continuous-command LIFT에서 다시 step36 bilateral loss로 정지했다. 따라서 추가 CLOSE 한 단계만으로 해결되지 않으며 live 재시작 조건을 충족하지 않는다.
- 기존 진단에 block-frame wrench/COM torque 및 optimistic vertical upper bound를 추가했다. MuJoCo contact force는 geom2에 작용하므로 block=geom1이면 부호를 반전한다. 새 값은 diagnosis 전용이며 is_load_ready_gate=false다.
- 실행 재현: SIM 디렉터리에서 기존 venv Python으로 `lift_transition_diagnostic.py --report test/fixtures/lift_transition.json --next-close --output /tmp/next-close.json`. 기본 진단과 기존 failure fixture도 유지한다.
- 집중 LIFT 회귀 1 PASS /46.702s. 전체 local suite와 원격 CI의 최종 결과·commit/merge SHA는 [후속 PR63](https://github.com/Alpenj/DAPIER/pull/63)에 연결한다. CI PASS 전 draft/merge 금지를 유지한다. 원시 A/B/C 산출물 및 후보 telemetry는 기존 KIT local-validation 아래 grasp-load-evidence-20260917에 보존한다.

### Result

실제 실행은 이번 턴 진행하지 않았다. 마지막 actual PASS=GRASP_CONFIRM, 기존 actual failure=LIFT_5MM/SIM17.474s, CENTER SUCCESS=false다. 추가 CLOSE/LIFT는 DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS다. 접촉 회복이나 기하 접촉 잔존을 stable grasp로 보고하지 않는다.

### Lesson / Next

CONTACT_CONFIRM(실제 양쪽 접촉)과 GRASP_LOAD_READY(하중 지지 근거)는 다른 의미다. 이번에는 runtime gate를 바꾸지 않았다. 현재처럼 힘이 작고 감쇠하며 table support가 남는 경우에는 bilateral force>0만으로 후자를 주장할 수 없다. 다음 blocker는 opposing-face 중심에 가까운 contact placement와 preload 유지의 원인 분리다. 마찰/힘 기준을 올리거나 닫힘 단계를 계속 추가해 통과시키지 않는다.

현재 staging30.378087mm는 일반30mm 대비.378087mm 여유의 단일 조건 결과이며 다중 초기조건·실물 강건성 증거가 아니다. Hardware/OS30A/multi-seed/ACT 미실행. 기존 untracked REAL_SCENE_GEOMETRY_AUDIT.md와 실패 state를 보존했다.

## 2026-09-18 — 비대칭 접촉 배치와 중력 wrench 진단

record_id: DAPIER-2026-09-18-contact-placement

### Problem

나는 LIFT를 통과시키기 전에 31.058mm 높이 차이의 접촉을 실제 하중 지지가 가능한
접촉 배치로 바꿀 수 있는지 확인했다. PR63 이후 main
`1c646eac01f14011d01899c2a2f65175be6d2449`에서 별도
`pro/grasp-placement-codex-20260918` writer worktree를 만들었다.
기존 dual-so101-codex-20260907 worktree와 untracked
REAL_SCENE_GEOMETRY_AUDIT.md는 읽기 전용으로 보존했다.

### Evidence

A는 정확한 접촉 면, B는 마찰 cone과 COM 토크 평형, C는 접촉 배치 후보를 분석했다.
동시 worker 2개 한도로 A/B를 병렬 실행하고, A 완료 thread를 C에 재사용했다.
Source writer는 coordinator 한 명이다. 모든 후보는
**DIAGNOSTIC COPY / NOT LIVE TASK SUCCESS**다.

- Pad1/2는 geom36/38, mesh17/19, body9/10(left PGripper jaw1/jaw2)의
  MESH다. 실제 world contact는
  (179.523748,12.405716,4.537328)/(219.059100,-20.174736,35.595154)mm,
  block COM-local은
  (-19.999264,12.013009,-15.459300)/(19.999259,-19.999879,15.595676)mm다.
  Jaw-body-local은
  (2.571241,7.059810,-41.002735)/(-2.571192,-15.069648,-16.002898)mm다.
- Compiled pad vertex 각각427개에서 inward convex-face polygon을 재구성했다.
  Closing 방향과 normal dot>0.9인 최대 planar patch 면적은671.380542/671.380201mm²,
  face center world는
  (176.650821,4.161000,18.708653)/(223.103605,-3.878379,23.500602)mm다.
  Plane grouping은 normal L2<5e-6, offset<0.1µm이다.
- Contact projection에서 **실제 polygon boundary**까지 거리는0.066917/0.075649µm다.
  이는 grouping tolerance보다 작아 내부 안전 여유라고 주장하지 않는다.
  AABB 여유3.46mm로 pad1이 중심 접촉이라고 해석하지 않는다.
  Raw CAD와 convex proxy의 해당 국소 nearest points 차이는0.000771/0.000517µm다.
  따라서31.057826mm 높이 차이는 frame 표현만의 문제가 아니다.
  선택한 usable patch는 CAD convex-face 분석 정의이며 실물 고무 패드 실측이 아니다.
  Nearest tip-cap/bevel triangle 하나를 유일한 하중 전달 면으로 단정하지 않는다.
- Closing axis는(.980517309,-.168651884,.100709227).
  Pad planar normals와 MuJoCo contact normals는 별개다.
  Finger 합력 block frame은(.001000670,.009109266,.000459545)N,
  COM 토크는(-.000487422,-.002192178,-.001634559)Nm다.
  두 접촉의 block-Y 토크 -0.001094535/-0.001097643Nm가 같은 방향으로 더해진다.

**Wrench feasibility:** 실제 elliptic condim4와 friction(1.6,1.6,0.02),
접촉 위치/normal,20g,중력9.81m/s²를 사용했다. Table을 제외하고 force3+COM torque3을
함께 평형시키며0≤Fn≤측정 Fn을 허용했다. Fn를 고정하는 것보다 낙관적인 relaxation이다.

| 상태 | full wrench 지지 질량 범위(512방향 LP) | force-only 범위 |
|---|---:|---:|
| 기존 B_confirm | 7.791–7.864g | 23.144–23.336g |
| 기존 추가 CLOSE17 terminal | 12.337–12.378g | 32.143g |
| B_confirm에서 HOLD100ms 후(128방향) | 5.051–5.214g | 15.439–15.523g |

단순 μΣFn>mg만으로는 토크 평형 실패를 놓친다. LP에서 얻은 covector ν를
독립적인 elliptic support inequality에 대입해 NumPy만으로 다시 검증했다.
각 contact wrench basis B에 대해 a=νB, α=ν·(-mg,0)>0이면

`λ ≤ Σ Fn_cap·max(0,a₀+norm(a₁:)) / α`

이다. B_confirm/CLOSE17의 독립 상한은 **λ0.393175904/0.618391128**,
즉7.86352/12.36782g다. Inner feasible witness로 upper bound를 증명하지 않는다.
Normal torsion과 COM lever arm을 포함한다. 이는 FP64 수치 부등식 검증이며
rounding까지 포함한 형식 증명, controller 도달 가능성, dynamic 성공 gate가 아니다.
SciPy LP는 이미 설치된 로컬 분석 환경에서만 사용했고 runtime/CI dependency는 추가하지 않았다.

### Decision

나는 먼저 closing-axis 수직 평면에서 analytic hand translation 두 개를 계산했다.
Usable face midpoint→COM은(+.083453,-.138093,-1.043767)mm,
contact midpoint→COM은(+.669253,+3.887726,-.005379)mm다.
두 벡터는 다른 목적이므로 더하지 않았다. 기존 material contact 두 점의
높이 차이는 rigid translation으로 줄어들지 않으며 새 feature 접촉이 생겨야 한다.

현재 closing→block+X 최소 회전을 tool axis에 적용한
(-.000822953,-.061589700,-.998101213) 방향도 검사했다.
Position3+axis-direction2만 사용하고 별도 closing15° gate를 유지했다.
이미 실패한 vertical IK, full-range sweep, retreat search를 다시 시작하지 않았다.

### Validation

같은 saved PREGRASP_NEAR full integration state에서 measured seed→coarse→fine
previous-q continuation을 사용했다. **비교 baseline도 재계획했으므로 과거 live의
bit-exact command replay가 아니다.** 모든 후보는 같은 추가100ms HOLD 후 continuous-command
LIFT를 시험했다. HOME/geometry/controller/timing/task-open/limits/0.5mm·2°·15°/
general30mm/penetration/measured tolerance를 변경하지 않았다.

| 후보 | 첫 blocker | 접촉 및 하중 결과 |
|---|---|---|
| 재계획 baseline | LIFT32step/64ms, pad1 force loss | table support 유지 |
| usable-face-center translation | LIFT33step/66ms, pad1 force loss | 높이차 약31.066mm, HOLD λ상한.203344 |
| contact-midpoint translation | APPROACH_COARSE 2.003240°>2° | 접촉 전 거부, TCP.121277mm |
| minimal-rotation axis | closing37.912678°>15° 및 경로 거부 | contact physics 실행 안 함 |
| contact-midpoint + 기존 planner reserve | LIFT33step/66ms, pad2 force loss | table support 유지 |

마지막 후보는 동일 XYZ/axis에 **기존1.94664° planner reserve만 재사용**했다.
실행 acceptance2°는 그대로다. CONFIRM Fn=.051613/.057298N→추가 HOLD100ms 후
.031431/.033428N, 합력40.45% 감소. 높이차30.841098→30.840343mm,
polygon boundary margin .047070/.052400→.028708/.030556µm다.
HOLD λ범위.190408–.197438로 전체20g 지지를 못한다.
두 contact normals가 서로 반대여도 closing-axis와 약10.96° 어긋난다.
LIFT 시 table4contacts/.143564N이 남고 maximum block lift는0.359µm뿐이다.
CONFIRM/HOLD TCP 최대 약.123mm, general clearance 최소약49.015mm,
block penetration 최대2.662µm였다. 이 수치들은 새 acceptance가 아니다.

Historical donor raw hash와 현재 raw hash는 경로/source 표현이 다르므로 같다고
보고하지 않았다. 기존 integration source profile의 unchanged-live raw 및 두 accepted XML,
fresh portable b6dafd…/augmented B exact identity를 대조했다.
Donor PRE time/qpos/qvel/TCP/axis/clearance 및 table4contacts 거리·Fn replay 오차0,
각 copied preflight 원본 state 불변, 초기 batch SHA 불변을 확인했다.
초기 reset flag 누락 harness 실패는 별도로 보존했고 task 후보 실패로 세지 않았다.

기존 tilt-feasibility의33방향/66branch에서는 이번 correction에 바로 재사용할
eligible 방향을 찾지 못했다. 다른 target/branch의 작은 closing 오차만으로
현재 경로가 가능한 것으로 처리하지 않는다. 전역 불가능을 증명한 것은 아니다.

### Result

**채택한 task candidate 없음. Live 재시작 안 함.**
실제 마지막 PASS는 여전히 GRASP_CONFIRM, 이전 actual LIFT_5MM FAIL/SIM17.474s,
CENTER SUCCESS=false다. 이번 수정은 기존 diagnostic에 정확한 geom/body/mesh와
contact frame 좌표를 추가하고, 저장 wrench 상한 회귀를 보존하는 범위다.
Task target/trajectory/success gate는 변경하지 않았다.

Focused/full/CI 결과는 아래 최종 검증 절과 후속 PR에 기록한다.
Raw geometry/force/후보 full-state·telemetry·재현 scripts는 기존 KIT local-validation의
contact-placement-20260918에 보존했다. 새 학습 원장은 만들지 않았다.
Viewer는 대표 baseline/refine의 **RECORDED PHYSICS REPLAY**이며 새 live 실행이 아니다.

### Lesson / Next

주 원인은 실제 edge 접촉 배치의 COM 토크 불균형과 HOLD 중 감소하는 정상력의 복합 문제다.
추가 CLOSE는 배치를 고치지 못하고, 작은 translation도 거의31mm 높이 차이를 남겼다.
다음 blocker는 현재5DoF limits 안에서 opposing usable faces에 더 가까운 접촉 배치를
만드는 orientation/position 조합이다. 현재 분석 patch와 실제 제조 패드의 차이도
확인 대상이며 모델을 임의 수정할 근거는 아니다.
CONTACT_CONFIRM과 GRASP_LOAD_READY는 다른 의미지만 이번에 gate를 새로 만들지 않았다.

Staging30.378087mm는30mm 기준 대비.378087mm 여유의 단일 조건 결과이며,
다중 초기조건이나 실물 강건성 근거가 아니다. Hardware/OS30A/multi-seed/ACT 미실행.

### 최종 로컬 검증 / handoff

- Focused 접촉·wrench·collision·MESH–BOX·BOX–BOX·near-support·manipulation:32 PASS/73.189s.
- `scripts/verify-mujoco-headless`: Research33 PASS/0.030s,
  MuJoCo356 PASS/1126.229s, canonical render 및 vision artifacts PASS, exit0.
  기존 dynamic-preflight/command-state/reset/SETTLE 회귀를 전체 suite에서 유지했다.
- `git diff --check` PASS. SIM source는 diagnostic metadata 추가뿐이며
  실제 task 제어/target/모델/limits/정책은 그대로다.
- 기존 viewer helper로 baseline/refine175개 저장 physics sample을 단일 창에서 재생했다.
  정상 종료exit0; 새 actual task 성공으로 세지 않는다.
- 원격 전체 CI가 통과하기 전에는 merge하지 않는다. 이번 부분 진척 PR의
  CI/commit/merge SHA는 GitHub PR과 기존 Notion 학습 기록의 handoff에 연결한다.

## 2026-09-18 — Opposing-face grasp family 탐색

record_id: DAPIER-2026-09-18-grasp-families

### Problem

기존 grasp의 접촉 높이차 31.058mm와 measured-force 기반 wrench 질량 상한
7.864g(CLOSE17 12.368g)은 20g block을 지지하지 못했다.
이번에는 기존 grasp 주변 translation tuning을 반복하지 않고,
현재 PGripper의 usable opposing faces에 COM 대칭 접촉을 만드는 family를 검사한다.

최신 origin/main `7ff415a707e3ea2c4e5e34452a61dab9a9dafdfb`에서
`pro/grasp-families-codex-20260918` writer worktree를 새로 만들었다.
기존 dual-so101 및 grasp-placement worktree/evidence와 unrelated audit는 보존한다.
Coordinator만 repository writer이며, 최대 두 worker의 read-only 분석을
A geometry → B kinematics → C copied physics 순서로 승인해 연결한다.

### Evidence

A는 settled block geometry를 offline oracle로 사용했다. Site X=approach,
site Z=pad1→pad2 closing axis다. Usable-face midpoint의 site offset은
(-1.260623mm, 약0, -2.208µm)이므로 ideal TCP는 block center 자체가 아니라
`blockCenter - Q * midpointOffset`이다. Q=[approach, closing×approach, closing]는
기하 설명용이며 full6D IK 목표가 아니다.

세 opposing face pair의 signed assignment 6개와 cardinal 방향24개를 생성했다.
±Z closing은 아래쪽 pad가 table을 약4.002mm 침범해 제외한다.
옆면 pinch의 위에서/수평 접근12개를 B에 넘겼다.
이상적 COM 대칭 접촉점은 실제 usable polygon 경계에서 최소11.230791mm 내부이며
plane residual 최대0.015857µm다. 두 polygon 면적 중심은 이상적 접촉점과
각26.265µm 다르므로 두 개념을 구분한다. 제조 pad 실측 검증은 아니다.
A의 clearance는 distal pad만 평가한 값이고 full arm/path 안전 판정이 아니다.

B는 기존 position3+axis-direction2 IK와 closing15° 후검사를 사용했다.
위치0.5mm/axis2°/joint limits 및 기존300iteration을 유지하고,
6개의 결정적 range-interior/branch seed를 사용했다.
기존 solver 내부 bounded step/ctrl clipping은 유지하지만 새 clamp는 추가하지 않았다.
그리퍼/right 채널은 고정했다. 기존 measured-derived1.94664° 내부 planner reserve를
명시적으로 재사용한 진단이며 runtime acceptance2°는 그대로다.

- Cardinal12×6 seed: 연결 PREGRASP→GRASP 통과0.
- +X top-down은 PREGRASP7.0892mm/axis3.8006°로 실패하지만,
  별도 GRASP_ONLY는0.3443mm/axis1.4598°/closing3.3052°로 통과했다.
  PRE wrist-flex margin0, GRASP margin0.307576rad다.
  GRASP_ONLY를 안전한 연결 경로로 세지 않는다.
- 다른 signed top-down GRASP는 closing164.14°/80.93°/74.20°로 실패했다.
- 네 signed side family에서 closing을 고정하고 접근축을 면 안에서
  수직 대비±15/30/45/60/75°로 확장한40개도 각각6seed로 평가했다.
  연결 통과0, 별도 GRASP_ONLY 통과0이다.
  예: Y−/+15°의 PRE 위치0.3190mm/axis8.5451°/closing12.0019°,
  GRASP 위치0.2958mm/axis8.0668°/closing11.7915°.
- 이 사례 PRE rank5, 최소 singular value3.464e-5m/rad, 최소 joint margin0.1864rad다.
  모든 실패를 joint limit/rank 소실로 단정하지 않는다.
  bounded solver/search에서 찾지 못한 결과이며 globally unreachable 증명이 아니다.

### Decision

고정된60mm stand-off에서 동일 축을 요구한 실패와 실제 GRASP 끝점의 도달성을 분리한다.
B가 통과한 +X vertical 끝점에 대해, 기존 verified PREGRASP prefix를 재사용한
하나의 새 copied 경로만 C에 허용했다:
기존 coarse 위치/축 → 새 usable-face-centered vertical GRASP.
각 구간은 이전 q에서 이어 풀며 전체 collision/path gate를 먼저 적용한다.
기존 작은 placement search를 다시 실행하지 않는다.

C는 PREFIX_END에서 시작한 DIAGNOSTIC COPY이며 fresh HOME/live task PASS가 아니다.
runtime source, model, friction, mass, controller, timing, limits와 모든 safety 기준은
copied evidence가 충분하기 전에는 수정하지 않는다.

### Validation / Result

C의 새 family는 `+blockX / vertical approach`이며 실제 physics는 donor
PREGRASP prefix 끝에서 시작했다. 기존 coarse 위치/축을 거쳐 새 fine으로 이어지는
계획은 각각11/9 interpolation samples PASS였다.
계획 coarse 위치0.111707mm/axis1.964057°/closing11.3222°,
fine 위치0.316947mm/axis1.089894°/closing9.901173°로 통과했다.
별도 GRASP_ONLY 해와 branch가 다르며 이전 q에서 이어 푼 결과다.
Copied actual fine 위치0.487900mm/axis1.035683°/closing9.8971°도 통과했다.

CLOSE22에서 first bilateral contact가 생겼고 GRASP_CONFIRM50step와
추가100ms HOLD50step는 통과했다. 그러나 다음과 같이 **채택 기준은 실패**했다.

| 항목 | CONFIRM 종료 | 추가 HOLD100ms 종료 |
|---|---:|---:|
| 실제 접촉 높이 차이 | 4.938480mm | 4.923809mm |
| pad1/pad2 polygon edge margin | .01835/.00696µm | .01101/.00381µm |
| 정상력 pad1/pad2 | .033950/.042197N | .019770/.023094N |
| polyhedral wrench 질량 상한 | 8.913322g | 5.191015g |
| 독립 elliptic support 부등식 상한 | **8.855380g** | **5.157271g** |

접촉 높이차는 기존31.058mm에서 줄었지만 edge margin은 기존 geometry grouping
0.1µm보다 작아 usable interior 확보로 세지 않는다. 동일한 snapshot의 force caps,
contact normals, COM torque, elliptic friction을 포함한 정적 상한도20g 아래다.
이는 실제 actuator가 만들 수 있는 힘의 보장이나 새 runtime gate가 아니다.
B가 C의 두 저장 상태를 독립 NumPy 지지 부등식으로 다시 확인했으며,
기존 `grasp_wrench.json`에 두 negative case만 추가했다. 이전 사례는 보존했다.

Continuous-command diagnostic LIFT는 **step31/62ms**, SIM20.072s에
pad1 force0 / pad2 .023502N으로 fail-closed 정지했다.
Table4contacts/.184514N이 남아 있었고 table support 이탈은 없었다.
LIFT 구간 최대 block bottom 변화는 약0.1003µm에 불과했다.
5mm 또는30mm lift PASS가 아니다. 그 순간5.236mm TCP 오차는 진행 중 trajectory의
최종+5mm 목표 대비 값이며 별도의 완료 endpoint 실패로 잘못 분류하지 않았다.
첫 blocker는 bilateral support loss다.

CONFIRM/HOLD/LIFT 구간 general clearance 최솟값52.577034mm,
최대 contact penetration1.615407µm다. 기존30mm/1mm 조건을 유지했다.
Full-state preflight 원본 및 단계 연결, donor/model identity,
recorded replay의 raw qpos/qvel·forces 불변 검증은 PASS다.
Viewer는 실제 저장 contact rows만 기존 helper로 재생하며
`DIAGNOSTIC COPY / RECORDED PHYSICS REPLAY / NOT LIVE TASK SUCCESS`를 표시한다.

**Teacher grasp target 채택 없음. Live 재시작 없음. CENTER SUCCESS=false.**
Actual 마지막 PASS는 여전히 기존 GRASP_CONFIRM이며 보존된 actual LIFT failure는
SIM17.474s다. 이번 copied 결과의 SIM20.072s와 혼동하지 않는다.
높이 대칭 개선만으로 load-bearing/LIFT milestone을 선언하지 않으므로
full local Research/MuJoCo/canonical render 및 remote full CI는 이번에 실행하지 않는다.

집중 검증:
- axis-direction finite difference / roll invariance 2 PASS(2.738s).
- 새2case를 포함한 wrench evidence/corrupt claim/대칭 contact 검증 2 PASS(.004s).
- 기존 actual failure/HOLD/continuous LIFT chronology 회귀 1 PASS(49.828s).
- A cardinal24 / refined40의 frame·COM·polygon·table negative verifier PASS.
- B cardinal12/refined40×6seed의 수용 조건 독립 재계산 PASS.
- C copied state continuation / endpoint / safety / first blocker / 실제 replay rows verifier PASS.
- 최초 unittest module-style invocation은 test package import에 실패했고,
  첫 discover는 명시적 asset env 누락으로 실패했다. 프로젝트의 discover 명령과
  기존 pinned DAPIER_SO101_MJCF로 바로잡은 뒤 위 결과를 얻었다.
  코드·의존성 변경으로 숨기지 않았다.

기존 KIT local-validation의 `grasp-families-20260918/{A,B,C}`에 전체 scripts,
frozen targets, q/margins/SVD, copied fullstate/telemetry, replay와 SHA manifest를 보존한다.
Raw physics JSON은 약109MiB로 Git에 추가하지 않았다.
C raw SHA256: `666430b2b6a2a819a45c5ff7eabc82e5c9118037ce1c4cc8a6b5aa8663647e6a`.
A cardinal/refined SHA256: `5aaaa0729a3b22b3a00477c5afc77929cae99b5e0b4b221cb221e5e99528d1fd` /
`ed7bf3c331e9e63436dc9d6f322be169d95ac2271219eaf6e8b94485c43e1ee6`.

다음 blocker는 **kinematically admissible한 접근축/closing 오차가 실제 pad interior
접촉을 보장하지 않는 것**이다. Height symmetry 하나보다 실제 face alignment와
접촉 lever arm/하중 지지를 함께 확인해야 한다. 이번 bounded family screen 실패는
전역 불가능 증명이 아니며 geometry/friction/threshold 변경 근거도 아니다.
Staging30.378087mm의 .378087mm 여유는 여전히 단일조건이며 실물 강건성 근거가 아니다.
Hardware/OS30A/multi-seed/ACT 미실행.


### Lesson / Next

MuJoCo truth는 이번 geometry/expert 분석의 oracle다. 후속 sensor milestone에서는
RGB-D/wrist RGB 기반 추정과 camera→arm transform으로 feasible family를 선택하고,
body pose는 estimator 평가 ground truth로만 사용한다.

Full local/remote suite는 후보마다 반복하지 않는다. 이번에 실제 load-bearing/LIFT 개선
또는 live family 채택 milestone이 있을 때만 전체 local 및 render를 실행하고,
main merge 직전에 원격 전체 CI를 요구한다. 미달이면 draft PR을 보존하고 merge하지 않는다.

### GitHub / handoff 범위

git diff --check PASS. 검증된 학습 기록과 기존 wrench fixture의 새 negative2case만
commit/normal push한다. 이전 dirty/untracked audit와 worktree HEAD는 보존했다.
이번 milestone 미달 commit은 [skip ci]로 PR/push 자동 전체 CI를 예약하지 않고
base=main draft PR로 남긴다. Workflow/safety assertion은 수정하지 않는다.
Skipped checks를 PASS로 세지 않으며 이번 PR은 merge하지 않는다.
후속 milestone에서 skip 없는 검증 commit과 요구된 full CI PASS가 있어야 merge한다.
GitHub의 commit skip semantics: https://docs.github.com/en/actions/how-tos/manage-workflow-runs/skip-workflow-runs
PR/commit과 기존 Notion 기록의 최종 링크는 turn handoff에 연결한다.

## 2026-09-18 — Gripper-only contact space와 최소 파지력 분리
record_id: DAPIER-2026-09-18-gripper-only-feasibility

### Problem

기존 +X grasp는 contact 높이차를 31.058mm에서 4.924mm로 줄였지만 실제 native
접촉점은 pad boundary이고, HOLD wrench 상한 5.157g < 20g였다.
이번에는 arm-pose local search를 반복하지 않고 PGripper/40mm cube 자체의
interior contact geometry와 필요한 힘, 실제 힘 유지 가능성을 분리했다.
PR65의 기존 writer pro/grasp-families-codex-20260918 / source HEAD
cf2b550e4e4089a5da792257001984a9395af979를 이어 사용했다. 시작 status clean.
Coordinator만 repository writer이며 A geometry/B wrench는 scratch-only로 분석했다.

### Evidence

**Geometry와 discrete contact는 다른 증거다.**
A는 compiled distal convex hull 전체를 기준으로 두 선택면이 실제 supporting
face임을 확인했다. Supporting patch 면적은 671.380628/671.380452mm²,
최대 내부 원 반경은 각각 12.499997mm다. COM 대칭으로 고른 opposing
facepoint의 polygon boundary margin은 11.230793/11.247968mm다.
두 pad 법선의 반대평행 오차는 0.0000476204°다.
Housing/cube exact OBB SAT separation은 13.747935mm다.

원본 full model의 별도 MjData에서 arm pose는 고정하고 block relative pose와
jaw 초기값만 구성했다. Kinematic 초기 배치이며 actual task 또는 arm motion이 아니다.
10µm per-pad 초기 압축은 native bilateral contact 각1점을 만들었지만
그 점은 경계였다(약0/0.015528µm margin).
Compiled MuJoCo3.3.7/native CCD/MULTICCD off/elliptic cone/condim4를 유지했다.
Sync helper가 반대 팔 passive jaw도 동기화한 것은 qpos 변경 audit에 명시했고
그 상태를 arm-motion/force 증거로 쓰지 않았다.

면적이 있는 supporting patch와 solver가 대표하는 discrete point를 구분한다.
MuJoCo의 general convex contact/multiple-contact 설명:
https://mujoco.readthedocs.io/en/3.3.7/computation/index.html
경계 sample만으로 내부 접촉면이 없다고 결론내리지 않는다. 반대로 abstract
pressure 분포를 실제 solver contact라고 보고하지 않는다.

**필요 파지력:** B는 기존 measured Fn cap을 요구조건으로 사용하지 않았다.
Actual friction (1.6,1.6,.02)의 elliptic condim4와 접촉 위치/법선/COM을 사용해
20g 중력 힘과 COM torque를 동시에 평형으로 만드는 최소 힘을 계산했다.

| 접촉 표현 | 최소 총 Fn 하한–상한(N) | 최소 finger별 최대 합산 Fn 하한–상한(N) |
|---|---:|---:|
| 실제 initialized native 두 점 | .183633781–.184562375 | .102047818–.102512115 |
| Abstract COM-center patch 두 점 | .142183339–.142724149 | .081322597–.081593002 |
| Abstract distributed patch 12점 | .121769233–.122144776 | .071115544–.071303316 |

하한은 exact elliptic-cone dual inequality로, 상한은 실제 cone 안의 force/torque
평형 witness로 각각 독립 검증했다. 단순 mu*sum(Fn)>mg 검사가 아니다.
실제 native 최소총힘 witness Fn은 .102512115/.082050260N이다.
최대 force/torque residual은 3.53e-15N/6.92e-17Nm.
무한 구동력 또는 기존 측정 force를 몰래 가정하지 않았으며, 이 정적 witness는
controller가 그 힘을 실제로 만들고 유지한다는 증명이 아니다.

### Decision

원본 gripper subtree를 rigid parent에 추출하고 free 20g cube만 남긴 **별도
gripper-only bench**에서 힘 형성/유지 가능성을 좁혀 확인했다.
원본 arm/table/environment가 없는 의도적인 격리이며 full-task preflight가 아니다.
Source model portable SHA b6dafd26e8e6bc34e9e5ecced05e2bb500b36c779a21f2e132efdc32f91811a4,
bench SHA 3c22a004c53af9fa0069cbdc0915bcae5f74418e2c3f683589a3621d05efbae1.
Pad meshes/geom/material contact fields, gripper actuator/equality/passive dynamics,
body mass/inertia와 option163항목 bit-exact 일치를 확인했다.
초기 world geom transform 차이는 1e-15 허용범위 안이다.
팔 wrist joint/actuator와 table 제거는 이 격리에서만 적용했고 task source/model은 그대로다.
Block 상대 orientation은 analytic pad-face 정렬이며 실제 desk reset pose 변경이 아니다.

기존 geometry에서 계산한10/50µm 압축 두 초기값만 각각100ms/50physics steps
검사했다. 기존 position actuator의 ctrl을 초기 motor reference에 유지했다.
매 step qpos reset/강제 jaw lock/물체 attachment/weld/mocap은 없다.
초기 압축 배치는 정상 APPROACH/CLOSE 과정으로 형성한 preload가 아니므로
이 bench를 실제 task trajectory 통과 근거로 사용하지 않는다.

### Validation

| 단독 bench | 10µm 초기 압축 | 50µm 초기 압축 |
|---|---:|---:|
| 50poststeps bilateral 유지 | 실패(소실 후 회복) | 유지 |
| 최소 pad1/pad2 Fn(N) | 0/0 | .215433/.193109 |
| 종료 Fn(N) | .155904/.135752 | .259592/.235115 |
| Block 하강량(mm) | .566323 | .101071 |
| 종료 block world Z속도(mm/s) | -1.016524 | -.969859 |
| 종료 jaw plane gap(mm) | 40.038016 | 39.984783 |
| 최대 penetration(mm) | .050394 | .050010 |

두 case 모두 nonfinite/warning/비pad block contact/관절 범위 위반 없음.
기존1mm penetration bound 이내다. 50µm의 최대 soft coupling residual은
48.996µm이며 raw state에 보존했다. 이는 새 tolerance 또는 task target이 아니다.
Native point는 종료에도 margin .238/.193nm로 경계에 있고,
독립 supporting-face compression overlap의 inradius12.5mm는 유지됐다.
면 안쪽 pressure가 실제 형성됐다는 증거로 대체하지 않는다.

50µm 각 step의 현재 contact geometry에서 measured-Fn cap을 별도로 넣은
instantaneous static capacity inner lambda 최솟값1.51889, 종료 구간
[2.503197,2.515586]이다. 필요한 힘의 계산에는 cap을 사용하지 않았고,
이 capacity 비교에만 실제 force를 넣었다.
종료 geometry의 최소 총 Fn은 [.185934518,.186955953]N.
실제 종료 finger force [.00401909,-.00043647,.19625422]N와 COM torque
[-2.14e-6,-1.44157e-4,3.52948e-4]Nm는 정적 평형이 아니며 미소 운동이 계속된다.
특히 약.97mm/s 하강이 남아 있어 장기 안정 또는 slip-free PASS가 아니다.

mj_step의 contact/force는 integration 전 평가, 저장 raw qpos/qvel은 integration
후 상태라는 시점을 분리했다. 같은 evaluation COM과 전후 속도 변화로
discrete dynamics residual을 확인했다(force 최대1.80e-16N, torque5.75e-18Nm).
Free joint angular qvel은 body-local이며 world velocity로 잘못 해석하지 않는다.
연속 가속도 검증 또는 이후 LIFT 성공이라고 주장하지 않는다.

Focused regression은 기존 wrench/그리퍼 planner/viewer와 새 primal/dual 증서
11 tests PASS(5.540s). 새 regression은3contact모델×2objective의 양의 증서,
force/COM torque/dual/NaN/누락 witness 변조 거부를 확인한다.
최초 broad discover에서 asset env가 빠져 viewer setup1error가 났고,
기존 pinned DAPIER_SO101_MJCF를 명시한 동일 invocation은 전부 통과했다.
의존성 또는 모델 assertion 변경으로 숨기지 않았다.
실행:
```bash
env DAPIER_SO101_MJCF=/tmp/dapier-pr62-pinned-assets/so101_new_calib.xml \
  /home/dapier-jhj/DAPIER/so101_imitation_learning/.venv/bin/python \
  -m unittest discover -s test -p 'test_gr*e*.py' -v
```
실행 디렉터리는 2ARM_ROBOT/sim/mobile_dual_so101이다.

### Result

**내부 접촉 patch의 기하학적 존재는 확인했지만, 요청한 actual usable-interior
stable load-bearing grasp 전체 조건은 미확정이다.**
A/B/C 중 A를 전체 성공으로 선언하거나 B(그런 geometry 없음)로 단정하지 않는다.
100ms의 힘 유지 가능성과 계속되는 sag, boundary native point, 정상 CLOSE 형성
미검증을 함께 남긴다. Jaw range가 geometry적 정렬을 차단하지 않았지만
장기 안정성/실물 pad fidelity는 별도다.

Arm mapping의 선행조건이 아직 충족되지 않았으므로 이번 arm IK/local search,
full-task copied APPROACH/CLOSE/LIFT, live 재시작은0회다.
Actual 마지막 PASS=GRASP_CONFIRM, 기존 LIFT FAIL/SIM17.474s 보존.
CENTER SUCCESS=false, teacher target 채택 없음.
Geometry/friction/mass/controller/timing/limits/acceptance/safety/measured tolerance 불변.
Milestone이 아니므로 full local Research/MuJoCo/canonical render/remote full CI를
실행하지 않고 PR65 draft를 유지한다. Skipped CI를 PASS로 세지 않는다.
기존 단일 viewer는 이전 recorded copied-physics replay를 유지하며 새 live로 표시하지 않는다.

### Lesson / Next

접촉점 boundary라는 관측과 물리 패드의 내부 면적 부재는 같은 명제가 아니다.
현재 blocker는 supporting patch와 native point representation의 관계 및
정상 CLOSE에서 형성되는 preload가 slip 없이 유지되는지다.
팔 search나 force/friction/geometry 수정으로 넘어가기 전에 이 경계를 검증한다.
후속 arm mapping에서는 IK residual이 실제 pad-contact margin을 얼마나 소모하는지
함께 평가해야 한다. 현재는 그 단계의 PASS evidence가 없다.
Staging30.378087mm는30mm 대비.378087mm의 단일조건 여유로 실물 강건성 근거가 아니다.
Hardware/OS30A/multi-seed/ACT는 미실행.

A/B scratch scripts, geometry/force witnesses, rawbench/fullstate/model 및 hash manifest는
기존 KIT local-validation/integration-task-20260916-aeg3vtmi/gripper-only-20260918에 보존한다.
Geometry SHA232f627c8cc2df5213655402a32e0e56b13f75aea6497ab81faf58a63c0c5899.
Bench raw SHA8f2bb0c519e292e97206e7465511a4928fb3ac82803f17bf871f3c9c5338ed12.
Public Git에는 최소 portable force fixture와 NumPy 검증기, 기존 학습 원장만 추가한다.
집중 회귀11 PASS와 git diff --check를 확인하고 normal push로 base=main PR65를 갱신한다. [skip ci]는 사용자가 요청한
milestone CI 주기 보존이며 main merge 허가가 아니다.

## DAPIER-2026-09-18-normal-close-support-hold

### Problem

이전50µm precompression은 정상 CLOSE 형성이나 안정 유지의 증거가 아니다.
이번 범위는 GRIPPER-ONLY BENCH이며 arm IK/teacher를 실행하지 않는다.
시작 HEAD fc3304785b96be864b2fab8bbe31ea90c8769147,
writer pro/grasp-families-codex-20260918. 기존 failure worktree는 읽기 전용으로 보존했다.

### Evidence

사용자가 승인한 TEST SUPPORT / NOT TASK ENVIRONMENT를 세 phase로 실행했다.
Formation: 정상 task-open2.202574638rad에서 기존 CLOSE generator/ctrl+mj_step 사용.
초기 finger contact/penetration0, block-support gap+1µm이며 reset 후 qpos 재배치 없음.
Compiled gripper/block/solver163항목 bitexact. Bench 전체를 회전하여 cube를 수평으로
놓았으므로 기존 precompressed 시험과 중력의 상대 방향이 다르다.
힘 차이를 순수 precompression A/B로 인과 해석하지 않는다.

받침은60×60×4mm frictionless condim1 plate와 수직 slide/시험용 actuator다.
Gripper actuator 변경이 아니다. Support 자동 collision mask0, explicit block pair만
사용하며 pad와의 독립 초기 수직 간격6.231775mm도 확인했다.
매 step vertical normal을 검사했고 최대 수평력1.59e-18N,
다른 block contact/solver warning0이다.

CLOSE stage49/SIM20.296s에서 bilateral 형성. Terminal command1.714220985rad,
Fn 약.503291/.503290N을 이후 고정했다. GRASP_CONFIRM50steps 뒤 받침만
.5s/-50mm physical slide로 철수했다. 받침 지지 중 Fn은 성공 근거에 포함하지 않는다.
Support contact count=0 AND Fn=0인 최초 force 평가20.426s부터 무지지 시간을 센다.
초기 OPEN의 무접촉 상태에서는 이 시계를 시작하지 않았다.

철수249/250step에서 이미49.536mm 떨어진 받침의6.022nm 반등으로 strict monotonic
fixture checker가 중단했다. Block에 전달된 contact/force/impulse는0이다.
원본 중단 trace를 보존하고 detached servo settling과 block에 가해지는 힘을 구분했다.
동일 full integration state를 bitexact 복원해 forward/rewind 없이 남은 기존 철수
sample과 HOLD를 이어갔다. 새로운 support contact/force/geometry overlap은 계속
거부하며 gain/command/timing을 바꾸지 않았다.

무지지1500steps×2ms=3s 관찰 완료. 마지막 force 평가23.426s, poststate23.428s.
Support count/Fn은 계속0, continuation 최소 독립 수직 gap47.117mm.
최종 Fn .503298/.503289N, jaw gap39.969460mm.
Reset 대비 하강2.901827mm, 동일 pre/pre 3s 무지지 구간 하강2.862446mm,
첫 pre에서 마지막 post까지3.002s의 하강2.864354mm를 구분했다.
종료 Vz=-.954146mm/s. Continuation 최대 penetration17.729µm, warnings0.
접촉력의 지속은 정지 평형이 아니다.

이전50µm도 보존된.1s fullstate에서 동일ctrl로3s까지 연장했다.
하강3.053025mm, 종료 Vz=-1.069531mm/s. 마지막1s Z 회귀-1.021770mm/s,
평균 Fn .256427/.235969N, jaw gap39.984548–39.985202mm.
접선 상대속도1.724985/1.792538mm/s인데 cone 사용률.400–.718/.423–.788로
포화가 아니다. 단순 μ 부족이나 jaw가 벌어지는 원인으로 단정하지 않는다.
모든 contact force/직접 torque/COM lever를 합산한 force residual6.30e-16N,
torque residual2.31e-17Nm. Force 평가 prestate와 postintegration,
free-joint angular qvel의 body-local 표현을 구분했다.

### Decision

분류C: 정상 CLOSE로 실제 접촉력이 형성되지만 현재 contact dynamics에서 slip이
지속된다. 무지지 retention의 제한된 근거이며 stable grasp milestone이 아니다.
Static feasible wrench와 실제 solver 평형, native boundary point와 overlap patch,
resultant line과 free couple을 구분한다. Friction/geometry/mass/solver/controller/
timing/limits/acceptance/safety/measured tolerance를 변경하지 않는다.

### Validation

Portable regression은 모든 contact의 force/COM torque/qacc/pre→post velocity를
재구성하고 contact 누락, COM lever 변조, support 잔류, fake zero velocity를 거부한다.
구간별 회귀 산술과 bilateral force 유지 중 지속 하강을 보존한다.
이는 관측 evidence regression이며 새 runtime success gate가 아니다.

### Result

GRIPPER-ONLY: 정상 CLOSE 형성과 무지지3s 관찰 완료, 안정성 FAIL.
이번 actual arm task 실행0회. 마지막 PASS=GRASP_CONFIRM, LIFT FAIL/SIM17.474s 보존,
CENTER SUCCESS=false. Milestone 미달로 full suites/render/remote CI/main merge 없음.
PR65 OPEN/DRAFT/base=main 유지. Skipped CI를 PASS로 세지 않는다.

### Lesson / Next

다음 blocker는 충분한 Fn과 지속 slip이 함께 존재하는 contact dynamics다.
Soft regularization, boundary representation, 미소 회전의 기여는 인과 분리되지 않았다.
Friction/preload 증가나 arm pose search로 돌아가지 않는다.
이전/현재 bench의 중력 상대 방향도 달라 직접 성능 A/B가 아니다.
단일 bench 증거로 실물/다중 초기조건 강건성을 주장하지 않는다.
기존 staging30.378087mm도30mm 대비.378087mm 단일조건 여유다.
Hardware/OS30A/multi-seed/ACT 미실행.
Raw/model/script는 기존 KIT validation/normal-close-20260918에 보존하고
public Git에는 최소 fixture/test/기존 학습 원장만 남긴다.

Bench SHA ab282e9deb11572e56e59f746e18a353e02b3e3ab8eb055224c49a5016cea8e3.
Stopped trace SHA40c583baab8ffec6923366b3fd85caa02817a86257ae8c1138be98bd6c8171fe.
Continuation SHA27ddf8577a4687c726d07045e85efe14db81a7f80224360f6c8358d476b8061d.

정상 CLOSE 독립 검증: 전체1501평가 source/pre/post/qacc 및 복원 일치.
마지막1s Z 회귀-.95414955mm/s, jaw39.96943–39.96955mm, cone 사용률.527–.539.
Instantaneous omega_z 약±.13934rad/s는2ms마다 부호가 바뀌며 yaw범위±.00806°;
순변화 약-1.98e-8°로 지속 한방향 회전과 다르다. Force residual1.20e-15N,
torque4.29e-17Nm. 최종 합력 Fz=.196199987N은중력과 거의 평형이지만
Vz는0이 아니므로 정지 grasp 증거가 아니다.

최종 focused13 PASS(4.219s): 기존 viewer/planner/wrench, static primal/dual,
precompressed 및 normal CLOSE의4실제 event/all-contact dynamics와 chronology.
최초 실행은 새 fixture의 support_Fn_N과 기존 support_force_N 명칭 차이로2error;
field를 증거 손실 없이 통일하여 해소했다. Runtime/safety 수정 없음.
명령: env DAPIER_SO101_MJCF=/tmp/dapier-pr62-pinned-assets/so101_new_calib.xml
기존 .venv/bin/python -m unittest discover -s test -p 'test_gr*e*.py' -v
(SIM 디렉터리). git diff --check PASS. PR65 draft 갱신, normal push, main merge 없음.

## DAPIER-2026-09-18-solver-contact-sensitivity

### Problem

나는 정상 CLOSE 후에도 무지지 HOLD에서 약.954mm/s slip이 남은 원인을
main solver 미수렴 / soft-friction regularization / 접촉 geometry로 분리했다.
시작 HEAD3a6ff44924ee79e633f351db3287a1b145d0cecd, 기존 writer branch/worktree를
그대로 사용했다. Runtime이나 teacher로 돌아가지 않고 세 prescribed copied case만 실행했다.

### Evidence

**VERIFIED BY PHYSICS — 동일 상태의3조건 비교**

이전 raw row10213 mjSTATE_INTEGRATION을 복원하고 다음 row10214의 실제 ctrl 전환을
적용했다. 최초 무지지 force 평가20.426s의 qpos/qvel/time/warmstart를 보존했다.
시작 state SHA62f8631095f4ee18018b85f075e44a647f93ac2e9e842cf680f0ee2db5d90dc3.
Restore 후 mj_forward 없이 실행했다. 남은 받침 철수 profile과 gripper terminal
command1.7142209851885606rad는 세 case 모두 동일하다.
Baseline1501평가 전체가 이전 raw와 bitexact, B도 A의 전체 state/qacc/contact wrench와
bitexact 일치했다. 각 case1500×2ms=3s이며 force prestate와 postintegration을 구분한다.

|항목|A baseline|B convergence|C NoSlip diagnostic|
|---|---:|---:|---:|
|main iterations|100|1000|100|
|main tolerance|1e-10|1e-12|1e-10|
|noslip_iterations|0|0|5|
|3s 수직 변위 mm|-2.862445924|-2.862445924|-.000472899|
|종료 Vz mm/s|-.954146418|-.954146418|-.00000692609|
|마지막1s Z 기울기 mm/s|-.954149554|-.954149554|-.00000692610|
|최종 Fn1/Fn2 N|.503298/.503289|동일|.521142/.521141|
|최대 penetration µm|17.729237|동일|18.853589|
|최대 cone 사용률|.538639|동일|.560622|
|support count/Fn, warning|0/0,0|0/0,0|0/0,0|

모든 case에서 bilateral contact, support 독립 양의 separation과 기존 penetration
bound를 유지했다. C의 Fn은최대.521142N, A는최대.507247N이며 증가량을 숨기지 않았다.
이3s에서 새 force-loss/비유한 상태/예상 외 contact/기존 bound 위반은 없었다.
d.solver_niter의 최대값은A/B1,C5이나 C통계를 main-only 반복횟수로 해석하지 않는다.
C의 native polygon margin은+.026/-.019nm로 boundary 표본이 유지된다.

**ANALYTIC / DIAGNOSTIC ONLY — 현재 solver/contact 계약**

아래 XML은 원래 task 입력MJCF가 아니라 MjSpec로 export한 isolated bench XML이다.
실제 physics는 SHA검증한 MJB를 로드했다. 생략된 XML 속성을 명시값으로 쓰지 않는다.

|항목|export XML/소스|실제 compiled|
|---|---|---|
|solver|XML 생략|Newton(enum2)|
|cone|elliptic; tabletop_replay.json 명시|elliptic(enum1)|
|integrator|implicitfast; builder 명시|implicitfast(enum3)|
|timestep|XML 생략|.002s|
|iterations|XML 생략|100|
|tolerance|XML/profile1e-10|1e-10|
|noslip_iterations|XML 생략|0|
|noslip_tolerance|XML 생략|1e-6|
|impratio|XML 생략|1|
|pad/block condim|4|4|
|geom friction|1.6 .02 .001|[1.6,.02,.001]|
|contact friction|geom에서 생성|[1.6,1.6,.02,.001,.001]|
|solref|-200000 -400|[-200000,-400]|
|contact solreffriction|pad-block explicit pair없음|[0,0]|
|solimp|export .95 .99; builder5값|[.95,.99,.001,.5,2]|

pgripper.py의 pad설정과 replay_recorded_episode.py의 builder,
tabletop_replay.json의 profile을 대조했다. 별도 freshdata mj_fwdPosition만으로
최초 contact geom/point/distance를 exact 재현해 actual contact parameter를 확인했다.
이 static probe는 physics step/force solve를 수행하거나 저장된 힘을 교체하지 않았다.
Full non-opt compiled array fingerprint:
1c9b054cb8113863c0923c089c01690c09f8650e11f7a8107524577f3f0f93b7.
세 case는 동일MJB/model에 opt허용필드만 변경했다. 기록된 물리 parameter hash는
매 case 검사했으며 full non-opt array hash 자체를매 step 수집한 것은 아니다.

MuJoCo3.3.7, Python3.12 기존 so101_imitation_learning/.venv 사용.
실제 loaded lib는 site-packages/mujoco/libmujoco.so.3.3.7,
SHA23841520ecf60ad648405674a757f260949f7cf6b44deb284398b9851b3f061d.
MJB SHAab282e9deb11572e56e59f746e18a353e02b3e3ab8eb055224c49a5016cea8e3.
전체 absolute library path와 XML/compiled snapshot은 로컬 archive에 보존했다.

공식3.3.7 XML reference에서 NoSlip은 main solver 이후 friction 차원의 slip/drift를
억제하는 후처리이고, tolerance는 early-stop 기준이지 속도 bound가 아님을 확인했다.
solreffriction[0,0]은 solref 재사용을 뜻하며 elliptic contact에 적용된다.
[MuJoCo3.3.7 option](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#option)
및 [contact pair](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#contact-pair).

### Decision

**INFERENCE — 사용자 분류A의 강한 근거.**
B는반복한도10배/tolerance100배강화해도 동일하고 C만slip이 크게 감소했다.
현재 조건에서는 단순main solver 미수렴보다 soft/regularized friction 처리가
주요 contributor라는 근거다. 이는 모든 수치오차가 없다는 전역 증명은 아니다.
C는normal force/penetration도 조금 바꾸므로 geometry·coupling의 잔여 영향을
완전히 배제하지 않는다. Native boundary point만으로 physical pad patch 부재를
주장하지 않는다.

이미 elliptic이므로 D는 생략했다. Impratio도 그대로다.
NoSlip은 DIAGNOSTIC COPY / NOT TASK SUCCESS이며 runtime 설정에 채택하지 않았다.
Friction/mass/geometry/gain/성공 threshold/joint limit/safety는 변경하지 않았다.

### Validation

Focused14 PASS(4.303s): 기존 viewer/planner/wrench/slip과 이번 parameter-isolation
회귀. 동일 state/model/command hash, 허용된 option delta만 존재, 3s구간의
변위=속도적분과 회귀 기울기를 확인했다. 시작state/command/model/impratio/support/
속도적분 변조를 거부한다. Saved-data verifier도 A의원본 replay, A/B fullstate와
wrench 동일, 모든case support0 및 기존bound 유지 PASS. 추가physics 재실행은 없다.
명령은 SIM에서 기존 venv Python과 pinned DAPIER_SO101_MJCF를 사용한
-m unittest discover -s test -p 'test_gr*e*.py' -v.
git diff --check PASS. 시간별 Z/Vz 및 log|Vz| 그래프 PNG/SVG를 저장·시각 검수했다.
사용자 범위에 따라 full local MuJoCo/Research/render suite/remote CI는 미실행.

### Result

세 copied diagnostic 완료. 분류A이며 runtime/model 채택이나 stable-grasp milestone은
아직 선언하지 않는다. 실제task는이번0회, 마지막 GRASP_CONFIRM PASS /
LIFT FAIL(SIM17.474s), CENTER SUCCESS=false를 보존했다.
PR65 draft/base=main 갱신, main merge 없음. SkippedCI는 PASS가 아니다.

### Lesson / Next

**UNVERIFIED:** NoSlip 조건에서 정상OPEN부터 CLOSE 형성 전체, 장시간/다른초기조건
안정성, 물리적 마찰/접촉 fidelity, 실제PGripper 대응은 확인하지 않았다.
현재는 baseline으로형성된 grasp의 동일 unsupported state에서 민감도만 비교했다.
후속 milestone에서 solver/model 후보의 물리 의미와 형성부터의 재현성을 검토해야 한다.
Arm IK/teacher/live/HW/OS30A/multi-seed/ACT로 이번결과를 확대하지 않는다.
Raw/script/compiledcontract/graph는 기존KIT validation/solver-contact-20260918에 보존한다.
Matrix SHA f14b0a9e7ed121ffe630fdcd1aaf97304cd2d222005708bdf60ec7ef68df8b8d.

## DAPIER-2026-09-18-contact-configuration-candidates

### Problem

NoSlip5가 이미 형성된 동일 unsupported state의 slip을 줄인 뒤,
정상 OPEN부터도 유효한 형성/철수를 거치는지(A), NoSlip 없이 impratio만으로
slow slip을 줄이는지(B)를 분리했다. Coordinator 단일 writer,
A/B scratch read-only-source 분석, 물리 실행은 A 다음 B 순서로 직렬화했다.
ACTIVE_WORKTREE=grasp-families-codex-20260918, 시작 HEAD
ea2e3f428da92dff9c240c9d90ef8cab3190f4e1 / clean.

### Evidence — VERIFIED BY PHYSICS

**A: NoSlip5 정상 formation.** 원래 OPEN fullstate와 command prefix를 정확히
재사용하고 noslip_iterations만 0→5. CLOSE49 / SIM20.296s에 bilateral:
Fn .514025/.513838N, terminal ctrl1.7142209851885606rad.
CONFIRM50step까지 완료했으나 WITHDRAW 첫step / SIM20.398s에 중단했다.
접촉 중 받침 실제 상향변위 +.210994µm, support contact3/Fn.227667N,
gap−8.805812µm. 받침/손가락/중력 합력의 Z는 +1.6054e−7N으로 작으나
접촉 중 받침 반등은 실제다. Detached6nm 사례처럼 취급하지 않았다.
unsupported 시작 전이므로 3s HOLD와 velocity convergence는 **UNVERIFIED**.
받침은 TEST SUPPORT / NOT TASK ENVIRONMENT다.

Matched-prefix Fn peak baseline .504753/.504754→.514083/.514829N
(+1.85/+2.00%). 최대 |dFn/dt| 6.960/6.903→27.187/17.590N/s.
최대 penetration18.739759µm, warning/unexpected cube contact0.
힘 peak 증가가 작다는 것과 변화율 증가를 모두 기록하며 임의 spike 허용치를
만들어 PASS라고 하지 않는다. Jaw final gap39.973042mm.
원시 q/qvel/힘/접촉/블록 pose는 그대로 보존했다.

**B: 동일 unsupported fullstate, NoSlip0, impratio만 변경.**

| impratio | 3s 하강 mm | 마지막1s Z slope mm/s | terminal Vz mm/s | final Fn pad1/pad2 N |
|---|---:|---:|---:|---|
| 1 기존 accepted replay | 2.862446 | −.954150 | −.954146 | .503298/.503289 |
| 10 | .286747 | −.095444 | −.095444 | .526790/.526789 |
| 100 | .031423 | −.010472 | −.010486 | .529672/.529598 |

두 새case 각각1501평가/3s, support count/force0, bilateral 지속, warnings0.
Slip은 약90.0%/98.9% 줄었으나 비영속도로 지속된다.
Fn peak .605743/.617772N: baseline 대비 약20%/22% 증가.
순힘 peak .001143→.006894/.007612N, max COM torque는 감소했다.
최대 penetration17.692111µm로 baseline17.729237µm보다 커지지 않았다.
Jaw는 약8.05/8.93µm 벌어진 뒤 후반 안정.
초기 warmstart 상태에서 impedance를 바꾼 과도응답을 포함한 결과다.

### Decision — INFERENCE / DIAGNOSTIC ONLY

Soft friction regularization이 주요 contributor라는 기존 분류A를 추가 지지한다.
impratio는 elliptic cone의 friction/normal constraint impedance 비율이며
마찰계수 자체가 아니다.
[MuJoCo3.3.7 공식 의미](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#option-impratio).

Runtime 후보 채택은 보류. NoSlip5는 후속 검증 우선 후보지만 정상 formation부터의
support withdrawal/unsupported HOLD가 미검증이다. A의 contacting support 반등을
무시하거나 block 순힘이 작다는 이유로 시험조건을 통과시키지 않았다.
Impratio10/100은 slip 감소와 과도응답의 tradeoff이며 stable equilibrium 증거가 아니다.
A는 normal OPEN부터, B는 baseline으로 이미 형성된 unsupported state부터 시작하므로
서로 동일 initial condition인 직접 A/B 비교라고 부르지 않는다.
Geometry/friction/mass/controller/timing/safety/runtime solver 설정은 보존했다.

### Validation

Focused16 PASS(4.345s). 기존14개와 formation-stop/parameter-isolation 회귀2개.
Fullstate 시작/명령 hash, option delta, displacement=velocity integral,
slope moments, residual slip, support 부재 및 잘못된 option/command/support/
zero-slip/integral 변조 rejection을 확인했다. Bilateral+support가 무지지 성공으로
승격되지 않도록 failing formation evidence를 보존했다.
Worker saved-only verification 둘 다 PASS(성공 trial을 뜻하지 않음).
B signed contact+tangential+torsional wrench와 dynamics 최대 잔차
force1.68e−15N / torque4.30e−17Nm. 비옵션 compiled fingerprint 전후 동일:
1c9b054cb8113863c0923c089c01690c09f8650e11f7a8107524577f3f0f93b7.
MuJoCo3.3.7/기존 Python3.12 venv, lib 및 model hash는 앞 기록과 동일.
git diff --check PASS. Full local suites/render/remote CI는 요청에 따라 미실행.

### Result

A의 정상 CLOSE formation은 확인했으나 전체 protocol은 WITHDRAW fail-closed.
B sensitivity 두 case 완료. Stable-grasp milestone 미달성, runtime adoption 없음.
실제 arm task 실행0회: 마지막 GRASP_CONFIRM PASS / 기존 LIFT FAIL SIM17.474s,
CENTER SUCCESS=false 보존. PR65 draft/base=main 갱신, main merge 없음.
원본/scripts/verifiers와 slip plot은 기존 KIT validation/contact-candidates-20260918 보존.
A rawSHA1c04d374ab8647d76b428e2ea836fa538a87ec57f9ce6e5b23cf22ef4e8dfad8;
B10 aa4db2d49805fca01219ca1357db0a1b25efe46118074e382c36756616b04f48;
B100 66aed4c3b879771eabefbf60fd74f5ecabba4f1b9c5b95fa95a87c394dd7ff91.

### Lesson / Next — UNVERIFIED

다음 blocker는 NoSlip 적용 시 TEST SUPPORT servo가 접촉 중 반등하는 원인과
시험 fixture의 무가압 하향 철수 재현성이다. Gripper/controller/runtime를 바꾸기 전에
fixture만의 정상적인 철수 절차를 검증해야 한다. 현재 checker를 완화하지 않았다.
그 뒤 정상 OPEN부터 완전한 unsupported HOLD를 확인해야 NoSlip 채택을 검토할 수 있다.
Impratio100의 남은 slip을 임의 tolerance로 안정이라고 선언하지 않는다.
Arm IK/live teacher/HW/OS30A/multi-seed/ACT는 실행하지 않았다.

## DAPIER-2026-09-18-prescribed-support-formation

### Problem

나는 NoSlip5 정상 CLOSE 이후 dynamic TEST SUPPORT의 반등 때문에 막힌
unsupported HOLD를 확인했다. 사용자가 fixture에 한해 prescribed mocap 하강을
허용했다. SIM writer는 기존 grasp-families worktree, REAL writer는 별도
real-sensor-pregrasp worktree로 분리했다. 이 기록은 SIM만 다룬다.

### Evidence — VERIFIED BY PHYSICS

받침의 dynamic Z joint/servo/mass만 제거하고 mocap boundary로 교체했다.
458개 named compiled 비교에서 gripper/block geometry, 질량·inertia,
friction, joint/actuator, passive coupling, contact pair, options가 동일했다.
Initial reset 뒤 block/gripper qpos 쓰기는 없고 actual ctrl+mj_step으로 CLOSE했다.
Fixture는 TEST SUPPORT / NOT TASK ENVIRONMENT다. Block/gripper를 mocap/weld로
부착하지 않았다.

받침은 .1m/s, 50mm/.5s 일정 하강했다. Command/actual Z 일치, upward step0.
처음에는 기존 septic withdrawal로 예비 진단했다. Baseline의 첫 count0 평가가
pre gap+.674µm/post−.936µm여서 중단됐고, 한 sample slope 요약 오류도 수정했다.
원본은 보존했다. 최종 비교는 사용자 요구인 constant velocity 두 case다.
Unsupported clock은 count0/Fn0와 양의 pre/post gap을 모두 확인한 뒤 시작했다.

| 항목 | NoSlip0 | NoSlip5 |
|---|---:|---:|
| 정상 OPEN 이후 첫 bilateral endpoint | CLOSE49 /20.296s | 동일 |
| terminal gripper ctrl rad | 1.7142209851885606 | 동일 |
| unsupported 시작 / 길이 |20.396s /3s|동일|
| 3s 수직 변위 mm |−2.861901215|−.000020820|
| 마지막1s slope mm/s |−.954149565|−.000006941|
| 최종 Fn pad1/pad2 N |.504725407/.504719811|.481440899/.481440654|
| 전체 Fn peak N |.506182033/.507246084|.481440899/.481440654|
| 최대 abs dFn/dt N/s |6.994468/6.936757|13.378062/17.203264|
| 무지지 순회전 rad |6.36529e−5|3.19518e−6|
| 최대 penetration µm |19.12|19.12|
| support contact/force, warning |0/0,0|0/0,0|

NoSlip5의 마지막1s 최대 선속도2.166e−8m/s, 각속도1.065e−6rad/s.
기존 dynamic-support baseline과의 차이도 확인했다: sag차이+.544709µm
(약.019%), tail slope는 사실상 같다. 첫 bilateral COM은23.948537µm
높아졌으나 첫 bilateral Fn 차이는 약1nN이다.
새 fixture가 baseline slow-slip 현상 자체를 없애지는 않았다.

### Decision — INFERENCE / ANALYTIC / DIAGNOSTIC ONLY

분류A: NoSlip5를 이 3초 gripper-only SIM 창에서 near-stationary contact-model
후보로 승격한다. Runtime/task 설정에 즉시 적용하지 않았다.
수직 속도가 수학적으로 정확히0은 아니며,20.82nm 잔여변위와 힘 변화율 증가도
그대로 기록한다. 힘 amplitude spike나 새 instability는 관측되지 않았지만
임의 force-derivative threshold를 추가하지 않는다.

분류D의 fixture 영향은 초기 높이/형성압력에 남는다. 따라서 기존 dynamic
fixture의 형성 이력과 이번 NoSlip을 동일-state 비교라고 부르지 않는다.
Native solver contact point는 여전히 pad polygon boundary 수준이다.
Physical overlap patch와 discrete point interior를 구분한다.
Mocap body의 native cvel0과 prescribed 위치의 finite-difference 속도는 다르다.
Frictionless vertical-only test boundary의 하강을 실제 물체 운동 모델로 일반화하지 않는다.

### Validation

Focused17 PASS(4.410s). 받침반등/접촉잔류/zero gap/비일정속도/
penetration초과/잘못된solveroption/command 변조를 거부하는 회귀를 추가했다.
Same model/initial state/ctrl, 옵션차이noslip5만, 변위=속도적분,
unsupported3s, support0 및 양의 separation 증거를 대조했다.
Saved-only verifier PASS;458compiled checks 및 per-step force/pose contract 보존.
Milestone 전체 검증 1회 완료: Research33 PASS, MuJoCo363 PASS(1122.395s),
canonical render 및 vision artifact 생성 PASS, verify-mujoco-headless exit0.
Canonical vision은 기존 tower baseline의 회귀이며 REAL calibration/현재 bench 성공과 구분한다.
반복 후보 full CI는 실행하지 않았다.

### Result

최종 gripper-only phase: UNSUPPORTED HOLD3s 완료.
SIM contact-model candidate이며 실제 arm task 성공이 아니다.
실제 arm은 재실행하지 않았고 기존 GRASP_CONFIRM PASS/LIFT FAIL,
CENTER SUCCESS=false/HOLD0s 상태를 보존했다.
Raw/scripts/compiled proof/이전 실패는 기존KIT validation 아래
prescribed-support-20260918에 보존한다.
Final raw SHA:
NoSlip0 6ece60726bd00ab45c794c6a841d6a6d9fef8827ae5f1edf45771603c1a8a2e7;
NoSlip5 698bc122438c23483dbe0c6537e291d5fa48f1ef82984665a58cdcb7ac315be6.
MODEL5af4835e39c7df7afaa4ca4669a27c2149214913bba22c88dfd53ffa78eb7e9c.

### Lesson / Next — UNVERIFIED

다음 SIM 단계는 copied arm-mounted grasp에서 contact-model 후보 검증이다.
긴 시간/다중 초기조건/실기 fidelity/pad sample interior는 아직 검증하지 않았다.
REAL perception/extrinsic/비접촉 PREGRASP는 별도 트랙이며 이 결과로 대체하지 않는다.


## DAPIER-2026-09-18-arm-noslip-copy

### Problem

Gripper-only NoSlip5 무지지 HOLD가 기존 arm-mounted 비대칭 grasp의 LIFT도
해결하는지 확인했다. 실제 arm task는 GRASP_CONFIRM PASS / LIFT FAIL이다.

### Evidence — VERIFIED BY PHYSICS / DIAGNOSTIC ONLY

보존 GRASP_CONFIRM의 mjSTATE_INTEGRATION 244값과 warmstart/terminal ctrl를
bitwise 복사했다. Portable model SHA 일치, compiled numeric fields/옵션은
noslip_iterations 0→5 외 동일하며 donor state/model은 불변이다.
기존 controller와 continuous-command septic LIFT의 50개 command도 baseline과 같다.
0.534920337s 궤적 중 50step/0.100s만 관찰했다.

첫 bilateral support loss는 NoSlip0 step36에서 NoSlip5 step33/66ms로 앞당겨졌다.
Step32에서만 table contact/force0 및 bottom gap +0.392161µm가 확인됐다.
다음 step33에는 table contact2/force0.168861N과 한쪽 finger loss가 발생했다.
이 한 sample의 분리는 지속 load-bearing LIFT가 아니다.
Step34–50은 실패 이후 bounded diagnostic extension이며 task PASS에 포함하지 않는다.
최종 finger Fn0/0.254030N, table Fn0.314623N, bottom lift−0.019765mm이다.
최대 normal-force 변화율127.015N/s, jaw1 cone utilization은 step32에서1.0이었다.
최소 protected clearance48.707677mm, 최대 penetration26.09µm, warnings0.

Force는 기존 helper의 ctrl→mj_step→integrated-pose mj_forward→contact gate
순서로 얻은 값이며 직전 integration impulse와 구분한다.
NoSlip0로 형성된 state에 NoSlip5를 적용한 첫 solve transient도 포함한다.
종료 TCP error5.039mm는 진행 중인 +5mm 최종 목표까지 거리이며
완료 endpoint acceptance 실패로 바꾸어 해석하지 않는다.

### Decision — INFERENCE

NoSlip이 gripper-only slow slip을 줄인 사실을 arm grasp 안정성으로 일반화할 수 없다.
현재 비대칭 contact placement/torque 수요를 유지한 상태에서는 지속 지지가 없었다.
NoSlip5 runtime 채택, full-path preflight와 live 재실행은 진행하지 않는다.
Geometry/friction/mass/controller/timing/joint limits/acceptance/safety는 유지했다.

### Validation

저장-only 독립 verifier PASS: fullstate/ctrl/options/donor/50commands/상태연속성.
새 LIFT negative fixture와 기존 gripper/contact/solver focused8 PASS(0.050s).
가짜 force/support/command/success 및30mm미달 기록을 거부한다.
이번 arm milestone은 미달이므로 fullsuite/새remoteCI는 실행하지 않는다.
지난 gripper-only milestone의 Research33/MuJoCo363/canonical 및remote checks는
별개로 PASS하여 PR65를 main c44d2c9dafc3cccaa088a33436750c4ff3c4bb56에 병합했다.

### Result

최종 copied phase LIFT_5MM FAIL(step33). 실제 live 상태는 변경하지 않았다.
Actual last PASS GRASP_CONFIRM, CENTER SUCCESS=false, HOLD0s.
Raw SHA187f3264af3dbec8db48dce64f5f5b9e30487ddb118121b9937b544bcb1b18df.
재현: 보존 KIT local-validation의 arm-noslip-20260918/run.py와 exact input;
저장 검증은 같은 폴더 verify.py. 공개 fixture는 기존 lift_transition.json에 추가했다.

### Lesson / Next — UNVERIFIED

한 sample의 table force0와 큰 finger force는 지속적인 무지지 grasp의 충분조건이 아니다.
다음 SIM blocker는 arm이 만들 수 있는 contact geometry와 load-bearing 안정성이다.
새 pose search나 physics 설정 변경은 이번 결과 없이 자동 확대하지 않았다.
REAL calibration 결과는 이 SIM 결과와 독립적으로 기록한다.

## DAPIER-2026-09-18-arm-coupling-comparison

### Problem
단독 gripper NoSlip5의 무지지 성공을 팔 장착 grasp로 일반화할 수 있는지,
팔의 움직임과 처음부터 다른 접촉 배치를 분리했다. 실제 task는 GRASP_CONFIRM
PASS / LIFT FAIL이고 새 live 실행은 하지 않았다.

### Evidence — VERIFIED BY PHYSICS / DIAGNOSTIC ONLY
동일한 보존 GRASP_CONFIRM full-state에서 terminal command HOLD50과 왼팔5관절
SOFT_LOCK HOLD50, 총100step/0.2s를 복제 실행했다. 원래qpos/qvel/warmstart/ctrl를
보존했고 per-step qpos reset은 없다. 29option/38physical arrays/기존4equalities가
같으며 신규5 eq_active만 추가했다. SOFT_LOCK은 유한 강성이고 강체 고정이 아니다.
최대 관절 변위2.76939e-7rad, 속도2.48973e-5rad/s이다.

HOLD 최종 Fn .080839/.075044N, table .197998N; SOFT_LOCK은
.081746/.074822N, table .196242N이다. 둘 다 양쪽 접촉을 유지하지만 table이
중량 .1962N을 계속 받친다. 성공한 정상 OPEN→CLOSE→prescribed TEST SUPPORT
철수→무지지3s bench는 Fn .481446/.481445N, 패드 수직합력 .1962N,
COM torque 약3.31e-15Nm, support0이다. 이전 preformed identical-state bench와 구분했다.

공통 block-COM frame 접촉 높이 차이: arm31.054978mm vs bench .017130mm.
Arm 접촉 법선은 완전 대향에서9.251도 어긋난다. Gripper terminal command도
arm2.043112221rad / bench1.714220985rad로 다르며 jaw opening47.385/39.967mm를
같은 압축량으로 비교할 수 없다. Bench는 NoSlip5로 형성했고 arm은 NoSlip0으로
형성한 상태에 NoSlip5를 적용했다. Friction/condim은 같지만 통제된 arm-only A/B는 아니다.

기존 NoSlip LIFT50은 원본hash/시작state/command 동일성을 확인해 재사용했다.
Step32 한 관측단계 table force0에서 pad Fz .172801N<weight .1962N,
step33 첫 bilateral support loss, step37 양쪽 기하접촉 상실이다.
Step50 남은 pad Fz -.060550N/table .314623N. 0.534920s 궤적의 .1s 진단이며
endpoint 도달 실패나 새로운 actual execution으로 표시하지 않는다.

### Decision — INFERENCE
가장 강한 차이는 형성 이력/대각 접촉/낮은 preload와 lift 중 wrench 붕괴다.
팔 compliance 단독 원인으로 확정하지 않는다. SOFT_LOCK은 HOLD에만 적용했으며
locked-LIFT는 미검증이다. NoSlip runtime 채택이나 friction/mass/geometry/controller/
timing/limits/acceptance/safety 변경을 하지 않았다.

### Validation — VERIFIED BY REGRESSION
저장-only verifier PASS: common full-state/모든 option/38arrays/원래 equality,
100행 모든 pad force와 COM moment를 contact frame에서 독립 재합산했다.
추가 mj_step 없음. 기존 lift_transition fixture에 대표1/25/50step과 비교값/원본hash를
추가해 명령 불변, table support 잔류, wrench 합산, 다른 접촉배치 의미를 회귀로 보존했다.
Focused9 PASS; fullsuite/remoteCI는 milestone 미달로 실행하지 않았다.
MuJoCo 한 창은 RECORDED STATE PLAYBACK / NOT NEW PHYSICS라고 표시하고
1:HOLD/2:SOFT_LOCK/3:LIFT를 비교한다. 표시용 data만 qpos를 복원한다.

### Result
Actual last PASS=GRASP_CONFIRM, LIFT FAIL, CENTER SUCCESS=false/HOLD0s 유지.
Copied HOLD retention은 unsupported HOLD 성공이 아니다. Raw SHA
189a8bee856647497ccab55443022cfb16a7a4a7a9b259900585decbdc70a1de.
재현은 기존 KIT local-validation/arm-coupling-20260918의 run.py;
저장 검증 verify.py, 표시 view_saved.py. 원본 모델/상태/manifest도 함께 보존한다.
Focused 명령은 test 디렉터리에서 python -m unittest
 test_lift_transition.LiftTransitionTest.test_arm_hold_retention_is_not_unsupported_grasp
 test_lift_transition.LiftTransitionTest.test_arm_noslip_copy_does_not_establish_load_bearing
 test_gripper_force_requirements -v 이다.

### Lesson / Next — UNVERIFIED
같은 solver라도 접촉 형성과 자세가 다르면 arm coupling 실험이 아니다.
Bench는 pre-integration force, arm은 integrated-pose mj_forward force여서 과도응답의
직접 인과 비교에도 한계가 있다. 다음은 검증된 접촉 배치/형성 이력을 맞춘 통제 비교다.
현재 기준을 완화하거나 추가 CLOSE/NoSlip sweep으로 성공을 만들지 않는다.
REAL extrinsic과 이 결과를 섞지 않고 PR67 draft를 유지한다.

## 2026-09-18 — 성공 상대 grasp를 팔에 매핑하고 기본 Z 동작 분리

record_id: DAPIER-2026-09-18-relative-grasp-basic-motion

### Problem

성공한 gripper-only 접촉과 arm-mounted 접촉이 다른 이유를 먼저 확인했다. 어려운 A–D 실측은 중단하고, camera extrinsic과 독립적인 LEFT +20 mm/정지/복귀 SIM 리허설을 준비했다. 실제 motor command는 보내지 않았다.

### Evidence

**ANALYTIC / DIAGNOSTIC:** 정상 형성 후 무지지 HOLD 성공 bench의 `T_block_housing`과 `T_block_TCP`를 원래 pre-step qpos에서 추출했다. `T_A_B`는 B 좌표를 A로 변환한다(m/rad). TCP translation은 block 기준 약 `(0, 0.000138, -1.257583) mm`, TCP approach는 거의 block −Z, closing은 +X다. Bench contact height 차이는 .017130 mm, Fn .481446/.481445 N, support0이다. 하우징/site local transform은 팔과 exact 동일했다.

기존 위치의 16개 제한된 seed/yaw 후보와 analytic wrist-axis 보정 6개를 평가했다. 선택 nominal 후보는 position .158510 mm, approach .210398°, closing .116392°이며 joint limits와 open 접근/CLOSE interpolation을 통과했다. 더 엄격한 별도 정밀 IK 자체는 .05° 목표에 수렴하지 않았으며, 기존 .5mm/2°/15° 수용과 구분했다. 전역 최소/불가능을 주장하지 않는다. Native bench-jaw state는 한쪽 접촉이므로 IK PASS를 양측 grasp로 간주하지 않았다.

**VERIFIED BY PHYSICS / COPIED DIAGNOSTIC:** copied pregrasp 초기 배치 후 runtime teleport 없이 기존 ctrl/step 경로로 APPROACH→CLOSE→CONFIRM을 실행했다. HOME→staging은 이번 candidate에서 검증하지 않았다. Contact 형성 뒤 실제 높이차는 **25.158028 mm**, rigidly carried bench contact 예측은 **.178175 mm**였다. CONFIRM Fn .102269/.116904 N, table .199456 N, pad vertical resultant −.003260 N. 거의 맞는 TCP pose가 동일 native contact manifold/하중을 재현하지 못했다.

기존 LIFT 첫 명령의 measured reseeding은 최대 .000616214 rad의 점프를 만들었다. 같은 CONFIRM full-state/같은 target에서 직전 verified ctrl로 정확히 연속 시작하면 첫 스텝 양측 접촉이 유지되지만 **step40 / 80ms**에 한쪽 normal force가0이 되어 기존 gate로 정지했다. Table 이탈은 step38–39 두 표본(.862/.696µm gap)에 불과했고 step40 재접촉했다. TCP 상승 .027776mm, 5mm endpoint 미도달, 무지지 HOLD 없음.

**한계:** bench 형성 NoSlip5 vs arm 형성/runtime0은 혼재 조건이다. 이번에는 값을 바꾸지 않았다. 작은 pose 오차와 contact feature 선택/형성 이력이 원인 후보이며 단일 solver bug나 arm compliance로 확정하지 않는다. Native representative contact 높이차와 전체 usable surface overlap은 같은 지표가 아니다. 원본 duplicated-preflight의 custom 상세 필드4종은 잘못된 donor capture라 제외; main/continuation 실제 state와 기존 core clone/final state만 사용했다. 3226 main+6947 continued+40 continuous actual copied steps, 별도 중복 preflight2828step을 구분해 보존했다.

### Decision

Teacher grasp/runtime, NoSlip, friction, mass, geometry, controller, timing, limits, safety policy를 바꾸지 않았다. 성공 relative transform의 FK 수용은 가능하지만 실제 접촉 재현은 미완료다. Grasp 후보를 live task에 채택하지 않는다.

기본 동작은 `basic_vertical_motion.py`에서 기존 integration scene/reset/settle, corrected axis IK, septic generator, `env.apply_action` 및 SAFE_STAGE 일반 30mm guard를 재사용한다. Measured q는 path/telemetry 시작값, 미조작 gripper/반대팔은 explicit ctrl를 유지한다. 새 hardware dispatch 경로는 없다.

### Validation

**VERIFIED BY PHYSICS:** 첫 +20mm 실행은 endpoint .630642mm로 거부했다. 계획 오차 .415475mm와 command FK→measured .346596mm를 분리했다. Acceptance .5mm는 유지하고 planner 내부 position tolerance만 `.5−.346596=.153404mm`로 설정했다. 단일 관측 기반 reserve이며 robustness bound가 아니다. Controller/속도/geometry는 그대로다.

재실행은 SETTLE100step/.2s → 상승514step/1.028s → 비접촉 HOLD250step/.5s → 복귀514step/1.028s, 총1378step. 실제 상승19.600463mm, endpoint .400950mm/.251168°, HOLD .400876mm, 복귀 .000792mm/.000105°. General min72.400006mm, max command tracking .001003409rad. Joint/command/near-support/runtime clearance 검사 PASS. 양 gripper/오른팔 command 불변, hardware0, runtime qpos 직접 쓰기0(소스 계약). Full dynamics robustness나 실물 profile 검증으로 확대하지 않는다.

**VERIFIED BY REGRESSION:** 기본 계획·저장 physics2 PASS, 접촉/LIFT/force focused10 PASS, 기존 REAL transform3 PASS. 원본 transform/FK22후보 및 복사-state chronology/hash/force 검증을 보존했다. Negative collision policy7 PASS/3.249s, near-support 음성2 PASS와 compact fixture 재검사1 PASS/3.128s, 양 worktree diffcheck PASS. 전체1378step max tracking이 기존 final .02rad보다 작음도 저장 trace에서 확인했다. Fullsuite/remoteCI 미실행.

### Result

실제 arm task는 여전히 GRASP_CONFIRM PASS/LIFT FAIL, CENTER SUCCESS=false/HOLD0s. Copied candidate의 continuous LIFT40step 실패를 분리했다. 기본 비접촉 SIM 상승/hold/return은 PASS이며 SIM 모델 시작 자세에서의 결과다.

SIM LEFT arm 순서 pan/lift/elbow/wrist-flex/roll:
- start/return q(rad): `[0, 0, 0, 0, 0]`
- target q(rad): `[0.00001045348, 0.01949969385, -0.16493163968, 0.14095875590, -0.00563163025]`
- delta(deg): `[0.0005989405, 1.11725016, -9.44988686, 8.07634180, -0.32266865]`
- SIM gripper command2.2028rad 불변. 이를 실물 gripper0..100에 복사하지 않는다.
- continuous septic nominal1.026733576s, timestep sampling1.028s/leg, hold.5s.

재실행(SIM 폴더, 기존 venv):
```bash
python basic_vertical_motion.py --viewer --report /tmp/dapier-basic-vertical.json
```
해당 Python은 기존 MuJoCo venv를 뜻한다. 현재 검증 asset은 기존 pinned SO101 source이다. 정확한 raw/report/script는 기존 local-validation KIT에 보존한다. Viewer는 실제 동일 model/data를 관찰했고, 별도 접촉 비교 창은 RECORDED PHYSICS / NOT NEW EXECUTION으로 표시했다.

### Lesson / Next

거의 같은 rigid pose와 source contact를 운반한 점은 native solver contact의 동일성을 보장하지 않는다. 다음은 접촉면/형성 이력을 통제한 원인 분리이며 이번 결과로 성공이라고 쓰지 않는다.

REAL extrinsic은 UNVERIFIED / BLOCKED BY DATUM METHOD. 새 측정 없이 원값 보존. BASIC에는 카메라가 필요 없지만 현재 실물 q 및 SIM↔REAL joint zero/sign/pose 대응은 아직 없다. 월요일 fresh readback→해당 실제 pose 기준 SIM 매핑/경로 검증→LEFT q/profile 제시→사용자 승인 순서다. 과거 start-pose나 위 SIM 절대 q를 현재 실물 q로 사용하지 않는다.
