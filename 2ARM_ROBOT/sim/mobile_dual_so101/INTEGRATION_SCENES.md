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

`shoe_task.py`의 block profile, reset/SETTLE, seeded XY/yaw,15 Hz/parallel rollout,
`center_block_teacher.py`, `grasp_debug.py`는 **이전 tower 장면용 regression**이다.
새 integration scene에 연결된 teacher가 아니다. 이전 HOME·workspace·IK·clearance·성공률을
새 장면 검증 근거로 재사용하지 않는다. 새 장면은 geometry→HOME→collision→workspace→camera→
seeded reset→IK→grasp→teacher 순서로 별도 검증해야 한다.
