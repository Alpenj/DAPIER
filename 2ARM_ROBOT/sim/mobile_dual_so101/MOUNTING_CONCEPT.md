# TurtleBot3 Waffle Pi + dual SO-101 printed mounting concept

이 문서는 3D 프린터로 제작 가능한 구조의 설계 기준이다. MuJoCo 형상과 원본
mass/inertia를 이용한 1차 강체 정역학·동역학 결과를 포함하지만, 실제 재료 시험이나
FEA를 대체하지 않는다. 구멍 위치, 출력 재료, 출력 방향과 실측 질량을 받기 전에는
제작 도면으로 확정하지 않는다.

MuJoCo의 회색/파란색 mount geom은 구조를 이해하기 위한 collision-disabled 형상이며
현재 합계 0.90 kg의 가정 질량은 포함한다. 형상을 그대로 solid 출력하라는 뜻은 아니다.

## 형상 기준과 변경 이유

- arm mount Z: 0.30 m
- 좌우 mount center 간격: 0.20 m
- TurtleBot3 LiDAR: 제거
- TurtleBot3 기존 소형 RGB camera_link: 제거
- top-view RGB-D: eYs3D R77 공식 URDF 기준 25.5 x 90 x 25 mm, 0.096 kg,
  tower 중앙 (-0.064, 0, 0.550) m, 아래 27도
- front Visual-SLAM RGB-D: Astra S 40 x 165 x 48 mm, 0.310 kg,
  TurtleBot3 전면 카메라 고정 프레임, optical center (0.076, 0, 0.093) m,
  body center (0.055, 0, 0.093) m, 정면 수평
- 출력 구조 가정 질량: 0.90 kg
- Waffle 상판 local Z: 0.094 m
- 8 mm deck 하단 local Z: 0.094 m (의도한 접촉, 공중 간격 없음)
- Waffle Pi nominal 크기: 281 x 306 x 141 mm
- SO-101 base AABB: 약 111 x 72 x 96 mm
- 양팔 base 전체 Y 폭: 약 339 mm
- Waffle 외곽 돌출: 약 16--17 mm/side
- 현재 자세의 gripperframe 바닥 여유: 약 59 mm

기존 0.30 m center 간격은 양팔 base 폭이 약 439 mm가 되어 Waffle 양쪽으로 약
67 mm씩 캔틸레버가 생긴다. 금속 프레임 없이 FDM 출력물만 쓸 때는 불리하므로 center
간격을 0.20 m로 줄였다. 팔은 계속 좌우 바깥을 향하지만 하중 작용선은 Waffle 지지면에
가까워진다.

## 출력 구조

```text
  left SO-101 vertical base face       right SO-101 vertical base face
              || M4                         M4 ||
       [left vertical wall]--------- [right vertical wall]
       | front/rear panels + top panel = closed torsion box |
       | internal gussets at wall/deck junctions             |
       [left 8 mm deck]--bolted seam--[right 8 mm deck]
       ================= Waffle top Z=0.094 m ================
           multiple existing Waffle M3 holes/support columns
       short front pads -> RGB-D optical center x=120, z=200 mm
```

1. SO-101의 `base_so101_v2`와 shoulder motor holder는 유지한다.
2. 전체 팔을 90도 회전했으므로 SO-101 원본 바닥 체결면도 수직이다. 이 면을 중앙
   torsion box의 좌우 10 mm 수직판 외측면에 M4 through-bolt, 큰 washer, lock nut로
   직접 잡는다. 팔 아래에 수평 선반을 만들지 않는다.
3. 좌우 수직판을 front/rear panel과 top panel로 닫아 한 팔만 움직일 때 생기는
   비대칭 비틀림을 반대편 벽과 deck으로 전달한다. 카메라 외함은 구조 tie로 쓰지 않는다.
4. 세로 몸통 한 덩어리를 세워 출력하지 않는다. 주요 y-z 측판과 x-z 측판을 베드에
   눕혀 출력한 뒤 M3 bolt와 captive nut로 box를 조립한다. 이렇게 해야 주 굽힘응력이
   약한 Z 적층 접착면을 직접 뜯는 방향으로 작용하지 않는다.
5. 하부 deck은 좌우 두 장으로 분할해 출력 베드에 들어가게 하고 중앙
   tongue/overlap과 through-bolt로 연결한다. deck 하단은 Waffle 상판에 면접촉하고,
   압축 가능한 spacer로 37 mm 같은 공중 간격을 만들지 않는다.
6. 수직판과 deck 접합부에는 전후 triangular rib/gusset를 둔다. 하중 경로는
   SO-101 수직 base flange -> side wall/rib -> deck -> 여러 Waffle M3 체결점 ->
   chassis support다.
7. Raspberry Pi, OpenCR, self-tapping screw와 케이블 가이드는 구조 하중
   경로로 사용하지 않는다.
8. RGB-D는 torsion box 전면에 짧은 좌우 pad로 체결하며 카메라 외함을 양팔 구조
   tie로 사용하지 않는다.

LiDAR를 제거한 중앙 공간은 닫힌 몸통의 구조 단면과 케이블 통로로 사용한다.

## 1차 정역학·동역학 결과

2026-08-26 재검증 모델은 Waffle와 양팔에 0.90 kg 출력 구조 가정 및 0.310 kg
카메라를 포함해 총 3.914 kg이다. 원본 URDF의 완성품 질량·실제 출력량과 다르므로
최종 계산은 실측 질량으로 교체해야 한다.

Waffle 원본 지면 접점은 x=0의 구동륜과 x=-0.177 m 부근 후방 caster다. 상판
281 x 306 mm를 직사각 지지면으로 사용하는 계산은 틀리다. 반대로 URDF의 단순 접점만
사용한 계산도 실물 배터리와 바닥 접촉을 반영하지 못한다. 따라서 외곽 caster를 억지로
추가하지 않고 다음 운용 제한을 둔다.

- 주행 중 팔은 작고 낮은 transport pose로 수납한다.
- 팔이 transport envelope 밖으로 나가기 전에 base 속도 명령을 0으로 만든다.
- 양팔 작업 중에는 base 정지와 brake/command interlock을 유지한다.
- 실제 팔·카메라·배터리·출력물 질량과 실물 접점을 측정한 후에만 전도 여유를 확정한다.

`design_validation.py`는 가정이 위험한 자세를 찾는 도구이지 현재 구조의 안전
인증서가 아니다.

## 설계 하중과 운용 결론

실제 신발 질량을 재기 전에는 0.5 kg nominal, 1.0 kg proof case를 사용한다.

- 34,097자세에서 arm 자중과 1 kg gripper payload를 합친 최대 정적 mount moment는
  약 6.65 N·m/arm이다.
- 관절 가감속, backlash 충격과 급정지를 위한 dynamic factor 2.5를 적용하면
  약 16.64 N·m/arm이다.
- printed torsion-box mount는 working moment 17 N·m/arm 이상, 손상 없는 proof
  moment 35 N·m/arm을 목표로 한다.
- 양팔이 같은 전후 방향 moment를 만들 수 있으므로 전체 deck/chassis interface는
  60 N·m proof case를 별도로 확인한다.

이는 재료 허용응력 계산이 끝났다는 뜻이 아니다. FDM 구조의 실제 한계는 bulk
filament 강도보다 layer adhesion, bolt-hole bearing, creep, seam과 nut pocket에서 먼저
나올 가능성이 크다.

권장 운용은 다음과 같다.

- TurtleBot 이동 중: 양팔을 낮고 중앙에 가까운 transport pose로 둔다.
- 측면으로 크게 뻗어 신발을 드는 동안: base 정지.
- 0.5 kg 이상을 측면 최대 reach에서 들고 base도 움직여야 한다면: 저상부 ballast만
  믿지 말고 해당 동작을 금지하고 base를 정지한다.
- 배터리는 가능한 낮고 중앙에 둔다. 1--1.5 kg급 저상부 질량은 0.5 kg 편심 하중의
  여유를 크게 늘리지만, 1 kg 최대 측면 하중까지 무조건 안전하게 만들지는 않는다.

## 충돌 방지 결론

팔을 물리적으로 벌려도 전체 joint range는 collision-free가 아니다. 양팔 동시 무작위
2,000자세 중 265자세가 좌우 팔 또는 전면 카메라에 대한 30 mm 보호거리
미만이었다. 따라서 `collision_guard.py`는 현재 자세부터 목표 자세까지 기본 2도
간격으로 보간하고 모든 좌우 collision geom pair와 카메라 거리를 검사한다.
하나라도 30 mm 미만이면 목표를 거부한다. 속도가 생기는 실물에서는 이 값에
braking-distance margin을 추가해야 한다.

## 출력 재료와 제작 방향

- PLA: 치수/조립 mock-up과 짧은 저속 시험용. 열과 지속하중 creep 때문에 최종 반복
  운용 구조로 확정하지 않는다.
- PETG 또는 ASA: 일반 장비에서 가능한 1차 기능 시제품 후보. 두꺼운 wall, 많은
  perimeter와 through-bolt를 우선하고 infill percentage만 높여 해결하지 않는다.
- PA-CF/유사 강화재: 프린터와 건조 조건이 지원될 때 최종 후보. 강화재라도 적층 방향과
  bolt-hole 국부응력 문제는 남는다.

최종 slicer 설정은 보유 프린터, nozzle, filament를 확인한 뒤 정한다. 주요 측판은
평면으로 눕혀 출력하고, bolt hole 주변은 solid modifier/perimeter를 늘리며, heat-set
insert 하나에 굽힘하중을 맡기지 않는다.

## shoulder/base 부품을 바꾸는 조건

기본 결정은 **교체하지 않음**이다. 원본 base는 첫 모터 정렬, 케이블 경로와 정비
기준을 담당한다. 90도 설치에서 공구 접근이나 cable bend가 불가능하다는 것이 실물
mock-up으로 확인될 때만 `base_so101_v2` 아래 interface shoe를 파생 설계한다.
shoulder motor holder 전체 교체는 마지막 선택이다.

## CAD와 최종 검증 전에 필요한 값

- 프린터 모델, bed 크기, nozzle, 사용 가능한 filament와 건조 가능 여부
- Waffle에서 사용 가능한 M3 구멍 중심 좌표와 하부 nut 접근성
- SO-101 base 4개 구멍의 X/Y 간격, 지름, 볼트 길이, 케이블 돌출
- 각 arm, 신발, battery, printed mount의 실측 질량과 COM
- 실제 arm center 간격과 허용되는 전체 폭

최종 검증은 static proof load, 1시간 creep 관찰, 반복 하중, 한 팔 비대칭 전개,
저속 base 가감속, emergency stop, modal/진동 관찰 순서로 한다. 균열·백화·bolt 풀림과
layer separation이 보이면 다음 단계로 넘어가지 않는다.

## 참고 자료

- ROBOTIS TurtleBot3 specifications/open hardware:
  https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
- eYs3D R77 official URDF, collision envelope, mass and mounting frames:
  https://github.com/eYs3D/eys3d-ros2/blob/ros2-master/eys3d_camera/urdf/eys3d_R77.urdf.xacro
- Orbbec Astra series dimensions, mass and field of view:
  https://www.orbbec.com/products/structured-light-camera/astra-series/
- SO-101 assembly:
  https://huggingface.co/docs/lerobot/main/en/assemble_so101
- SO-ARM base interface example:
  https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/4040_Base_Mount
