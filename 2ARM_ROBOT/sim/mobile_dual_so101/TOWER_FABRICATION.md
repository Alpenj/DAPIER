# 분할 `assem_base` 지지대 제작 입력과 출력 패키지

현재 MuJoCo visual은 조원이 제공한 하부·상부 STL을 수정 없이 사용한다. upper 좌우
socket이 stock SO-101의 `base_so101_v2`만 대체하고, base motor holder, Waveshare
mounting plate, base servo와 shoulder 이후 관절 체인을 socket 축에 끼워 체결하는 구조다.
하부는 Z=0~170 mm, 상부는 Z=160~316 mm에 놓여 10 mm가 중첩된다. 단순 box는
보이지 않는 collision proxy에만 남겼다. 직접 실행한 simulation에서 결합 접촉면,
bare depth camera와 1,000-step finite state를 확인했다. 다만 실제 체결 강도, 출력
공차, 카메라 고정, 재료와 질량은 아직 실측하지 않았으므로 실물 안전이 확인됐다고
쓰지 않는다.

upper는 `base_so101_v2` 역할만 대체한다. `base_motor_holder_so101_v1`,
`waveshare_mounting_plate_so101_v2`, 검은 base servo와 `shoulder_pan` 이후 체인은
유지한다. 따라서 같은 위치에 stock `base_so101_v2`만 다시 조립하지 않는다.

## 확보된 입력

- Waffle `base_link`: +X 전방, +Y 좌측, +Z 위
- 실제 top plate plane: Z=91.5 mm
- 단순 URDF collision proxy top: Z=94 mm
- Waffle plate 단품: 128 x 64 x 9 mm
- deck 체결 중심 6개와 좌표 변환: `WAFFLE_COORDINATE_REFERENCE.md`
- `assem_base.step` SHA-256:
  `f9f77f71a77f962aac3c7a3898bf5df7232b12982be3f16e3e2fc20d39c1bb3b`
- lower STL: 160 x 180 x 170 mm,
  SHA-256 `f91c58b14bd9932757787d9fea1f72104bae537d38b003d2a430576fecc70076`
- source upper STL: 110.963 x 254 x 156 mm,
  SHA-256 `ed6218f3ba83459fc7c436416ef708130e2622a27df6772d4d62bf6a9d822bf7`
- active upper: 110.963 x 254 x 156 mm, source STL 그대로 사용
- assembly overlap: Z=160~170 mm, 10 mm
- bottom hole: `(+/-77, +/-87)` mm, radius 3 mm
- upper socket axis: `(0, +/-127, 302.550896)` mm, radius 8.5 mm
- Waffle 기준 socket axis: `(-64, +/-127, 394.050896)` mm
- SO-101 arm frame: `(-64, +/-93.4, 387.686186)` mm
- camera center: `(-64, 0, 550)` mm, 27도 하향
- 프린터: Creality K1 Max, build volume 300 x 300 x 300 mm
- filament: Creality Hyper PLA Black, 1.75 mm, 1 kg
- K1 Max 출고 사양 nozzle: 0.4 mm, 0.6/0.8 mm 호환

마지막 항목은 출고 사양일 뿐 현재 장착 nozzle 확인을 대신하지 않는다. 최종 G-code는
실제 nozzle과 slicer/version을 확인한 뒤에만 만든다.

## 최종 제공할 파일

현재 상·하부 STL과 조립 STEP을 하나의 revision으로 보존한다. 추가 실측값이 들어오면
다음 파일을 같은 revision으로 생성한다.

- 파라메트릭 CAD 원본과 중립 교환용 STEP
- 제공된 lower/upper STL과 필요한 Waffle adapter plate
- 조립도, fastener BOM, 출력 방향과 support 표시
- fit-check용 저재료 STL
- 사용 프린터와 slicer profile에 고정된 G-code
- CAD/STL/G-code의 SHA-256과 생성 조건

STL은 mesh라 치수 수정 원본으로 사용하지 않는다. G-code는 프린터 운동 한계,
nozzle, filament, bed와 start/end sequence에 종속되므로 범용 파일로 만들지 않는다.

## CAD 전에 필요한 실측값

단위는 mm로 기록하고 사진 한 장만으로 추정하지 않는다.

1. 선택한 Waffle M3 중심 6개의 하부 nut 접근 공간과 기존 support 간섭
2. 상판 외곽, wheel/caster, Raspberry Pi/OpenCR, connector와 cable keep-out envelope
3. upper socket에 base servo를 넣는 순서, bolt·shaft 접근 방향과 service clearance
4. 두 팔의 실측 질량, base 기준 COM, 최대 작업 payload
5. depth camera 실제 폭/높이/깊이, bottom/back hole, optical origin과 cable bend radius
6. upper socket 중심 높이 302.550896 mm·간격 254 mm의 허용오차와 전체 통과 폭
7. 프린터 모델, build volume, nozzle 지름, 사용 filament, 건조 가능 여부
8. slicer 이름/버전, layer height, wall/perimeter, infill, support와 bed adhesion profile

## 출력 순서

1. Waffle adapter hole coupon과 camera cradle fit-check만 먼저 출력한다.
2. 중앙 support bottom coupon을 팔 없이 Waffle에 조립해 체결 간섭과 cable route를 확인한다.
3. 큰 shoulder hole/interface coupon 하나를 출력해 축 정렬, 변형, 백화, bolt seating을 확인한다.
4. 중앙 support를 조립하고 팔과 bare camera 위치에는 dummy mass를 둔다.
5. 정적 proof load와 1시간 creep 관찰을 통과한 뒤에만 실제 부품을 장착한다.
6. 균열, 층분리, bolt 풀림, 영구변형이 있으면 G-code revision을 폐기하고 재설계한다.

실물 motor command, torque enable, base 주행은 이 출력 검증과 별개의 hardware 승인
절차다. 출력물이 조립됐다는 사실만으로 움직임을 승인하지 않는다.

K1 Max의 300 mm Z보다 assembly 전체 높이 316 mm가 크지만, lower 170 mm와 upper
156 mm는 각각 build volume 안에 들어간다. 따라서 한 번에 세워 출력하지 않고 제공된
두 STL을 별도 slicing한다. 최종 G-code 전에는 10 mm 중첩부의 실제 clearance,
결합 fastener/접착 방식, 출력 방향, support와 layer 방향에 따른 강도를 확인해야 한다.
