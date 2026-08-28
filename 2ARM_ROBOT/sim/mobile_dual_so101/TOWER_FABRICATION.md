# Dual-tower 제작 입력과 출력 패키지

현재 상태는 MuJoCo concept envelope다. 직접 실행해 본 simulation에서는 두 tower,
중앙 crossbar와 depth camera 배치가 컴파일되고 1,000 step 동안 finite state를
유지했다. 아직 Waffle 체결홀, SO-101 flange와 카메라 bracket을 실측하지 않았으므로
제작 가능한 CAD나 실물 안전이 확인됐다고 쓰지 않는다.

## 최종 제공할 파일

실측값이 들어오면 다음 파일을 같은 revision으로 생성한다.

- 파라메트릭 CAD 원본과 중립 교환용 STEP
- 부품별 STL: 좌/우 deck, 좌/우 tower interface, crossbar, camera cradle, gusset
- 조립도, fastener BOM, 출력 방향과 support 표시
- fit-check용 저재료 STL
- 사용 프린터와 slicer profile에 고정된 G-code
- CAD/STL/G-code의 SHA-256과 생성 조건

STL은 mesh라 치수 수정 원본으로 사용하지 않는다. G-code는 프린터 운동 한계,
nozzle, filament, bed와 start/end sequence에 종속되므로 범용 파일로 만들지 않는다.

## CAD 전에 필요한 실측값

단위는 mm로 기록하고 사진 한 장만으로 추정하지 않는다.

1. Waffle 상판에서 사용 가능한 M3 hole 중심 좌표, thread 깊이와 하부 접근 공간
2. 상판 외곽, wheel/caster, Raspberry Pi/OpenCR, connector와 cable keep-out envelope
3. 각 SO-101 vertical flange의 4-hole pitch, hole 지름, flange 두께와 bolt 접근 방향
4. 두 팔의 실측 질량, base 기준 COM, 최대 작업 payload
5. depth camera 실제 폭/높이/깊이, 체결홀, optical origin과 cable bend radius
6. 목표 tower 높이/간격의 허용오차와 전체 로봇 통과 폭
7. 프린터 모델, build volume, nozzle 지름, 사용 filament, 건조 가능 여부
8. slicer 이름/버전, layer height, wall/perimeter, infill, support와 bed adhesion profile

## 출력 순서

1. hole coupon과 camera cradle fit-check만 먼저 출력한다.
2. 좌/우 interface를 팔 없이 Waffle에 조립해 체결 간섭과 cable route를 확인한다.
3. tower 한쪽만 장착하고 변형, 백화, bolt seating을 확인한다.
4. 양쪽 tower와 crossbar를 조립하되 팔과 camera는 dummy mass로 대체한다.
5. 정적 proof load와 1시간 creep 관찰을 통과한 뒤에만 실제 부품을 장착한다.
6. 균열, 층분리, bolt 풀림, 영구변형이 있으면 G-code revision을 폐기하고 재설계한다.

실물 motor command, torque enable, base 주행은 이 출력 검증과 별개의 hardware 승인
절차다. 출력물이 조립됐다는 사실만으로 움직임을 승인하지 않는다.
