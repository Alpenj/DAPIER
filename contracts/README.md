# DAPIER cross-language contracts

이 디렉터리는 Python 연구 계층, C++ 인식·지역화 런타임, C++ 실시간 제어 계층
사이의 언어 중립 계약을 보관한다.

- [`research_realtime_control_v1.json`](research_realtime_control_v1.json):
  intent 종류, TTL, 단위, 소유권과 Python의 하드웨어 비승인 원칙
- [`localization_runtime_v1.json`](localization_runtime_v1.json):
  pose·frame·tracking state·map/reset generation, receiver-local TTL,
  simulator-truth 금지와 localization의 하드웨어 비승인 원칙
- control C++ generated header:
  `so101_ros2/dapier_so101_core/include/dapier_so101_core/generated/`
- localization C++ generated header:
  `so101_ros2/dapier_localization_core/include/dapier_localization_core/generated/`
- 생성 명령:
  - `scripts/generate-control-contract-header`
  - `scripts/generate-localization-contract-header`
- 동기화 검사: `scripts/verify-architecture-boundaries`

계약을 수정할 때는 JSON을 먼저 수정하고 generated header를 다시 만든다. Python,
localization C++, control C++ 어느 한쪽의 상수만 직접 바꾸지 않는다.

서로 다른 호스트의 monotonic timestamp는 비교하지 않는다. 모든 TTL은 수신기가
자신의 monotonic clock으로 시작하며 source timestamp는 trace용이다.
