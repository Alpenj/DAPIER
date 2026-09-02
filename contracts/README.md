# DAPIER cross-language contracts

이 디렉터리는 Python 연구 계층과 C++ 실시간 제어 계층 사이의 언어 중립 계약을
보관한다.

- [`research_realtime_control_v1.json`](research_realtime_control_v1.json):
  intent 종류, TTL, 단위, 소유권과 Python의 하드웨어 비승인 원칙
- C++ generated header:
  `so101_ros2/dapier_so101_core/include/dapier_so101_core/generated/`
- 생성 명령: `scripts/generate-control-contract-header`
- 동기화 검사: `scripts/verify-architecture-boundaries`

계약을 수정할 때는 JSON만 수정하고 generated header를 다시 만든다. Python과
C++ 어느 한쪽의 상수만 직접 바꾸지 않는다.
