# 프로젝트 문서

- [`requirements-ledger.md`](requirements-ledger.md): 현재 확정·미확정 요구사항 원장
- [`PHASE0_HANDOFF.md`](PHASE0_HANDOFF.md): Ubuntu ROS 2 PC 개발 인수인계
- [`이동형_양팔_로봇_신발_정리_프로젝트_요구사항_원장.docx`](이동형_양팔_로봇_신발_정리_프로젝트_요구사항_원장.docx): 팀 공유용 Word 요구사항 원장
- [`DYNA_SLIM_팀_의사결정_비교_메모.docx`](DYNA_SLIM_팀_의사결정_비교_메모.docx): 기존안과 선택안 B 비교 자료
- [`DYNA_SLIM_적용_조사_참고자료.md`](DYNA_SLIM_적용_조사_참고자료.md): DYNA/SLIM 조사 참고자료이며 필수 채택 사양이 아님
- [`GEN1.5_양팔_적용_조사_참고자료.md`](GEN1.5_양팔_적용_조사_참고자료.md): Generalist GEN-1.5 조사 원문 기반 참고자료
- [`GEN15_ADOPTION.md`](GEN15_ADOPTION.md): GEN-1.5에서 실제 채택·보류·팀 결정 대상을 분리한 기록
- [`LEROBOT_DECONSTRUCTION_AND_PERSONALIZATION.md`](LEROBOT_DECONSTRUCTION_AND_PERSONALIZATION.md): 최신 공식 LeRobot 구동 흐름 해체분석과 2ARM_ROBOT ACT 개인화·검증 기록
- [`ASTRA_RGBD_PAYLOAD_CONTRACT.md`](ASTRA_RGBD_PAYLOAD_CONTRACT.md): Astra Pro lossless RGB/Depth raw payload·timestamp·integrity 계약
- [`research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md`](research/LATEST_RGBD_DATA_CONTRACT_RESEARCH_20260821.md): 최신 LeRobot v3·DROID·ROS2·Orbbec 기반 Stage 1 연구 보강
- [`RESEARCH_ADOPTION_LEDGER.md`](RESEARCH_ADOPTION_LEDGER.md): 단계별 최신 논문·공식 정본, 확인 주장, 채택·보류 결정과 코드 증거 원장
- [`NATIVE_LEROBOT_V3_ENCODER.md`](NATIVE_LEROBOT_V3_ENCODER.md): LeRobot/Torch를 ROS2 base에서 분리한 lazy native v3 encoder와 Stage 3 gate
- [`research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md`](research/LATEST_LEROBOT_V3_ENCODER_RESEARCH_20260821.md): 공식 v3 writer/tests/extras와 Robo-DM 기반 Stage 2 보강
- [`ACT_DATASET_ROUNDTRIP_SMOKE.md`](ACT_DATASET_ROUNDTRIP_SMOKE.md): 2×3 native v3 reopen·ACT temporal window·padding mask·one-forward 검증
- [`research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md`](research/LATEST_ACT_DATALOADER_ROUNDTRIP_RESEARCH_20260821.md): Stage 3에 직접 관련된 LeRobot 공식 정본과 최소 논문 참고 기록
- [`OFFLINE_EVALUATOR_ACTION_CHUNK.md`](OFFLINE_EVALUATOR_ACTION_CHUNK.md): held-out split·padding 제외·양팔 joint/group metric·inspection 계약
- [`research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md`](research/LATEST_OFFLINE_EVALUATOR_CHUNK_RESEARCH_20260821.md): Stage 4 공식 정본과 직접 관련 최신 자료의 채택·보류 기록
- [`JDCOBOT_ROLLOUT_SAFETY_SUPERVISOR.md`](JDCOBOT_ROLLOUT_SAFETY_SUPERVISOR.md): policy-independent lifecycle/fault latch와 ROS2-shaped dry-run adapter
- [`research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md`](research/LATEST_ROLLOUT_SAFETY_SUPERVISOR_RESEARCH_20260821.md): ROS2·LeRobot·ROBOTIS 정본과 최소 최신 연구의 Stage 5 판정

팀의 현재 결정은 `ACT baseline + DYNA-lite + 4주차 이후 보조학습
go/no-go`다. 조사 참고자료의 전체 SLIM/DYNA 구성을 무조건 적용하지 않는다.
