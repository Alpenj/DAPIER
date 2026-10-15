# Sweepick GitHub 작업 규칙

저장소는 Alpenj/DAPIER, 통합 브랜치는 `pro/sweepick-review-20261015`, 작업 범위는 `2ARM_ROBOT/sweepick/`다. main 자동 merge, 기존 공유 이력 재작성, 강제 push는 하지 않는다.

| 구분 | 현재 검토 경로 | 변경 관리 |
|---|---|---|
| A 제어 | native/, support/field_tools/ | Owner·writer·mapping·Calibration·승인·STOP 계약을 유지하고 독립 검수 |
| B 학습·Shadow | 이번 스냅샷에서 새로 추가/수정하지 않음 | 실제 B 소스 위치를 확인한 뒤 별도 작업 브랜치·커밋으로 관리 |
| Console | console/ 및 docs/console16/ | 기존 실행기 relay·읽기 전용 검수·웹 계약. 별도 장치 owner 생성 금지 |
| 공통 의존성 | support/tjj_runtime/ | 호출되는 공통 소스임을 표시하고 A/B 변경과 구분 |

서로 다른 작업자가 동시에 소스를 수정하면 통합 브랜치에서 함께 쓰지 않고 각자 별도 작업 브랜치와 worktree를 사용한다. 예: `pro/sweepick-a-<task>`, `pro/sweepick-b-<task>`, `pro/sweepick-console-<task>`. 관련 테스트·소스 대조·공개 범위 검수가 끝난 변경만 지정 통합 브랜치에 반영한다. 단일 통합 담당자가 공개 검토본을 동기화한다.

각 공개 소스의 실제 실행 원본 SHA와 공개본 SHA를 SOURCE_INDEX.json에 기록한다. 공개 경로 정리로 달라진 바이트는 원본과 구분한다. 기존 실행 기록과 당시 Source SHA는 현재 파일로 덮어쓰지 않는다.

민감한 Calibration·EEPROM·실제 장치 식별정보·승인 자료·전체 실물 영상·대용량 모델 바이너리는 업로드하지 않는다. 로컬 작업용 AGENTS.md, 도구 설정, 대화·프롬프트·작업 로그도 제외한다. 공개 기능에 필요한 실제 AI/ML 설명, 라이선스와 제3자 출처는 유지한다.

변경 후 관련 무장치 시험을 수행하고, 변경 파일 목록과 검증 한계 및 커밋 SHA를 보고한다. 실물 시험은 기존 현장 승인 계약에 따른 별도 작업이다. Fake/Mock 통과를 실물 성공으로 표시하지 않는다.
