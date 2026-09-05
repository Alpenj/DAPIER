# 채용 검토용 문서 정리 범위

기준일: 2026-09-05. 이 기록은 정리 범위를 추적하기 위한 것으로 전체 코드·보안·실물 검증 완료 보고서가 아닙니다.

## GitHub 계정 목록

연결된 Alpenj 계정에서 조회된 저장소 11개를 공개 범위에 따라 분류했습니다. 비공개 저장소를 임의로 공개하거나 삭제·보관 처리하지 않습니다.

| 저장소 | 조회 시 공개 범위 | 이번 범위 |
|---|---|---|
| physical-ai-lab | public | 루트·글 목록·공개 학습 안내 정리 |
| soccerhelper | public | 제품·구조·실행·QA·운영 문서 탐색 정리 |
| so101_imitation_learning | public | 교육용 fork 출처, 전체 최상위 실습 폴더, ACT 읽기 순서 |
| deepThinkCar_mini | public | 원본 키트 자료와 개인 검증 구분, 전체 최상위 폴더·스크립트 안내 |
| DAPIER | private | main 기반 문서 전용 변경과 개발 PR의 상태·근거 분리 |
| Trading_ | private | 목록·공개 범위만 확인; 내용·branch·코드 변경 안 함 |
| naver_cafe_auto_dm | private | 목록·공개 범위만 확인; 내용 변경 안 함 |
| socceranalyst | private | 목록·공개 범위만 확인; 내용 변경 안 함 |
| WorkwithYara | private | 목록·공개 범위만 확인; 내용 변경 안 함 |
| ttfb_specialmatch_lookup | private | 목록·공개 범위만 확인; 내용 변경 안 함 |
| TTFB_WED | private | 목록·공개 범위만 확인; 내용 변경 안 함 |

`Trading_`의 조회 시 기본 브랜치는 `pro-rescue/cycle2-source-projection-sol-pro`입니다. 이름만 보고 main으로 정리하거나 개발 상태를 변경하지 않습니다.

## 완료로 간주하지 않는 항목

- 모든 재귀 하위 파일·과거 commit·binary·dataset 내용의 전수 감사.
- 로봇·시뮬레이터·모델 학습·전체 CI·웹 서비스의 신규 실행 검증.
- 각 개인·팀 기여의 정량 비율, 외부 자료의 재배포 권리 확인 완료.
- 비공개 repo 또는 Notion의 공개 전환, GitHub profile·pin·About 설정 변경.
- 현재 Netlify 사이트 본문의 모든 외부 링크를 익명 브라우저에서 검증한 상태.

## 남은 공개 준비

1. DAPIER에서 외부 검토에 필요한 코드·비식별 근거·합성 fixture를 선별하고 원본·이력의 민감정보 및 이용 권리를 확인합니다.
2. 공개 후보의 실행 환경과 데이터 없이 가능한 최소 검사 범위를 고정합니다.
3. 개인 역할을 실제 PR·변경·검증으로 연결합니다. 팀 전체 기능과 학습용 원본 구현은 별도로 표기합니다.
4. 공개 전환 또는 별도 showcase 여부는 사용자가 결정한 뒤 적용합니다. 그 전에는 DAPIER 링크만으로 외부 검토가 가능하다고 안내하지 않습니다.

## 보존 범위

이번 DAPIER 변경은 루트 README와 `docs/portfolio/` 문서입니다. ROS 2 package, Python import, launch, 모델·mesh·dataset 경로, 안전 설정, 기존 테스트·의존성·workflow와 실험 원본을 이동·삭제·수정하지 않습니다. 문서 전용 PR은 진행 중인 기능 PR과 분리합니다.
