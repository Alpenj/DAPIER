# GEN-1.5 참고 내용의 DAPIER 반영 결정

이 문서는 GEN-1.5 조사 결과를 프로젝트 요구사항과 혼동하지 않기 위한
결정 기록이다. GEN-1.5 자체 모델을 사용하거나 재현한다는 계획이 아니다.

## 이번에 채택·구현한 것

| 항목 | DAPIER 구현 | 경계 |
|---|---|---|
| one-shot perception exemplar | 신발 임베딩 예시를 등록하고 cosine similarity와 top-2 margin으로 `match/abstain` 판정 | 짝 후보만 반환하며 관절 제어 권한은 항상 `false` |
| typed skill exemplar | accepted quality-gated `train/exemplar` episode에서 precondition, postcondition, timeout, parameter, tag를 생성 | 같은 platform, calibration, robot config, action/state schema에서만 검색 |
| held-out leakage audit | exemplar와 평가 episode의 object, session, recording span, background, fixture 중복 검사 | object/session/span은 error, background/fixture는 warning |
| provenance 강화 | episode에 object/background/fixture/span/attempt ID 기록 | frame random split 대신 object×session split을 지원 |

## 참고만 하고 구현하지 않은 것

- GEN-1.5 checkpoint/API/weight adapter
- 30초 context window와 100Hz action을 프로젝트 요구사항으로 고정
- 사람 손 영상에서 JDcobot 관절 행동을 직접 생성
- simulation demonstration의 실기체 zero-shot 실행
- 1~10 gradient step proprietary adaptation 재현
- GEN-1.5의 59%/83%와 DAPIER 신발 정리 성능의 직접 비교

이 항목들은 공개 artifact와 controller 계약이 없으므로 현재 구현하면 재현성
없는 명목상 integration이 된다.

## 팀 회의에서 아직 정할 값

다음 값은 조사 보고서의 권고이며 확정 요구사항이 아니다.

- 6주 MVP를 seen 6켤레 + unseen 2켤레로 제한할지
- atomic ACT 진행 기준을 20 trial success 70%로 둘지
- pair similarity threshold와 abstention margin
- background/fixture 중복을 평가 hard error로 승격할지
- exemplar retrieval을 5주차 offline ablation에 포함할지

팀이 값을 잠근 뒤에는 평가 결과를 보고 threshold를 소급 완화하지 않는다.
