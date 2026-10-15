# 확인한 범위와 재현 제한

2026-10-15 로컬 실행 소스에서 직접 확인한 결과다. 공개 경로·식별값 정리 이후 이 폴더에서 수행한 구문/SHA 검사는 아래 결과와 별개다.

| 검사 | 결과 | 의미 |
|---|---:|---|
| 기존 콘솔 + Console-15 단위 시험 | 92 pass | parser/replay/승인 relay/검수 export·원본 보존 |
| A-16/A-17 native 관련 시험 | 36 pass | Fake Bus 바인딩·최초 Goal·STOP·기존 allowlist/postwrite |
| 콘솔 브라우저 검사 | 32 시나리오 pass | 실제 제어 POST 0, viewport/저장 근거/문서 SHA 확인 |
| 핵심 기존 원본 SHA 대조 | 32개 일치 | 원본 세션·이전 Source SHA 보존 |
| 최신 수정본 실물 재현 | 미실행 | 실제 CLOSE/LIFT 성공으로 해석하지 않음 |

Console-16 후속 변경은 콘솔 단위·HTTP 검사 **103개**와 브라우저 **39개**를 통과했다. 신규 제한 범위 거부·서버 capability·보유/STOP·시각 근거 검사와 기존 replay/승인/Export 회귀를 포함한다. 실제 브라우저 제어·데이터 검수 POST는 0회다. 핵심 원본 32개 및 A17 native 소스 4개의 SHA가 유지됐다. [시험 보고서](console16/CONSOLE16_TEST_REPORT.json), [Fake 검증](console16/CONSOLE16_FAKE_RUN_VALIDATION.json)을 참고한다.

후속 Gripper 초기 범위 검수에서는 새 회귀 111개, 기존 A16/A17·allowlist·명령 순서 36개, 기존 grip 7개로 **154개가 통과**했다. 64조건을 수정 전후 비교했고, 별도로 첫 CLOSE 추종 지연 24조건을 검사했다. 관련 없는 기존 trim 시험 1개는 수정 전후 같은 `KeyError: provisional`이었다. [결함·수정·실물 한계](gripper_initial_range/GRIPPER_INITIAL_RANGE_REVIEW.md), [시험 결과와 원본 SHA](gripper_initial_range/GRIPPER_INITIAL_RANGE_TEST_REPORT.json)를 참고한다. 이번에는 실물 장치 접근과 Motor/Torque write 모두 0회다. 기존 콘솔·브라우저 코드는 변경하지 않아 위 검사를 반복하지 않았다.

소형 검수 ZIP의 모든 member SHA와 실제 원본 이미지를 다시 대조했다. 10월 14일 패키지는 252,279바이트/실제 이미지 5개, A16 실패 패키지는 291,740바이트/4개다. A16/A17/Console16의 분석 문서 19개를 HTTP로 조회할 수 있다. 원시 이미지와 ZIP 자체는 이 공개 브랜치에 포함하지 않는다.

native 실행 인터프리터는 기존 로봇 Python 3.12 venv, console은 기존 FastAPI venv다. ROS pytest plugin의 무관한 lark 의존성 수집 오류가 있어 native 시험은 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 실행했다. 패키지나 시스템 설정은 바꾸지 않았다.

구형 native suite의 9개 실패는 그대로 남는다. fixture의 fresh other-arm reader 누락, 현재 관측/yaw 정책과 다른 assertion, 과거 Scene fixture 누락을 [A17_LEGACY_FAILURE_TRIAGE.md](a17/A17_LEGACY_FAILURE_TRIAGE.md)에 분류했다. 모든 native 시험이 green이라고 표시하지 않는다.

공개 폴더에는 private Calibration·현재 모델 바이너리·원시 세션이 없다. 관련 fixture가 이 로컬 자원을 요구하므로 공개본만으로 원래 92/36 시험을 전부 재현할 수 있다고 주장하지 않는다. `tests/check_public_snapshot.py`는 외부 자원 없이 소스 파일 SHA/구문/공개 경계를 확인한다.

게시 전 공개 사본에서도 native A-16/A-17/allowlist/postwrite 36개를 다시 실행해 통과했다. 이 검사는 이 PC에 있던 로컬 입력을 읽고 Fake Bus만 사용했다. 필요한 archived 모듈 한 개는 기존 Fake 클래스의 import 의존성으로 포함했으며 현재 live executor로 선택한 것이 아니다. 공개 사본 140개 파일의 SHA·구문·상대 문서 링크·공개 경계 검사도 통과했다.

Gripper 수정 공개본에서도 동일한 관련 154개를 검사했다. 공개 스냅샷 검사는 154개 원본 대응 파일의 SHA·구문·상대 문서 링크·공개 경계를 확인했다. `SOURCE_INDEX.json`의 A_CONTROL 변경과 CONSOLE 보존을 구분하며 과거 140/148개 스냅샷의 검증 기록을 소급 변경하지 않는다.
