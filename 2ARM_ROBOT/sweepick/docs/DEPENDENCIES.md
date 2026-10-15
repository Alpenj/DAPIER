# 외부 입력과 의존성

실행 코드가 참조하는 다음 입력은 공개 사본에 포함하지 않았다. 코드에 있는 `/home/robot`은 개인 홈 경로를 정리한 표시이며 실제 배포 경로가 아니다.

| 입력/의존성 | 공개 상태 |
|---|---|
| 양팔 Calibration, EEPROM/device identity, q12 mapping/영점 | 비공개 로컬 입력; API와 검증 코드만 공개 |
| wrist pairing, USB topology, camera intrinsic/extrinsic | 비공개 로컬 입력; 실제 식별값을 예제로 재사용하지 않음 |
| `RECEIVE_FIELD_PLAN.json`, 승인·prepared manifest | 현재 현장 실행 입력이므로 제외 |
| compiled model `.mjb`, robot meshes/MJCF 자산 | 제외; source support는 포함 |
| 세션 RGB/RGB-D, joints/events 전체, owner log | 제외; 익명화한 기술 검토와 Fake Bus 결과만 포함 |
| canonical field tools | `support/field_tools`에 실제 reader/import 대조용 소스 포함 |
| TJJ factory/scene/IK/teacher/task manager | `support/tjj_runtime`에 선택한 프로젝트 소스 포함 |
| LeRobot/Feetech + scservo_sdk | 설치 의존성; vendor 코드와 장치 설정은 복사하지 않음 |
| OS30A SDK/capture helper | 외부 SDK/binary; 소유권·프레임 수신 인터페이스만 소스에 있음 |
| NumPy, MuJoCo, SciPy, PyTorch, OpenCV | 외부 라이브러리; 선택 경로에 따라 필요 |
| FastAPI, Starlette, Uvicorn, AnyIO, Pillow | 콘솔 의존성; 기존 [requirements.txt](../console/requirements.txt) 참조 |

측정 기반 motion/jaw/progress profile은 알고리즘의 기존 PROVISIONAL 실행 파라미터를 읽기 위한 자료다. 제조사 physical limit 또는 새로운 하드웨어 검증값으로 해석하지 않는다. full-chain의 오른 jaw 등 추가 runtime reference가 현재 장비에서 준비됐다고 이 폴더가 보증하지 않는다.
