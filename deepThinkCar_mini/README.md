# deepThinkCar-mini | 비전·모방학습 주행 실습

**카메라 영상 → OpenCV 차선 인식 → 데이터 라벨링 → PC 학습 → 차량 추론**의 연결을 배우는 Raspberry Pi 기반 교육용 저장소입니다.

[전형주 포트폴리오](https://julianjeonresume.netlify.app/) · [개인 학습 아카이브](https://github.com/Alpenj/physical-ai-lab) · [변경 이력](https://github.com/Alpenj/deepThinkCar_mini/commits/main/)

> 기반 자료는 JD-edu의 deepThinkCar-mini 교육 코드와 키트 문서입니다. 원본이 제공하는 기능, 개인이 수정한 코드, 직접 수행한 실험 결과를 구분합니다. 키트의 주행·ADAS 소개나 저장된 모델·영상이 곧 개인의 독자 구현 또는 검증된 주행 성능을 뜻하지 않습니다.

## 검토 시작점

| 확인하려는 내용 | 읽을 위치 |
|---|---|
| 영상에서 조향까지의 흐름 | [`jd_opencv_lane_detect.py`](jd_opencv_lane_detect.py) → [`jd_deep_lane_detect.py`](jd_deep_lane_detect.py) |
| 데이터 수집·가공 | [`jd_1_record_lane_video.py`](jd_1_record_lane_video.py), [`jd_2_get_train_data.py`](jd_2_get_train_data.py) |
| PC에서 학습하는 부분 | [`PC_run_code/`](PC_run_code/) |
| 제어와 장치 경계 | [`jd_car_motor_l9110.py`](jd_car_motor_l9110.py), [`test_code/`](test_code/) |
| 원본 준비·조립·수업 순서 | 아래 원본 문서 안내 및 [`doc/`](doc/) |

코드 검토에는 차량을 움직일 필요가 없습니다. 개인 기여를 판단할 때에는 변경 이력과 실행 기록을 원본 자료와 함께 비교합니다.

## 전체 폴더·파일 지도

| 경로 | 역할 |
|---|---|
| [`PC_run_code/`](PC_run_code/) | PC 측 실행·학습 코드 |
| [`Package_part_label/`](Package_part_label/) | 키트 부품 라벨 자료 |
| [`doc/`](doc/) | OS·설치·조립·주행 학습 문서 |
| [`test_code/`](test_code/) | 장치와 기능별 시험 코드; 자동화된 무장비 테스트 모음으로 간주하지 않음 |
| [`data/`](data/) | 기존 데이터 자산 |
| [`models/`](models/) | 기존 모델 자산; 개인 학습 결과 여부는 출처·실행 기록으로 별도 확인 |
| [`jd_1_record_lane_video.py`](jd_1_record_lane_video.py) | 차선 영상 기록 단계 |
| [`jd_2_get_train_data.py`](jd_2_get_train_data.py), [`jd_label_data_compress.py`](jd_label_data_compress.py) | 학습 데이터 생성·라벨 데이터 압축 관련 코드 |
| [`jd_opencv_lane_detect.py`](jd_opencv_lane_detect.py) | OpenCV 차선 인식 |
| [`jd_deep_lane_detect.py`](jd_deep_lane_detect.py), [`jd_4_lane_follower_deep.py`](jd_4_lane_follower_deep.py) | 학습 모델 기반 차선 인식·주행 |
| [`jd_5_object_detection_opencv.py`](jd_5_object_detection_opencv.py), [`jd_opencv_dnn_objectdetect_v3.py`](jd_opencv_dnn_objectdetect_v3.py) | 객체 인식 관련 예제 |
| [`jd_car_motor_l9110.py`](jd_car_motor_l9110.py), [`jd_remote_control.py`](jd_remote_control.py) | 모터 제어·원격 조작 |
| [`car_video.avi`](car_video.avi) | 기존 영상 파일; 촬영자·조건·개인 수행 여부를 확인하기 전에는 실적 근거로 사용하지 않음 |

원본 문서, Python import와 데이터 경로의 호환성을 유지하기 위해 기존 파일을 이동하거나 이름을 바꾸지 않았습니다.

## 원본 문서와 학습 순서

기존 README가 안내하던 준비·조립·실습 자료를 모두 유지합니다. 아래 내용은 **원본 교육 과정의 안내**이며 이 저장소 관리자의 실습 완료 목록이 아닙니다.

| 순서 | 원본 문서 | 확인할 내용 |
|---|---|---|
| 준비 1 | [Raspberry Pi OS 이미지](https://jd-edu.github.io/deepThinkCar_mini/doc/os) | 보드와 OS 구성 |
| 준비 2 | [소프트웨어 설치·셋업](https://jd-edu.github.io/deepThinkCar_mini/doc/setup) | OpenCV, TensorFlow, Adafruit 서보 관련 라이브러리 |
| 준비 3 | [키트 조립](https://jd-edu.github.io/deepThinkCar_mini/doc/assembly) | 차체·카메라·구동·조향 연결 |
| 준비 4 | [VNC 환경](https://jd-edu.github.io/deepThinkCar_mini/doc/vnc) | 원격 개발 환경 |
| 준비 5 | [하드웨어 테스트](https://jd-edu.github.io/deepThinkCar_mini/doc/hardware) | 카메라, DC 모터, 조향 서보, 조향 오프셋, 전원 |
| 실습 1 | [OpenCV 차선 인식 주행](https://jd-edu.github.io/deepThinkCar_mini/doc/step_1) | 차선 인식과 학습용 영상 수집 |
| 실습 2 | [데이터 라벨링](https://jd-edu.github.io/deepThinkCar_mini/doc/step_2) | 영상과 학습 정답의 연결 |
| 실습 3 | [PC 딥러닝 학습](https://jd-edu.github.io/deepThinkCar_mini/doc/step_3) | 학습과 모델 파일 생성 |
| 실습 4 | [딥러닝 차선 인식 주행](https://jd-edu.github.io/deepThinkCar_mini/doc/step_4) | 저장한 모델을 차량에서 실행 |

원본 README의 호환성 안내는 **Raspberry Pi 3B·3B+·4에서 시험, Pi 5는 미시험**입니다. 이는 원본의 기록이며 이번 문서 정리에서 보드·OS별 재시험을 수행하지 않았습니다. 현재 사용 장비와 의존성 버전의 호환성을 별도로 확인합니다.

## 실행과 안전 경계

이 저장소는 PC 학습 코드와 Raspberry Pi 장치 제어 코드가 섞여 있으므로 루트에서 모든 스크립트를 일괄 실행하지 않습니다. 파일의 import, 모델·데이터 경로, 카메라·GPIO·모터 접근 여부를 읽은 뒤 필요한 예제만 선택합니다.

실물 시험 전에는 원본 하드웨어 안내를 기준으로 전원과 배선, 조향 범위·오프셋, 정지 방법을 확인합니다. 원격 접속이 된다는 사실이나 추론 파일이 있다는 사실만으로 차량이 안전하게 주행할 준비가 끝났다고 판단하지 않습니다.

## 개인 재현 기록

실험을 포트폴리오에 추가할 때 다음 항목을 함께 남깁니다.

| 구분 | 필요한 기록 |
|---|---|
| 출처·기여 | 기준 원본, 수정 commit, 변경 목적과 범위 |
| 환경 | 보드·OS·카메라·구동부, Python·라이브러리 버전 |
| 데이터 | 촬영 조건, 라벨 정의, 학습·평가 데이터 분리 |
| 실행 | 실제 명령, 모델 버전, 제어 주기, 정지 조건 |
| 결과 | 코스·조명·속도·반복 횟수, 실패·개입 사례, 로그 또는 영상 |

주행 성공률, 일반화 성능, ADAS 성능은 해당 조건과 측정 기록이 있을 때만 소개합니다. 현재 README에는 새 성능 수치를 추가하지 않았습니다.

## 출처·공개 범위

원본 문서와 코드·모델·데이터의 권리 표기를 유지합니다. 2026-09-05 루트 목록에서는 별도 LICENSE 파일을 확인하지 못했으며, 문서 정리는 새로운 이용 허락을 부여하지 않습니다. 민감한 네트워크 정보, 인증 정보, 타인의 개인정보는 실행 예시나 공개 결과에 넣지 않습니다.

문서·구조 점검: 2026-09-05. 이번 변경은 안내 문서에 한정되며 차량 주행·모델 학습·전체 코드 보안 감사를 새로 수행한 결과가 아닙니다.
