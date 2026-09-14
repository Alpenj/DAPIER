# STEP 1 — 웹캠 데이터 수집

PDF의 STEP 1을 직접 실행하기 위해 만든 로컬 데이터 수집 도구다. 브라우저의 웹캠과 내 컴퓨터의 Python 저장 서버를 사용하며 이미지는 외부로 전송하지 않는다.

## 실행

    conda activate lerobot-vision
    cd ~/DAPIER/pytorch/resnet_dataset_studio_step1
    python serve.py --open

브라우저가 열리지 않으면 http://127.0.0.1:8765 에 접속한다.

## 수집 순서

1. 카메라 시작을 누르고 카메라 권한을 허용한다.
2. class_A부터 class_D까지 실제 라벨 이름으로 변경한다.
3. 데이터 폴더 준비를 눌러 저장 위치와 기존 이미지 수를 확인한다.
4. 객체를 중앙 ROI 안에 크게 배치한다.
5. 누르고 있는 동안 촬영 또는 자동 촬영 시작을 사용한다.
6. 클래스마다 정확히 1,000장을 채운다.

기본 저장 위치는 이 폴더 아래 collected_images다. 다른 위치를 사용하려면 실행할 때 --output 경로를 추가한다. 저장 구조는 PyTorch datasets.ImageFolder가 바로 읽을 수 있다.

    선택한_폴더/
    ├── class_A/
    ├── class_B/
    ├── class_C/
    ├── class_D/
    └── collection_state.json

촬영 결과는 224×224 JPEG다. 흐림 필터는 Laplacian 분산을 사용하고, 중복 필터는 최근 프레임의 dHash를 비교한다. 필터가 너무 엄격하면 화면에서 개별 옵션을 끌 수 있다.

## 데이터 수집 시 바꿀 조건

- 객체의 앞·뒤·옆과 기울어진 각도
- 카메라와의 거리
- 밝기와 그림자
- ROI 내부의 좌·우·위·아래 위치
- 손으로 잡은 상태와 바닥에 놓은 상태

배경만 바뀌고 객체가 거의 움직이지 않는 1,000장은 좋은 데이터가 아니다. 촬영 도중 의도적으로 위 조건을 바꾼다.

## 무결성 검사

    python integrity_check.py /path/to/선택한_폴더 \
      --target 1000 \
      --expected-classes 4 \
      --json integrity_report.json

검사는 클래스 수, 클래스별 장수, 손상 이미지, 64×64 미만 이미지, 완전히 동일한 파일, ImageFolder 로딩을 확인한다. 모든 조건을 만족해야 PASS가 출력된다.

## 카페 공유본

`ResNet_Dataset_Studio_STEP1.zip`은 코드, README, 테스트만 담은 배포본이다.
웹캠으로 촬영한 개인 이미지, `collected_images`, Python 캐시와 PC별 절대 경로는
포함하지 않았다.

- 압축 크기: 약 17 KiB
- SHA-256: `72c7e37b8a079ad5222ca78b3d3fbc38f63acb612ade9a95a3394e43fa3e40f9`
- 압축 무결성: `unzip -t` 통과

## 확인한 항목

- `node --check app.js`
- Python 문법 컴파일
- `test_integrity_check.py` 단위 테스트 2개
- 임시 JPEG 4장을 사용한 API 저장·무결성 검사

원본 촬영 데이터는 Git에 올리지 않는다. 각 사용자가 직접 수집하고
`integrity_check.py`로 클래스별 장수와 손상 여부를 확인한다.
