# JDcobot200 원본 자산 고지

## 출처

- 저장소: <https://github.com/JD-edu/jdcobot200_imitation_learning>
- 원본 commit: `99d196d23a3b3b3dfa1a195f2040fc889ecb1042`
- 권리자/제공자: JD-edu 강사 저장소
- DAPIER 반입일: 2026-08-24

## 허가 기록

원본 저장소에는 표준 LICENSE 파일이 확인되지 않았다. 2026-08-24에 DAPIER 저장소 사용자가
강사에게 이 프로젝트에서 원본 JDcobot200 자료를 사용·개인화하는 허가를 받았다고 확인했다.
허가 문구 원문은 이 저장소에 포함하지 않았다. 이 고지는 제3자에게 일반적인 재사용 권리를
부여하는 표준 오픈소스 라이선스를 대신하지 않는다.

## 반입한 원본

`upstream/`에는 다음 자료를 원본 그대로 보존한다.

- `robot.urdf`: MuJoCo 변환용으로 경로가 조정된 원본 URDF
- `urdf_to_mjcf.py`: 원본 URDF→MJCF 변환 예제
- `jdcobot200.xml`: 그리퍼 actuator/equality가 추가된 원본 MJCF
- `assets/*.stl`: 위 모델이 참조하는 14개 mesh

DAPIER 개인화 코드는 원본과 섞지 않고 상위 폴더의 `dual_model.py`에 둔다.
