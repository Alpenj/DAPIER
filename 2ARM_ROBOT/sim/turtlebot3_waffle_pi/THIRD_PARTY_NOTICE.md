# TurtleBot3 Waffle Pi 원본 자산 고지

## 출처

- 저장소: https://github.com/ROBOTIS-GIT/turtlebot3
- 브랜치: jazzy
- 원본 commit: 0c0be84e3f5c3194fb2adea8426a58a96060eab5
- package: turtlebot3_description 2.3.7
- 라이선스: Apache License 2.0
- DAPIER 반입일: 2026-08-24
- 공식 jazzy 재검증: 2026-08-27, path latest commit 2d7e80ace1636c2a2260cd6080ee6dea49115188
- 재검증 결과: base, tire 2개, LDS, Waffle Pi URDF의 Git blob SHA가 공식 jazzy와 모두 일치

Apache License 2.0 전문은 upstream/LICENSE에 보존한다.

## 원본으로 보존한 파일

- Waffle Pi와 Waffle URDF
- common_properties.urdf
- package.xml
- waffle_pi_base.stl
- left_tire.stl과 right_tire.stl
- lds.stl

## DAPIER 변경 파일

turtlebot3_waffle_pi_mujoco.urdf는 원본 Waffle Pi xacro/URDF에서 생성한 파생 파일이다.
MuJoCo가 base_link와 공식 visual mesh를 보존하도록 compiler 설정을 추가했고, package URI를 이
폴더 기준 상대 경로로 변경했다. loader는 collision proxy를 투명한 group 3으로 분리하지만 접촉 bit는
유지한다. 로봇 치수, 질량, 관성, joint와 frame 값은 변경하지 않았다.
