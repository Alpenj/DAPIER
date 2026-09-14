# 외부 Feetech ROS 2 driver 로컬 patch

`$HOME/so101_ros2_ws/src/so101-ros-physical-ai`는 DAPIER 소스가 아니라
`legalaspro/so101-ros-physical-ai` checkout이다. 그 저장소의 submodule에서
발견한 로컬 변경 5개 파일을 `feetech-driver-local.patch`로 보존했다.

## 기준과 범위

- parent remote: `https://github.com/legalaspro/so101-ros-physical-ai.git`
- parent revision: `58318c905a2c61289fa907de85cb8473322fbe68`
- submodule remote: `https://github.com/legalaspro/feetech_ros2_driver.git`
- submodule base: `4c0fdbfe16c84c686f8ace09526c52d98d0110ca`
- 변경량: 5 files, 292 insertions, 116 deletions
- license: `LICENSE.bsd-3-clause.txt`

이 patch는 serial protocol과 hardware interface를 바꾸므로 DAPIER의
[`so101/ros2_ws`](../../ros2_ws/README.md)에 자동으로 합치지 않는다. 먼저
mock/build 검증을 반복하고, 실제 장비는 별도 safety Gate와 승인 뒤에만 다룬다.

깨끗한 submodule checkout에서 복원할 때는 다음처럼 적용한다.

```bash
git checkout --detach 4c0fdbfe16c84c686f8ace09526c52d98d0110ca
git apply /path/to/DAPIER/so101/integrations/feetech_ros2_driver/feetech-driver-local.patch
```

`launcher/run_follower_real.sh`도 이 외부 stack에 종속된다. 가져온 뒤 accidental
real-hardware launch를 막기 위해 기본값을 `mock`으로 바꾸고, real backend에서는
`SO101_USB_PORT`를 반드시 명시하도록 수정했다.

2026-08-07에는 clean submodule base에 patch가 적용되는지 확인하고 ROS 2 Jazzy
workspace에서 `colcon build --packages-select feetech_ros2_driver`를 통과시켰다.
`colcon test` 결과는 test 0개, error/failure 0개였다. 빌드 성공을 serial 통신이나
실기체 검증으로 확대하지 않는다. launcher는 shell syntax만 검사했고 real launch는
실행하지 않았다.
