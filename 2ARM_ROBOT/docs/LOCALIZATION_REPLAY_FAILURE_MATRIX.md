# Localization replay and recovery gate

## 목적

실제 카메라와 모터를 연결하기 전에 Visual SLAM 출력의 정상·실패 상태가
base/arm motion에 어떻게 전파되는지 결정론적으로 검증한다. 이 경로는 카메라,
ROS graph, SSH, serial port와 actuator endpoint를 열지 않는다.

## 데이터 흐름

```text
recorded-like RGB/depth timestamps + LocalizationEstimate v1
    -> RGB/depth skew 검사
    -> schema/sequence/TTL/pose/covariance 검사
    -> map/session/reset identity 검사
    -> tracking quality gate
    -> proceed / hold / reject
    -> stale action chunk 폐기 및 replan acknowledgement
```

첫 localization identity, tracking loss, quality 저하, relocalization, map restart와
receiver-local interarrival watchdog 만료는 기존 action chunk를 폐기하고 fresh plan을
요구한다. identity와 invalidation sequence가 정확히 일치하는 acknowledgement 전에는
tracking이 회복돼도 motion을 재개하지 않는다.

## 고정 failure matrix

- low texture / insufficient tracked features
- motion blur / high reprojection error
- temporary occlusion
- frame drop and receiver-local interarrival timeout
- duplicate or out-of-order sequence
- RGB/depth timestamp skew
- invalid TF frame pair
- recently lost / lost tracking
- relocalized pose jump
- map/session restart and reset generation change

모든 report에서 `hardware_execution=false`이며 simulator truth를 fallback으로 사용하지
않는다.

## ORB-SLAM3 adapter 경계

`OrbSlam3StateAdapter`는 공식 ORB-SLAM3 tracking state 숫자를 DAPIER 계약으로
변환한다. `OK`, `RECENTLY_LOST`, `LOST`를 명시적으로 구분하며, loss 뒤 `OK` 복귀나
`MapChanged()` 이벤트는 `relocalized`로 내보내 stale target을 폐기한다. 이 core는
ORB-SLAM3를 링크하지 않으므로 GPL 소스를 복사하거나 vendor하지 않는다. 실제
ORB-SLAM3 node는 별도 optional package에서 이 경계를 호출한다.

## 재현

```bash
cd ~/DAPIER
scripts/replay-localization-scenarios \
  --output /tmp/dapier-localization-failure-matrix.json
cat /tmp/dapier-localization-failure-matrix.json

scripts/verify-hardware-free
```

ROS 2 Jazzy가 설치된 로컬에서는 다음도 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dapier_localization_core
colcon test --packages-select dapier_localization_core
colcon test-result --verbose
```

## 아직 실측으로 남는 항목

- 실제 Astra RGB/depth timestamps와 alignment
- camera-to-base, IMU-to-camera extrinsics
- 실제 ORB-SLAM3 처리율과 tracked-feature/reprojection 분포
- Raspberry Pi와 laptop 사이 network jitter
- 실제 E-stop과 저속 commissioning
