# Jetson Nano(P3450) TurtleBot3 실물 구동 런북

이 문서는 [`turtlebot3_ws`](../turtlebot3_ws/README.md)에서 Gazebo로만 검증한
SLAM·Nav2 절차를, 실제 TurtleBot3(온보드 SBC = Jetson Nano)로 옮길 때 같은
실수를 반복하지 않으려고 적어둔 작업 순서다. **아직 실물로 검증한 항목은
하나도 없다** — 체크박스는 실제로 명령을 실행하고 화면·로그·로봇 움직임을
확인한 뒤에만 채운다.

보드 각인 확인 결과 정확한 모델은 **Jetson Nano Developer Kit (P3450 =
모듈 P3448 + 캐리어보드 P3449)**, 즉 2019년형 오리지널 Jetson Nano다.
Jetson Orin Nano가 아니므로 아래 절차 전체가 이 전제(JetPack 4.x 상한,
Ubuntu 22.04/24.04 네이티브 미지원)를 기준으로 한다. 보드를 교체하면 이
문서는 다시 써야 한다.

## 0. 이 문서에서 AI가 도운 범위

Jetson Nano를 되살려서 실물 주행까지 간다는 목표와 순서는 사용자가 정했다.
AI는 ROBOTIS 공식 문서(docs.robotis.com, emanual.robotis.com)와 NVIDIA
공식/커뮤니티 자료(dusty-nv/jetson-containers)를 확인해서 명령어와 버전
호환성을 정리했다. 배선, 전원 인가, SD카드 굽기, 보드 조립, OpenCR 연결,
실제 바퀴 움직임 확인은 사용자가 직접 한다. AI가 제안한 명령도 그대로
믿지 말고 한 줄씩 실행한 결과를 다음 단계의 기준으로 삼는다.

## 1. 핵심 결정: 왜 Docker로 ROS2 Humble을 쓰는가

`turtlebot3_ws`는 PC에서 **ROS2 Jazzy**(Ubuntu 24.04)로 되어 있다. 하지만
오리지널 Jetson Nano(P3450)는 NVIDIA가 만든 마지막 JetPack이 **JetPack
4.6.x (L4T r32.7.x, Ubuntu 18.04 기반)**로 끝났고, Ubuntu 22.04/24.04를
네이티브로 못 올린다 → **ROS2 Humble도, Jazzy는 더더욱 apt로 못 깐다.**

검증된 우회로는 `dusty-nv/jetson-containers`가 미리 빌드해 둔 Docker
이미지(`dustynv/ros:humble-desktop-l4t-r32.7.1` 계열)를 쓰는 것이다.
JetPack 4.6.x에는 Docker와 nvidia-container-runtime이 기본 포함돼 있다.

**결론(2026-08-11 수정)**: docs.robotis.com 공식 문서가 이제 **Ubuntu
24.04 + ROS2 Jazzy를 정식 지원**한다(SLAM/Navigation/Teleop/Simulation
전부 O, Manipulation·Home Service Challenge·자율주행 챌린지·ML만 X —
이번 목표엔 안 걸림). 따라서 **PC는 기존 `~/DAPIER/turtlebot3_ws`
(Jazzy)를 그대로 실물 로봇 연결에 쓴다.** PC에 별도 Humble 워크스페이스를
새로 만들 필요 없음 — 이전 버전의 이 문서에 있던 그 항목은 취소.

Nano만 하드웨어 제약(JetPack 4.6.x = Ubuntu 18.04 상한)으로 Docker
Humble 우회가 불가피하다. 결과적으로 **PC=Jazzy, Nano=Docker Humble**로
배포판이 섞인다. 공식 지원 조합은 아니지만, 실제로 부딪힐 지점은 거의
`/cmd_vel` 타입(Humble 쪽 turtlebot3_node는 plain `Twist`, Nav2
`enable_stamped_cmd_vel` 설정에 따라 `TwistStamped`) 하나 정도이고, 이건
`turtlebot3_ws/README.md`에 이미 기록된 Chapter 6 시뮬레이션 트러블슈팅과
같은 종류의 문제라 대응 가능하다. 실물에서 이 문제가 나오면 PC측
`nav2_params.yaml`의 `enable_stamped_cmd_vel`을 Nano 쪽 turtlebot3_node
기본값에 맞춰 끄면 된다.

## 2. 준비물 체크리스트

- [x] Jetson Nano(P3450) 보드, **5V/4A DC 배럴잭 어댑터 + J48 점퍼** —
      노트북 USB 포트로는 전류 부족해서(0.5~1A vs 필요치 2~2.5A+) 배럴잭
      어댑터 구매 후 사용, MAXN(기본) 모드로 부팅 중
- [x] microSD 카드 32GB(실사용 29.8G), 노트북 USB 카드리더로 굽기
- [x] 노트북(ASUS TUF A16) — 이미지 굽기 + 이후 SSH 원격작업 겸용
- [x] 모니터+키보드+마우스로 최초 부팅 계정 생성 완료, 이후 SSH 전환
- [ ] TurtleBot3 실물 섀시 + Dynamixel 모터 — 조립/배선 상태 미확인
- [x] OpenCR 보드, Nano와 USB로 연결 확인(`/dev/ttyACM0`)
- [ ] LDS(라이다) 연결 — 아직 확인 안 됨

## 3. Nano 접근 경로 확보 — SD카드 굽기 → 최초 부팅

```bash
# 다른 PC에서 (Nano 아님)
sudo dd if=/home/dapier-jhj/Downloads/sd-blob-b01.img of=/dev/sda bs=4M status=progress conv=fsync
sync
```

- [x] SD카드 굽기 완료 — 실제 이미지 파일명은 `sd-blob-b01.img`(JetPack
      4.6.1 zip 안에 이 이름으로 들어있음, B01 리비전 Nano용). **1차
      시도는 `dd: fsync failed for '/dev/sda': Input/output error`로
      실패**(카드리더 접촉 불량으로 추정) — 파티션 테이블은 반영됐지만
      끝부분 데이터 무결성 불확실. 카드리더 재삽입 후 재실행해서 2차
      시도는 `records in`=`records out`(3294+0) 일치, 에러 없이 성공.
      **`dd` 재시도 전엔 이전 프로세스가 완전히 죽었는지(`ps aux | grep
      dd`) 꼭 확인할 것** — 두 개가 동시에 같은 디스크에 쓰면 카드가
      확실히 깨진다.
- [x] SD카드 삽입 후 모니터+키보드로 최초 부팅, oem-config로 계정 생성
      (`dapierttb`), nvpmodel은 MAXN(배럴잭 4A 전원 확보돼서 선택)
- [x] Nano 보드엔 RTC 배터리가 없어서 최초 부팅 시 시계가 이미지 빌드
      시점(2021년)으로 떠있음 — 정상, 네트워크 연결 후 systemd-timesyncd로
      자동 동기화됨(`timedatectl`로 `synchronized: yes` 확인)
- [x] Nano 데브킷은 **Wi-Fi 미내장** — USB Wi-Fi 동글로 연결 성공
- [x] `sudo apt install openssh-server -y && sudo systemctl enable --now ssh`
- [x] 노트북에서 `ssh dapierttb@192.168.0.253` 키 기반 인증 성공
      (`~/.ssh/authorized_keys`에 노트북 공개키 등록). **sudo NOPASSWD**도
      `/etc/sudoers.d/nopasswd`에 `dapierttb ALL=(ALL) NOPASSWD:ALL`로
      설정 — 이후 전 단계를 노트북에서 SSH로 원격 진행

## 4. Nano: Docker 기반 ROS2 Humble

jetson-containers 레포를 따로 클론하지 않고, 미리 알려진 이미지 태그를
바로 `docker pull`했다(레포 클론은 `autotag` 스크립트 편의 기능일 뿐,
태그를 이미 알고 있으면 불필요).

```bash
cat /etc/nv_tegra_release   # R32.7.1 확인됨 (예상과 일치)
sudo docker pull dustynv/ros:humble-desktop-l4t-r32.7.1
sudo docker run -d --name turtlebot3_humble \
  --runtime nvidia --network host \
  --device=/dev/ttyACM0 \
  -v /home/dapierttb/turtlebot3_ws:/turtlebot3_ws \
  dustynv/ros:humble-desktop-l4t-r32.7.1 tail -f /dev/null
```

- [x] `/etc/nv_tegra_release` → `R32 (release), REVISION: 7.1` 확인
- [x] `docker pull` 성공 (37개 레이어, 수 GB)
- [x] 컨테이너 기동 확인 (`docker ps`), `ros2 pkg list`로 307개 패키지
      정상 로드 확인. **주의**: 이 이미지는 ROS2가 apt가 아니라 **소스
      빌드**라 setup 스크립트 경로가 `/opt/ros/humble/setup.bash`가
      아니라 **`/opt/ros/humble/install/setup.bash`**다(`ROS_ROOT`
      환경변수로 확인 가능, `/ros_entrypoint.sh` 참고)
- [x] OpenCR `--device=/dev/ttyACM0`로 패스스루 확인, 컨테이너 안에서
      장치 보임

## 5. Nano(컨테이너 안): TurtleBot3 워크스페이스 (humble 브랜치)

```bash
apt-get update   # 최초 1회 필요 — 아래 "겪은 문제" 참고
apt-get install -y python3-argcomplete python3-colcon-common-extensions \
  libboost-system-dev build-essential

# ⚠️ ros-humble-{hls-lfcd-lds-driver,turtlebot3-msgs,dynamixel-sdk,xacro}는
#    bionic(18.04)용 데비안이 없음(공식 Humble 데비안은 jammy 전용) →
#    apt 대신 전부 소스로 클론
mkdir -p /turtlebot3_ws/src && cd /turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone -b ros2    https://github.com/ros/xacro.git   # xacro는 distro별 브랜치가 아니라 ros2 브랜치
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
rm -rf turtlebot3/turtlebot3_cartographer turtlebot3/turtlebot3_navigation2

cd /turtlebot3_ws
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
# gcc-8/g++-8 지정 이유는 아래 "겪은 문제" 참고
```

- [x] 4GB 모델이라 zram 스왑(1.9GB, 4개 파티션)이 이미 기본 활성화돼
      있어서 별도 swapfile 생성 안 함(2GB 모델이면 필요)
- [x] colcon build 성공 — 13개 패키지 전부 빌드(`DynamixelSDK`,
      `turtlebot3_msgs`, `xacro`, `hls_lfcd_lds_driver`,
      `turtlebot3_description`, `turtlebot3_node`, `turtlebot3_teleop`,
      `turtlebot3_example`, `turtlebot3_bringup`, `ld08_driver`,
      `dynamixel_sdk_examples`, `dynamixel_sdk_custom_interfaces`,
      `turtlebot3`). `turtlebot3_bringup`에 `robot.launch.py` 확인,
      `turtlebot3_node`에 `turtlebot3_ros` 실행파일 확인
- [x] 실물 확인 결과 **`TURTLEBOT3_MODEL=burger`**, 라이다는 별도 USB
      케이블로 연결(USB2LDS 방식) → **`LDS_MODEL=LDS-01`**. 컨테이너를
      `-e TURTLEBOT3_MODEL=burger -e LDS_MODEL=LDS-01 -e
      ROS_DOMAIN_ID=30 --restart unless-stopped`로 재생성해서 환경변수
      고정(컨테이너 안 `~/.bashrc` 대신 `docker run -e`로 처리 — 재부팅
      후에도 컨테이너가 자동 재시작되며 값 유지)
- [x] udev rule 등록: `--symlink-install`이라 install 폴더의 룰 파일이
      심볼릭 링크라 `docker cp`가 계속 깨짐(아래 트러블슈팅 참고) →
      `docker exec ... cat | sudo tee /etc/udev/rules.d/`로 우회 성공.
      이 규칙에 `ID_MM_DEVICE_IGNORE=1`(OpenCR을 ModemManager가 모뎀으로
      오인해서 잡아가는 것 방지)과 라이다 `SYMLINK+="tb3_lidar"` 포함

### 겪은 문제 (실제로 부딪힌 것들)

**ROS2 apt 저장소 GPG 키 만료.** `apt-get update`가
`EXPKEYSIG F42ED6FBAB17C654 Open Robotics`로 실패. 2025년에 있었던 OSRF
GPG 키 로테이션 때문에, 이 컨테이너 이미지(그보다 오래된 빌드)에 박혀있는
키가 만료된 상태였음. 해결:
```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key
```
(`--batch --yes` 없으면 기존 파일 덮어쓸지 물어보는데 `/dev/tty`가 없는
비대화형 셸이라 `gpg: cannot open '/dev/tty'`로 죽는다.)

**bionic(Ubuntu 18.04)엔 `ros-humble-*` 확장 패키지 데비안이 아예 없음.**
공식 ROS2 Humble 데비안 배포는 jammy(22.04) 전용이라, `apt-get install
ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs
ros-humble-dynamixel-sdk ros-humble-xacro`가 전부
`E: Unable to locate package`. dusty-nv 컨테이너는 ROS2 코어 자체를
소스로 빌드해뒀을 뿐, 이런 다운스트림 패키지까지 데비안으로 미리 채워주진
않는다. 해결: 위 4개 패키지 전부 GitHub에서 소스 클론해서 같은
워크스페이스에 넣고 같이 `colcon build`.

**GCC 7.5.0에서 `turtlebot3_node` 컴파일 실패.** `reset.cpp`가 아니라
`rclcpp/client.hpp:821`(`get_and_erase_pending_request`)에서
`std::variant<...std::promise<...>...>` → `std::optional<variant>`
암시적 변환이 안 된다며 에러. `std_srvs::srv::Trigger` 서비스 클라이언트를
쓰는 코드에서만 이 템플릿이 실제 인스턴스화되기 때문에 rclcpp 자체
빌드 때는 안 걸리고 여기서 처음 걸림 — GCC 7의 `std::variant`+이동전용
타입(`std::promise`) 조합 관련 알려진 버그(9 이상에서 해결). 해결:
```bash
apt-get install -y gcc-8 g++-8
colcon build --cmake-args -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```
이미 성공적으로 빌드된 패키지(DynamixelSDK, turtlebot3_msgs, xacro,
hls_lfcd_lds_driver, turtlebot3_description)는 colcon이 재사용하고,
실패/미처리 패키지만 gcc-8로 재컴파일됨.

**gcc-8 재빌드가 install 경로를 `/usr/local`로 잘못 잡음.** 위 방법으로
`turtlebot3_node`는 컴파일 자체는 성공했는데, `ros2 pkg list`에 안
잡히는 문제 발생. 확인해보니 `make install` 로그가
`-- Installing: /usr/local/lib/turtlebot3_node/...`로 찍혀있었음 —
`/turtlebot3_ws/install/turtlebot3_node/`가 아니라 시스템 전역 경로로
설치된 것. 원인은 컴파일러를 바꾸면서 CMake가 "cache 재설정 필요" 경고를
띄웠는데, 이 재설정 과정에서 첫 번째(실패한) 시도 때 만들어진
`CMakeCache.txt`가 꼬여서 colcon이 넘겨주는 `CMAKE_INSTALL_PREFIX`가
반영이 안 된 것으로 추정. 해결: 컴파일러를 바꿔서 재시도할 땐
`--cmake-args`만 추가하지 말고, 해당 패키지의 `build/`와 `install/`
디렉터리를 **완전히 지우고** 처음부터 다시 configure:
```bash
rm -rf /turtlebot3_ws/build/turtlebot3_node /turtlebot3_ws/install/turtlebot3_node
colcon build --packages-select turtlebot3_node \
  --cmake-args -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```
재시도 후 `install/turtlebot3_node/{lib,share}` 정상 생성, `ros2 pkg
list`에도 정상으로 잡힘.

**`docker cp`가 `--symlink-install`로 만든 심볼릭 링크에서 계속 깨짐.**
`turtlebot3_bringup`의 udev rule 파일을 호스트로 꺼내려고
`docker cp turtlebot3_humble:.../99-turtlebot3-cdc.rules ...`를 여러 번
시도(`-L` 옵션 포함)했는데 전부
`stat /turtlebot3_ws/src/.../script: no such file or directory`로 실패.
`install/` 밑의 파일이 `--symlink-install` 때문에 `src/`의 원본을
가리키는 심볼릭 링크인데, 이 Docker 버전(20.10.7)의 `cp`가 이 링크를
컨테이너 안에서 제대로 못 따라감. 우회: `docker cp` 대신
`docker exec ... cat <파일> | sudo tee <호스트경로>`로 내용만 스트리밍.
(중간에 깨진 심볼릭 링크가 `/tmp`에 남아서 `sudo rm`으로 지우기 전까진
이후 명령들도 `bash: FILE: No such file or directory`라는 헷갈리는
에러를 냈음 — root 소유로 만들어진 잔재는 꼭 `sudo rm`으로 지울 것.)

**OpenCR 펌웨어 업로드가 `Fail to open port` + `Device or resource
busy`로 계속 실패.** `sudo fuser -v /dev/ttyACM0`으로 확인하니
`ModemManager`가 이 포트를 물고 있었음 — 새로 나타난 USB 시리얼
장치를 모뎀인지 확인하려고 자동으로 열어보는 게 기본 동작. 방금 등록한
udev rule에 이미 `ID_MM_DEVICE_IGNORE=1`이 있었는데도(`udevadm test`로
규칙 자체는 정상 인식 확인됨) `ModemManager` 재시작만으로는 이미 잡고
있던 포트를 안 놓음. 이 로봇엔 셀룰러 모뎀 쓸 일이 없어서 아예
서비스를 껐다: `sudo systemctl stop ModemManager && sudo systemctl
disable ModemManager`. 이후 `fuser`에 아무것도 안 잡히는 것 확인,
`update.sh` 재실행해서 성공(`flash_erase`/`flash_write`/`CRC Check`
전부 OK, `burger.opencr` V230127R1 정상 업로드).

## 6. OpenCR 펌웨어 플래시

```bash
sudo dpkg --add-architecture armhf
sudo apt-get update && sudo apt-get install -y libc6:armhf
export OPENCR_PORT=/dev/ttyACM0   # 실제 포트명 재확인
export OPENCR_MODEL=burger        # 실물 모델에 맞게
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

- [x] 업로드 성공 — `burger.opencr` V230127R1, flash_erase 0.98s /
      flash_write 0.69s / CRC Check 일치(D92222 D92222) / jump_to_fw
      전부 OK. (리커버리 모드는 필요 없었음 — ModemManager 끄고 나니
      바로 성공)
- [ ] OpenCR 테스트 시 바퀴가 실제로 도는지 — **아직 미확인, 다음 단계**

## 7. PC 쪽: 기존 `turtlebot3_ws`(Jazzy) 그대로 사용

새 워크스페이스 안 만든다. docs.robotis.com이 Ubuntu 24.04+Jazzy를
SLAM/Navigation/Teleop/Simulation 전 영역에서 정식 지원하므로, 이미 있는
`~/DAPIER/turtlebot3_ws`를 그대로 실물 로봇 연결에 쓴다.

- [ ] PC↔Nano 같은 `ROS_DOMAIN_ID=30`, 같은 네트워크(공유기, Nano 유선
      권장)에서 `ros2 topic list`로 서로 topic 보이는지 확인 (DDS
      discovery 확인 — 방화벽 때문에 안 보이면 멀티캐스트 허용 확인)
- [ ] `/cmd_vel` 타입 불일치(Nano=Humble plain `Twist` vs PC Nav2가
      `enable_stamped_cmd_vel`에 따라 `TwistStamped` 발행) 발생하면
      PC측 `nav2_params.yaml`의 `enable_stamped_cmd_vel: False`로 맞춘다

## 8. 실물 SLAM (교재 Part1과 동일 목적, 실물판)

```bash
# Nano
ros2 launch turtlebot3_bringup robot.launch.py

# PC
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run turtlebot3_teleop teleop_keyboard   # gentle_explorer.py는 시뮬레이션 전용, 실물은 직접 조작 권장(첫 시도)
ros2 run nav2_map_server map_saver_cli -f ~/map
```

- [x] `robot.launch.py` 정상 기동 — 라이다는 아직 미연결이라
      `hlds_laser_publisher`만 예상대로 죽음(exit code 255), `odom`은
      정상 발행. 나머지 노드(`turtlebot3_node`, `diff_drive_controller`,
      `robot_state_publisher`)는 라이다와 무관하게 독립적으로 정상 기동
- [x] **바퀴 실제 이동 확인 완료(2026-08-11)** — `ros2 topic pub -r 5
      /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}}"`로 실제
      전진, `odom.pose.pose.position.x`가 0→0.35m로 변화 + 사용자가
      눈으로 직접 회전 확인. 아래 트러블슈팅 참고(원인은 하드웨어가
      아니라 테스트 방법 문제였음)
- [ ] 지도 저장 성공 — 라이다 연결 후 진행 예정

## 9. 실물 Nav2 자율주행 (교재 Part2와 동일 목적, 실물판)

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

- [ ] `/initialpose` 전송 후 AMCL 수렴
- [ ] `/navigate_to_pose` 액션으로 목표까지 실제 이동 + `SUCCEEDED` 수신
- [ ] 충돌·이탈 없이 완료

## 10. 예상 트러블 포인트 (아직 실물로 안 겪어봤으므로 추정 — 겪으면 갱신)

- Nano USB 배럴잭 5V/2A 전원으로는 Wi-Fi+카메라+USB 허브(OpenCR+LDS) 동시
  부하 시 브라운아웃 가능 → 문제 생기면 5V/4A 배럴잭 전원으로 교체
- Docker 컨테이너 안에서 시리얼 장치(`/dev/ttyACM0`, `/dev/ttyUSB0`)가 안
  보이면 `--device` 플래그 누락이거나 컨테이너 밖에서 장치명이 바뀐 것
  (재부팅/재연결마다 번호가 바뀔 수 있음, `dialout` 그룹 권한도 확인)
- PC(Jazzy 계열 최근 설치)에 Humble을 apt 네이티브로 못 깔면 PC도 Docker로
  통일 — 이 경우 절차 7을 Docker 버전으로 다시 씀
- `turtlebot3_ws`(Jazzy) 교재에서 겪은 `/cmd_vel` 타입 이슈
  (`TwistStamped` vs `Twist`)는 Humble 쪽엔 없음 — Jazzy 전용 이슈였다는
  점을 착각하지 말 것

## 11. 바퀴 첫 구동 디버깅 — "하드웨어 문제인 줄 알았는데 테스트 방법 문제였다"

`teleop_keyboard`는 raw 터미널 입력(termios)이 필요해서 SSH 비대화형
세션으로는 못 돌리므로, 대신 `ros2 topic pub`으로 `/cmd_vel`을 직접
발행해서 테스트했다. 처음 몇 번은 전혀 안 움직였는데, 다음 순서로
원인을 좁혔다:

1. **배터리 연결·전원 확인** — 문제 없음(OpenCR 정상 부팅)
2. **손으로 바퀴 돌려보기** — 단단히 고정(토크 ON 확인, 통신 자체는
   살아있다는 뜻)
3. **`turtlebot3_node`의 `cmd_vel_callback` 소스 확인** — 실제 쓰기
   실패 메시지(`sdk_msg`)가 `RCLCPP_DEBUG`로만 찍혀서 기본 로그
   레벨에선 안 보인다는 걸 발견
4. **`--log-level debug`를 `ros2 launch`에 직접 못 넘김** (이 CLI
   버전은 `--log-level NODE:=LEVEL` 문법 미지원, `unrecognized
   arguments` 에러) → `turtlebot3_ros` 바이너리를 launch 없이 직접
   실행(`-i /dev/ttyACM0 --ros-args --params-file ... --log-level
   debug`)해서 우회. 이 과정에서 `namespace` 파라미터가 launch가 암묵적으로
   채워주던 값이라 직접 실행하면 `UninitializedStaticallyTypedParameterException`으로
   죽음 → `--params-file`로 `namespace: ""`를 명시하는 별도 yaml을 추가해서 해결
5. **디버그 로그로 실제 원인 확인**: `timeout 1.5 ros2 topic pub -r 5
   ...`로 보낸 명령이 `turtlebot3_node`의 `lin_vel` 디버그 로그에
   **한 줄도 안 찍힘** — 즉 애초에 노드까지 도달을 안 했다. 원인은
   **DDS discovery 지연**: 새로 띄운 `ros2 topic pub` 퍼블리셔가
   기존 구독자(`turtlebot3_node`)와 매칭되는 데 1\~2초가 걸리는데,
   `timeout 1.5`가 매칭 완료 전에 프로세스를 죽여버려서 메시지가
   단 하나도 실제로 발행되지 않았다(`--once`는 자체적으로 매칭을
   기다리는 로직이 있어서 이 문제를 안 겪음 — 그래서 정지 명령만
   항상 성공한 것처럼 보였다).
6. **해결**: `timeout`을 4초로 늘려서 재시도 → `lin_vel: 0.080000 ...
   msg: Succeeded to write data`가 9번 정상 로깅, `odom.pose.pose.
   position.x`가 0→0.35m로 실측 변화, 사용자가 바퀴 회전 육안 확인.

**교훈**: 실물 로봇에서 "명령을 보냈는데 안 움직인다"는 하드웨어
문제라고 바로 단정하지 말고, **명령이 애초에 목적지 노드까지
도달했는지부터**(구독자 매칭, discovery 시간, `--once` vs 반복 발행의
차이) 확인해야 한다. 이번 경우 하드웨어·펌웨어·배선 전부 처음부터
멀쩡했다.

Related: [[turtlebot3-ros-dd-study-book-progress]]
