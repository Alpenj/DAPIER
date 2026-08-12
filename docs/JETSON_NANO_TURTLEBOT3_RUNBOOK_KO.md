# Jetson Nano(P3450) TurtleBot3 실물 구동 런북

이 문서는 [`turtlebot3_ws`](../turtlebot3_ws/README.md)에서 Gazebo로만 검증한
SLAM·Nav2 절차를, 실제 TurtleBot3(온보드 SBC = Jetson Nano)로 옮길 때 같은
실수를 반복하지 않으려고 적어둔 작업 순서다. 체크박스는 실제로 명령을
실행하고 화면·로그·로봇 움직임을 확인한 뒤에만 채운다.

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
- [x] TurtleBot3 실물 섀시 + Dynamixel 모터 — 조립·배선 및 실제
      전진·회전 확인(저전압 발생 후 재검증은 13절에 별도 미완료로 기록)
- [x] OpenCR 보드, Nano와 USB로 연결 확인(`/dev/ttyACM0`)
- [x] LDS-01(라이다) USB 연결 — 물리 회전과 `/scan` 약 4.99Hz 확인

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
- [x] OpenCR와 Dynamixel 실제 구동 확인 — 전진·제자리 회전 및 odom 변화
      모두 확인(11절)

## 7. PC 쪽: 기존 `turtlebot3_ws`(Jazzy) 그대로 사용

새 워크스페이스 안 만든다. docs.robotis.com이 Ubuntu 24.04+Jazzy를
SLAM/Navigation/Teleop/Simulation 전 영역에서 정식 지원하므로, 이미 있는
`~/DAPIER/turtlebot3_ws`를 그대로 실물 로봇 연결에 쓴다.

- [x] PC↔Nano DDS discovery 확인(2026-08-12) — PC의
      `ROS_LOCALHOST_ONLY=1`을 실물 전용 셸에서 해제하고, 양쪽
      처음에는 `ROS_DOMAIN_ID=30` + `rmw_cyclonedds_cpp` + 기본 멀티캐스트로
      연결. 전체 토픽 발견, `/odom` 19~20Hz, `/cmd_vel` 구독자 1개 확인.
      Claude가 만든 `~/cyclonedds_pc.xml`은 multicast를 끄고 static peer를
      지정했지만 실제 discovery에 실패했으므로 실물 스크립트에서 사용하지
      않는다.
- [x] **실물 전용 domain 73으로 격리(2026-08-12)** — 공유 Wi-Fi의 domain
      30에서 로봇과 무관한 `TwistStamped /cmd_vel` endpoint가 발견됐고,
      `/odom`도 기대 주기보다 많이 수신되며 timestamp가 역행했다.
      PC 실물 환경과 Jetson systemd bringup을 함께 domain 73으로 옮겨
      다른 ROS participant가 같은 이름의 토픽에 섞이지 않게 분리한다.
- [x] teleop의 `/cmd_vel` 타입 불일치 해결 — PC Jazzy의 ROBOTIS
      `teleop_keyboard`는 `TwistStamped`를 발행하지만 Nano Humble의
      `turtlebot3_node`는 plain `Twist`를 구독한다. 실물 전용
      `scripts/tb3_real_teleop.py`가 plain `Twist`를 20Hz로 발행하도록
      분리했다. 직선 가속도는 0.35m/s², 각가속도는 1.2rad/s²로 제한해
      `a/d` 조향이 갑자기 튀지 않게 했고, `s`·Space·종료 시 정지는
      지연 없이 즉시 보낸다.

## 8. 실물 SLAM (교재 Part1과 동일 목적, 실물판)

```bash
# Nano bringup은 turtlebot3-bringup.service가 부팅 시 자동 실행

# 터미널 1: Cartographer + RViz (지도 저장 전까지 열어둘 것)
tb3-slam

# 터미널 2: 고정 저속 teleop
tb3-teleop

# 터미널 3: 지도 상태 확인 후 저장
tb3-slam-check
tb3-map-save my_room
```

- [x] `robot.launch.py` 정상 기동 — `turtlebot3_node`, `diff_drive_
      controller`, `robot_state_publisher`, `hlds_laser_publisher`
      전부 살아있음. Jetson 호스트의 `turtlebot3-bringup.service`로
      등록해 Docker 이후 자동 시작한다. 단, `Restart=on-failure`는 부모
      launch가 살아 있는 상태의 자식 노드 사망을 감지하지 못했으며,
      이 문제의 수정·재검증 상태는 13절에 기록한다.
- [x] **바퀴 실제 이동 확인 완료(2026-08-11)** — `ros2 topic pub -r 5
      /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}}"`로 전진,
      `{angular: {z: 0.3}}`로 제자리 회전. `odom` 위치·orientation
      quaternion 둘 다 유의미하게 변화, 사용자가 눈으로 직접 전진·회전
      확인. 아래 11절 트러블슈팅 참고(원인은 하드웨어가 아니라 테스트
      방법 문제였음)
- [x] **라이다 `/scan` 정상 발행 확인(2026-08-11)** — 물리적으로도
      회전 중이고, `ros2 topic hz /scan`으로 ~4.99Hz 안정적 발행,
      `ranges` 값도 0.38~0.68m대 실측치로 확인. 아래 12절 트러블슈팅
      참고(컨테이너에 `--device=/dev/ttyUSB0` 누락이 원인이었음)
- [x] **PC 키보드 teleop 경로 검증(2026-08-12)** — 실물 실행 전 점검,
      plain `Twist` publisher/subscriber 매칭, 0속도 메시지, Ctrl-C 때
      정지 메시지 3회 발행 후 정상 종료까지 확인. 바퀴를 띄운 실물에서
      전진·후진·좌회전·우회전·좌곡선·우곡선 6단계를 실행했고, 모든
      단계에서 좌우 wheel joint 위치와 속도가 명령 방향대로 변해 통과했다.
- [x] **실물 Cartographer 지도 생성 확인(2026-08-12)** — `/scan`
      약 4.99Hz와 `/odom` 약 19.27Hz를 입력으로 받아 `/map` 발행,
      `map -> odom` TF 연속 발행 확인. 정지 상태의 첫 지도는 0.05m/cell,
      61x71 cells로 생성됨
- [x] **지도 저장 경로 확인(2026-08-12)** — `map_saver_cli`로 생성 중인
      지도를 임시 경로에 저장해 61x71 `PGM`과 `YAML`이 모두 생성되고
      YAML의 image/resolution/origin 메타데이터가 정상임을 확인.
      사용자 명령은 `tb3-map-save <이름>`, 결과는 `~/maps/`에 저장
- [x] **실제 주행 지도 최종 저장(2026-08-12)** — 복도와 장애물 주변을
      TELEOP으로 주행한 뒤 정지 상태를 25초 동안 확인하고
      `~/maps/dapier_real_20260812.{yaml,pgm}`으로 저장했다. 최종 지도는
      0.05m/cell, 887x206 cells이며 PGM에 장애물 3,225셀, 자유 공간
      44,367셀, 미확인 공간 135,130셀이 들어 있다. YAML/PGM 구조,
      raster 길이, 비정상 픽셀 0개와 SHA-256까지 확인했다. 진행 중
      체크포인트 `dapier_real_checkpoint_20260812`도 별도로 보존했다.
- [x] **Cartographer odometry 시간 역행 근본 해결(2026-08-12)** — domain
      30에서는 12초 동안 PC 수신 338개 중 70회 timestamp 역행이 있었고
      Cartographer가 fatal 종료됐다. 전용 domain 73 격리 후 직접 콜백
      audit 결과 PC 238개/Jetson 241개 모두 중복 0, 역행 0이었다.
      따라서 `use_odometry=false` 임시안은 채택하지 않고 삭제했으며,
      ROBOTIS 공식 `use_odometry=true` 설정을 그대로 사용한다.

### 새 터미널에서 실제로 갖고 놀면서 지도 만들기

1. 로봇 전원을 켜고 약 30초 기다린다. Jetson의 Docker 컨테이너와
   `turtlebot3-bringup.service`는 자동 시작하므로 SSH 터미널은 필요 없다.
2. **터미널 1**에서 `tb3-slam`을 실행한다. 연결 점검이 통과하면 RViz가
   열리고 지도가 나타난다. 지도 저장 전까지 이 터미널을 닫지 않는다.
   모터/OpenCR 전원만 껐다 켜고 Jetson은 계속 살아 있었다면 먼저
   `tb3-restart`를 한 번 실행한다. 이 명령은 Jetson bringup을 재시작하고,
   CycloneDDS의 이전 endpoint가 사라져 `/cmd_vel` 구독자와 `/odom`
   발행자가 각각 정확히 1개가 될 때까지 기다린다. 그 다음
   `tb3-slam`을 실행한다.
   평상시 실행은 `tb3-ready --mode slam`을 자동 호출한다. ROS daemon을
   재시작하지 않고 odom 5개, TF 2개, 배터리/torque 2개, 라이다 2개를
   동시에 받아 조건이 채워지는 즉시 Cartographer를 시작한다. endpoint가
   정확히 하나인지, timestamp 중복·역행이 없는지, 배터리가 11.1V
   이상인지, torque가 켜졌는지, 라이다에 유효 거리값이 있는지는 그대로
   검사한다. 즉 안전 조건을 빼서 빨라진 것이 아니라 순차 대기와 반복적인
   daemon reset을 없앤 것이다.
3. **터미널 2**에서 `tb3-teleop`을 실행한다. `w` 전진, `x` 후진,
   `a/d` 주행 중 좌/우 조향, `r` 직진 복귀, `s` 또는 Space 정지다.
   직선 속도는 0.02m/s 단계로 최대 +/-0.18m/s, 회전은 0.15rad/s 단계다.
   곡선주행은 최대 1.10rad/s, 제자리 회전은 최대 1.50rad/s다. 좌우 바퀴
   각각 0.22m/s 이하이고 안쪽 바퀴가 바깥쪽의 20% 이상 돌도록 자동
   제한된다. 스포츠 최고속에서 조향하면 직선속도를 필요한 만큼 자동으로
   낮춘다. 명령은 20Hz로 갱신하고, 전진·조향은 가속도 제한을 거쳐
   부드럽게 바뀌지만 정지 키는 즉시 0속도를 보낸다. 처음 설치한 날에만
   `tb3-wheel-test`로 바퀴를 띄워 6방향을 확인한 뒤 바닥에 내려놓는다.
   넓은 바닥에서 지도 작성이 아닌 주행만 할 때는
   `tb3-teleop --sport`로 공식 직선 상한 0.22m/s를 쓸 수 있다.
4. 직선과 완만한 회전을 섞고, 지나간 곳을 다시 방문하면서 천천히
   주행한다. 라이다 앞을 손이나 몸으로 가리지 않는다.
5. **터미널 3**에서 `tb3-slam-check`을 실행한다. `OK`와 지도 크기가
   나오면 `tb3-map-save my_room`처럼 저장한다. 결과는
   `~/maps/my_room.yaml`과 `~/maps/my_room.pgm`이다.
6. 저장 파일 두 개가 출력된 것을 확인한 뒤 터미널 2에서 `Ctrl+C`,
   터미널 1에서 `Ctrl+C` 순서로 종료한다.

### 빠른 시작 점검과 정밀 진단의 구분

- `tb3-ready`: 평상시 시작용이다. `tb3-teleop`, `tb3-slam`, `tb3-nav`가
  목적에 맞는 mode로 자동 실행한다. TELEOP은 라이다를 사용하지 않으므로
  drive mode에서는 라이다를 기다리지 않는다.
- `tb3-check`: 전원을 껐다 켠 뒤, 빨간 LED/비프음/모터 이상이 있었을 때,
  DDS endpoint가 중복됐을 때, 또는 `tb3-ready` 실패 원인을 자세히 볼 때
  수동 실행한다. ROS daemon을 초기화하고 각 항목을 더 길게 순차 검사한다.
- `tb3-restart`: Jetson bringup을 실제로 재시작해 예전 DDS endpoint가
  사라지는지까지 봐야 하므로 의도적으로 `tb3-check` 정밀 경로를 유지한다.

로봇 전원이 꺼진 상태의 빠른 점검은 실제 측정에서 약 3.1초 안에 실패했고,
0개의 endpoint/odom/TF/배터리/torque를 구체적으로 보고했다. 정상 로봇의
실제 통과 시간은 다음 가동 때 별도로 기록한다.

## 9. 실물 Nav2 자율주행 (교재 Part2와 동일 목적, 실물판)

```bash
tb3-nav my_room
```

`tb3-nav`은 `~/maps/my_room.yaml`과 PGM을 확인하고, SLAM이 완전히
종료된 뒤에만 추적 가능한 독립 패키지 `dapier_turtlebot3_real`의 Nav2를
시작한다. 타입이 보존된 실물 전용 YAML에서 모든
`enable_stamped_cmd_vel`을 false로 두고 자율주행 직선 속도는 0.18m/s로
제한한다. 전달받은 지도 절대경로를 `map_server`에 직접 주입하므로 패키지의
가상 예제 `map.yaml`을 잘못 여는 일이 없다. RViz에서 먼저
**2D Pose Estimate**로 로봇의 실제 위치·방향을 찍고, AMCL이 수렴한 뒤
새 터미널에서 `tb3-nav-check my_room`이 `OK`인지 확인한다. 이 검사는
`map_server`가 지정한 YAML을 정확히 열었는지와 실시간 `/map`의
크기·해상도가 저장된 PGM/YAML 쌍과 일치하는지까지 검사한다. 그 다음에만
새 터미널에서 `tb3-nav-watch`를 실행하고 **Nav2 Goal**을 찍는다. watcher는
실행 전에 남아 있던 action status를 기준선에서 제외하고 새 goal UUID의
상태만 추적하며, `SUCCEEDED`일 때만 성공 종료한다. `CANCELED`, `ABORTED`,
action server 없음과 timeout은 모두 실패다.

- [x] 실물 전용 패키지 빌드, YAML 타입, 지도 경로 로드 확인 — 정지 상태
      검증 지도 `domain73_official_validation`을 정확히 37x32 cells로 로드
- [x] 충전된 배터리로 `/odom` 및 `odom -> base_footprint` TF 재검증
- [x] `/initialpose` 전송 후 AMCL 수렴 — 라이다 끝점 82.3%가 지도 벽
      10cm 이내에 일치
- [x] `FollowWaypoints`로 6개 목표 실제 이동 + `SUCCEEDED` 수신
- [x] 충돌·이탈·누락 없이 19.52m 계획 완주, odometry 누적 21.69m

2026-08-12 첫 시도는 preflight에서 좌측 Dynamixel 빨간 LED와
`torque=false`를 검출해 목표를 보내지 않았다. 충전·냉각 후 재시작한 두 번째
시도에서 정밀 preflight(odom 74개, 중복/역행 0, 배터리 최저 11.76V,
torque true, 라이다 18 scan)를 통과했다. 초기 자세가 늦게 들어오며 Nav2가
부분 활성화된 상태는 lifecycle manager `RESUME`으로 정상화했고 이후 필수
노드는 모두 active였다.

처음 0.30m 목표는 upstream `SimpleGoalChecker.xy_goal_tolerance=0.25` 때문에
약 0.11m만 움직이고도 `SUCCEEDED`가 됐다. 성공으로 과장하지 않고 tolerance를
0.08m, yaw tolerance를 0.15rad로 낮췄다. 그 뒤 3m → 6m → 9m → 6m →
3m → 출발점의 6개 웨이포인트를 `ComputePathThroughPoses`로 먼저 검사했다.
계획은 757 pose, 19.52m였고 실제 결과는 `status=4`, `error_code=0`,
`missed=[]`, odometry 누적 21.69m였다. `/cmd_vel_nav`,
`/cmd_vel_smoothed`, 최종 `/cmd_vel` 모두 최대 0.18m/s와 1.0rad/s를 기록해
계획·제어·충돌감시·실물 전달 전체를 확인했다.

재현 명령은 다음과 같다. 첫 명령은 좌표를 실행하지 않고 전체 연결 경로만
검사한다. 두 번째 명령의 `--execute`는 실물 구동을 명시적으로 허용한다.

```bash
tb3-waypoints \
  --pose 3.025 0.152 0 --pose 6.025 0.152 0 \
  --pose 9.025 0.102 180 --pose 6.025 0.152 180 \
  --pose 3.025 0.152 180 --pose 0.100 0.030 -47.3

tb3-waypoints --execute \
  --pose 3.025 0.152 0 --pose 6.025 0.152 0 \
  --pose 9.025 0.102 180 --pose 6.025 0.152 180 \
  --pose 3.025 0.152 180 --pose 0.100 0.030 -47.3
```

다른 지도에서 이 좌표를 재사용하지 않는다. RViz에서 새 지도의 흰색
자유공간 좌표를 확인해 바꾸고, 계획 명령의 `PLAN OK`를 먼저 확인한다.

## 10. 예상 트러블 포인트

- Nano USB 배럴잭 5V/2A 전원으로는 Wi-Fi+카메라+USB 허브(OpenCR+LDS) 동시
  부하 시 브라운아웃 가능 → 문제 생기면 5V/4A 배럴잭 전원으로 교체
- Docker 컨테이너 안에서 시리얼 장치(`/dev/ttyACM0`, `/dev/ttyUSB0`)가 안
  보이면 `--device` 플래그 누락이거나 컨테이너 밖에서 장치명이 바뀐 것
  (재부팅/재연결마다 번호가 바뀔 수 있음, `dialout` 그룹 권한도 확인)
- PC(Jazzy 계열 최근 설치)에 Humble을 apt 네이티브로 못 깔면 PC도 Docker로
  통일 — 이 경우 절차 7을 Docker 버전으로 다시 씀
- `turtlebot3_ws`(Jazzy) 교재에서 겪은 `/cmd_vel` 타입 이슈
  (`TwistStamped` vs `Twist`)는 실물 전용 teleop 스크립트에서 plain
  `Twist`로 해결했다. Jazzy↔Humble 연결 시 출력되는 type-hash 경고는
  관찰되지만, 실제 topic discovery·구독 매칭·odom 데이터 수신은 정상이다.

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

## 12. 라이다 조립 후 `hlds_laser_publisher`가 계속 죽던 문제

라이다를 나중에 조립해서 USB로 연결한 뒤 `robot.launch.py`를 다시
띄웠는데, `hlds_laser_publisher`가 `exit code 255`로 계속 죽었다.
`/dev/ttyUSB0`는 udev rule 덕분에 호스트에선 정상 인식됐는데(모터도
물리적으로 실제 회전 중이었음), 소프트웨어만 실패하는 게 이상했다.

디버깅 순서:
1. `hlds_laser_publisher`를 launch 없이 직접 실행해보니 처음엔
   `frame_id` 파라미터 누락 예외 — 이건 내가 파라미터를 안 챙겨서
   생긴 재현 오류였고 실제 원인이 아니었음
2. `port`/`frame_id`를 제대로 넘겨서 재실행해도 여전히 조용히
   `exit 255`. 소스(`hlds_laser_publisher.cpp`)를 보니
   `catch (boost::system::system_error & ex) { return -1; }`로
   **예외 메시지 자체를 버리고 종료 코드만 반환**하는 구조라 로그로는
   원인을 알 수 없는 게 정상이었음
3. `stty -F /dev/ttyUSB0 ...`로 직접 열어보려 하니
   `stty: /dev/ttyUSB0: No such file or directory` — **호스트에는
   있는데 Docker 컨테이너 안에는 이 장치 노드가 없었다**
4. 원인: 라이다가 아직 연결 안 된 시점에 컨테이너를 만들면서
   `--device=/dev/ttyACM0`만 넣었었고, 나중에 라이다를 연결해도
   이미 떠있는 컨테이너에는 새 장치가 자동으로 추가되지 않음
   (Docker `--device`는 컨테이너 생성 시점에 고정)
5. **해결**: 컨테이너를 `--device=/dev/ttyACM0 --device=/dev/ttyUSB0`
   둘 다 넣어서 재생성. 이후 `hlds_laser_publisher`가 안 죽고
   `/scan`이 `ros2 topic hz`로 ~4.99Hz 안정적으로 발행되는 것 확인

**교훈**: USB 주변기기를 나중에 추가로 연결할 계획이면, 컨테이너를
처음 만들 때 **아직 안 꽂혀 있어도 예상되는 `/dev/ttyACM*`,
`/dev/ttyUSB*` 전부 미리 `--device`로 넣어두는 게 낫다** (또는
`--device-cgroup-rule`로 문자 디바이스 클래스 전체를 허용하거나,
`--privileged`로 띄우는 방법도 있지만 이번엔 device 목록을 명시하는
쪽을 택함). 나중에 장치를 추가했으면 컨테이너 재생성이 필요하다는 것도
기억해둘 것.

## 13. OpenCR USER3 빨간 LED + 반복 비프음 (2026-08-12)

실물 Nav2 검증 직전 `turtlebot3_node`에서 Dynamixel
`There is no status packet`이 연속 발생했고, 이어 stack-smashing으로
프로세스가 종료됐다. 부모 `ros2 launch`는 살아 있어 systemd 서비스는
`active`로 보였지만 `/odom`과 `odom -> base_footprint` TF는 사라진
상태였다. 재시작 직후에는 `Failed connection with Devices`로 다시 종료됐다.

현장에서 확인한 **OpenCR USER3 빨간 LED + 반복 비프음**은 공식 OpenCR
TurtleBot3 펌웨어 소스상 저전압 경보다. `LED_LOW_BATTERY=2`가 USER LED
배열의 세 번째인 USER3을 가리키고, 입력이 11.1V 미만이면 점등한다. 평균
전압이 약 11.0V 미만으로 5회 확인되면 Dynamixel 전원을 끄고 1kHz 비프를
0.5초씩 반복한다. 따라서 이번 `No status packet`은 저전압 보호로 모터
전원이 차단된 결과와 일치한다.

복구 순서:

1. 로봇 전원을 즉시 끈다.
2. 방전 배터리를 OpenCR에서 분리한다.
3. 정품/규격에 맞는 3S LiPo 충전기로 완충한다. OpenCR에 연결한 채 충전과
   방전을 동시에 하지 않는다.
4. 충전된 배터리를 다시 연결하고 전원을 켠 뒤 USER3과 반복 비프가 없는지
   확인한다.
5. `tb3-restart` 후 `/cmd_vel` 구독자, `/odom`, TF, 실제 양쪽 바퀴를 다시
   확인한다. 이 검증 전에는 SLAM/Nav2 성공으로 기록하지 않는다.

자식 `turtlebot3_ros`가 죽어도 부모 launch가 살아 systemd가 놓치는 문제는
`turtlebot3_ws/patches/turtlebot3_bringup_respawn.patch`로 노드에 5초
respawn을 적용했다. 2026-08-12 충전 후 Jetson의 Humble 소스에 패치를
적용해 `turtlebot3_bringup` 재빌드와 서비스 재시작을 완료했다. 정상 상태의
`turtlebot3_ros` PID 490을 의도적으로 `SIGTERM` 종료하자 5초째 PID 565로
재생성됐고, 이어서 `/odom` 81개(중복·역행 0), typed TF 2회, 배터리 최저
12.240V, torque 3회 연속 true, LiDAR 19스캔(유효 거리점 4,833개)을 모두
다시 통과했다. 따라서 이 자동복구 경로는 실물에서 검증 완료다.

### 2022년 OpenCR ROS2 모터 무반응 결함과의 구분

ROBOTIS 포럼의 2022-06-10 게시글(Post 2591905)은 토픽·라이다·SLAM은
정상인데 teleop/Nav2 명령에 모터가 전혀 반응하지 않은 사례다. 공식 답변은
당시 OpenCR ROS2 펌웨어 결함이라고 밝혔고, 연결된 수정 커밋은 초기화 중
누락된 `dxl_slave.begin()`을 추가한다. 이 수정은 OpenCR 1.5.0부터 포함돼
있고, 공식 ROS2 바이너리 저장소에도 2022-06-24에 수정 펌웨어가 올라왔다.

현재 보드에 실제로 업로드한 파일은 공식 `ROS2/latest`와 같은
**0.2.1 / `V230127R1`** 바이너리다. 이는 위 수정 배포보다 뒤 버전이고,
같은 펌웨어에서 실제 전진·제자리 회전도 이미 성공했다. 따라서 2022년의
`dxl_slave.begin()` 누락을 이번 USER3+비프음의 직접 원인으로 보지 않는다.
USER3과 반복 비프는 공식 펌웨어의 저전압 분기와 정확히 일치한다.

다만 충전 후 다음 두 조건을 별도로 다시 확인한다.

1. `tb3-check`에서 배터리 11.1V 이상, OpenCR torque `true`, `/odom`과
   `/scan`이 모두 정상인지 확인한다.
2. 바퀴를 띄운 상태에서 `tb3-teleop`으로 전진·회전시켜 양쪽 모터가 모두
   반응하는지 확인한다. 센서·SLAM은 정상인데 모터만 계속 무반응이면 그때
   펌웨어 재플래시와 Dynamixel 통신을 다시 조사한다.

참고: ROBOTIS Forum Post 2591905, ROBOTIS-GIT/OpenCR PR #309 commit
`4e60a84e`, ROBOTIS-GIT/OpenCR-Binaries ROS2 0.2.1.

Related: [[turtlebot3-ros-dd-study-book-progress]]

## 14. 좌측 XL430 빨간 LED + Torque OFF (2026-08-12)

실제 매핑을 끝내고 Nav2로 전환하던 중 좌측 XL430의 빨간 LED가 다시
점등됐다. 이번에는 OpenCR USER3 저전압 LED와 구분해야 한다. Nav2
preflight가 읽은 OpenCR 배터리는 최저 11.87V였지만 `/sensor_state.torque`가
`false`로 바뀌었고 곧 Jetson endpoint도 사라졌다. 저장 지도에는 영향이
없으며 자율주행 목표도 보내기 전이었다.

XL430 공식 제어표에서 보호 shutdown의 기본 원인은 과부하(bit 5), 회로
충격/구동 전력 부족(bit 4), 과열(bit 2)이다. 설정에 따라 엔코더(bit 3)와
입력전압(bit 0)도 원인이 될 수 있다. shutdown이 발생하면 Torque Enable이
0으로 지워지고 모터 출력이 0%가 되며, 펌웨어 v41 이상에서는 모터 LED가
1초 주기로 깜빡인다. 재부팅 전까지 torque를 다시 켤 수 없고, 과열이면
최소 20분 이상 식힌 뒤 재사용하라는 것이 공식 지침이다.

현재 ROS2 OpenCR 펌웨어와 `turtlebot3_node`가 노출하는 외부 제어표에는
두 모터의 현재 전류·속도·위치와 합산 torque 상태만 있고, 각 XL430의
`Hardware Error Status(70)`, `Present Input Voltage(144)`, `Present
Temperature(146)`는 없다. 따라서 이번 기록만으로 과열·과부하·전원 중
하나를 단정하지 않는다. 다음 재시도 순서는 다음과 같다.

1. 전원을 끄고 모터가 뜨거우면 20분 이상 식힌다. 뜨거운 상태에서 바로
   재가동하지 않는다.
2. 전원이 꺼진 상태에서 좌우 바퀴를 손으로 같은 힘으로 돌려 좌측만
   뻑뻑한지, 타이어/프레임/혼이 닿는지, 축이 비뚤어졌는지 확인한다.
   전원이 들어온 상태에서는 케이블을 뽑거나 끼우지 않는다.
3. 좌측 XL430의 양쪽 TTL 케이블과 OpenCR 연결부가 완전히 체결됐는지
   확인한다. 케이블 피복 손상과 눌림도 확인한다.
4. 바퀴를 바닥에서 띄우고 전원을 켠 뒤 `tb3-restart`, `tb3-check`를
   실행한다. LED가 다시 켜지거나 torque가 한 번이라도 false면 주행하지
   않는다.
5. 냉간·무부하에서도 좌측만 곧바로 재발하면 단순 배터리 문제로 처리하지
   않는다. 정확한 오류 비트를 읽으려면 OpenCR에 공식 `usb_to_dxl`
   진단 스케치를 올려 DYNAMIXEL Wizard 2.0으로 좌측 모터를 검사해야 한다.
   이 작업은 현재 TurtleBot3 펌웨어를 지우므로 진단 후 ROS2 Burger
   펌웨어를 다시 플래시해야 하며, 실제 재발 시 별도 정비 단계로 수행한다.

참고: ROBOTIS XL430-W250 e-Manual의 Shutdown(63), Hardware Error
Status(70), Present Input Voltage(144), Present Temperature(146), ROBOTIS
DYNAMIXEL Wizard 2.0 문서, ROBOTIS-GIT/OpenCR `turtlebot3_ros2` 소스.
