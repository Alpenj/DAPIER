# TurtleBot3 Raspberry Pi 원격 접속 설정

현재 공개 가능한 계정 식별자는 다음 하나뿐이다.

```text
username: user
```

비밀번호는 저장소, PR 본문, 명령행 인자, shell history 또는 CI 로그에 기록하지
않는다. 원격 접속은 SSH key를 기본으로 사용한다.

## 로컬 설정 파일

```bash
cd ~/DAPIER/2ARM_ROBOT
mkdir -p config/local
cp config/turtlebot3.env.example config/local/turtlebot3.env
chmod 700 config/local
chmod 600 config/local/turtlebot3.env
```

`config/local/turtlebot3.env`에서 `TB3_HOST`를 실제 IP 또는 확인된 hostname으로
바꾼다. `TB3_USER=user`와 `TB3_AUTH_MODE=ssh-key`는 유지한다. 이 로컬 파일은
Git에서 제외된다.

## SSH key 등록

```bash
ssh-keygen -t ed25519 -f ~/.ssh/dapier_turtlebot3
ssh-copy-id -i ~/.ssh/dapier_turtlebot3.pub user@<TURTLEBOT3_IP>
```

`ssh-copy-id`의 최초 대화형 인증에서만 현재 비밀번호를 직접 입력한다. 비밀번호를
스크립트에 넣거나 `sshpass`, URL, environment variable로 자동 전달하지 않는다.
등록 후 로컬 설정에 다음 경로를 넣는다.

```text
TB3_SSH_KEY_PATH=/home/<LOCAL_USER>/.ssh/dapier_turtlebot3
```

접속 확인:

```bash
cd ~/DAPIER/2ARM_ROBOT
set -a
source config/local/turtlebot3.env
set +a

ssh_args=(-o BatchMode=yes -p "$TB3_SSH_PORT")
if [[ -n "${TB3_SSH_KEY_PATH:-}" ]]; then
  ssh_args+=(-i "$TB3_SSH_KEY_PATH")
fi
ssh "${ssh_args[@]}" "$TB3_USER@$TB3_HOST"
```

## 아키텍처 경계

SSH는 배포와 사람이 승인한 진단에만 사용한다. Python 연구 계층이 SSH를 통해
`/cmd_vel`, trajectory, motor register 또는 torque 명령을 보내는 경로를 만들지
않는다. 실제 제어는 Raspberry Pi에서 실행되는 C++ 제어 계층이 local watchdog,
limit, interlock과 safe-stop을 적용한 뒤 담당한다.

원격 접속 자체도 실물 장비 접근이다. 실제 ROS graph 또는 장치가 연결된 상태의
명령은 루트 `AGENTS.md`의 하드웨어 승인 조건을 만족한 뒤 사람의 현장 확인 아래
실행한다.
