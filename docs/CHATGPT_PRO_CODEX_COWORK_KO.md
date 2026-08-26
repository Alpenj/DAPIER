# ChatGPT Pro + 로컬 Codex GitHub 협업

이 저장소는 `pro/<task>` 원격 브랜치를 교환 지점으로 사용한다. ChatGPT의 Pro
모델은 Codex에서 어려운 문제나 UX 개선을 구현해 같은 브랜치에 커밋하고, 로컬
Codex는 별도 worktree에서 결과를 동기화해 빌드·테스트·실패 원인을 확인한다.
기존 작업 트리의 미커밋 파일은 건드리지 않는다.

## 최초 한 번: ChatGPT와 GitHub 연결

1. ChatGPT에서 **Settings → Apps → GitHub**를 연다.
2. GitHub의 ChatGPT 앱을 승인하고 `Alpenj/DAPIER` 저장소만 선택한다.
3. ChatGPT의 **Codex**에서 `Alpenj/DAPIER` 환경을 만든다.
4. 일반 Chat에서 GitHub 앱을 붙이는 것만으로는 push할 수 없다. 코드를 수정하고
   커밋하려면 Codex의 Code 작업 또는 Codex 안의 Quick Chat을 사용한다.
5. 새 저장소가 보이지 않으면 GitHub 검색에서 `repo:Alpenj/DAPIER import`를 한 번
   실행하고 인덱싱을 기다린다.

저장소 권한은 `All repositories` 대신 `Only select repositories`로 제한한다.

## 매 작업 흐름

저장소 루트에서 작업 이름을 소문자 slug로 정한다.

```bash
cd ~/DAPIER
scripts/cowork start shoe-task-ux
scripts/cowork prompt shoe-task-ux
```

첫 명령은 다음 두 항목을 만든다.

- GitHub 브랜치: `pro/shoe-task-ux`
- 격리된 로컬 worktree: `.local-workspaces/pro/shoe-task-ux`

두 번째 명령의 출력에서 `Task` 부분을 구체적으로 채워 Codex Quick Chat의 Pro
모델에 전달한다. 저장소와 **기존 브랜치 이름을 명시**하고, 완료하면 같은
브랜치에 commit/push하도록 요청한다.

원격 작업이 끝나면 로컬에서 가져와 확인한다.

```bash
scripts/cowork sync shoe-task-ux
scripts/cowork status shoe-task-ux
scripts/cowork verify shoe-task-ux
```

기본 검증은 장비 없이 실행 가능한 `dapier_sim_first`와 `casino_dealer` 단위
테스트다. 변경 영역에 자체 README나 테스트 명령이 있으면 그것도 해당 worktree
안에서 실행한다. SIM/MOCK 통과를 HW 성공으로 해석하지 않는다.

결과가 괜찮으면 PR을 연다.

```bash
gh pr create \
  --repo Alpenj/DAPIER \
  --base main \
  --head pro/shoe-task-ux \
  --fill
```

실패하면 로컬 Codex에 아래처럼 요청한다.

```text
pro/shoe-task-ux 브랜치의 원격 커밋을 검토해 줘.
재현 가능한 테스트를 먼저 실행하고, 빌드/테스트 실패의 원인을 분류한 뒤
최소 수정으로 고쳐. 실제 하드웨어는 구동하지 말고 SIM/MOCK/HW 근거를 구분해.
```

수정 후 다시 같은 브랜치에 push하고 Pro 모델에 남은 UX 문제만 좁혀서 맡긴다.

## 운영 원칙

- 작업 하나당 브랜치 하나를 사용한다.
- main이나 진행 중인 사람의 브랜치에 Pro가 직접 커밋하게 하지 않는다.
- 원격 결과는 테스트 전까지 제안으로 취급한다.
- token, `.env`, serial ID, 개인 calibration, 원시 dataset은 GitHub에 올리지 않는다.
- 실제 로봇 동작 검증은 사람이 현장에서 안전 조건을 확인한 뒤 별도로 기록한다.
