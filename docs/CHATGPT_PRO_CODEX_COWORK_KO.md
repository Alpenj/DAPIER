# Codex cloud + 로컬 Codex GitHub 협업

이 저장소는 `pro/<task>` 원격 브랜치를 교환 지점으로 사용한다. Codex cloud는
어려운 문제나 UX 개선을 기존 브랜치에서 구현하고, 로컬 Codex는 별도 worktree에서
결과를 동기화해 빌드·테스트·실패 원인을 확인한다. 기존 작업 트리의 미커밋
파일은 건드리지 않는다.

이 문서의 `scripts/cowork` 흐름은 **Codex cloud와 command-line/local Codex 사이의
원격 브랜치 handoff** 전용이다. ChatGPT 데스크톱 앱의 Codex-managed Worktree와
Handoff를 사용할 때는 이 흐름을 함께 사용하지 않는다.

## 최초 한 번: ChatGPT와 GitHub 연결

1. ChatGPT에서 **Settings → Apps → GitHub**를 연다.
2. GitHub의 ChatGPT 앱을 승인하고 `Alpenj/DAPIER` 저장소만 선택한다.
3. ChatGPT의 **Codex**에서 `Alpenj/DAPIER` 환경을 만든다.
4. 일반 Chat에서 GitHub 앱을 붙이는 것만으로는 push할 수 없다. 코드를 수정하고
   커밋하려면 Codex의 Code 작업 또는 Codex 안의 Quick Chat을 사용한다.
5. 새 저장소가 보이지 않으면 GitHub 검색에서 `repo:Alpenj/DAPIER import`를 한 번
   실행하고 인덱싱을 기다린다.

저장소 권한은 `All repositories` 대신 `Only select repositories`로 제한한다.

## 모드 A: Codex cloud와 `scripts/cowork`

Codex cloud에서 원격 브랜치를 작업하고 로컬 command-line Codex에서 검증할 때만
이 모드를 사용한다.

### 매 작업 흐름

저장소 루트에서 작업 이름을 소문자 slug로 정한다.

```bash
cd ~/DAPIER
scripts/cowork start shoe-task-ux
scripts/cowork prompt shoe-task-ux
```

첫 명령은 다음 두 항목을 만든다.

- GitHub 브랜치: `pro/shoe-task-ux`
- 격리된 로컬 worktree: `.local-workspaces/pro/shoe-task-ux`

두 번째 명령의 출력에서 `Task` 부분을 구체적으로 채워 Codex cloud에 전달한다.
저장소와 **기존 브랜치 이름, Starting SHA, Expected remote SHA**를 유지하고,
완료하면 같은 브랜치에 normal push하도록 요청한다. remote SHA 변경이나
non-fast-forward가 발생하면 merge, rebase, reset, force push로 해결하지 않고
중단한다.

원격 작업이 끝나면 로컬에서 가져와 확인한다.

```bash
scripts/cowork sync shoe-task-ux
scripts/cowork status shoe-task-ux
scripts/cowork verify shoe-task-ux
```

기본 검증은 cowork shell 통합 테스트와 장비 없이 실행 가능한
`dapier_sim_first`, `casino_dealer` 단위 테스트다. `verify`는 clean worktree와
`HEAD == origin/pro/<task>`를 확인하고 검증한 정확한 SHA를 출력한다. 변경 영역에
자체 README나 테스트 명령이 있으면 그것도 해당 worktree 안에서 실행한다.
SIM/MOCK 통과를 HW 성공으로 해석하지 않는다.

결과가 괜찮으면 PR을 연다.

```bash
gh pr create \
  --repo Alpenj/DAPIER \
  --base main \
  --head pro/shoe-task-ux \
  --template .github/pull_request_template.md
```

`--fill`은 commit 정보로 본문을 만들기 때문에 이 저장소의 안전 체크 항목을
보장하지 않는다. PR을 제출하기 전에 template의 SHA, writer, SIM/MOCK/HW,
미실행 검증과 rollback 항목을 직접 채운다.

실패하면 로컬 Codex에 아래처럼 요청한다.

```text
pro/shoe-task-ux 브랜치의 원격 커밋을 검토해 줘.
재현 가능한 테스트를 먼저 실행하고, 빌드/테스트 실패의 원인을 분류한 뒤
최소 수정으로 고쳐. 실제 하드웨어는 구동하지 말고 SIM/MOCK/HW 근거를 구분해.
```

수정 후 다시 같은 브랜치에 push하고 다음 writer에게 순차적으로 handoff한다.
두 writer가 같은 브랜치를 동시에 작업하지 않는다.

## 모드 B: ChatGPT 데스크톱 Codex Worktree/Handoff

같은 PC의 ChatGPT 데스크톱 앱에서 **Worktree** 또는 **Hand off**를 사용할 때는
`scripts/cowork start`를 실행하지 않는다.

1. ChatGPT 데스크톱에서 프로젝트를 열고 새 chat의 **Worktree**를 선택한다.
2. 시작 branch를 고르면 Codex가 관리 worktree를 만든다. 기본 시작 상태는
   detached HEAD다.
3. worktree 안에서 계속 작업하려면 **Create branch here**로 별도 branch를 만든다.
4. 기존 local checkout으로 옮길 때는 같은 branch를 두 worktree에 checkout하지
   말고 **Hand off**를 사용한다. Handoff가 필요한 Git 작업을 처리한다.

Git은 같은 branch를 두 worktree에 동시에 checkout하지 못한다. 따라서 custom
`.local-workspaces/pro/<task>`와 Codex-managed Worktree 중 하나만 선택한다.

## 운영 원칙

- 한 branch에는 writer 한 명만 둔다.
- 병렬 시도는 `pro/shoe-task-ux-cloud-01`, `pro/shoe-task-ux-local-01`처럼
  writer별 branch로 분리한다.
- main이나 진행 중인 사람의 브랜치에 Codex가 직접 커밋하게 하지 않는다.
- 원격 결과는 테스트 전까지 제안으로 취급한다.
- token, `.env`, serial ID, 개인 calibration, 원시 dataset은 GitHub에 올리지 않는다.
- 로컬·원격 모든 에이전트는 승인 없이 serial open, torque 변경, EEPROM write,
  motor jog, 실물 `/cmd_vel`이나 actuator command를 실행하지 않는다.
- 실제 로봇 동작 검증은 사람이 현장에서 E-stop과 장치 profile을 확인하고 정확한
  명령을 승인한 뒤 interactive TTY에서 별도로 기록한다.
- `scripts/cowork`, Actions, Codex review는 GitHub ruleset과 사람의 merge 판단을
  대신하지 않는다.

## 공식 참고

- [Codex Worktrees](https://developers.openai.com/codex/app/worktrees)
- [Codex GitHub code review](https://developers.openai.com/codex/integrations/github)
- [Codex AGENTS.md 안내](https://developers.openai.com/codex/guides/agents-md)
- [GitHub CLI `gh pr create`](https://cli.github.com/manual/gh_pr_create)
