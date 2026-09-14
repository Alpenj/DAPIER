# Codex cloud + 로컬 Codex GitHub 협업

이 저장소는 `pro/<task>` 원격 브랜치를 교환 지점으로 사용한다. Codex cloud는
어려운 문제나 UX 개선을 기존 브랜치에서 구현하고, 로컬 Codex는 별도 worktree에서
결과를 동기화해 빌드·테스트·실패 원인을 확인한다. 기존 작업 트리의 미커밋
파일은 건드리지 않는다.

이 문서의 `scripts/cowork` 흐름은 **Codex cloud와 command-line/local Codex 사이의
원격 브랜치 handoff** 전용이다. ChatGPT 데스크톱 앱의 Codex-managed Worktree와
Handoff를 사용할 때는 이 흐름을 함께 사용하지 않는다.

## main 통합 이후 작업·업로드 — 2026-09-07

record_id: `DAPIER-2026-09-07-post-merge-upload-workflow`

나는 #53으로 통합한 main을 새 작업의 출발점으로 사용한다. 코드·자체 작성 문서는
`Alpenj/DAPIER`의 작업 브랜치에 올리고 PR 목적지는 항상 `main`으로 지정한다.
Codex와 Hermes는 브랜치를 따로 사용한다. 원시 dataset, 장치 identity/calibration과
개인 설정은 로컬에 보관하며 코드와 함께 일괄 add하지 않는다.

Mode A의 새 작업 예:

```bash
cd ~/DAPIER
scripts/cowork start dual-so101-codex-20260907 origin/main
cd .local-workspaces/pro/dual-so101-codex-20260907
git config branch.pro/dual-so101-codex-20260907.gh-merge-base main
```

Hermes 새 작업은 `dual-so101-hermes-20260907`처럼 writer를 바꾼 slug를 사용한다.
`start`는 원격 작업 브랜치와 upstream까지 설정한다. 변경 파일을 선별하고 해당 영역의
검증을 마친 뒤 같은 브랜치에서 commit과 `git push`를 수행한다. PR은 다음처럼 지정한다.

```bash
gh pr create --repo Alpenj/DAPIER --base main \
  --head pro/dual-so101-codex-20260907 \
  --template .github/pull_request_template.md
```

이미 작성 중이던 월요일 Hermes WIP는 새 작업과 구분해
`pro/dual-so101-monday-prep-hermes-20260907`에 보존한다. 이 브랜치의 원래 기준은
`3ff180a`이며, 작업 파일을 보존하기 위해 최신 main으로 강제 이동하지 않는다.
이전 로컬 source remote는 `local-source`로 보존하고, 업로드용 `origin`은 GitHub를 가리킨다.
WIP를 실제 commit/push하기 전 main의 converter/planner/checkpoint 수정과 겹치는 부분을
검토해야 한다. upstream 설정이나 빈 시작 브랜치 push는 미커밋 코드의 검증·업로드가 아니다.

작업 종료 후에는 main에 포함됐는지, 미푸시 커밋이 없는지, worktree가 clean인지 확인한
브랜치만 정리한다. 작업 브랜치를 삭제한 다음 작업은 다시 최신 main에서 시작한다.
Mode B의 Codex-managed worktree에서는 아래 별도 Handoff 절을 따른다.

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
이 모드를 사용한다. 이 모드에만 root `AGENTS.md`의 `pro/<task>` branch 고정
규칙이 적용된다.

### 매 작업 흐름

저장소 루트에서 작업 이름을 소문자 slug로 정한다.

```bash
cd ~/DAPIER
scripts/cowork doctor
scripts/cowork start shoe-task-ux
scripts/cowork prompt shoe-task-ux
```

`doctor`는 origin, Git worktree, primary checkout의 dirty 상태, Python/NumPy,
GitHub CLI 인증, repository ruleset과 active `pro/**` branch를 한 번에 점검한다.
경고는 작업을 막지 않지만 `FAIL`은 먼저 해결한다.

`start`는 다음 세 항목을 만든다.

- GitHub 브랜치: `pro/shoe-task-ux`
- 격리된 로컬 worktree: `.local-workspaces/pro/shoe-task-ux`
- Git common directory의 private handoff manifest: task, writer, base SHA,
  expected remote SHA, required tests와 `hardware_allowed=false`

Manifest는 worktree 안에 두지 않으므로 Git status와 `verify`의 clean 조건을
오염시키지 않는다. `sync`는 expected remote SHA를 명시적으로 갱신하고 기존
검증 기록을 비운다. `verify`가 성공해야 현재 SHA가 verified SHA로 기록된다.

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

결과가 괜찮으면 검증한 exact SHA로 PR을 연다.

```bash
scripts/cowork pr shoe-task-ux --draft
```

`pr`은 clean worktree, `HEAD == origin/pro/<task>`, manifest expected SHA와
verified SHA의 일치를 요구한다. 변경 파일, writer, base/remote/verified SHA,
SIM/MOCK/HW 구분과 rollback 항목을 body file로 생성해 `gh pr create`에 전달한다.
현재 SHA에서 `verify`하지 않았거나 열린 PR이 이미 있으면 새 PR을 만들지 않는다.

실패하면 로컬 Codex에 아래처럼 요청한다.

```text
pro/shoe-task-ux 브랜치의 원격 커밋을 검토해 줘.
재현 가능한 테스트를 먼저 실행하고, 빌드/테스트 실패의 원인을 분류한 뒤
최소 수정으로 고쳐. 실제 하드웨어는 구동하지 말고 SIM/MOCK/HW 근거를 구분해.
```

수정 후 다시 같은 브랜치에 push하고 다음 writer에게 순차적으로 handoff한다.
두 writer가 같은 브랜치를 동시에 작업하지 않는다.

사람의 검토와 required checks를 거쳐 PR이 병합되면 로컬 작업을 정리한다.

```bash
scripts/cowork finish shoe-task-ux
```

`finish`는 clean·synchronized·verified 상태, task tip의 `origin/main` 포함과
GitHub merged PR을 확인한 뒤 worktree, local branch와 manifest만 제거한다. 원격
branch는 기본적으로 보존한다. `--delete-remote`는 모든 검사를 통과한 뒤 원격
삭제부터 시도하며, `pro/**` deletion ruleset이 차단하면 로컬 정리를 시작하지
않는다.

## 모드 B: ChatGPT 데스크톱 Codex Worktree/Handoff

같은 PC의 ChatGPT 데스크톱 앱에서 **Worktree** 또는 **Hand off**를 사용할 때는
`scripts/cowork start`를 실행하지 않는다.

1. ChatGPT 데스크톱에서 프로젝트를 열고 새 chat의 **Worktree**를 선택한다.
2. 시작 branch를 고르면 Codex가 관리 worktree를 만든다. 기본 시작 상태는
   detached HEAD이며 root `AGENTS.md`가 이를 정상 상태로 허용한다.
3. worktree 안에서 계속 작업하려면 **Create branch here**로 별도 branch를 만든다.
4. 기존 local checkout으로 옮길 때는 같은 branch를 두 worktree에 checkout하지
   말고 **Hand off**를 사용한다. Handoff가 필요한 Git 작업을 처리한다.

Git은 같은 branch를 두 worktree에 동시에 checkout하지 못한다. 따라서 custom
`.local-workspaces/pro/<task>`와 Codex-managed Worktree 중 하나만 선택한다.
Desktop 모드에서는 managed Worktree를 기존 `pro/<task>` branch에 직접
checkout하려고 하지 않는다. detached HEAD에서 앱의 Create branch 또는 Handoff
흐름을 사용하며, 생성되는 branch 이름은 `pro/<task>`일 필요가 없다.

## `AGENTS.md` 로딩 범위와 안전 불변조건

Codex는 실행을 시작할 때 한 번, project root에서 세션의 current working
directory까지 `AGENTS.md` instruction chain을 구성한다. 저장소 root에서 시작한
세션은 편집 대상이 하위 경로라는 이유만으로 그 아래의 nested `AGENTS.md`를
나중에 자동으로 추가하지 않는다.

따라서 실물 안전에 필수적인 다음 조건은 모두 root `AGENTS.md`에 둔다.

- exact confirmation string과 현장 human gate
- device role/profile 및 motor identity/state 불일치 시 중단
- bounded command, timeout/watchdog, fail-safe stop
- SIM/MOCK namespace 또는 격리 ROS domain
- firmware upload, hardware snapshot, serial open의 실물 접근 취급
- CI self-hosted runner와 attached-hardware discovery 금지

네 하위 `AGENTS.md`는 이 root 불변조건을 완화하지 않고 경로별 추가 실행 맥락과
`Code Review Rules`를 제공한다. 경로별 세부 지침까지 coding session에 로드해야
하면 새 세션을 해당 하위 디렉터리에서 시작한다. 단순히 세션 중 `cd`만 하는 것은
이미 구성된 instruction chain을 다시 로드한다는 보장이 없다.

## 운영 원칙

- 한 branch에는 writer 한 명만 둔다.
- Mode A의 병렬 시도는 `pro/shoe-task-ux-cloud-01`,
  `pro/shoe-task-ux-local-01`처럼 writer별 branch로 분리한다. Mode B는 detached
  HEAD에서 시작해 Create branch 또는 Handoff가 만든 별도 branch를 사용한다.
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
