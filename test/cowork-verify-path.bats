#!/usr/bin/env bats

setup() {
  SOURCE_ROOT="$(cd "$BATS_TEST_DIRNAME/.." && pwd)"
  COWORK_SOURCE="$SOURCE_ROOT/scripts/cowork"
  VERIFY_SOURCE="$SOURCE_ROOT/scripts/verify-hardware-free"
  TEST_ROOT="$BATS_TEST_TMPDIR/cowork-verify-path-fixture"
  REMOTE="$TEST_ROOT/remote.git"
  SEED="$TEST_ROOT/seed"
  REPO="$TEST_ROOT/primary"

  mkdir -p "$TEST_ROOT"
  git init --bare -q "$REMOTE"
  git init -q -b main "$SEED"
  configure_identity "$SEED"

  mkdir -p "$SEED/scripts"
  cp "$COWORK_SOURCE" "$SEED/scripts/cowork"
  cat >"$SEED/scripts/verify-hardware-free" <<'VERIFY'
#!/usr/bin/env bash
set -euo pipefail
exit 0
VERIFY
  chmod +x "$SEED/scripts/cowork" "$SEED/scripts/verify-hardware-free"
  printf 'primary fixture\n' >"$SEED/primary-only.txt"
  git -C "$SEED" add .
  git -C "$SEED" commit -qm "test: initial verify-path fixture"
  git -C "$SEED" remote add origin "$REMOTE"
  git -C "$SEED" push -q -u origin main
  git --git-dir="$REMOTE" symbolic-ref HEAD refs/heads/main

  git clone -q "$REMOTE" "$REPO"
  configure_identity "$REPO"
}

configure_identity() {
  git -C "$1" config user.name "Cowork Verify Path Test"
  git -C "$1" config user.email "cowork-verify-path@example.com"
}

worktree_path() {
  printf '%s/.local-workspaces/pro/%s\n' "$REPO" "$1"
}

cowork() {
  (
    cd "$REPO"
    "$REPO/scripts/cowork" "$@"
  )
}

commit_and_push_task() {
  local worktree="$1"
  local message="$2"

  git -C "$worktree" add .
  git -C "$worktree" commit -qm "$message"
  git -C "$worktree" push -q origin HEAD
}

make_fake_verification_tools() {
  local fake_bin="$1"

  mkdir -p "$fake_bin"

  cat >"$fake_bin/shellcheck" <<'SHELLCHECK'
#!/usr/bin/env bash
set -euo pipefail
printf 'shellcheck cwd=%s\n' "$(pwd -P)" >>"$VERIFY_TRACE"
exit 0
SHELLCHECK

  cat >"$fake_bin/bats" <<'BATS'
#!/usr/bin/env bash
set -euo pipefail
observed_root="$(pwd -P)"
printf 'bats cwd=%s\n' "$observed_root" >>"$VERIFY_TRACE"
if [[ "$observed_root" == "$EXPECTED_TASK_ROOT" ]]; then
  exit 73
fi
exit 0
BATS

  cat >"$fake_bin/python3" <<'PYTHON'
#!/usr/bin/env bash
set -euo pipefail
printf 'python3 cwd=%s\n' "$(pwd -P)" >>"$VERIFY_TRACE"
exit 0
PYTHON

  chmod +x "$fake_bin/shellcheck" "$fake_bin/bats" "$fake_bin/python3"
}

verify_from_primary_with_path() {
  local fake_bin="$1"
  local task_worktree="$2"

  (
    cd "$REPO"
    PATH="$fake_bin:$PATH" bash "$task_worktree/scripts/verify-hardware-free"
  )
}

@test "cowork verify runs the task verifier from the task worktree and propagates failure" {
  cowork start verify-cwd >/dev/null
  wt="$(worktree_path verify-cwd)"
  expected_root="$(git -C "$wt" rev-parse --show-toplevel)"
  trace="$TEST_ROOT/cowork-verify-cwd.trace"

  cat >"$wt/scripts/verify-hardware-free" <<'VERIFY'
#!/usr/bin/env bash
set -euo pipefail
observed_cwd="$(pwd -P)"
observed_root="$(git rev-parse --show-toplevel)"
printf 'cwd=%s\nroot=%s\n' "$observed_cwd" "$observed_root" >"$VERIFY_TRACE"
[[ "$observed_cwd" == "$EXPECTED_TASK_ROOT" ]] || exit 91
[[ "$observed_root" == "$EXPECTED_TASK_ROOT" ]] || exit 92
exit 73
VERIFY
  chmod +x "$wt/scripts/verify-hardware-free"
  printf 'task fixture\n' >"$wt/task-only.txt"
  commit_and_push_task "$wt" "test: install failing task verifier"

  export VERIFY_TRACE="$trace"
  export EXPECTED_TASK_ROOT="$expected_root"

  run cowork verify verify-cwd
  [ "$status" -eq 73 ]
  [ "$(sed -n '1p' "$trace")" = "cwd=$expected_root" ]
  [ "$(sed -n '2p' "$trace")" = "root=$expected_root" ]
}

@test "verify-hardware-free resolves its repository from BASH_SOURCE when called from primary" {
  cowork start source-root >/dev/null
  wt="$(worktree_path source-root)"
  expected_root="$(git -C "$wt" rev-parse --show-toplevel)"
  trace="$TEST_ROOT/verify-source-root.trace"
  fake_bin="$TEST_ROOT/fake-verification-tools"

  cp "$VERIFY_SOURCE" "$wt/scripts/verify-hardware-free"
  chmod +x "$wt/scripts/verify-hardware-free"
  printf 'task fixture\n' >"$wt/task-only.txt"
  commit_and_push_task "$wt" "test: install real task verifier"

  # The old cwd-based implementation would select this primary directory and
  # finish successfully through the fake tools, masking the task failure.
  mkdir -p "$REPO/casino_dealer"
  make_fake_verification_tools "$fake_bin"

  export VERIFY_TRACE="$trace"
  export EXPECTED_TASK_ROOT="$expected_root"

  run verify_from_primary_with_path "$fake_bin" "$wt"
  [ "$status" -eq 73 ]
  grep -Fqx "shellcheck cwd=$expected_root" "$trace"
  grep -Fqx "bats cwd=$expected_root" "$trace"
  ! grep -Fq "python3 cwd=" "$trace"
}
