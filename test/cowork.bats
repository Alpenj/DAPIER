#!/usr/bin/env bats

setup() {
  COWORK_SOURCE="$(cd "$BATS_TEST_DIRNAME/.." && pwd)/scripts/cowork"
  TEST_ROOT="$BATS_TEST_TMPDIR/cowork-fixture"
  create_fixture "$TEST_ROOT" "repo"
}

create_fixture() {
  local root="$1"
  local repo_name="$2"

  ROOT="$root"
  REMOTE="$ROOT/remote.git"
  SEED="$ROOT/seed"
  REPO="$ROOT/$repo_name"

  mkdir -p "$ROOT"
  git init --bare -q "$REMOTE"
  git init -q -b main "$SEED"
  configure_identity "$SEED"

  mkdir -p "$SEED/scripts"
  cp "$COWORK_SOURCE" "$SEED/scripts/cowork"
  cat >"$SEED/scripts/verify-hardware-free" <<'VERIFY'
#!/usr/bin/env bash
set -euo pipefail
exit "${COWORK_FAKE_VERIFY_STATUS:-0}"
VERIFY
  chmod +x "$SEED/scripts/cowork" "$SEED/scripts/verify-hardware-free"
  printf 'base\n' >"$SEED/base.txt"
  git -C "$SEED" add .
  git -C "$SEED" commit -qm "test: initial fixture"
  git -C "$SEED" remote add origin "$REMOTE"
  git -C "$SEED" push -q -u origin main
  git --git-dir="$REMOTE" symbolic-ref HEAD refs/heads/main

  git clone -q "$REMOTE" "$REPO"
  configure_identity "$REPO"
}

configure_identity() {
  git -C "$1" config user.name "Cowork Test"
  git -C "$1" config user.email "cowork-test@example.com"
}

worktree_path() {
  printf '%s/.local-workspaces/pro/%s\n' "$REPO" "$1"
}

remote_ref_sha() {
  git --git-dir="$REMOTE" rev-parse "refs/heads/$1"
}

manifest_path() {
  common="$(git -C "$REPO" rev-parse --path-format=absolute --git-common-dir)"
  printf '%s/cowork/manifests/%s.json\n' "$common" "$1"
}

manifest_field() {
  python3 - "$(manifest_path "$1")" "$2" <<'PYTHON'
import json
import sys
from pathlib import Path
print(json.loads(Path(sys.argv[1]).read_text())[sys.argv[2]])
PYTHON
}

merge_task_into_remote_main() {
  local task="$1"
  local actor="$ROOT/merge-actor-${task//\//-}"

  git clone -q "$REMOTE" "$actor"
  configure_identity "$actor"
  git -C "$actor" fetch -q origin "pro/$task"
  git -C "$actor" merge -q --no-ff --no-edit "origin/pro/$task"
  git -C "$actor" push -q origin main
}

make_fake_gh() {
  local fake_bin="$1"
  mkdir -p "$fake_bin"
  cat >"$fake_bin/gh" <<'GH'
#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == auth && "${2:-}" == status ]]; then
  exit 0
fi
if [[ "${1:-}" == pr && "${2:-}" == list ]]; then
  exit 0
fi
if [[ "${1:-}" == pr && "${2:-}" == create ]]; then
  shift 2
  body_file=""
  while [[ "$#" -gt 0 ]]; do
    if [[ "$1" == --body-file ]]; then
      body_file="$2"
      shift 2
    else
      shift
    fi
  done
  [[ -n "$body_file" ]]
  cp "$body_file" "$GH_BODY_CAPTURE"
  printf 'https://github.example/test/repo/pull/1\n'
  exit 0
fi
exit 1
GH
  chmod +x "$fake_bin/gh"
}

create_remote_branch() {
  local task="$1"
  local marker="${2:-remote}"
  local actor="$ROOT/actor-${task//\//-}-$marker"

  git clone -q "$REMOTE" "$actor"
  configure_identity "$actor"
  git -C "$actor" switch -q -c "pro/$task" origin/main
  printf '%s\n' "$marker" >"$actor/$marker.txt"
  git -C "$actor" add "$marker.txt"
  git -C "$actor" commit -qm "test: $marker task commit"
  git -C "$actor" push -q -u origin "pro/$task"
  git -C "$actor" rev-parse HEAD
}

advance_remote_main() {
  local marker="$1"
  local actor="$ROOT/main-actor-$marker"

  git clone -q "$REMOTE" "$actor"
  configure_identity "$actor"
  printf '%s\n' "$marker" >"$actor/$marker.txt"
  git -C "$actor" add "$marker.txt"
  git -C "$actor" commit -qm "test: advance main $marker"
  git -C "$actor" push -q origin main
  git -C "$actor" rev-parse HEAD
}

advance_remote_task() {
  local task="$1"
  local marker="$2"
  local actor="$ROOT/task-actor-${task//\//-}-$marker"

  git clone -q "$REMOTE" "$actor"
  configure_identity "$actor"
  git -C "$actor" switch -q -c "pro/$task" --track "origin/pro/$task"
  printf '%s\n' "$marker" >"$actor/$marker.txt"
  git -C "$actor" add "$marker.txt"
  git -C "$actor" commit -qm "test: advance task $marker"
  git -C "$actor" push -q origin "pro/$task"
  git -C "$actor" rev-parse HEAD
}

commit_in_worktree() {
  local checkout="$1"
  local filename="$2"
  local content="$3"

  printf '%s\n' "$content" >"$checkout/$filename"
  git -C "$checkout" add "$filename"
  git -C "$checkout" commit -qm "test: $content"
}

assert_output_contains() {
  [[ "$output" == *"$1"* ]]
}

cowork() {
  (
    cd "$REPO"
    "$REPO/scripts/cowork" "$@"
  )
}

cowork_with_path() {
  local path_prefix="$1"
  shift
  (
    cd "$REPO"
    PATH="$path_prefix:$PATH" "$REPO/scripts/cowork" "$@"
  )
}

@test "new branch is created and pushed with the exact upstream" {
  run cowork start new-branch
  [ "$status" -eq 0 ]
  assert_output_contains "Start state: new-branch-pushed"

  wt="$(worktree_path new-branch)"
  [ -d "$wt" ]
  [ "$(git -C "$wt" branch --show-current)" = "pro/new-branch" ]
  [ "$(git -C "$wt" rev-parse --abbrev-ref '@{upstream}')" = "origin/pro/new-branch" ]
  [ "$(git -C "$wt" rev-parse HEAD)" = "$(remote_ref_sha pro/new-branch)" ]
}

@test "existing remote branch is resumed when no local branch exists" {
  expected_sha="$(create_remote_branch resume remote-resume)"
  ! git -C "$REPO" show-ref --verify --quiet refs/heads/pro/resume

  run cowork start resume
  [ "$status" -eq 0 ]
  assert_output_contains "Start state: remote-branch-resumed"

  wt="$(worktree_path resume)"
  [ "$(git -C "$wt" rev-parse HEAD)" = "$expected_sha" ]
  [ "$(git -C "$wt" rev-parse --abbrev-ref '@{upstream}')" = "origin/pro/resume" ]
}

@test "local-only branch is pushed without replacing the existing local branch" {
  git -C "$REPO" branch --no-track pro/local-only main
  local_sha="$(git -C "$REPO" rev-parse refs/heads/pro/local-only)"

  run cowork start local-only
  [ "$status" -eq 0 ]
  assert_output_contains "Start state: local-only-branch-pushed"
  [ "$(remote_ref_sha pro/local-only)" = "$local_sha" ]
  [ "$(git -C "$(worktree_path local-only)" rev-parse --abbrev-ref '@{upstream}')" = "origin/pro/local-only" ]
}

@test "divergent local and remote branches stop before worktree creation" {
  create_remote_branch divergence remote-side >/dev/null
  git -C "$REPO" fetch -q --prune origin
  git -C "$REPO" branch --no-track pro/divergence origin/main
  local_checkout="$ROOT/local-divergence"
  git -C "$REPO" worktree add -q "$local_checkout" pro/divergence
  commit_in_worktree "$local_checkout" local-side.txt local-side
  git -C "$REPO" worktree remove "$local_checkout"
  git -C "$REPO" branch --set-upstream-to=origin/pro/divergence pro/divergence >/dev/null

  run cowork start divergence
  [ "$status" -ne 0 ]
  assert_output_contains "have diverged"
  [ ! -e "$(worktree_path divergence)" ]
}

@test "sync refuses a dirty task worktree before fetching" {
  cowork start dirty-sync >/dev/null
  wt="$(worktree_path dirty-sync)"
  printf 'dirty\n' >>"$wt/base.txt"

  run cowork sync dirty-sync
  [ "$status" -ne 0 ]
  assert_output_contains "worktree has local changes"
}

@test "status detects a worktree checked out on the wrong branch" {
  cowork start wrong-branch >/dev/null
  wt="$(worktree_path wrong-branch)"
  git -C "$wt" switch -q -c unrelated-branch

  run cowork status wrong-branch
  [ "$status" -ne 0 ]
  assert_output_contains "expected 'pro/wrong-branch'"
}

@test "status detects detached HEAD" {
  cowork start detached >/dev/null
  wt="$(worktree_path detached)"
  git -C "$wt" switch -q --detach

  run cowork status detached
  [ "$status" -ne 0 ]
  assert_output_contains "detached HEAD"
}

@test "failed initial push rolls back only the new worktree and branch" {
  cat >"$REMOTE/hooks/pre-receive" <<'HOOK'
#!/usr/bin/env bash
exit 1
HOOK
  chmod +x "$REMOTE/hooks/pre-receive"

  run cowork start rejected-push
  [ "$status" -ne 0 ]
  assert_output_contains "rolled back the worktree and local branch"
  [ ! -e "$(worktree_path rejected-push)" ]
  ! git -C "$REPO" show-ref --verify --quiet refs/heads/pro/rejected-push
  ! git --git-dir="$REMOTE" show-ref --verify --quiet refs/heads/pro/rejected-push
}

@test "status refreshes a stale origin main before reporting SHAs" {
  cowork start stale-main >/dev/null
  old_main="$(git -C "$REPO" rev-parse refs/remotes/origin/main)"
  new_main="$(advance_remote_main main-advanced)"
  [ "$old_main" != "$new_main" ]
  [ "$(git -C "$REPO" rev-parse refs/remotes/origin/main)" = "$old_main" ]

  run cowork status stale-main
  [ "$status" -eq 0 ]
  assert_output_contains "origin/main SHA: $new_main"
  [ "$(git -C "$REPO" rev-parse refs/remotes/origin/main)" = "$new_main" ]
}

@test "prompt falls back to the origin URL when gh is unauthenticated" {
  cowork start gh-fallback >/dev/null
  fake_bin="$ROOT/fake-bin"
  mkdir -p "$fake_bin"
  cat >"$fake_bin/gh" <<'GH'
#!/usr/bin/env bash
exit 1
GH
  chmod +x "$fake_bin/gh"
  origin_url="$(git -C "$REPO" remote get-url origin)"

  run cowork_with_path "$fake_bin" prompt gh-fallback
  [ "$status" -eq 0 ]
  assert_output_contains "Use repository $origin_url"
}

@test "repository and worktree paths containing spaces are handled safely" {
  rm -rf "$TEST_ROOT"
  create_fixture "$BATS_TEST_TMPDIR/root with spaces" "primary checkout with spaces"

  run cowork start spaces
  [ "$status" -eq 0 ]
  wt="$(worktree_path spaces)"
  [ -d "$wt" ]

  run cowork status spaces
  [ "$status" -eq 0 ]
  assert_output_contains "Local HEAD SHA:"
}

@test "deleted remote branch is not accepted through a stale tracking ref" {
  cowork start deleted-remote >/dev/null
  git -C "$REPO" show-ref --verify --quiet refs/remotes/origin/pro/deleted-remote
  git --git-dir="$REMOTE" update-ref -d refs/heads/pro/deleted-remote

  run cowork status deleted-remote
  [ "$status" -ne 0 ]
  assert_output_contains "remote branch does not exist"
  ! git -C "$REPO" show-ref --verify --quiet refs/remotes/origin/pro/deleted-remote
}

@test "wrong upstream is rejected even when branch and worktree are otherwise correct" {
  cowork start wrong-upstream >/dev/null
  wt="$(worktree_path wrong-upstream)"
  git -C "$wt" branch --set-upstream-to=origin/main pro/wrong-upstream >/dev/null

  run cowork status wrong-upstream
  [ "$status" -ne 0 ]
  assert_output_contains "upstream is 'origin/main'"
}

@test "an unregistered directory at the expected path is rejected" {
  cowork start unregistered >/dev/null
  wt="$(worktree_path unregistered)"
  git -C "$REPO" worktree remove "$wt"
  mkdir -p "$wt"
  git init -q -b pro/unregistered "$wt"

  run cowork status unregistered
  [ "$status" -ne 0 ]
  assert_output_contains "not a registered Git worktree"
}

@test "verify rejects local ahead by default and permits only pure local ahead with the flag" {
  cowork start local-ahead >/dev/null
  wt="$(worktree_path local-ahead)"
  commit_in_worktree "$wt" ahead.txt ahead

  run cowork verify local-ahead
  [ "$status" -ne 0 ]
  assert_output_contains "requires HEAD == origin/pro/local-ahead"

  run cowork verify local-ahead --allow-local-ahead
  [ "$status" -eq 0 ]
  assert_output_contains "State:          local-ahead"
  assert_output_contains "Verified SHA:"
}

@test "verify allow-local-ahead flag rejects a synchronized branch" {
  cowork start synchronized-flag >/dev/null

  run cowork verify synchronized-flag --allow-local-ahead
  [ "$status" -ne 0 ]
  assert_output_contains "permits only a purely local-ahead state"
}

@test "prompt includes the starting and expected remote SHA plus the non-FF stop rule" {
  cowork start prompt-sha >/dev/null
  expected_sha="$(remote_ref_sha pro/prompt-sha)"

  run cowork prompt prompt-sha
  [ "$status" -eq 0 ]
  assert_output_contains "Starting SHA:            $expected_sha"
  assert_output_contains "Expected remote SHA:     $expected_sha"
  assert_output_contains "push is rejected as non-fast-forward"
}


@test "start fast-forwards a purely behind local branch" {
  remote_sha="$(create_remote_branch behind-start remote-behind)"
  git -C "$REPO" fetch -q --prune origin
  git -C "$REPO" branch --no-track pro/behind-start origin/main
  git -C "$REPO" branch --set-upstream-to=origin/pro/behind-start pro/behind-start >/dev/null

  run cowork start behind-start
  [ "$status" -eq 0 ]
  assert_output_contains "Start state: local-branch-fast-forwarded"
  [ "$(git -C "$(worktree_path behind-start)" rev-parse HEAD)" = "$remote_sha" ]
}

@test "start preserves a purely ahead local branch without pushing it" {
  create_remote_branch ahead-start remote-base >/dev/null
  git -C "$REPO" fetch -q --prune origin
  git -C "$REPO" branch --track pro/ahead-start origin/pro/ahead-start
  local_checkout="$ROOT/local-ahead-start"
  git -C "$REPO" worktree add -q "$local_checkout" pro/ahead-start
  commit_in_worktree "$local_checkout" local-ahead-start.txt local-ahead-start
  local_sha="$(git -C "$local_checkout" rev-parse HEAD)"
  remote_sha="$(remote_ref_sha pro/ahead-start)"
  git -C "$REPO" worktree remove "$local_checkout"

  run cowork start ahead-start
  [ "$status" -eq 0 ]
  assert_output_contains "Start state: local-branch-ahead"
  [ "$(git -C "$(worktree_path ahead-start)" rev-parse HEAD)" = "$local_sha" ]
  [ "$(remote_ref_sha pro/ahead-start)" = "$remote_sha" ]
}

@test "sync fast-forwards a clean local branch that is behind remote" {
  cowork start sync-behind >/dev/null
  new_remote_sha="$(advance_remote_task sync-behind sync-remote-update)"

  run cowork sync sync-behind
  [ "$status" -eq 0 ]
  assert_output_contains "State:          synchronized"
  [ "$(git -C "$(worktree_path sync-behind)" rev-parse HEAD)" = "$new_remote_sha" ]
}

@test "sync refuses divergence without changing the local HEAD" {
  cowork start sync-divergent >/dev/null
  wt="$(worktree_path sync-divergent)"
  commit_in_worktree "$wt" local-divergent.txt local-divergent
  local_sha="$(git -C "$wt" rev-parse HEAD)"
  advance_remote_task sync-divergent remote-divergent >/dev/null

  run cowork sync sync-divergent
  [ "$status" -ne 0 ]
  assert_output_contains "have diverged"
  [ "$(git -C "$wt" rev-parse HEAD)" = "$local_sha" ]
}

@test "verify fails when the remote task head changes during verification" {
  cowork start verify-race >/dev/null
  wt="$(worktree_path verify-race)"
  cat >"$wt/scripts/verify-hardware-free" <<VERIFY
#!/usr/bin/env bash
set -euo pipefail
remote='$REMOTE'
old=\$(git --git-dir="\$remote" rev-parse refs/heads/pro/verify-race)
tree=\$(git --git-dir="\$remote" rev-parse "\$old^{tree}")
new=\$(printf 'remote race\n' | \\
  GIT_AUTHOR_NAME='Race Test' GIT_AUTHOR_EMAIL='race@example.com' \\
  GIT_COMMITTER_NAME='Race Test' GIT_COMMITTER_EMAIL='race@example.com' \\
  git --git-dir="\$remote" commit-tree "\$tree" -p "\$old")
git --git-dir="\$remote" update-ref refs/heads/pro/verify-race "\$new" "\$old"
VERIFY
  chmod +x "$wt/scripts/verify-hardware-free"
  git -C "$wt" add scripts/verify-hardware-free
  git -C "$wt" commit -qm "test: simulate remote race"
  git -C "$wt" push -q origin pro/verify-race
  cowork sync verify-race >/dev/null

  run cowork verify verify-race
  [ "$status" -ne 0 ]
  assert_output_contains "remote head changed during verification"
}

@test "registered path pointing at another Git common directory is rejected" {
  cowork start wrong-common >/dev/null
  wt="$(worktree_path wrong-common)"
  other="$ROOT/other-repository"
  git init -q -b main "$other"
  configure_identity "$other"
  printf 'other\n' >"$other/other.txt"
  git -C "$other" add other.txt
  git -C "$other" commit -qm "test: other repository"
  printf 'gitdir: %s/.git\n' "$other" >"$wt/.git"

  run cowork status wrong-common
  [ "$status" -ne 0 ]
  assert_output_contains "different Git common directory"
}

@test "git check-ref-format rejects an invalid derived branch" {
  run cowork start bad..slug
  [ "$status" -ne 0 ]
  assert_output_contains "invalid Git branch name"
}


@test "start writes a private handoff manifest with the exact remote SHA" {
  run cowork start manifest-created
  [ "$status" -eq 0 ]
  manifest="$(manifest_path manifest-created)"
  [ -f "$manifest" ]
  [ "$(stat -c '%a' "$manifest")" = "600" ]
  [ "$(manifest_field manifest-created task)" = "manifest-created" ]
  [ "$(manifest_field manifest-created branch)" = "pro/manifest-created" ]
  [ "$(manifest_field manifest-created expected_remote_sha)" = "$(remote_ref_sha pro/manifest-created)" ]
  [ "$(manifest_field manifest-created hardware_allowed)" = "False" ]
}

@test "verify records the exact verified SHA in the manifest" {
  cowork start manifest-verified >/dev/null
  expected="$(remote_ref_sha pro/manifest-verified)"

  run cowork verify manifest-verified
  [ "$status" -eq 0 ]
  [ "$(manifest_field manifest-verified verified_sha)" = "$expected" ]
}

@test "prompt rejects a remote SHA that differs from its handoff manifest" {
  cowork start manifest-lease >/dev/null
  manifest="$(manifest_path manifest-lease)"
  python3 - "$manifest" <<'PYTHON'
import json
import sys
from pathlib import Path
path = Path(sys.argv[1])
data = json.loads(path.read_text())
data["expected_remote_sha"] = "0000000000000000000000000000000000000000"
path.write_text(json.dumps(data) + "\n")
PYTHON

  run cowork prompt manifest-lease
  [ "$status" -ne 0 ]
  assert_output_contains "remote head differs from manifest"
}

@test "doctor reports active task branches without failing on a non-GitHub fixture" {
  cowork start doctor-active >/dev/null

  run cowork doctor
  [ "$status" -eq 0 ]
  assert_output_contains "Cowork doctor"
  assert_output_contains "pro/doctor-active"
  assert_output_contains "Doctor summary: 0 failure(s)"
}

@test "pr refuses a synchronized but unverified remote head" {
  cowork start pr-unverified >/dev/null
  wt="$(worktree_path pr-unverified)"
  commit_in_worktree "$wt" pr-unverified.txt pr-unverified
  git -C "$wt" push -q origin pro/pr-unverified
  cowork sync pr-unverified >/dev/null

  run cowork pr pr-unverified
  [ "$status" -ne 0 ]
  assert_output_contains "requires cowork verify"
}

@test "pr creates a metadata body only for the verified remote head" {
  cowork start pr-created >/dev/null
  wt="$(worktree_path pr-created)"
  commit_in_worktree "$wt" pr-created.txt pr-created
  git -C "$wt" push -q origin pro/pr-created
  cowork sync pr-created >/dev/null
  cowork verify pr-created >/dev/null
  fake_bin="$ROOT/fake-gh-bin"
  body_capture="$ROOT/pr-body.md"
  make_fake_gh "$fake_bin"
  export GH_BODY_CAPTURE="$body_capture"
  expected="$(remote_ref_sha pro/pr-created)"

  run cowork_with_path "$fake_bin" pr pr-created --draft
  [ "$status" -eq 0 ]
  assert_output_contains "https://github.example/test/repo/pull/1"
  grep -Fqx -- "- Remote head SHA: $expected" "$body_capture"
  grep -Fqx -- "- Local verified SHA: $expected" "$body_capture"
  grep -Fqx -- "- 접근 여부: 없음" "$body_capture"
  grep -Fqx "pr-created.txt" "$body_capture"
}

@test "finish refuses an unmerged task without removing local state" {
  cowork start finish-unmerged >/dev/null
  wt="$(worktree_path finish-unmerged)"
  commit_in_worktree "$wt" finish-unmerged.txt finish-unmerged
  git -C "$wt" push -q origin pro/finish-unmerged
  cowork sync finish-unmerged >/dev/null
  cowork verify finish-unmerged >/dev/null
  manifest="$(manifest_path finish-unmerged)"

  run cowork finish finish-unmerged
  [ "$status" -ne 0 ]
  assert_output_contains "not contained in origin/main"
  [ -d "$wt" ]
  [ -f "$manifest" ]
  git -C "$REPO" show-ref --verify --quiet refs/heads/pro/finish-unmerged
}

@test "finish removes verified merged local state and preserves the remote branch by default" {
  cowork start finish-merged >/dev/null
  wt="$(worktree_path finish-merged)"
  commit_in_worktree "$wt" finish-merged.txt finish-merged
  git -C "$wt" push -q origin pro/finish-merged
  cowork sync finish-merged >/dev/null
  cowork verify finish-merged >/dev/null
  merge_task_into_remote_main finish-merged
  manifest="$(manifest_path finish-merged)"

  run cowork finish finish-merged
  [ "$status" -eq 0 ]
  assert_output_contains "Remote branch: preserved"
  [ ! -e "$wt" ]
  [ ! -e "$manifest" ]
  ! git -C "$REPO" show-ref --verify --quiet refs/heads/pro/finish-merged
  git --git-dir="$REMOTE" show-ref --verify --quiet refs/heads/pro/finish-merged
}

@test "finish delete-remote removes the remote only after merge and verification" {
  cowork start finish-delete >/dev/null
  wt="$(worktree_path finish-delete)"
  commit_in_worktree "$wt" finish-delete.txt finish-delete
  git -C "$wt" push -q origin pro/finish-delete
  cowork sync finish-delete >/dev/null
  cowork verify finish-delete >/dev/null
  merge_task_into_remote_main finish-delete

  run cowork finish finish-delete --delete-remote
  [ "$status" -eq 0 ]
  assert_output_contains "Remote branch: deleted"
  ! git --git-dir="$REMOTE" show-ref --verify --quiet refs/heads/pro/finish-delete
}

@test "failed remote deletion leaves the worktree branch and manifest untouched" {
  cowork start finish-delete-fails >/dev/null
  wt="$(worktree_path finish-delete-fails)"
  commit_in_worktree "$wt" finish-delete-fails.txt finish-delete-fails
  git -C "$wt" push -q origin pro/finish-delete-fails
  cowork sync finish-delete-fails >/dev/null
  cowork verify finish-delete-fails >/dev/null
  merge_task_into_remote_main finish-delete-fails
  manifest="$(manifest_path finish-delete-fails)"
  cat >"$REMOTE/hooks/pre-receive" <<'HOOK'
#!/usr/bin/env bash
set -euo pipefail
while read -r old new ref; do
  if [[ "$ref" == refs/heads/pro/finish-delete-fails && "$new" == 0000000000000000000000000000000000000000 ]]; then
    exit 1
  fi
done
exit 0
HOOK
  chmod +x "$REMOTE/hooks/pre-receive"

  run cowork finish finish-delete-fails --delete-remote
  [ "$status" -ne 0 ]
  assert_output_contains "no local cleanup was performed"
  [ -d "$wt" ]
  [ -f "$manifest" ]
  git -C "$REPO" show-ref --verify --quiet refs/heads/pro/finish-delete-fails
}
