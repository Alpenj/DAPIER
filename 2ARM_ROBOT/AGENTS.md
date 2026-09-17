# Mobile dual-arm agent rules

These rules add to the repository root `AGENTS.md` for this directory.

- PGripper learning follows execution → failure analysis → corrective demonstrations →
  retraining, not unlimited updates on unchanged data. Before further training, check
  expert replay under the policy's actual control clock and safety limits. Preserve
  failures, keep evaluation data separate, and read the current decisions in
  `sim/mobile_dual_so101/LEARNING_STRATEGY_ACT_FIRST_KO.md`.

- MuJoCo-only models, offline data validation, static analysis, and tests that cannot
  connect to ROS or serial devices are allowed by default.
- New hardware-agnostic Python research code belongs in `2ARM_ROBOT/research/`.
  It may create versioned control intents, but it must not import low-level motor or
  serial SDKs, open device paths, authorize hardware, or publish direct actuator commands.
- Runtime perception and planning must not use MuJoCo object body poses, geom IDs, or
  segmentation labels to create grasp/IK targets. Use rendered sensor observations and
  transforms that have a physical calibration/TF equivalent. Simulator truth is limited
  to reset, reward, offline labels, and test-only error measurement.
- Visual SLAM, localization quality, map/session identity, reset generation, and TF adapter
  logic belong in the C++ localization packages under `so101_ros2/dapier_localization_*`.
  Localization code must not publish actuator commands, open motor/serial devices, or
  authorize hardware. Its language-neutral boundary is
  `contracts/localization_runtime_v1.json`.
- Real-time command validation, limits, watchdogs, safe stop, and hardware I/O belong in
  the C++ control packages under `so101_ros2/dapier_so101_*`. Python and localization
  packages may not bypass those gates.
- The Python-to-control boundary is `contracts/research_realtime_control_v1.json`.
  Change contract JSON first and regenerate the corresponding C++ header instead of
  editing generated values.
- A localization `relocalized`, `lost`, or map/reset-generation change must invalidate
  stale navigation goals, shoe targets, and action chunks before motion resumes.
- Do not run hardware teleop, motor jog, torque control, EEPROM/register writes,
  hardware snapshot scripts, or actuator publishers without the root hardware approval
  gate. Read-only snapshots still require approval when they contact physical devices.
- Treat `/cmd_vel`, JointTrajectory, JointState command publishers, and any unknown ROS
  graph as physical until its SIM/MOCK isolation is demonstrated.
- Never reuse committed example serial IDs or calibration files as live device identity.
- TurtleBot3 public configuration may contain `TB3_USER=user`, but must never contain a
  password, token, private key, or automated password-input mechanism. Use a Git-ignored
  local env file and SSH key authentication.
- Do not add a self-hosted runner or make CI discover attached hardware.

## Engineering learning and portfolio record rules

Before substantive 2ARM_ROBOT work, read
`docs/ENGINEERING_LEARNING_RECORD_KO.md` and apply its record contract.

- For non-obvious safety, geometry, frame/unit, transmission, numerical-tolerance, or
  simulator/sensor boundary logic, leave a concise 1–3 line source comment explaining
  **why the invariant exists**, not a line-by-line description of what the code does.
- For meaningful debugging or design changes, preserve enough evidence to explain
  `Problem → Evidence → Decision → Validation → Result → Lesson / Next`. Prefer a
  regression fixture or negative test when the failure can recur.
- Keep long debugging narratives out of source comments. Put reproducible commands,
  tests, artifacts, limitations, and the first failed phase in Markdown or the PR body.
  Private learning notes and screenshots belong in Notion, not in public repository URLs.
- When the work is visually meaningful, provide a viewer/render/plot/progress path so the
  user can watch the real execution. Distinguish kinematic preview from actual physics or
  hardware execution in both the UI and report.
- Final reports must distinguish SIM/MOCK/HW evidence, state the actual final phase, and
  name unverified follow-up work. A passing regression is not task success.

## End-of-turn GitHub and Notion sync

Treat one **turn** as one substantive Codex work cycle that produces a validated change,
new reproducible evidence, or a clearly identified next blocker. At the end of every such
turn, perform the record/sync steps below unless doing so would destroy unrelated work or
publish unvalidated/unsafe changes.

- Update the existing engineering learning record with the current
  `Problem → Evidence → Decision → Validation → Result → Lesson / Next`. Do not create a
  duplicate log for the same investigation.
- Run the smallest relevant regression set plus `git diff --check`; state anything not
  executed and why.
- Commit only this writer's validated, self-contained changes to the current named branch
  and normal-push it. Preserve unrelated dirty work and never reset, clean, force-push, or
  silently absorb another writer's changes.
- Open or update a PR whose base is `main`. If the turn's changes are self-contained,
  regression-clean, and safe to integrate, merge the PR into `main` and report the PR URL
  and resulting main SHA. Overall task incompleteness is allowed when the merged change is
  independently valid; clearly record the next blocker. Do not merge failing, ambiguous,
  hardware-unsafe, or partially edited code merely to satisfy this rule.
- After the GitHub result is known, update the private Notion DAPIER learning record with
  the same concise engineering summary plus branch/PR/merge SHA, actual reached phase,
  evidence, limitations, and next blocker. Never put private Notion URLs or IDs into the
  public repository.
- If the current Codex environment has no authenticated Notion write path, do not claim
  that Notion was updated. Instead, finish the local/GitHub record and emit a compact
  `NOTION_SYNC_PAYLOAD` containing the exact summary needed for ChatGPT or the user to
  write into the existing DAPIER Notion record.
- A turn is not considered fully handed off until the final report states the GitHub
  status (`merged to main`, `PR open`, or `not safe to merge`) and the Notion status
  (`updated` or `sync payload produced`).

## Code Review Rules

- Flag new mobile-base or arm command paths that lack an explicit human gate,
  bounded motion, or a short execution time limit. Apply the root distinction between
  attended manual recording and autonomous operation; do not reintroduce a mandatory
  independent watchdog/communication-loss stop verification gate for manual recording.
- Simulation entrypoints must remain usable without importing or opening hardware APIs.
- Flag Python research changes that acquire direct hardware dispatch responsibility,
  localization changes that acquire actuator authority, or C++ real-time changes that
  embed/launch Python or Visual SLAM inside the bounded command loop.
- Flag any runtime IK/grasp target derived from simulator truth rather than RGB/RGB-D
  perception, calibrated transforms, and an explicit confidence/uncertainty gate.
- Flag localization consumers that ignore tracking state, receiver-local TTL, covariance,
  or map/reset generation when accepting navigation or manipulation motion.
