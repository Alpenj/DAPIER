# Goal observation evidence and model preview

This B17 review supports obstacle handling followed by cleaning resume. `OBSERVE_HELD_OBJECT` is the representative primitive. C10/R13 remains a joint-action predictor; this change adds no goal predictor or training labels.

`b16_goal_bridge.py` carries the B16 goal contract forward with separate temporal diagnostics, geometry deficits, model view candidates and physical execution requirements. The historical B16 source/results stay frozen. No A control, writer, calibration or motion profile is changed.

Sequential arm feedback receipts and a camera receipt before a joint receipt do not block pure model preview. Left wrist references left state, right wrist references right state, and top references both. Signed time differences are computed only within the same qualified clock, with units normalized before subtraction. A host receipt is not a servo measurement or verified camera exposure. Unknown sampling/exposure alignment remains a geometric uncertainty without an invented time tolerance.

An unverified calibration does not prevent a model calculation. Missing measured object geometry still leaves the goal at `NEEDS_OBSERVATION`; an existing model target can be inspected separately as `MODEL_VIEW_CANDIDATE`. A model candidate never becomes `MEASURED_GOAL`. `shadow_only=true`, `executable=false`, `motor_write_authorized=false`, `motion_requested=false` and `insertion_requested=false` remain explicit.

`GoalProposalBridge.propose(record, preview)` accepts the existing A preview callback. Its result includes `status`, `ik_solution_exists` and `collision_result`. The bridge validates recorded A `CURRENT_TOOL` targets with the existing pure A contract; it does not invoke a motor executor. [The schema](B17_GOAL_PROPOSAL_SCHEMA.json) describes the extended output.

`stored_replay.py` reruns existing A RGB-D and wrist feature functions on saved files. It retains the original search/robot masks as model-assisted inputs; this is feature-stage replay, not independent target acquisition. The four pure wrist surface functions are extracted unchanged from the A source so that importing the pick executor is unnecessary. Partial planes, 2D contours and CAD pad seeds do not certify object centre, full pose or physical pad identity.

Validated results are available in the five anonymized [reports](reports/B17_GOAL_BRIDGE_REGRESSION.json). Ten stored RGB-D bundles were replayed. Top returned five partial planes and five missing features; wrist RGB supplied additional 2D contours. No metric centre, complete orientation or receiving inner-pad identity was confirmed. The two existing A model previews reproduced B16 waypoints exactly and remained model-only. Their historical movement did not execute, so actual information gain is unproven.

Run portable contract and timing tests with the project's existing Python environment containing NumPy:

```bash
python -m unittest discover -s 2ARM_ROBOT/sweepick/goal_bridge -v
```

The public test set has 22 portable cases and three private A geometry cases. Without `B16_PRIVATE_ROOT`, those three are explicitly skipped. Full local validation ran all 17 B16 cases, eight additional temporal/model cases and four stored REAL replay cases: 29 passed, zero skipped. Schema checks accepted five REAL-derived proposals plus one labelled synthetic fixture and rejected five illegal mutations. Tests do not establish physical success.

To reproduce full verification, supply existing private B16 evidence, the recorded A model, and a private replay manifest. Nothing is downloaded or created as substitute camera data:

```bash
python 2ARM_ROBOT/sweepick/goal_bridge/run_b17.py \
  --b16-root "$B16_PRIVATE_ROOT" \
  --replay-manifest "$B17_REPLAY_MANIFEST" \
  --output "$B17_NEW_OUTPUT_DIRECTORY"
```

The manifest has `a_vision_source`, `wrist_calibration` and `samples`. Each sample has `sample_id`, `folder` and a source-qualified `clock`. The optional `archive_oct13` and `historical_view_outcome` fields identify existing historical evidence. Output must be a new directory and contains private evidence: keep it outside the published source tree. The existing A optical model is used only for model-facing face hypotheses. Its field verification and physical error bound remain unresolved.

No device is opened, policy loaded, optimizer run, dynamics stepped or new motion target sent. The replay audit rejects device opens, executor/policy imports, subprocesses and network connections. Local validation recorded `motor_commands_sent=0`, `executor_calls=0` and `optimizer_updates=0`.

The original train/validation/test split remains 15/0/0. Teacher intention, actual object state and desired pose labels are separate. Recorded source/admission links do not resolve every shared seed/setup parent, and no parent grouping or desired pose label is inferred. [Dataset readiness](reports/B17_DATASET_READINESS.md) remains `INSUFFICIENT_DATA`.

Private recordings, actual joint values, camera parameters, device identity, machine paths, full model binaries and calibration files are excluded. This directory is a reviewed source snapshot; it grants no physical execution approval.
