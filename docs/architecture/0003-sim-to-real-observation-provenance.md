# ADR 0003: Sim-to-real observation provenance is mandatory

- Status: Accepted for staged implementation
- Date: 2026-09-02
- Scope: DAPIER mobile dual-arm research policy and dataset boundaries

## Context

PR #42 introduced an RGB-D path that reconstructs a shoe target without using MuJoCo
object poses. The existing state task and episode recorder still expose simulator truth
for evaluation and baseline experiments. A policy or dataset adapter could therefore
accidentally consume privileged fields even when RGB-D frames are also present.

Presence of camera data is not proof that policy input is sensor-derived. The producer,
source class, truth usage, and consumer must be checked explicitly.

## Decision

Every observation crossing a sim-to-real policy, training, or control-monitor boundary
must carry `dapier.observation-provenance.v1`.

Two source classes are defined:

1. `sensor_runtime`: only information available through physical sensors or calibrated
   transforms.
2. `simulator_privileged`: object/body pose, simulator IDs, reward or reset oracle.

`policy_runtime`, `training_episode`, and `control_monitor` accept only
`sensor_runtime`. `evaluation_oracle` and `reset_reward` may consume privileged state.
Missing provenance is rejected rather than inferred.

The existing simulation task and recorder remain available as legacy evaluation
baselines. They are not declared sim-to-real compatible. A separate dataset gate rejects
an episode unless both the manifest and every policy observation prove sensor-runtime
provenance.

## Consequences

### Positive

- RGB-D rendering can no longer be confused with RGB-D policy conditioning.
- Detection failure remains visible and cannot silently fall back to object truth.
- Legacy simulation results remain reproducible without being mislabeled as deployment
  evidence.
- ACT/LeRobot conversion has a concrete gate to enforce before training.

### Costs

- Producers must attach provenance and list sensor frames.
- Existing episode files fail the new sim-to-real gate until migrated.
- Policy integration needs a strict wrapper even when the underlying simulation executor
  remains unchanged.

## Verification

- pure-Python provenance and dataset-gate unit tests
- MuJoCo adapter tests for world-pose sanitization
- policy-query count remains zero when privileged observations are rejected
- sensor detection failure remains a sensor observation with no truth fallback
- `hardware_execution=false` remains mandatory

## Follow-up

1. Add a sensor-runtime episode producer.
2. Require the dataset gate in ACT/LeRobot conversion.
3. Connect the accepted policy observation to the C++ mock safety bridge.
4. Add ROS 2 camera/joint provenance and clock-domain validation.
