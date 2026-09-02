# ADR 0004: C++ Visual SLAM localization runtime is separate from real-time control

- Status: accepted for staged implementation
- Date: 2026-09-02
- Depends on: ADR 0001, ADR 0002, ADR 0003
- Source basis: Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*, DOI 10.1109/TRO.2021.3075644

## Context

DAPIER needs two different camera-derived estimates:

1. the robot pose in a map for navigation, relocalization and docking;
2. the shoe pose in camera/base/map frames for manipulation.

MuJoCo provides exact object and robot world state, but the physical system does
not. Runtime navigation and manipulation therefore need sensor-derived
localization and object perception with explicit provenance.

ORB-SLAM3 separates tracking, local mapping, loop/map merging, Atlas/keyframe
database and full bundle adjustment. Mapping, merging and full BA can have
variable latency and must not share the motor servo or safety execution path.
The paper lists low-texture environments as a main failure case and reports
compute time on an Intel Core i7-7700 with 32 GB RAM, not on the TurtleBot3
Raspberry Pi.

## Decision

DAPIER uses three execution responsibilities:

1. **Python research layer** — shoe perception, ACT/VLA, planning, simulation,
   datasets and offline evaluation.
2. **C++ perception/localization runtime** — Visual SLAM adapter, pose/TF,
   tracking quality, map/session/reset generation and relocalization state.
3. **C++ real-time control/safety layer** — command ingress, limits,
   interlocks, watchdog, safe stop and hardware I/O.

The localization runtime cannot authorize hardware or publish actuator
commands. The control layer never embeds or launches Python or ORB-SLAM3 inside
the bounded command loop. It consumes only a validated localization estimate.

## Contract-first integration

`contracts/localization_runtime_v1.json` is the language-neutral source for the
C++ constants. An estimate includes:

- schema version, sequence and receiver-local TTL;
- source timestamp for trace only;
- parent/child frames;
- map, session and reset generation identity;
- tracking state;
- position, quaternion and 6x6 covariance;
- tracked features, reprojection error and confidence;
- explicit false values for simulator truth and control authorization.

Cross-host monotonic timestamps are never compared. The receiver starts TTL from
its own monotonic clock. A map/reset generation mismatch rejects the estimate so
old shoe targets and action chunks cannot cross a relocalization or map reset.

## Motion semantics

- `tracking` may proceed only when confidence, tracked features, reprojection
  error and covariance pass configured thresholds.
- `initializing`, `recently_lost` and `lost` force hold.
- `relocalized` forces hold, target invalidation and replanning.
- malformed, stale, replayed or privileged estimates are rejected.

## ORB-SLAM3 scope interpretation

The paper states support for monocular, stereo and RGB-D visual sensors, and
presents tightly integrated monocular-inertial and stereo-inertial systems. The
RGB-D support statement alone is not treated as proof that the selected DAPIER
camera/IMU combination has a ready-made tightly coupled RGB-D+IMU path. The
chosen adapter and sensor configuration require separate source and replay
validation.

## Hardware-ready sequence

1. contract and standalone C++ validator;
2. deterministic fake and bag/MCAP replay producer;
3. failure injection for low texture, blur, occlusion, frame drop, timestamp
   skew, lost tracking, relocalization and map reset;
4. ORB-SLAM3 adapter behind the same contract;
5. Nav2 and C++ safety integration;
6. physical intrinsics/extrinsics, time offset, wheel scale and low-speed commissioning.

No stage before step 6 is recorded as hardware validation.
