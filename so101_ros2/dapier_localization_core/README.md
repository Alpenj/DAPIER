# dapier_localization_core

C++ contract and fail-closed quality gate for localization estimates used by
DAPIER navigation and manipulation.

This package is deliberately independent of ORB-SLAM3, Nav2, camera drivers and
motor drivers. A future ORB-SLAM3 adapter, a recorded-data replay producer and a
test fake must all emit the same `LocalizationEstimate` contract.

## Responsibilities

- receiver-local TTL and sequence validation
- map/reset generation matching
- frame and identifier validation
- finite pose and normalized quaternion validation
- covariance and quality validation
- tracking-state gate for motion
- rejection of simulator truth and hardware authorization

## Non-responsibilities

- feature extraction, mapping or loop closing
- ROS image transport
- TF publication
- Nav2 goal generation
- actuator commands or hardware I/O

## Hardware-free smoke test

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Iso101_ros2/dapier_localization_core/include \
  so101_ros2/dapier_localization_core/src/localization_runtime.cpp \
  so101_ros2/dapier_localization_core/test/localization_runtime_contract_smoke.cpp \
  -o /tmp/dapier-localization-contract-smoke
/tmp/dapier-localization-contract-smoke
```

`TrackingState::kRelocalized` intentionally produces a hold decision. Consumers
must invalidate old object targets and action chunks, acknowledge the new map
reset generation, and replan before motion resumes.
