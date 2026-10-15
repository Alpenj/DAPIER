# Image-based visual servo foundation

B18 implements image features, local image-to-motion Jacobians and unapproved tool corrections. A retains its stages, IK, collision checks, motor writer and STOP contract. No running A or Console code is changed.

REAL evidence currently supports left-wrist X features only. The recorded two local corrections reduce X error, but actual height, transverse motion, rotation, image scale and target also change. They cannot identify a general 2D Jacobian. Task Y remains `Y_REFERENCE_UNOBSERVED`; a visible component centroid or a bottom-row Jaw scan does not supply it. Missing evidence returns an explicit incomplete result rather than a fabricated matrix or two-axis correction.

The code reuses the B17 file-only A vision functions and temporal diagnostics. Top RGB-D provides separate object/worktable context; right wrist stays in the observation without unverified left-control fusion. Host receipt chronology is distinct from camera exposure and servo sampling. No new time cutoff is added.

| File | Responsibility |
|---|---|
| [features.py](features.py) | Observable contours/centroids, current Jaw X evidence, explicit target/error and camera/state binding |
| [image_jacobian.py](image_jacobian.py) | SVD identifiability, observed-subspace least squares and numerical regularization |
| [servo_proposal.py](servo_proposal.py) | `VisualServoProposal`, new-image checks and response-direction re-evaluation |
| [offline_model.py](offline_model.py) | Existing A pure IK/collision and virtual camera reprojection |
| [test_visual_servo.py](test_visual_servo.py) | Independent signed-error, rank, missing-input, identity and response regressions |
| [run_b18.py](run_b18.py) | Private saved-data audit and full offline verification |

Run portable tests with the existing Python environment containing NumPy and OpenCV:

```bash
cd 2ARM_ROBOT/sweepick
python -m unittest visual_servo.test_visual_servo -v
```

The private A-model test class is explicitly skipped by this portable command. Full verification supplies the existing frozen model and evidence through a private input manifest:

```bash
python visual_servo/run_b18.py \
  --inputs "$B18_PRIVATE_INPUT_MANIFEST" \
  --output "$B18_NEW_OUTPUT_DIRECTORY"
```

The manifest has `b16_root`, `b17_root`, saved `sessions` with `id/path`, and the `excluded_session` record. Output must be a new directory outside the public snapshot: it contains private image/feedback provenance. No data or model is downloaded, replaced or generated as REAL evidence. The original B17 regression is run unchanged with its private inputs; its 29 cases remain separate from the new B18 tests.

Model verification uses an explicit virtual fixed point, a reference model pose, and the existing model camera/FOV. The model's declared viewport is not REAL optical calibration. Four signed X/Y cases close the loop through A pure IK/collision and new projections. Model results are `SIM_OR_MODEL_CONVERGED` only, and the model X response can have the opposite sign to REAL response. REAL convergence remains unverified.

Every proposal has `executable=false` and `motor_write_authorized=false`. There is no actuator writer, step cap, move clipping, arbitrary physical threshold, new calibration, training or optimizer. Damping and finite-difference probes are numerical parameters; model test precision and iteration budget are not physical gates.

The [integration contract](B18_A_INTEGRATION_CONTRACT.md) specifies signatures, fields, the replaceable A calculation and minimal future evidence. Anonymized technical results are in [reports/B18_OFFLINE_SERVO_REGRESSION.json](reports/B18_OFFLINE_SERVO_REGRESSION.json). Raw recordings, machine paths, actual joint values, camera parameters, calibration, models and ACT weights remain private. Publication uses the separate B18 branch; Console-17 integration and main are left to their owners.
