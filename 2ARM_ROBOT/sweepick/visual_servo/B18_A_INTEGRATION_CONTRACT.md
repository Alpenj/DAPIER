# B18 visual servo calculation contract

This change prepares image-based correction for obstacle handling and cleaning resume. It does not connect to the running A controller or Console. The review base is B17 commit `5295c6aaa7e561a3bebfe8379591fffb9625545a`; publication stays on `pro/sweepick-b18-vservo-foundation-20261015`. Console-17 integration is managed separately.

The current REAL image reference supports X only. `jaw_tips()` reads inner-tip X positions on bottom image rows. Those rows do not define a physical/task Y reference. The visible red-component centroid is an image measurement and can be clipped or occluded; it is not the full object centre or a target. Missing Y returns `Y_REFERENCE_UNOBSERVED`. No image-centre target is supplied automatically.

## Pure calculation signatures

```python
extract_features(observation, *, jaw_reference=None, target=None,
                 previous=None, a_source=None) -> dict

estimate_jacobian(delta_tool_mm, delta_features_px, *, feature_axes,
                  motion_basis_model, context, feature_definitions,
                  source_sha256, source="REAL_MEASURED",
                  comparable_context=False) -> dict

propose_servo(observation, features, target_features, jacobian, *,
              previous_features=None, previous_response=None,
              damping=0.0) -> dict

evaluate_response(previous_features, current_features, proposal,
                  actual_control_displacement_mm) -> dict
```

The observation contains `observation_identity={run_id, observation_id}`, a source kind, finite canonical `q12` (five arm radians and one gripper fraction per side), B17 `reference_times`, and the three `views`: `top`, `left_wrist`, `right_wrist`. Each view has its own image identity, source SHA, packet association and availability. Images use uint8 BGR in `image_bgr`. An optional recorded burst and its frame identities support the unchanged A image-stability diagnostic. Unavailable views are explicit; they are never filled with another view or simulated pixels.

The left view binds to left feedback, right to right feedback, and top to both. B17 receipt/exposure/measurement distinctions remain. Sequential host receipts are not a failure; unknown exposure and servo sampling times remain uncertainty. Matching packet association does not claim simultaneous exposure or physical stillness. Before correction, the feature record must match the current q12 hash, source kind, left-image identity, recorded file SHA, pixel-array SHA, shape and dtype. Changing the image under an unchanged packet ID requires fresh extraction.

`jaw_reference` includes the recorded X inner positions/midline, camera key, source kind and source SHA. Existing A current-tip or same-opening evidence confirms that reference. `target` contains `values_px`, `observed_axes`, matching `feature_definitions`, camera/source provenance and a local `context`. The legacy u definition is `A_WIDEST_ROW_OBJECT_MIDPOINT_X`; v is `VISIBLE_COMPONENT_CENTROID_Y`. A different feature definition requires its own Jacobian. Y enters control only when a corresponding task target has separate evidence.

The recorded A X target is the confirmed Jaw midline plus its planned lateral target converted using the recorded image scale. That conversion uses the existing development-object/model assumptions. Raw object-centre movement, normalized error movement and a changing target/scale are reported separately.

Top RGB-D retains the existing object/worktable spatial context and candidate frame assumptions. Right wrist remains a separate input without an unverified contribution to left-arm control. No 6D pose or receiving-pad identity is inferred.

## Jacobian and proposal schema

Jacobian rows are named image features in pixels. Its columns are millimetres along orthonormal columns of `motion_basis_model`, expressed in the assembled A model world. A rank-one fit returns one observed column and its basis, with the unobserved column absent. Each local fit records candidate/valid pairs, independent excitation directions, rank, condition number, residuals and source SHA. A fit is not automatically a validated REAL calibration. Incomparable height, camera, target or object conditions cannot be pooled.

`VisualServoProposal` contains:

| Field | Contract |
|---|---|
| `observation_identity` | Exact current packet identity |
| `feature_error_px` | Current minus target; absent axes remain null |
| `observed_control_dimensions` | Number of independent computed controls; zero if evidence is insufficient |
| `jacobian`, `jacobian_source` | `REAL_MEASURED`, `MODEL_ONLY_JACOBIAN`, or labelled test fixture |
| `estimated_tool_correction` | Independent coefficients in mm, their model-frame basis, and derived model translation; no joint goal |
| `expected_image_error_change` | Predicted change on the included image axes |
| `missing_evidence`, `validity` | Explicit reason for a missing or diagnostic calculation |
| `source_sha256`, `model_only` | Input/feature provenance and model scope |
| `executable`, `motor_write_authorized` | Always false |

Least squares solves `J * correction = -error`. Damping defaults to zero and, when explicitly supplied, is numerical regularization in px/mm. SVD rank uses the library's machine-precision convention, not a sensor or physical safety threshold. No move clipping, maximum travel, error hard stop, new margin or dwell rule is introduced. Numerical condition and residuals do not establish physical accuracy. See the versioned [NumPy least-squares documentation](https://numpy.org/doc/2.2/reference/generated/numpy.linalg.lstsq.html) and [rank documentation](https://numpy.org/doc/2.2/reference/generated/numpy.linalg.matrix_rank.html).

The proposal boundary reuses `jacobian_record()` to validate external matrix dimensions, finite values and the orthonormal motion basis. A validated REAL record can accompany only REAL observations/features/targets; test fixtures stay within test fixtures. A MODEL Jacobian may produce only a model-only diagnostic calculation, including when supplied with REAL input. Marking a record validated does not establish calibration: the caller must retain its measurement evidence and context. Non-finite corrections or responses cannot be reused for the next calculation.

## Future A connection point

The replaceable calculation in `sweepick_pick01.visual_align()` is the image error/gain calculation followed by `move = -err_px / g`. A keeps `look()` and `judge()`, its current stage/hold rules, `correct(out, along_jaw_mm, basis, assisted=False, axis_xy=None, above_b=None)`, planning and collision checks, `run_stage("align", ...)`, and the next look. B supplies no serial writer, stage executor or joint goal.

The present scalar `correct()` contract is a correction along the observed Jaw axis. A one-dimensional proposal must use that same axis and sign. A two-dimensional extension needs an explicit A-owned mapping from the proposed translation into its existing `plan_pick(..., tool_offset=...)` path; it cannot silently rename a perpendicular displacement as a Jaw-axis correction. This review changes no A call site.

After A returns actual feedback and a new frame, `evaluate_response()` compares actual feature change with `J * actual_displacement`. An opposite response requires Jacobian re-identification; no new correction is computed when this response is passed back as `previous_response`. No new image returns `NO_NEW_IMAGE`. This is calculation validity, not a new physical STOP or automatic retry. A remains responsible for IK, collision, stage execution, operator authority and STOP.

## Verification scope and minimum future observations

The stored REAL audit verifies local X error reduction in two recorded corrections. Actual candidate FK height, transverse motion, rotation, image scale and planned target also change. These pairs are separate local responses, not a two-axis calibration or independent convergence guarantee. JPEG re-extraction differences are retained alongside the recorded in-memory features. The full task did not succeed. REAL convergence remains `REAL_CONVERGENCE_UNVERIFIED`.

The model branch uses the existing `left_wrist_rgb` camera pose/FOV and a declared virtual viewport. It has no verified REAL left-wrist optical calibration. A virtual target comes from an explicit reference model pose, not the image centre. Four X/Y signed cases pass through existing A pure IK, path collision checks and camera reprojection; their result is `SIM_OR_MODEL_CONVERGED` only. The model response can have the opposite sign to recorded REAL X response and must not be deployed as a measured Jacobian.

The finite-difference probe, eight-iteration test budget and 0.2 px model-test acceptance in the regression are numerical fixture settings, not hardware step limits, dwell requirements or task tolerances. No dynamics are stepped; [MuJoCo forward computation](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-forward) updates only the private model calculation here.

No new physical collection is requested or performed in B18. A future approved collection needs only:

1. An observed, task-defined Jaw/object Y reference in the same left-camera pixel frame, with its source and feature definition. A cropped silhouette, bottom scan row or image midpoint is insufficient.
2. Baseline and post-correction frames for two independent in-plane displacement directions at a documented comparable height/camera pose, with stationary-object evidence. A common baseline and two post-motion observations are the algebraic minimum; repeated same-pose frames are needed to assess measurement variation separately.
3. The actual q12/ticks bracketing those frames, commanded tool displacement and feedback-derived actual displacement, motion-plane basis and units. Preserve transverse/height/rotation changes rather than forcing them into a 2D fit.
4. Camera stream/frame identity, host receipt clock, joint measurement/receipt clock, and native exposure timestamp/mapping when available. Unknown fields stay null. Top may support object/worktable/stationarity evidence; right wrist is not fused without its own verified role.

A chooses any physical displacement through the existing approved execution path. This document sets no physical move amount, new calibration procedure or execution permission. No ACT training, goal predictor, A controller change or B19 work is included.
