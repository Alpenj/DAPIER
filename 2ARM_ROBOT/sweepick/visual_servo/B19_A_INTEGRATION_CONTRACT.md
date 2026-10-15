# B19 right relative servo and performance contract

The review base is B18 `c44dd1d83083ac0496181c8f97690b2d717ba4f6`. This code reads saved observations and performs pure calculations. It does not connect to the running Native controller or Console. A retains actual-pose IK, path checking, collision certification, stage ownership, STOP and motor writing.

## Pure interface

```python
RightRelativeServoProposal(
    actual_q12,
    right_wrist_observation,
    held_object_evidence,
    right_pad_reference,
    jacobian_evidence,
    *, a_source=None,
)
```

`actual_q12` is the finite canonical twelve-axis feedback vector: five arm radians and one gripper fraction on each side. It must exactly equal the observation's recorded q12. Actual feedback is never clipped or converted into a joint target.

The observation uses B18 `observation_identity={run_id, observation_id}`, `source_kind`, `q12`, `reference_times` and three `views`. The right view has an image identity, file SHA, uint8 BGR image and `recorded_camera` with the existing `fresh` and `reader_valid` evidence. Missing freshness does not create a time cutoff: it leaves REAL correction incomplete. Top and left remain explicit separate inputs. Receipt times are not exposure or servo sampling times.

The shared `observation_binding(..., camera_key='right_wrist')` retains observation ID, q12 hash, camera identity, source, pixel/file SHA, shape and dtype. Its default stays left wrist for the existing PICK interface. Right RECEIVE definitions never reuse the left widest-row/Jaw X definition.

`held_object_evidence` binds the same `observation_identity`, `feedback_sha256`, `camera_state_binding`, source kind and source SHA. It also carries `held_by_left` and `target_association_verified` from current evidence. A powered left hold or a red component alone does not establish that association.

`right_pad_reference` has the same binding fields, `camera_key='right_wrist'`, a source SHA, `role='CURRENT_OBSERVED_PAD'`, `physical_correspondence_verified=true`, explicit task reference `values_px`, `observed_axes`, `feature_definitions` and a local `context`. The target pixels must describe the identified receiving Pad/task relation. An image midpoint, generic dark jaw tips or a hidden CAD Pad projection cannot supply this record. A MODEL observation can carry an explicitly MODEL reference, but cannot produce a REAL result.

The two feature definitions are `RIGHT_HELD_COMPONENT_MINUS_OBSERVED_PAD_TARGET_U/V`. Each measured relative feature is the visible target component centroid minus its observed, task-defined Pad reference in the same raw image. The goal is zero of that explicit relative feature; it is not the centre of the image. Missing axes remain null. The visible component is not promoted to a complete object centre, physically identified approach face or certified graspable region.

`jacobian_evidence` uses B18 `jacobian_record()`: named image-feature rows, px/mm matrix, orthonormal model-frame motion basis, source/context/feature-definition/SHA evidence and validation state. REAL right corrections require `source='REAL_MEASURED'` and `validated=true`. A caller's flag is not a new calibration certificate: its paired physical correspondence and actual-displacement evidence must be retained. Missing or model-only evidence yields no REAL tool correction. External matrices/bases are revalidated by the existing shared calculation.

The returned `RightRelativeServoProposal` includes:

| Field | Meaning |
|---|---|
| `relative_feature_error_px` | Observed object/Pad task error; absent axes are null |
| `observed_control_dimensions` | Independent computed controls, zero when a valid right Jacobian is missing |
| `estimated_tool_correction` | Coefficients in mm and model-frame translation in metres; never a joint goal |
| `expected_image_error_change` | Jacobian prediction on the included image axes |
| `reference_provenance`, `feature_evidence` | Current held/Pad binding, visible contour/graph, clipping and separate model diagnostics |
| `missing_evidence`, `validity` | Exact missing or invalid input |
| `actual_q12_for_A_IK` | Recorded measured basis, for comparison with A's own new actual feedback |
| `executable`, `motor_write_authorized` | Always false |

Raw object contours and existing A face graphs are current pixel evidence. Stored `inner_pad_face_contract` CAD projections and `current_rgbd_faces` model-association diagnostics remain labelled separately. A small PnP reprojection residual does not verify physical corner/Pad identity or TCP accuracy. Existing optical K/D/T artifacts are present but offline solved, not independently field verified. The wrist surface producer's fixed registration string does not establish that those optical inputs were absent.

## Shadow connection after RIGHT_APPROACH

A can publish one immutable observation/actual-feedback packet after RIGHT_APPROACH. The existing OBSERVATION_VIEW workflow and B shadow calculation may consume that packet independently. B opens no camera or bus, launches no controller and requests no new observation move. If A has no new packet, B returns incomplete evidence; a shadow calculation never marks an observation move as reached or successful.

A associates a result with its exact packet/frame/q12/source before considering it. A obtains its own current actual state and runs the existing IK, path checks, collision certificate and stage/STOP authority. B's recorded `actual_q12_for_A_IK` is not permission to reuse stale feedback. This review supplies no executor call site, background worker, retry loop or motor command.

After an approved A move and a new image, the shared `evaluate_response()` checks each Jacobian axis's predicted/observed direction and the whole task image-error norm. Norm axes are all axes with explicit task targets, including a defined Y target when a rank-one Jacobian controls only X. A missing required response feature cannot be silently dropped. An unobserved Y target remains absent. A positive vector dot product cannot hide a reversed axis. Unpredicted axis change, non-finite data, source/feature mismatch or increased total error requires re-identification; this is calculation validity, not a new physical STOP threshold.

The saved additional observation moves remain recorded as target-not-reached. The later RGB observation does not make the requested object approach face or receiving Pad physically identified. A resolves that execution/observation problem separately.

## Latency and left performance interpretation

Historical camera receipt/frame-wait, producer and planner scopes retain their original clocks and nesting. Unknown exposure/acquisition, PICK feature/error, complete command and isolated settling durations remain UNKNOWN. A pre/post image gap is not attributed to CPU or motor movement. A certificate's planned trajectory duration is not certificate CPU time.

CPU benchmarks use the existing interpreter and frozen input files, load functions before timing their pixel work, and report warm samples separately from module initialization. They include pure FK, model-only IK and path-collision fixtures; these are not historical actuator or full certificate durations. OpenCV uses one thread within the benchmark process only.

The baseline calculation remains A's `g=0.8/mm_per_px`, `move=-error/g`. Native A may subsequently replace its prior with same-height local feedback; the fixed-prior comparison does not disable that behavior in A. Candidates B/C/D/E are offline calculations only: other-context local gain, scalar DLS, error-dependent gain, and global-component-selection/ROI feature extraction. The ROI keeps the global target scan and full-image coordinate system; its four-pixel halo comes from two existing 5x5 morphology passes, not a physical margin.

Predicted correction counts use an explicitly fitted local scalar response model, including the recorded difference between commanded motion and actual candidate FK motion. The same pair's post-outcome response defines that model, while candidate B draws its controller gain from other contexts. This is not independent REAL validation or a feasible-motion guarantee. Numerical iteration/error targets are comparison fixtures only. No physical time saving, success probability, new dwell/step cap, TCP accuracy or calibration is inferred.

Image-only Jaw reference availability is counted from the existing A matching-tip evidence. Native A reference confirmation is counted separately because it can also use unchanged measured opening when tips are hidden or image candidates disagree. A native confirmation therefore does not establish a visible Pad/Jaw reference in that image. Encoded burst variation and differences from saved in-memory features are diagnostics, not a sensor-noise or physical accuracy bound.

## Minimum evidence for a later approved right response estimate

The missing inputs are a physically identified receiving Pad/task reference, the same observable held-object feature tracked against it, and paired actual right displacement in an explicit basis. Start with relative X only if that is the only observed axis. Keep actual q12, command/feedback distinction, per-stream frame identities, camera/feedback clock metadata and height/rotation/left-hold changes. Y, depth and rotation remain absent until independently observed. Existing images and optical files should be reused where sufficient; this document requests no new physical measurement or calibration.

All B17/B18 regression cases are retained. Publication is confined to the B19 branch and anonymized technical results. No main/A integration merge, Native/Console change, hardware run, ACT training or subsequent task is included.
