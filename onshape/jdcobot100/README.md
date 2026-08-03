# jdcobot100 Onshape export

This directory contains reproducible `onshape-to-robot` configurations for the
jdcobot100 four-DOF arm. Credentials are read from environment variables and
must never be committed.

The root configs point at the repaired DAPIER assembly. On 2026-08-03 it was
rebuilt around five subassemblies and verified to export one rooted, four-joint
kinematic tree. The `reference/` configs point at the JD-edu exercise assembly
used as the topology reference:

```text
base_sub_assembly
  -> dof_base
shoulder_sub_assembly
  -> dof_shoulder
arm_1_sub_asssembly
  -> dof_elbow
arm_2_sub_asssembly_copy_1
  -> dof_wrist_pitch
end_arm_sub_assembly
```

The local credentials file is loaded from
`~/.config/onshape-to-robot/env`. Regenerate and normalize the repaired working
models with:

```bash
onshape/jdcobot100/export_working.sh
```

Regenerate and normalize both reference models with:

```bash
onshape/jdcobot100/export_reference.sh
```

To inspect the repaired DAPIER workspace directly without normalization:

```bash
onshape-to-robot --safe onshape/jdcobot100/config.json
onshape-to-robot --safe onshape/jdcobot100/config.mujoco.json
```

The first export generates `jdcobot100.urdf`; the second generates
`jdcobot100.xml`. Meshes are written below `assets/`. The working wrapper maps
the exporter names `base`, `shoulder`, `elbow`, and `wrist_pitch` to the stable
simulator names shown above. The raw Onshape mate names are `dof_base`,
`dof_shoulder`, `dof_elbow`, and `dof_wrist_pitch`; onshape-to-robot removes the
`dof_` prefix while exporting.

The repaired Onshape assembly contains eight hidden explicit mate connectors
and four Revolute mates. Two legacy `ParametricPartStudio` features remain
suppressed so that they do not regenerate the old flattened servo instances.

Run the headless engine checks with the Python environment installed for
`onshape-to-robot`:

```bash
~/.local/share/uv/tools/onshape-to-robot/bin/python \\
  onshape/jdcobot100/validate_models.py \\
  --urdf onshape/jdcobot100/jdcobot100.urdf \\
  --mjcf onshape/jdcobot100/jdcobot100.xml
```

The check loads the URDF in PyBullet and the MJCF in MuJoCo, verifies all four
joint names, applies small position commands, and advances each simulation for
240 steps.

The reference STEP inputs and expected outputs come from:
https://github.com/JD-edu/so101_imitation_learning/tree/main/105_MUJOCO_basic/107_jdcobot100_MUJOCO_load

The extracted, simulator-validated reference bundle is self-contained below
`reference/`: `jdcobot100.urdf`, its `assets/`, and the nine original CAD files
under `step/`. The URDF exposes the four revolute joints `dof_base`,
`dof_shoulder`, `dof_elbow`, and `dof_wrist_pitch`.
