# jdcobot100 Onshape export

이 폴더는 제가 CAD 조립 상태와 시뮬레이터 모델이 맞는지 비교하기 위해
남겨둔 export 실험입니다. 완성된 모델만 모아둔 폴더가 아니며, 현재
조립본과 참고용으로 검증된 모델을 의도적으로 나눠 두었습니다.

This directory contains reproducible `onshape-to-robot` configurations for the
jdcobot100 four-DOF arm. Credentials are read from environment variables and
must never be committed.

The root configs point at the in-progress DAPIER assembly. It currently exports
zero joints because the parts are separate root nodes and the revolute mates
have not been added yet. I keep this incomplete state visible instead of
replacing it with the reference model. The `reference/` configs point at the completed
four-joint assembly used by the JD-edu exercise and provide the expected
topology:

```text
base_sub_assembly
  -> dof_base (Onshape mate: base_shoulder)
shoulder_sub_assembly
  -> dof_shoulder (Onshape mate: shoulder_arm1)
arm_1_sub_asssembly
  -> dof_elbow (Onshape mate: arm1_arm2)
arm_2_sub_asssembly_copy_1
  -> dof_wrist_pitch (Onshape mate: arn2_end_arm)
end_arm_sub_assembly
```

The local credentials file is loaded from
`~/.config/onshape-to-robot/env`. Regenerate and normalize both reference
models with:

```bash
onshape/jdcobot100/export_reference.sh
```

To inspect the unfinished DAPIER workspace directly:

```bash
onshape-to-robot --safe onshape/jdcobot100/config.json
onshape-to-robot --safe onshape/jdcobot100/config.mujoco.json
```

The first export generates `jdcobot100.urdf`; the second generates
`jdcobot100.xml`. Meshes are written below `assets/`. The wrapper then maps the
Onshape mate names to the stable simulator names shown above.

Run the headless engine checks with the Python environment installed for
`onshape-to-robot`:

```bash
~/.local/share/uv/tools/onshape-to-robot/bin/python \\
  onshape/jdcobot100/validate_models.py
```

The check loads the URDF in PyBullet and the MJCF in MuJoCo, verifies all four
joint names, applies small position commands, and advances each simulation for
240 steps.

The reference STEP inputs and expected outputs come from:
https://github.com/JD-edu/so101_imitation_learning/tree/main/105_MUJOCO_basic/107_jdcobot100_MUJOCO_load

The extracted, simulator-validated reference bundle is self-contained below
`reference/`: `jdcobot100.urdf`, its `assets/`, and the nine original CAD files
under `step/`. I use it as a comparison point while repairing the live export;
it is not evidence that the unfinished DAPIER assembly is already correct. The
URDF exposes the four revolute joints `dof_base`, `dof_shoulder`, `dof_elbow`,
and `dof_wrist_pitch`.

AI 도움은 export 명령과 joint-name 오류를 정리하는 데 사용하지만, 실제
모델이 맞는지는 export 로그와 PyBullet·MuJoCo 검사 결과를 직접 확인합니다.
