"""Build revision/sweepick_mounted_floor_scene.mjb: the A runtime's 'mobile' integration scene (TurtleBot3 Waffle Pi +
aluminium mount + both SO-101 + camera stand; the runtime labels its layout USER_DIMENSIONS_UNVERIFIED) with the two parts
the existing planner and checks need:
  * 'table': a thin box whose top is the floor (z = 0), so the existing table/desk code reads the floor as its support;
  * 'red_block': the existing A block (40 mm, free joint), parked far away until model() places it.
The Waffle Pi URDF comes from git e1267f1 (the runtime folder does not carry it). Writes a provenance JSON next to the mjb."""
import hashlib, json, os, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
H = Path.home()
RT = H / 'DAPIER/tjj-runtime/v1'
WAFFLE = H / 'sweepick_261016_upper_body/mounted/waffle_pi/turtlebot3_waffle_pi_mujoco.urdf'
# 2026-10-16 evening: the depth-fitted left mount offset (0.036, -0.016, -0.016) is REMOVED. The operator states both arms are
# mounted the same (symmetric, both bases 140 mm above the floor); the residual the fit absorbed is not a base offset.
LEFT_MOUNT_FIT_M = (0.0, 0.0, 0.0)
# 2026-10-16 operator ruler (photo): TurtleBot3 top plate 104 mm (= the model), the arm bases 140 mm above the floor; the
# model puts the mount stack 16 mm too low (right base 124 mm). Raising the whole mount (both arms and the camera stand on it)
# by 16 mm puts the floor the top depth sees at -0.1 mm in run 20261016_164052_9b3c13 (was -16.1 mm) with the arm fit
# unchanged (left 0.77 / right 0.80 within 6 mm) and the board check within 2.2 mm.
MOUNT_RAISE_M = 0.016
os.environ.setdefault('DAPIER_SO101_MJCF', str(H / 'DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml'))
sys.path[:0] = [str(RT / 'factory/v1'), str(RT / 'source/production_source_g000'), str(RT / 'source/turtlebot3_waffle_pi')]


def build():
    import mujoco
    import waffle_pi_model
    waffle_pi_model.BASE_URDF = WAFFLE; waffle_pi_model.PROJECT_DIR = WAFFLE.parent
    import integration_scenes
    spec = integration_scenes.build_scene('mobile', grippers='both')
    # 2026-10-16 left mount from the REAL top depth: the model left arm moved by LEFT_MOUNT_FIT_M puts the depth points of the
    # left arm within 6 mm of its meshes 0.33 -> 0.77 at the home pose and 0.66 -> 0.88 at the pre-grasp pose of run
    # 20261016_164052_9b3c13 (two poses, one fit). The right arm already fits (0.80 at home) and is unchanged.
    left = spec.body('left_base'); left.pos = [float(v) + float(o) for v, o in zip(left.pos, LEFT_MOUNT_FIT_M)]
    mount = spec.body('aluminum_mount_UNVERIFIED'); mount.pos = [float(mount.pos[0]), float(mount.pos[1]), float(mount.pos[2]) + MOUNT_RAISE_M]
    box = mujoco.mjtGeom.mjGEOM_BOX
    spec.worldbody.add_geom(name='table', type=box, pos=[0.05, 0.0, -0.005], size=[0.60, 0.80, 0.005], rgba=[.55, .55, .55, 1])
    b = spec.worldbody.add_body(name='red_block', pos=[2.0, 0.0, 0.02]); b.add_freejoint()
    b.add_geom(name='red_block_geom', type=box, size=[0.02] * 3, rgba=[.8, .1, .1, 1])
    m = spec.compile()
    return m, dict(integration_scenes=hashlib.sha256((RT / 'source/production_source_g000/integration_scenes.py').read_bytes()).hexdigest(),
                   waffle_pi_urdf=hashlib.sha256(WAFFLE.read_bytes()).hexdigest(), so101_mjcf=os.environ['DAPIER_SO101_MJCF'])


if __name__ == '__main__':
    import mujoco
    m, src = build()
    out = HERE / 'sweepick_mounted_floor_scene.mjb'
    mujoco.mj_saveModel(m, str(out))
    (HERE / 'sweepick_mounted_floor_scene.json').write_text(json.dumps(dict(
        scene=out.name, sha256=hashlib.sha256(out.read_bytes()).hexdigest(), sources=src,
        layout='mobile_aluminum_USER_DIMENSIONS_UNVERIFIED (runtime label)', support='table top = floor z 0',
        block='red_block 40 mm free joint (existing A block model)', left_mount_fit_m=list(LEFT_MOUNT_FIT_M),
        mount_raise_m=MOUNT_RAISE_M, mount_raise_basis='operator ruler 2026-10-16: arm bases 140 mm above the floor, TurtleBot3 top 104 mm; checked against the top depth floor of run 20261016_164052_9b3c13 (-16.1 -> -0.1 mm)',
        left_mount_fit_basis='real top depth of run 20261016_164052_9b3c13: share of left-arm points within 6 mm 0.33->0.77 (home) and 0.66->0.88 (pre-grasp)'), indent=1) + '\n')
    print(out, m.ngeom, m.nbody)
