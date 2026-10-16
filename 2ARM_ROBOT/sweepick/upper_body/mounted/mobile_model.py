"""Build the existing A 'mobile' integration scene (TurtleBot3 Waffle Pi + aluminium mount + two SO-101 + camera stand)
without changing the runtime folder: the Waffle Pi URDF is read from the repository's sim folder. Dimensions are the
runtime's own USER_DIMENSIONS_UNVERIFIED layout; nothing here measures the real robot."""
import os, sys
from pathlib import Path
H = Path.home()
RT = H / 'DAPIER/tjj-runtime/v1'
os.environ.setdefault('DAPIER_SO101_MJCF', str(H / 'DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml'))
sys.path[:0] = [p for p in (str(RT / 'factory/v1'), str(RT / 'source/production_source_g000'), str(RT / 'source/turtlebot3_waffle_pi')) if p not in sys.path]
import waffle_pi_model
WAFFLE_URDF = Path(__file__).resolve().parent / 'waffle_pi' / 'turtlebot3_waffle_pi_mujoco.urdf'   # git e1267f1 'render official Waffle Pi meshes', read-only copy
waffle_pi_model.BASE_URDF = WAFFLE_URDF
waffle_pi_model.PROJECT_DIR = WAFFLE_URDF.parent
import integration_scenes
import mujoco


def build(grippers='both'):
    return integration_scenes.build_scene('mobile', grippers=grippers).compile()


def build_holding(side='left', half_extent_m=0.02):
    """The same scene with the block held at `side`'s grasp site (the existing A block model, half extent 0.02 m), so a
    transport pose is checked with what it carries. Collision-active like the arm's own geoms."""
    spec = integration_scenes.build_scene('mobile', grippers='both')
    site = spec.site(f'{side}_cube_grasp')
    body = site.parent
    ref = next(g for g in spec.geoms if g.parent.name == body.name and (g.contype or g.conaffinity))
    body.add_geom(name=f'{side}_held_block_MODEL_PRIOR', type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(site.pos), quat=list(site.quat),
                  size=[half_extent_m] * 3, rgba=[.8, .1, .1, 1], contype=ref.contype, conaffinity=ref.conaffinity, group=ref.group)
    return spec.compile()
