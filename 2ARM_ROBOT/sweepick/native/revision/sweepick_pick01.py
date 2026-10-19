"""PICK01 / PICK02: a supervised real pick with the left arm, sensor-planned (no ACT, no streaming):
observe -> pre-grasp -> align from the wrist image -> descend -> close on the object -> lift -> lower -> open -> retreat -> home.

Existing code joined together:
  observation  top camera through the SDK reader (SdkTop), left wrist frames (WristCam), joints through the read-only bus
  frame chain  camera <- printed board (ArUco PnP of this session's frame) <- arm datum (ruler record 2026-09-23) <- model
  planning     sim_data_factory.teacher (the SIM teacher's IK, pre-grasp / descend / lift geometry, collision check)
  arm moves    sweepick_move_a.run, one stage per call, level "model"; a stage's path AND its settle trims are checked first
  gripper      sweepick_move_a.grip: the existing sweepick_grasp.Jaw primitive owns the closing command (contact onset from the
               residual of the free stroke, squeeze to the device preload, hold); arm progress rules are not used for it
  scene        the object stays in the collision scene; only left pad - block and block - desk contact are allowed
A stage is opened by the previous controller result AND by an observation (wrist image / top depth / jaw residual):
arriving at a joint target (HOLDING) is not 'pads beside the block', and a stop of the jaws is not a grasp.
The joint mapping, the frame chain and the wrist-image direction are CANDIDATES / records of one session.

usage: sweepick_pick01.py session OUT [--execute]   the whole PICK02 session (without --execute: observe and plan only)
       sweepick_pick01.py observe OUT | look OUT TAG | stage OUT NAME [--execute] | correct OUT --along-jaw-mm X (ASSISTED)
"""
import argparse, contextlib, hashlib, json, subprocess, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
RT = Path.home() / "DAPIER/tjj-runtime/v1"
sys.path[:0] = [p for p in (str(RT / "factory/v1"), str(RT / "source/production_source_g000"), str(RT / "source/turtlebot3_waffle_pi")) if p not in sys.path]
import sweepick_real_command as rc, sweepick_kin, sweepick_move_a as mv
import sweepick_setup as setup                                                 # 2026-10-16: DESK or MOUNTED_FLOOR (sweepick_setup.json)
MOUNTED = setup.SETUP == setup.MOUNTED_FLOOR

IN = Path.home() / "sweepick_261007_commission/inputs"
CAND, CAL, ASM = IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"
SCENE = setup.MOUNTED_SCENE if MOUNTED else RT / "model/dual_scene_desk_both.mjb"
# the arm datum on the printed board: operator ruler record of 2026-09-23 (registered-capture-07-board-input.json), axes assumed parallel
T_DATUM_FROM_BOARD = np.array([[0, -1, 0, 0.201], [-1, 0, 0, -0.034], [0, 0, -1, -0.04], [0, 0, 0, 1.0]])
DATUM_IN_MODEL_BASE = np.array([0.0388353, 0.0, 0.0254])                       # motor-1 lower shaft face in the model base (motor1-cad-datum-audit.json)
BOARD_Z_IN_WORLD = setup.FLOOR_Z if MOUNTED else float(DATUM_IN_MODEL_BASE[2] + T_DATUM_FROM_BOARD[2, 3])    # the support top in the model's world: desk -14.6 mm / mounted: the floor
OBJECT_FILE = Path(__file__).resolve().parent / "sweepick_object.json"          # 2026-10-19: the object is configuration (size, colour), not code
OBJECTS = json.loads(OBJECT_FILE.read_text())
OBJECT = dict(OBJECTS["objects"][OBJECTS["active"]], name=OBJECTS["active"])
OBJECT_COLOUR_RULE = OBJECTS["colour_rules"][OBJECT["colour"]]
BLOCK_HALF_M = float(OBJECT["size_m"]) / 2                                      # half the cube side (the A block model was 40 mm)
# MOUNTED_FLOOR: the OS30A depth reads long. On the printed board of run 20261016_164052_9b3c13 the board pose from its markers
# (image geometry, marker size; no depth) put the board top at -14.6 mm in the robot frame and the depth points on it at
# -24.5 mm, 0.612 m from the camera: 1.6 % long (READ02 measured about 2 %). The depth is scaled by this before use.
DEPTH_SCALE_MOUNTED = 1.0 / 1.016
ARM = list(rc.JOINTS[:5]); ALL = list(rc.JOINTS)
STAGES = ("pregrasp", "align", "descend_a", "descend_b", "descend", "descend_settle", "close", "lift", "carry", "carry_back", "lower", "open", "retreat", "home", "retreat_arm", "home_arm", "abort_home")
GRIP_ENDED = ("GRIP_CONTACT_HOLDING", "GRIP_NO_CONTACT_HOLDING")                 # the jaw primitive ended normally (with or without a contact); a fault / stop / unusable feedback is not in here
ENDED_OK = ("HOLDING",) + GRIP_ENDED


def allowed(state, name):
    """May this stage run now? From what the session has actually done: the stages that reached the executor, in order, how
    each ended, and which joints they powered. The normal chain and the planned ways back after a 'not ready' observation
    are in here; after a stage that did not end normally (fault, bus loss, stop, no progress, refusal) only the manual
    'abort_home' is left, and the session never calls that."""
    st = state["stages"]; order = state.get("order", []); res = lambda n: (st.get(n) or {}).get("controller_result")
    last = order[-1] if order else None
    if name == "abort_home":
        return True, "manual tool; not part of a session"
    align_short = last in SHORT_OK and res(last) == "TARGET_NOT_REACHED_HOLDING"     # an arm move that completed and settled a little short of its joint target (loaded proportional servos do): the arm holds, and where it stands is judged by the look / the next stage's own checks, not by the joint target
    if last is not None and res(last) not in ENDED_OK and not align_short:
        return False, f"the last stage ('{last}') ended as {res(last)}: nothing is commanded after it by the session"
    gripper_on = "close" in st                                                 # the gripper is powered from the close stage on, and only then
    height = state.get("height"); pending = bool(state.get("pending_align"))
    if name == "pregrasp":
        return last is None, "first stage"
    has_b = "descend_b" in ((state.get("plan") or {}).get("ticks") or {})
    if name == "align":
        return pending and height in ("pregrasp", "descend_a", "descend_b") and not gripper_on, "a correction computed at this height is waiting to be moved"
    if name == "descend_a":
        return height == "pregrasp" and last in ("pregrasp", "align") and not pending, "the (corrected) pre-grasp pose is held"
    if name == "descend_b":
        return has_b and height == "descend_a" and last in ("descend_a", "align") and not pending, "the first stop above the block is held"
    if name == "descend":
        need = "descend_b" if has_b else "descend_a"
        return height == need and last in (need, "align") and not pending, "the last stop above the block is held, pads still above the top face"
    if name == "descend_settle":
        return last == "descend", "the last descent ended short of its joint target and the image shows the block undisturbed: the same target once more, with the settle trim"
    if name == "close":
        return last in ("descend", "descend_settle"), "the grasp pose is held"
    if name == "lift":
        return last == "close" and res("close") == "GRIP_CONTACT_HOLDING", "the jaws hold a contact candidate"
    if name == "carry":
        return last == "lift" and res("lift") == "HOLDING", "the lift pose is held with the block"
    if name == "carry_back":
        return last == "carry", "the carry pose is held"
    if name == "lower":
        return last in ("lift", "carry_back"), "the lift pose is held (straight after the lift, or back from the carry pose)"
    if name == "open":
        return last == "lower" or last == "close", "after the lowering, or straight after a close that is not followed by a lift (no contact / block not seen): the jaws are powered and the arm is at the grasp height"
    if name == "retreat":
        return last == "open", "the jaws are open at the grasp height (all six joints powered)"
    if name == "home":
        return last == "retreat", "the pre-grasp pose is held with all six joints"
    if name == "retreat_arm":
        return height in ("descend_a", "descend_b", "descend") and not gripper_on, "below the pre-grasp height with the gripper never powered: back up with the five arm joints only"
    if name == "home_arm":
        return height == "pregrasp" and not gripper_on, "at the pre-grasp height with the gripper never powered: home with the five arm joints only"
    return False, "unknown stage"


SHORT_OK = ("pregrasp", "align", "descend_a", "descend_b", "descend", "descend_settle", "lift", "carry", "carry_back", "lower", "retreat", "retreat_arm")     # arm positioning stages; never the gripper, and not the final return (that one must be confirmed before torque is released)
HOLDING_OK = ("HOLDING",)
CLOSE_OK = ("GRIP_CONTACT_HOLDING",)                                          # the existing Jaw primitive ended holding a contact candidate
GATE = {"descend_settle": "block_undisturbed_after_descent", "descend_a": "ready_to_descend", "descend_b": "ready_for_final_descent", "descend": "ready_for_final_descent", "close": "ready_to_close", "lift": "ready_to_lift"}   # observation needed before the stage, besides the previous controller result
TRIM = ("pregrasp", "align", "descend_a", "descend_settle", "lift", "carry", "carry_back", "retreat", "home", "retreat_arm", "home_arm", "abort_home")   # descend_a: the pads are above the block by the vertical uncertainty, so a shortfall there is the arm's own; never descend_b / descend / lower or the gripper           # loaded free moves only; never where a shortfall may be a contact (descend, lower) or on the gripper


def mapping():
    return rc.load_mapping(CAND, CAL, ASM)


def home_command_ticks(home_ticks, mp, side="left"):
    """Keep the observed start ticks intact, but make a return target that the servo's own configured range can accept."""
    command, adjusted = {}, {}
    for name, raw in home_ticks.items():
        value = float(raw)
        lo, hi = float(mp[side][name]["lo"]), float(mp[side][name]["hi"])
        target = min(max(value, lo), hi) if np.isfinite(value) else value
        command[name] = target
        if target != value:
            adjusted[name] = dict(observed_tick=value, command_tick=target, configured_range=[lo, hi], outside_by_ticks=value - target)
    return command, dict(source="fresh observed home_ticks plus the active calibrated servo lo/hi; sweepick_move_a verifies the device EEPROM Min/Max matches this mapping before a write",
                         basis="only an observed home tick outside the servo's actual configured range is replaced by its nearest endpoint; no margin is added",
                         observed_ticks={name: float(value) for name, value in home_ticks.items()}, command_ticks=command, adjusted=adjusted)


def model(block=None, table_z=None):
    """The assembled dual model with the desk top at the real height and the block at `block` (centre, yaw) or far away.
    MOUNTED_FLOOR: the floor is where the observation put it, so with a block the support top is the block's bottom."""
    import mujoco
    if table_z is None:
        table_z = float(block[0][2]) - BLOCK_HALF_M if (MOUNTED and block is not None) else BOARD_Z_IN_WORLD
    m = mujoco.MjModel.from_binary_path(str(SCENE))
    m.geom_pos[m.geom("table").id][2] += table_z
    if MOUNTED:                                                                 # the mobile scene's ground plane is a static world plane fixed at z 0 (moving its pos does not move it:
        g = m.geom("floor").id                                                  # MuJoCo 3.3.7 keeps its xpos); the 'table' box at the observed floor is the floor instead
        m.geom_contype[g] = 0; m.geom_conaffinity[g] = 0
    for g in range(m.ngeom):                                                    # the scene's block is the 40 mm A block: give it the active object's size
        if m.geom_bodyid[g] == m.body("red_block").id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            m.geom_size[g][:] = BLOCK_HALF_M                                     # (geom_rbound stays the larger 40 mm bound: conservative for the broad phase)
    adr = m.jnt_qposadr[m.body_jntadr[m.body("red_block").id]]
    if block is None:
        m.qpos0[adr:adr + 3] = [2.0, 0.0, BLOCK_HALF_M]
    else:
        (x, y, z), yaw = block
        m.qpos0[adr:adr + 7] = [x, y, z, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    return m


def world_from_camera(T_camera_from_board, m):
    if MOUNTED:                                                                 # camera and arms are both on the robot: fixed, no board in the chain
        return setup.mounted_robot_from_camera(m)
    Tb = np.eye(4); Tb[:3, 3] = DATUM_IN_MODEL_BASE
    Tw = np.eye(4); Tw[:3, 3] = m.body("left_base").pos
    return Tw @ Tb @ T_DATUM_FROM_BOARD @ np.linalg.inv(np.asarray(T_camera_from_board, float))


def robot_boxes(m, d):
    return [(g, m.body(m.geom_bodyid[g]).name) for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(("left_", "right_"))]


def inside(m, d, pts, prefix, margin):
    out = np.zeros(len(pts), bool)
    for g, name in robot_boxes(m, d):
        if not name.startswith(prefix):
            continue
        a = m.geom_aabb[g]; loc = (pts - d.geom_xpos[g]) @ d.geom_xmat[g].reshape(3, 3) - a[:3]
        out |= np.all(np.abs(loc) <= a[3:] + margin, axis=1)
    return out


def block_sized_patch(top, object_half, cell=0.01, min_points=50):
    """The block's top face among the points at block height. Those points are split into connected patches (1 cm grid,
    8-neighbour); only a patch whose spread fits one block can be the block (longest side at most 6 halves, shortest at least 0.75 half: 1.5 .. 6 cm for the 4 cm block; seen: 3.6 .. 4.1 cm). A long strip - the foot of a wall or a box edge at the same height (2026-10-18: the wall-floor
    edge 0.42 m ahead outvoted the block when all points were pooled into one median) - is listed and refused.
    Returns (points of the chosen patch or None, every patch with its verdict)."""
    from scipy import ndimage
    if len(top) < min_points:
        return None, []
    ij = np.floor(top[:, :2] / cell).astype(int); lo = ij.min(0); ij -= lo
    grid = np.zeros(ij.max(0) + 1, int); np.add.at(grid, (ij[:, 0], ij[:, 1]), 1)
    lab, n = ndimage.label(grid > 0, structure=np.ones((3, 3), int)); which = lab[ij[:, 0], ij[:, 1]]
    out, best = [], None
    for k in range(1, n + 1):
        p = top[which == k]
        ext = [float(np.percentile(p[:, i], 98) - np.percentile(p[:, i], 2)) for i in (0, 1)] if len(p) else [0.0, 0.0]
        diag = float(np.hypot(*ext))
        if len(p) < min_points:
            verdict = "too few points"
        elif max(ext) > 6 * object_half:
            verdict = "longer than a block (strip: wall foot / edge)"
        elif min(ext) < 0.75 * object_half:
            verdict = "narrower than a block face"
        else:
            verdict = "BLOCK_SIZED"
        out.append(dict(points=int(len(p)), centre_xy=[float(v) for v in np.median(p[:, :2], 0)], extent_xy_m=ext, diagonal_m=diag, verdict=verdict))
        if verdict == "BLOCK_SIZED" and (best is None or len(p) > len(best)):
            best = p
    out.sort(key=lambda r: -r["points"])
    if best is not None:
        for r in out:
            r["chosen"] = bool(r["verdict"] == "BLOCK_SIZED" and r["points"] == len(best))
    return best, out


def analyse(bgr, depth_mm, K, ticks, *, object_half=BLOCK_HALF_M, lite=False):
    """Scene from one top FrameSet and the joint readings: frame chain, how well the depth points of the arms fall on the
    model arms (the joint mapping and the chain are both in this number), and the object's top face."""
    import mujoco, tjj_perception as tp
    from sweepick_read03_capture import board_pose
    K = np.asarray(K, float).reshape(3, 3); bp = board_pose(bgr, K)
    if bp is None and not MOUNTED:
        return dict(ok=False, why="the printed board was not found in the top image: no frame chain")
    mp = mapping(); m = model(); d = mujoco.MjData(m)
    q12 = rc.ticks_to_q12(mp, ticks); sweepick_kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    Twc = world_from_camera(None if bp is None else bp["T"], m); Kd = K.copy(); Kd[:2] /= 2
    scale = DEPTH_SCALE_MOUNTED if MOUNTED else 1.0
    pts = tp.back_project(np.asarray(depth_mm, float) * scale / 1000.0, Kd, Twc)
    floor = None
    if MOUNTED:                                                                 # the floor as this observation sees it (it is not at the model's z 0: run 20261016_173652_ea3c03)
        floor = floor_plane(pts, ~inside(m, d, pts, "", 0.03) & ~((pts[:, 0] > -0.207) & (pts[:, 0] < 0.095) & (np.abs(pts[:, 1]) < 0.210)))
        if not floor["ok"]:
            return dict(ok=False, setup=setup.SETUP, why=f"the floor could not be found in the top depth: {floor['why']}", floor=floor)
        if bp is not None:                                                      # the board lies on the floor: its pose from the markers (no depth) must agree with the depth floor
            Twb = Twc @ np.asarray(bp["T"], float); bz = float(Twb[2, 3]); fz = floor_z(floor, Twb[0, 3], Twb[1, 3])
            floor["board_check"] = dict(board_z_by_markers_m=bz, floor_z_by_depth_m=fz, difference_m=bz - fz, tolerance_m=3 * floor["residual_m"],
                                        tolerance_basis="3 x the floor fit residual of this observation")
            if abs(bz - fz) > 3 * floor["residual_m"]:
                return dict(ok=False, setup=setup.SETUP, why=f"the board (markers) and the depth floor disagree by {1000 * (bz - fz):.1f} mm: the camera-to-robot transform or the depth scale does not hold", floor=floor)
        zt = floor_z(floor, 0.20, 0.0)
    else:
        zt = float(np.median(pts[(np.abs(pts[:, 2] - BOARD_Z_IN_WORLD) < 0.03) & ~inside(m, d, pts, "", 0.03)][:, 2])) if len(pts) else None    # desk height as the DEPTH sees it
    arms = {}
    for side in (() if lite else ("left", "right")):                         # lite (alignment looks, 2026-10-18 timing): no arm-on-model share, no block search (about 1.2 s)
        near = inside(m, d, pts, side, 0.06) & (pts[:, 2] > zt + 0.03)
        arms[side] = dict(points=int(near.sum()), **{f"share_within_{int(k * 1000)}mm": (float(inside(m, d, pts[near], side, k).mean()) if near.sum() else None) for k in (0.0, 0.006, 0.012, 0.025)})
    if lite:
        top_all = None
    else:
        if MOUNTED:                                                                 # the floor around the robot, outside its own outline (the mounted scene footprint: base x -0.197..0.085 with the front camera, rails |y| <= 0.200; +1 cm)
            free = ~inside(m, d, pts, "", 0.02) & ~((pts[:, 0] > -0.207) & (pts[:, 0] < 0.095) & (np.abs(pts[:, 1]) < 0.210)) & (pts[:, 0] > -0.30) & (pts[:, 0] < 0.55) & (np.abs(pts[:, 1]) < 0.60)
        else:
            free = ~inside(m, d, pts, "", 0.02) & (pts[:, 0] > 0.05) & (pts[:, 0] < 0.45) & (np.abs(pts[:, 1]) < 0.25)
        h = pts[:, 2] - (floor_z(floor, pts[:, 0], pts[:, 1]) if MOUNTED else zt)  # height above the support under each point
        top_all = pts[free & (h > 2 * object_half - 0.01) & (h < 2 * object_half + 0.01)]
    top, top_candidates = block_sized_patch(top_all, object_half) if not lite else (None, [])
    obj = None
    if top is not None and len(top) >= 50:
        ex = [float(np.percentile(top[:, i], 98) - np.percentile(top[:, i], 2)) for i in (0, 1)]
        yaw, n = tp.top_face_yaw(np.asarray(depth_mm, float) / 1000.0, Kd, Twc, np.median(top, 0), surface_z=zt, robot_mask=lambda p: inside(m, d, p, "", 0.02))
        obj = dict(top_points=int(len(top)), centre_xy=[float(v) for v in np.median(top, 0)[:2]], extent_xy_m=ex, top_height_above_depth_desk_m=float(np.median(top[:, 2]) - zt), yaw_rad=None if yaw is None else float(yaw), yaw_points=int(n),
                   centre_world=[float(np.median(top, 0)[0]), float(np.median(top, 0)[1]), (floor_z(floor, *np.median(top, 0)[:2]) if MOUNTED else BOARD_Z_IN_WORLD) + object_half],
                   class_match=bool(all(abs(e - 2 * object_half) <= object_half / 2 for e in ex)))
        obj["geometry"] = dict(classification="MODEL_PRIOR", half_extents_m=[object_half] * 3,
                               source="existing A block model", observed_patch_extent_xy_m=ex,
                               class_match_role="PROVISIONAL visible-patch diagnostic; partial depth coverage does not measure hidden object dimensions")
    if obj is not None:
        obj["candidates"] = top_candidates
        try:                                                                   # 2026-10-18: the top face's outline from the head depth (min-area rectangle of the block-top points, world xy).
            import cv2                                                         # Four runs: 0 deg blocks read 7.4 / -8.4, 45 deg blocks -43.6 / 43.8 (the depth yaw read -36.5 for a 0 deg block)
            (_, _), (rw_, rh_), ra_ = cv2.minAreaRect((top[:, :2] * 1000.0).astype(np.float32))
            obj["yaw_rect_rad"] = float(np.radians(((ra_ + 45.0) % 90.0) - 45.0)); obj["yaw_rect_extent_mm"] = [float(rw_), float(rh_)]
        except Exception as e:
            obj["yaw_rect_error"] = f"{type(e).__name__}: {e}"
    return dict(ok=True, setup=setup.SETUP, top_candidates=top_candidates, board=None if bp is None else dict(markers=bp["markers"], rms_px=bp["rms_px"], T_camera_from_board=bp["T"], role="DIAGNOSTIC_ONLY" if MOUNTED else "FRAME_CHAIN"), T_world_camera=Twc.tolist(), camera_source=setup.mounted_camera_source() if MOUNTED else dict(source='BOARD_CHAIN'), camera_in_world=Twc[:3, 3].tolist(),
                desk_z=(dict(by_chain=None, by_depth=zt, floor=floor, depth_scale=scale, note="mounted: heights come from the floor fitted in this observation's depth (scaled 1/1.016); the board, when seen, only checks it") if MOUNTED else
                        dict(by_chain=BOARD_Z_IN_WORLD, by_depth=zt, note="the depth reads about 2 % long (READ02); the chain value is used for heights")), q12=[float(v) for v in q12], ticks=ticks, arms_on_model=arms, object=obj,
                tool_left=[float(v) for v in d.site_xpos[m.site("left_cube_grasp").id]], tool_right=[float(v) for v in d.site_xpos[m.site("right_cube_grasp").id]], **tool_axis(m, d, SIDE))


def floor_plane(pts, mask, *, band_m=(-0.12, 0.06), min_points=2000):
    """A plane z = a x + b y + c through the floor points (robot frame) by trimmed least squares: points standing on the
    floor (a block, a cable) are left out as they fall off the plane. Not found: too few points."""
    p = pts[mask & (pts[:, 2] > band_m[0]) & (pts[:, 2] < band_m[1])]
    if len(p) < min_points:
        return dict(ok=False, why=f"{len(p)} depth points around the robot (need {min_points})", points=int(len(p)))
    keep = np.abs(p[:, 2] - np.median(p[:, 2])) < 0.02
    for trim in (0.012, 0.008, 0.006):
        A = np.c_[p[keep, 0], p[keep, 1], np.ones(int(keep.sum()))]; co = np.linalg.lstsq(A, p[keep, 2], rcond=None)[0]
        keep = np.abs(p[:, 2] - np.c_[p[:, :2], np.ones(len(p))] @ co) < trim
    r = p[keep, 2] - np.c_[p[keep, :2], np.ones(int(keep.sum()))] @ co
    if keep.sum() < min_points:
        return dict(ok=False, why=f"only {int(keep.sum())} depth points lie on one plane", points=int(keep.sum()))
    return dict(ok=True, coef=[float(v) for v in co], points=int(keep.sum()), residual_m=float(np.std(r)),
                slope_deg=[float(np.degrees(np.arctan(co[0]))), float(np.degrees(np.arctan(co[1])))],
                x_range_m=[float(p[keep, 0].min()), float(p[keep, 0].max())], y_range_m=[float(p[keep, 1].min()), float(p[keep, 1].max())])


def floor_z(floor, x, y):
    a, b, c = floor["coef"]
    return a * np.asarray(x, float) + b * np.asarray(y, float) + c if np.ndim(x) else float(a * x + b * y + c)


def tool_axis(m, d, side="left"):
    rot = d.site_xmat[m.site(f"{side}_cube_grasp").id].reshape(3, 3); jaw = rot[:, 2].copy(); jaw[2] = 0.0; jaw /= np.linalg.norm(jaw)
    return dict(jaw_axis_xy=[float(jaw[0]), float(jaw[1])], tool_along_jaw_mm=float(1000 * np.dot(d.site_xpos[m.site(f"{side}_cube_grasp").id][:2], jaw[:2])), jaw_axis_side=side)


PAD_GEOMS = ("left_pgripper_pad_1", "left_pgripper_pad_2")                    # the LEFT arm's; the session arm's are pad_geoms()


def pad_geoms(side=None):
    """The two pad meshes of the session arm (2026-10-18 R1: the left names were used for the right arm -> KeyError)."""
    s_ = side or SIDE; return (f"{s_}_pgripper_pad_1", f"{s_}_pgripper_pad_2")


def geom_vertices(m, d, name):
    """World vertices of one mesh geom of the model at the pose in d (the real shape, not a box around it)."""
    import mujoco
    g = m.geom(name).id; k = m.geom_dataid[g]; R = d.geom_xmat[g].reshape(3, 3)
    return d.geom_xpos[g] + m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]] @ R.T


def pad_placement(m, d, centre, yaw, half=BLOCK_HALF_M, side="left"):
    """Where the two pads of the left gripper are against the block, from the pad meshes at the pose in d, in one frame:
      vertical_mm   lowest pad vertex that is over the block's top face (or within 3 mm of its edge), above the top face
      gaps_mm       per pad: distance along the jaw axis from the pad's vertices at or below the top face to the block's side
                    face (None while the pad is still above the top face); negative = that pad is over / inside the block
      tool_minus_pad_low_mm   how far the lowest pad vertex is below the tool point (the model's own number)
    PICK01 / PICK02 run 5 were planned with 'pad 5 mm below the tool point'; the meshes say about 16 .. 18 mm."""
    pads = (f"{side}_pgripper_pad_1", f"{side}_pgripper_pad_2")
    top = centre[2] + half; rot = d.site_xmat[m.site(f"{side}_cube_grasp").id].reshape(3, 3); jaw = rot[:, 2].copy(); jaw[2] = 0.0; jaw /= np.linalg.norm(jaw)
    tool = d.site_xpos[m.site(f"{side}_cube_grasp").id]; Rb = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    rel = np.arctan2(jaw[1], jaw[0]) - yaw; half_along = half * (abs(np.cos(rel)) + abs(np.sin(rel)))
    out = dict(jaw_axis_xy=[float(jaw[0]), float(jaw[1])], block_half_along_jaw_mm=float(1000 * half_along), tool_above_top_mm=float(1000 * (tool[2] - top)), tool_along_jaw_mm=float(1000 * np.dot(tool[:2] - centre[:2], jaw[:2])), pads={})
    low_over, low_all = [], []
    for n in pads:
        v = geom_vertices(m, d, n); loc = (v[:, :2] - centre[:2]) @ Rb.T; near = (np.abs(loc) <= half + 0.003).all(axis=1)
        s_ = (v[:, :2] - centre[:2]) @ jaw[:2]; side = float(np.sign(s_.mean())); band = v[:, 2] <= top + 0.002
        low_all.append(v[:, 2].min()); low_over += [v[near, 2].min()] if near.any() else []
        out["pads"][n] = dict(side=int(side), lowest_above_top_mm=float(1000 * (v[:, 2].min() - top)), gap_mm=float(1000 * ((side * s_[band]).min() - half_along)) if band.any() else None)
    out["vertical_mm"] = float(1000 * (min(low_over) - top)) if low_over else None
    out["tool_minus_pad_low_mm"] = float(1000 * (tool[2] - min(low_all)))
    g = [out["pads"][n]["gap_mm"] for n in pads]
    out["gaps_mm"] = g; out["room_each_side_mm"] = None if None in g else float(sum(g) / 2)
    out["recentre_along_jaw_mm"] = None if None in g else float(out["pads"][pads[0]]["side"] * (g[1] - g[0]) / 2)     # tool shift along +jaw that makes the two gaps equal: a pad on side s gains s * shift
    return out


def taught_heights(side):
    """How far above the taught GRASP pose the taught pre-grasp and just-above poses hold the tool (m, model FK of the hand-
    taught joints; the same arm in nearby poses, so the model's own height error cancels). None when not taught."""
    import mujoco
    f = {k: TAUGHT_REFS[side] / d / "ticks.json" for k, d in (("grasp", "H0_grasp"), ("above", "H2_above_c"), ("pregrasp", "H6_pregrasp"))}
    if not all(x.exists() for x in f.values()):
        return None
    m = model(); d = mujoco.MjData(m); mp = mapping(); z = {}
    for k, fp in f.items():
        t = json.loads(fp.read_text()); sweepick_kin.set_q12(m, d, rc.ticks_to_q12(mp, {s_: t[s_]["ticks"] for s_ in ("left", "right")})); mujoco.mj_forward(m, d)
        z[k] = float(d.site_xpos[m.site(f"{side}_cube_grasp").id][2])
    return dict(pregrasp=z["pregrasp"] - z["grasp"], above=z["above"] - z["grasp"], grasp_z=z["grasp"], source=str(TAUGHT_REFS[side]))


_TAUGHT_YAW = {}


def taught_block_yaw(side):
    """The block yaw (mod 90 deg) the hand-taught grasp of this arm was made at: the jaw axis of the taught grasp joints (model FK).
    2026-10-18 195343_auto: the head-depth yaw of a block put at the taught place read -51.8 deg (the wrist then started 38 deg turned);
    earlier R1 looks of one unmoved block read 4..29 deg. The wrist image corrects the rest. None when not taught."""
    if side in _TAUGHT_YAW:
        return _TAUGHT_YAW[side]
    import mujoco
    f = TAUGHT_REFS.get(side, Path("/nonexistent")) / "H0_grasp" / "ticks.json"; val = None
    if f.exists():
        t_ = json.loads(f.read_text()); mp = mapping(); m = model(); d = mujoco.MjData(m)
        sweepick_kin.set_q12(m, d, rc.ticks_to_q12(mp, {s_: t_[s_]["ticks"] for s_ in ("left", "right")})); mujoco.mj_forward(m, d)
        a = tool_axis(m, d, side)["jaw_axis_xy"]; y = float(np.arctan2(a[1], a[0])); val = float((y + np.pi / 4) % (np.pi / 2) - np.pi / 4)
    _TAUGHT_YAW[side] = val; return val


def plan_pick(scene, cfg=None, tool_offset=None, above_b=None, keep=None, side="left", approach_dz=0.0, above_dz=0.0):
    """Pre-grasp / intermediate / grasp / lift joint targets of one arm (`side`; the other arm stays where it was measured) from the SIM teacher's planner (IK + its path
    collision check), at the measured start pose.
    tool_offset (world, m): where the HAND is aimed relative to the observed block centre (visual correction). The block stays
    at its OBSERVED pose in the scene: a hand correction is not a measurement of the block.
    The grasp is chosen from every tilt / jaw direction / lift height the arm can do, by the room the PAD MESHES leave on
    both sides of the block against how far a joint residual moves the pads along the jaw axis; the hand is re-centred so
    that the two pad gaps are equal (a leaning jaw puts its lower pad nearer the block than the tool point suggests)."""
    import mujoco
    import sweepick_trajectory as tt
    from sim_data_factory.teacher import HandoverTeacher, TeacherFailure, UP
    o = scene["object"]; yaw = o["yaw_rad"] or 0.0; centre = np.array(o["centre_world"]); width = 2 * BLOCK_HALF_M
    if keep is None and o.get("yaw_source") != "wrist":                         # a first plan: the taught grasp's block yaw, not the head-depth one (see taught_block_yaw)
        ty_ = taught_block_yaw(side)
        if ty_ is not None:
            yaw = ty_
            if o.get("yaw_rect_rad") is not None:                              # the head's top-face rectangle (mod 90), folded to the symmetric yaw nearest the taught one
                r_ = float(o["yaw_rect_rad"]); dr_ = (r_ - ty_ + np.pi / 4) % (np.pi / 2) - np.pi / 4
                if abs(dr_) > np.radians(HEAD_RECT_YAW_TRUST_DEG):             # the rectangle read 0 deg blocks 7..10 deg off: within that the taught yaw stays (the wrist image corrects the rest)
                    yaw = ty_ + dr_
    o.setdefault("geometry", dict(classification="MODEL_PRIOR", half_extents_m=[width / 2] * 3,
                                 source="existing A block model; not the measured visible-patch footprint"))
    m = model((centre, yaw)); t = HandoverTeacher(m); cfg = t.cfg; d = mujoco.MjData(m); q12 = np.array(scene["q12"]); sweepick_kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    other = "right" if side == "left" else "left"; arm_i, oth_i = (0, 1) if side == "left" else (1, 0)
    mp = mapping(); observed_own = ((scene.get("ticks") or {}).get(side) or rc.q12_to_ticks(mp, q12)[side])
    command_own, command_seed = home_command_ticks(observed_own, mp, side)
    start_gear = d.qpos[t.plan.adr].copy(); lo, hi = float(mp[side]["gripper"]["lo"]), float(mp[side]["gripper"]["hi"])
    start_gear[5 + 6 * arm_i] = t.g_a[arm_i] + t.g_b[arm_i] * ((command_own["gripper"] - lo) / (hi - lo))
    observed_other = ((scene.get("ticks") or {}).get(other) or rc.q12_to_ticks(mp, q12)[other])
    command_other, other_seed = home_command_ticks({"gripper": observed_other["gripper"]}, mp, other)
    other_seed["role"] = f"NUMERICAL_IK_SEED_ONLY; actual {other} ticks remain in the execution collision scene; no {other} command"
    lo, hi = float(mp[other]["gripper"]["lo"]), float(mp[other]["gripper"]["hi"])
    # IK validates a command seed; measured q12 stays intact for FK and the actual execution-path collision check.
    start_gear[5 + 6 * oth_i] = t.g_a[oth_i] + t.g_b[oth_i] * ((command_other["gripper"] - lo) / (hi - lo))
    t.cmd = start_gear.copy(); prof = tt.load_profile(); pg = __import__("sweepick_progress").load()
    user = np.zeros(3) if tool_offset is None else np.asarray(tool_offset, float)
    seen = centre + np.array([user[0], user[1], 0.0])                          # where the wrist image puts the block relative to the hand (lateral only); used for the pad gaps, never for the collision scene
    rb = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]]); jaws = dict(x=rb[:, 0], y=rb[:, 1])
    t.plan.pose(d.qpos, t.cmd); rl = t.plan.radial(side, centre); reasons, cands = [], []
    here = {n: float(v) for n, v in rc.q12_to_ticks(mp, q12)[side].items()}; rng = {n: (mp[side][n]["lo"], mp[side][n]["hi"]) for n in ARM}
    own = slice(0, 6) if side == "left" else slice(6, 12)
    def ticks(w):
        q = t.to_canonical(np.array(w)); keep_q = np.array(q12, float).copy(); keep_q[own.start:own.start + 5] = q[own.start:own.start + 5]     # this arm's 5 joints from the plan; its gripper and the other arm as measured
        tk = rc.q12_to_ticks(mp, keep_q)[side]; return {n: float(round(tk[n])) for n in ARM}
    def pose_of(tk_):
        arms = {side: dict(here, **tk_), other: {n: float(v) for n, v in rc.q12_to_ticks(mp, q12)[other].items()}}
        dd = mujoco.MjData(m); sweepick_kin.set_q12(m, dd, rc.ticks_to_q12(mp, arms)); mujoco.mj_forward(m, dd); return dd
    def solve(start, pos, ap, jaw, label, allow=()):
        dd = mujoco.MjData(m); dd.qpos[:] = d.qpos; dd.qpos[t.plan.adr] = start; t.cmd = np.array(start, float).copy()
        w, info = t.plan_move(dd, side, pos, ap, [jaw], allow=allow, label=label)
        if len(w) != 1:
            raise TeacherFailure(f"{label}: the planner needs a via point; this session runs single joint-space segments only")
        return w[0], info
    # 191754_auto: the hand-taught poses are nearly vertical (tool 1..5 deg from vertical in the model) but IK started from the start pose did not find
    # a vertical pre-grasp there and took tilt 10 with the elbow 22 deg and the wrist 17 deg away from the taught pose: another view in the wrist camera and
    # another model error. The taught pre-grasp joints are tried first as the IK start (the same arm configuration as the taught view), then the start pose.
    seeds = [start_gear]
    tf_ = TAUGHT_REFS.get(side, Path("/nonexistent")) / "H6_pregrasp" / "ticks.json"
    if tf_.exists():
        try:
            tt_ = json.loads(tf_.read_text()); arms_t = {side: {n: float(tt_[side]["ticks"][n]) for n in ARM + ["gripper"]}, other: {n: float(v) for n, v in rc.q12_to_ticks(mp, q12)[other].items()}}
            dt_ = mujoco.MjData(m); sweepick_kin.set_q12(m, dt_, rc.ticks_to_q12(mp, arms_t)); mujoco.mj_forward(m, dt_)
            sg_ = dt_.qpos[t.plan.adr].copy(); sg_[5 + 6 * arm_i] = start_gear[5 + 6 * arm_i]; sg_[5 + 6 * oth_i] = start_gear[5 + 6 * oth_i]
            seeds = [sg_, start_gear]
        except Exception as e:
            reasons.append(f"taught pre-grasp seed not used: {type(e).__name__}: {e}")
    for tilt in np.deg2rad(cfg.pick_tilt_deg):
        if cands:                                                              # the least feasible tilt wins (below): larger tilts are not planned (select_arm took 10.9 s for all six)
            reasons.append(f"tilt {np.degrees(tilt):.0f}: not planned (a smaller tilt is feasible)"); continue
        ap = np.cos(tilt) * -UP + np.sin(tilt) * rl
        for jname, jaw in jaws.items():
            tag = f"tilt {np.degrees(tilt):.0f} jaw {jname}"
            if keep is not None and ((keep.get("tilt_deg") is not None and abs(np.degrees(tilt) - keep["tilt_deg"]) > 1e-6) or jname != keep["jaw"]):
                continue                                                       # a re-plan inside a session keeps the grasp it started with: a correction is a small shift, never another jaw direction (tilt None: any tilt of that jaw)
            try:
                aim = centre + user
                th_ = taught_heights(side)                                     # 2026-10-18: the approach heights of the hand-taught views (the wrist image is compared at the same height)
                # 200250_auto: the grasp was planned 5 mm below the taught grasp (head block centre + offset), the pads met the floor and the hand slid
                # forward past the block centre. The tool goes no lower than the taught grasp (model FK of the taught joints, the same floor and block).
                g_off = max(cfg.left_grasp_offset_m, th_["grasp_z"] + GRASP_ABOVE_TAUGHT_M - float(aim[2])) if th_ else cfg.left_grasp_offset_m
                pre_h = (g_off + th_["pregrasp"] if th_ else cfg.pregrasp_height_m) + approach_dz
                for _ in range(2):                                             # second pass: the hand re-centred so that both pad gaps are equal
                    err_ = None
                    for sd_ in seeds:
                        try:
                            pre, info = solve(sd_, aim + [0, 0, pre_h], ap, jaw, f"{side} pregrasp"); err_ = None; break
                        except TeacherFailure as e_:
                            err_ = e_
                    if err_ is not None:
                        raise err_
                    des, _ = solve(pre, aim + [0, 0, g_off], ap, jaw, f"{side} grasp", allow=(side,))
                    pl = pad_placement(m, pose_of(ticks(des)), seen, yaw, side=side)
                    if pl["room_each_side_mm"] is None:
                        raise TeacherFailure(f"{side} grasp: a pad does not reach below the block's top face at the grasp pose")
                    shift = pl["recentre_along_jaw_mm"] / 1000.0 * np.array([pl["jaw_axis_xy"][0], pl["jaw_axis_xy"][1], 0.0])
                    if abs(pl["recentre_along_jaw_mm"]) < 0.1:
                        break
                    aim = aim + shift
                # how far joint residuals of the size the executor accepts as 'reached' move the pads: along the jaw axis and vertically
                tkd = ticks(des); base = pose_of(tkd); p0 = base.site_xpos[m.site(f"{side}_cube_grasp").id].copy(); jx = np.array(pl["jaw_axis_xy"]); u_lat = u_ver = 0.0
                for n in ARM:
                    pp = pose_of(dict(tkd, **{n: tkd[n] + pg["reach"]})).site_xpos[m.site(f"{side}_cube_grasp").id]
                    u_lat += abs(float(np.dot(pp[:2] - p0[:2], jx))); u_ver += abs(float(pp[2] - p0[2]))
                # intermediate height: the lowest pad vertex stays above the top face by the vertical uncertainty (and 1 mm for the block's own height)
                h_a = 0.02 + pl["tool_minus_pad_low_mm"] / 1000.0 + u_ver + 0.001
                if th_:                                                        # the taught just-above height, never below the pads' safe height
                    h_a = max(h_a, g_off + th_["above"] + above_dz)           # 200250_auto: the rises made at the pre-grasp (approach_dz) are not carried to this stop (one 7.8 cm joint-space descent curved forward); only those made at it (above_dz)
                if h_a >= pre_h:
                    raise TeacherFailure(f"the pads would already be within the vertical uncertainty ({u_ver * 1000:.1f} mm) of the top face at the pre-grasp height")
                mid, _ = solve(pre, aim + [0, 0, h_a], ap, jaw, f"{side} above block")
                mid_b = None
                if above_b is not None:                                        # a second stop: pads this far above the top face (from the residual MEASURED at the first stop)
                    mid_b, _ = solve(mid, aim + [0, 0, 0.02 + pl["tool_minus_pad_low_mm"] / 1000.0 + above_b], ap, jaw, f"{side} just above block")
                des2, _ = solve(mid if mid_b is None else mid_b, aim + [0, 0, g_off], ap, jaw, f"{side} grasp from above", allow=(side,))
                pl_mid = pad_placement(m, pose_of(ticks(mid)), seen, yaw, side=side); pl = pad_placement(m, pose_of(ticks(des2)), seen, yaw, side=side)
                lift = None
                for lh in (cfg.lift_height_m, 0.045, 0.03):                    # the lift only has to take the block clear of the desk; a lower lift may be within the joint ranges where the full one is not
                    try:
                        l_, _ = solve(des2, aim + [0, 0, g_off + lh], ap, jaw, f"{side} lift", allow=(side,))
                    except TeacherFailure:
                        continue
                    tk = dict(pregrasp=ticks(pre), descend_a=ticks(mid), descend=ticks(des2), lift=ticks(l_)); bad = []
                    if mid_b is not None:
                        tk["descend_b"] = ticks(mid_b)
                    for a_, b_ in (("start", "pregrasp"), ("pregrasp", "descend_a")) + ((("descend_a", "descend_b"), ("descend_b", "descend")) if mid_b is not None else (("descend_a", "descend"),)) + (("descend", "lift"),):          # the executor's own range gates on each segment, before anything moves
                        initial_seed = dict(here, **{n: command_own[n] for n in ARM})
                        seg = tt.make(dict(here, **({} if a_ == "start" else tk[a_])), tk[b_], prof,
                                      **({"command_start": initial_seed} if a_ == "start" and initial_seed != here else {}))
                        if not seg["moving"]:
                            continue
                        v = tt.validate(side, mp[side], seg, prof, mapping_sha="plan", level="model")
                        bad += [f"{b_}: {g_['gate']} {g_.get('joint', '')}" for g_ in v["hard"] + v["provisional"] if g_["gate"] in ("servo_range", "model_joint_range_candidate")]
                    if not bad:
                        lift = (lh, tk); break
                    last_bad = bad
                if lift is None:
                    raise TeacherFailure(f"no lift height within the joint ranges ({locals().get('last_bad')})")
            except TeacherFailure as e:
                reasons.append(f"{tag}: {str(e)[:150]}"); continue
            lh, tk = lift; room = pl["room_each_side_mm"]
            cand = dict(ok=True, tilt_deg=float(np.degrees(tilt)), jaw=jname, ik=info if not isinstance(info, dict) else {k: float(v) for k, v in info.items()}, ticks=tk, object_width_m=width, lift_height_m=float(lh), side=side, command_seed={f"{side}_gripper": command_seed, f"{other}_gripper": other_seed},
                        room_each_side_m=float(room / 1000.0), pad_gaps_mm=pl["gaps_mm"], joint_residual_moves_pads_mm=dict(along_jaw=float(u_lat * 1000), vertical=float(u_ver * 1000), residual_ticks=pg["reach"],
                        basis="each arm joint off by the executor's reach tolerance, effects added: the worst case the executor would still call 'reached'"),
                        room_over_residual=float(room / max(u_lat * 1000, 1e-6)), jaw_lean_deg=float(np.degrees(np.arcsin(abs(float(pose_of(tk["descend"]).site_xmat[m.site(f"{side}_cube_grasp").id].reshape(3, 3)[2, 2]))))),
                        placement_at_grasp=pl, placement_above_block=pl_mid, placement_just_above_block=None if mid_b is None else pad_placement(m, pose_of(ticks(mid_b)), seen, yaw, side=side), above_b_m=above_b, model_jaw_gap_now_m=float(width + 2 * room / 1000.0), pad_overlap_m=float(-pl["pads"][f"{side}_pgripper_pad_1"]["lowest_above_top_mm"] / 1000.0),
                        descend_a=dict(tool_above_centre_m=float(h_a), pads_above_top_mm=pl_mid["vertical_mm"], lowest_pad_vertex_above_top_mm=min(v_["lowest_above_top_mm"] for v_ in pl_mid["pads"].values()), fraction=None,
                                       basis="IK pose straight above the grasp: the lowest vertex of the pad meshes stays above the block's top face by what joint residuals can move it vertically, plus 1 mm"),
                        approach=dict(pregrasp_height_m=pre_h, above_height_m=h_a, approach_dz_m=approach_dz, taught=th_),
                        geometry=dict(pregrasp_height_m=cfg.pregrasp_height_m, grasp_offset_m=cfg.left_grasp_offset_m, lift_height_m=float(lh), source="sim_data_factory.teacher.TeacherConfig; lift height lowered only when the full one is outside a joint range"),
                        object_observed_world_m=[float(v) for v in centre], object_observed_yaw_rad=float(yaw), tool_offset_world_m=[float(v) for v in (aim - centre)], visual_offset_world_m=[float(v) for v in user], recentring_world_m=[float(v) for v in (aim - centre - user)],
                        room_note="from the pad meshes at the grasp pose, after the hand is re-centred between the pads; candidate model")
            reasons.append(f"{tag}: feasible, room {room:.1f} mm each side, a joint residual moves the pads {u_lat * 1000:.1f} mm along the jaw axis (ratio {cand['room_over_residual']:.2f}), lift {lh * 1000:.0f} mm")
            cands.append(cand)
    if not cands:
        return dict(ok=False, why=reasons, side=side)
    lowest = min(c["tilt_deg"] for c in cands)                                 # 2026-10-18 (user): the gripper is built for a vertical approach - the least tilt that is feasible wins,
    best = max((c for c in cands if c["tilt_deg"] == lowest), key=lambda c: c["room_over_residual"])   # then the most room against what joint residuals do to the pads along the jaw axis
    best["considered"] = reasons
    return best


RIGHT_NOT_WIRED = ("the executor runs either arm, but each with its OWN measured references: the left's are PICK01/PICK02 (wrist gain/"
                   "direction) and sweepick_jaw_profile_left.json; the right needs sweepick_wrist_record_right.json and sweepick_jaw_profile_right.json "
                   "(a no-load close of the right gripper and the right wrist response at pre-grasp), which do not exist yet")


def select_arm(scene):
    """MOUNTED_FLOOR: plan the pick for both arms from the same observation and choose one.
    Only one arm has a path -> that arm. Both -> the larger room_over_residual (room each side of the block over how far
    the executor's reach tolerance moves the pads along the jaw axis): one definition, computed by this same planner for
    both arms. Equal -> left (the arm with REAL pick evidence). Neither -> None."""
    # 2026-10-18 timing: planning the far arm took 8.7 s of failing IK (block on the left: the right arm). The nearer arm is planned first; the other
    # one only when it is not much farther (SKIP_FAR_ARM_M) or the nearer one has no path. Both near the middle: both planned, as before.
    import mujoco
    m0 = model(); d0 = mujoco.MjData(m0); mujoco.mj_forward(m0, d0); c0_ = np.array(scene["object"]["centre_world"][:2], float)
    dist0 = {s_: float(np.linalg.norm(d0.xpos[m0.body(f"{s_}_base").id][:2] - c0_)) for s_ in ("left", "right")}
    near_ = min(dist0, key=dist0.get); far_ = "right" if near_ == "left" else "left"
    plans = {near_: plan_pick(json.loads(json.dumps(scene, default=str)), side=near_)}
    if plans[near_]["ok"] and dist0[far_] - dist0[near_] > SKIP_FAR_ARM_M:
        plans[far_] = dict(ok=False, why=[f"not planned: {(dist0[far_] - dist0[near_]) * 1000:.0f} mm farther from the block than the {near_} arm, which has a path"], side=far_)
    else:
        plans[far_] = plan_pick(json.loads(json.dumps(scene, default=str)), side=far_)
    ok = [s_ for s_ in ("left", "right") if plans[s_]["ok"]]
    cost = {s_: (plans[s_]["room_over_residual"] if plans[s_]["ok"] else None) for s_ in ("left", "right")}
    if not ok:
        selected, reason = None, "no pick path for either arm"
    elif len(ok) == 1:
        selected, reason = ok[0], f"only the {ok[0]} arm has a pick path"
    else:                                                                     # 2026-10-18 (user): the arm nearer to the block moves (horizontal distance from its shoulder-pan axis)
        import mujoco
        m_ = model(); d_ = mujoco.MjData(m_); mujoco.mj_forward(m_, d_); c_ = np.array(scene["object"]["centre_world"][:2], float)
        dist = {s_: float(np.linalg.norm(d_.xpos[m_.body(f"{s_}_base").id][:2] - c_)) for s_ in ("left", "right")}
        hi, lo_ = max(cost.values()), min(cost.values())
        if hi > 0 and (hi - lo_) / hi > 0.10:                                  # the success measure decides: more room against what joint errors do to the pads
            selected = "right" if cost["right"] > cost["left"] else "left"; why_ = "larger room over the reach residual (the planner's measure of how safely the pads clear the block)"
        else:                                                                  # within 10 %: the nearer arm (a less stretched pose)
            selected = min(dist, key=dist.get); why_ = "rooms within 10 %; nearer to the block"
        reason = f"both arms have a path; {selected}: {why_} (room/residual {cost['left']:.2f} left vs {cost['right']:.2f} right; distance {dist['left'] * 1000:.0f} mm left vs {dist['right'] * 1000:.0f} mm right)"
    return dict(schema="sweepick.arm-selection.v1", selected=selected, reason=reason, executable=selected in executable_arms() if selected else False,
                block_centre_robot_m=[float(v) for v in scene["object"]["centre_world"]],
                cost=dict(value=cost, definition="room_over_residual = pad room each side of the block / pad motion along the jaw axis from the executor's reach tolerance (higher is better)", source="sweepick_pick01.plan_pick"),
                arms={s_: dict(plan_ok=plans[s_]["ok"], why=None if plans[s_]["ok"] else plans[s_]["why"][:3], room_each_side_mm=plans[s_]["room_each_side_m"] * 1000 if plans[s_]["ok"] else None,
                               tilt_deg=plans[s_].get("tilt_deg"), jaw=plans[s_].get("jaw"), executable=s_ in executable_arms()) for s_ in ("left", "right")},
                plans=plans)


# ---- reading the cell (no write) ---------------------------------------------------------------------------------------
EPISODE = None        # the session's continuous recorder (sweepick_episode.Episode) when one runs: it owns the camera readers and lends them; it lets go of an arm's port for every read or stage here
DEFERRED_COLLISION_DIAGNOSTICS = []
SESSION_WRIST_PAIRING = None  # the selected session's confirmed registry is also used by standalone recovery


def read_ticks():
    if EPISODE is not None:
        with EPISODE.lease("left", "right"):
            return _read_ticks()
    return _read_ticks()


def _read_ticks():
    from sweepick_local_loop import open_readonly_bus
    out = {}
    for side in ("left", "right"):
        b = open_readonly_bus(side)
        try:
            out[side] = dict(ticks={n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}, torque={n: int(v) for n, v in b.sync_read("Torque_Enable", normalize=False).items()},
                             status={n: int(v) for n, v in b.sync_read("Status", normalize=False).items()})
        finally:
            b.disconnect()
    return out


KEEP_TOP = False                                                               # the CLI session / replay keep ONE head reader for the whole process (each start costs about 14 s; 2026-10-18)
_KEPT_TOP = None


def _kept_top(workdir):
    global _KEPT_TOP
    from sweepick_local_loop import SdkTop
    if _KEPT_TOP is None or _KEPT_TOP.proc.poll() is not None:
        import atexit
        _KEPT_TOP = SdkTop(Path(workdir), 30 * 60 * 60)
        atexit.register(lambda t=_KEPT_TOP: t.close())
    return _KEPT_TOP


def top_frame(workdir, wait_s=30.0, depth_frames=30):
    """One image and a depth map from the SDK reader. The depth is the per-pixel median of the valid readings of
    `depth_frames` consecutive FrameSets of the static scene (a single FrameSet leaves holes on plain surfaces)."""
    from sweepick_local_loop import SdkTop
    resident = (EPISODE is not None and EPISODE.top is not None) or KEEP_TOP     # the session's one SDK reader: only FrameSets received AFTER this call are used
    s = (EPISODE.top if (EPISODE is not None and EPISODE.top is not None) else _kept_top(workdir)) if resident else SdkTop(workdir, depth_frames + 40); t0 = time.monotonic(); fs, stack = None, []
    first = s.get(); seen = first["n"] if resident and first is not None else 0
    try:
        while time.monotonic() - t0 < wait_s and len(stack) < depth_frames:
            if not resident and s.proc.poll() is not None and s.get() is None:
                break                                                           # the helper ended without any FrameSet: report now, not after wait_s
            cur = s.get()
            if cur is not None and cur["n"] != seen and cur["n"] >= 5 and cur["bgr"] is not None:
                seen, fs = cur["n"], cur; stack.append(cur["zd"].astype(np.float32))
            time.sleep(0.01)
        K = s.K
    finally:
        closed = dict(resident_reader="kept open by the session's recorder", framesets_received=s.count, reader_error=s.error) if resident else s.close()
    if fs is None:
        rc_ = s.proc.poll() if hasattr(s, "proc") else None
        raise RuntimeError(f"no FrameSet from the top camera after {time.monotonic() - t0:.1f} s of {wait_s} s (reader_error={s.error}, helper_returncode={rc_}, see {getattr(s, 'workdir', '?')}/sdk.stderr.log)")
    z = np.stack(stack); z[z == 0] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(z, axis=0)
    n_valid = np.sum(~np.isnan(z), axis=0); med[(n_valid < max(3, len(stack) // 5)) | np.isnan(med)] = 0
    return dict(bgr=fs["bgr"].copy(), depth_mm=med, K=K, depth_frames=len(stack), valid_share=dict(single=float((stack[-1] > 0).mean()), merged=float((med > 0).mean())),
                stamps={k: fs[k] for k in ("sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s", "recv_monotonic_s")}, close=closed)


def wrist_frame(path, frames=3, tries=3, once=None):
    """One reading of the wrist camera, asked again when the camera did not open or did not deliver (run 12b of 2026-10-08:
    the driver refused one open, the next open would have worked). Reading only: nothing is sent to a motor here. Every try is recorded."""
    log = []
    for k in range(tries):
        try:
            r = (once or _wrist_frame_once)(path, frames)
        except Exception as e:
            r = dict(readable=False, camera_failed=True, why=f"{type(e).__name__}: {e}")
        log.append(dict(k=k, camera_failed=bool(r.get("camera_failed")), why=r.get("why") if r.get("camera_failed") else None))
        if not r.get("camera_failed"):
            break
        time.sleep(1.0)
    r["camera_tries"] = log
    return r


def _wrist_frame_once(path, frames=3):
    """`frames` DIFFERENT consecutive frames of the left wrist reader (by the reader's own frame counter), each measured;
    the middle one is saved. The measurement counts only when the frames agree (a blurred, torn or changing image does not)."""
    import cv2
    from sweepick_local_loop import WristCam
    resident = EPISODE is not None and f"{SIDE}_wrist" in EPISODE.wrists
    if not resident and SESSION_WRIST_PAIRING is not None:
        import sweepick_local_loop as loop
        if not Path(SESSION_WRIST_PAIRING).is_file():
            raise RuntimeError("selected session wrist pairing is unavailable; no recovery camera fallback")
        loop.WRIST_PAIRING = Path(SESSION_WRIST_PAIRING)
    c = EPISODE.lend(f"{SIDE}_wrist") if resident else WristCam(f"{SIDE}_wrist", f"/dev/dapier/{SIDE}_wrist_rgb"); got, t0 = [], time.monotonic()
    try:
        if resident:
            l0, _ = c.get(); got.append(dict(n=l0["n"] if l0 is not None else -1, skip=True))     # only frames received after this call
        else:
            time.sleep(1.0)
        while len([g for g in got if not g.get("skip")]) < frames and time.monotonic() - t0 < 4.0:
            l, _ = c.get()
            if l is not None and (not got or l["n"] != got[-1]["n"]):
                got.append(dict(n=l["n"], recv_monotonic_s=l["recv_monotonic_s"], frame=l["frame"].copy()))
            time.sleep(0.005)
    finally:
        c.close()
    got = [g for g in got if not g.get("skip")]
    if len(got) < frames:
        return dict(readable=False, camera_failed=True, why=f"only {len(got)} new wrist frames in 4 s")
    cv2.imwrite(str(path), got[len(got) // 2]["frame"])
    return wrist_features([g["frame"] for g in got], ids=[dict(n=g["n"], recv_monotonic_s=g["recv_monotonic_s"]) for g in got], saved=str(path))


def wrist_features(frames, ids=None, saved=None):
    """Object and jaw-tip features of consecutive frames and whether they agree with each other."""
    objs = [wrist_object(f) for f in frames]; tips = [jaw_tips(f) for f in frames]; mid = len(frames) // 2
    vis = [o for o in objs if o.get("visible")]
    stable = len(vis) == len(objs) and max(o["centre_px"] for o in vis) - min(o["centre_px"] for o in vis) <= WRIST["measure_px"] and max(o["width_px"] for o in vis) - min(o["width_px"] for o in vis) <= 2 * WRIST["measure_px"]
    obj = dict(objs[mid], stable=bool(stable)) if objs[mid].get("visible") else dict(visible=False, stable=False, seen_in=len(vis), of=len(objs))
    return dict(saved=saved, frames=ids, object=obj, tips=tips[mid], jaw_reference=jaw_reference(frames[mid]))


def jaw_tips(bgr):
    """Inner edges of the jaw tips in THIS image (dark and not reddish, on the bottom rows). None where a tip cannot be told
    from what is next to it (the block's shaded side, or the tip hidden behind the block)."""
    import cv2
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(int); dark = (lab[..., 0] < 95) & (lab[..., 1] < 136); H, W = dark.shape; out = []
    for y in (H - 2, H - 4):
        d = dark[y]; l = int(np.argmax(d[:8])) if d[:8].any() else 0        # as in jaw_reference: the border column or two may not be dark
        while l < W // 2 and d[l:l + 3].any(): l += 1
        r = W - 1 - int(np.argmax(d[::-1][:8])) if d[-8:].any() else W - 1
        while r > W // 2 and d[max(r - 2, 0):r + 1].any(): r -= 1
        out.append((l - 1 if 3 < l < W // 2 else None, r + 1 if W // 2 < r < W - 4 else None))
    left = out[0][0] if out[0][0] is not None and out[1][0] is not None and abs(out[0][0] - out[1][0]) <= 2 else None
    right = out[0][1] if out[0][1] is not None and out[1][1] is not None and abs(out[0][1] - out[1][1]) <= 2 else None
    return dict(left_px=left, right_px=right)


WRIST = dict(
    gain_px_per_mm=3.0, direction="the object moves towards image-RIGHT when the tool moves along +jaw axis (model site z axis)",
    basis="PICK01 2026-10-08, pick01_run2 looks 01 -> 02 -> 03 at the pre-grasp height: tool -3.8 mm along the jaw axis moved the object centre 139.0 -> 130.0 px (2.4 px/mm); tool +5.8 mm moved it 130.0 -> 146.5 px (2.9 px/mm); 01 -> 03 directly: +2.0 mm for +7.5 px (3.8 px/mm). 3.0 is used and the result is measured again after the move. "
          "The model's wrist camera gives the opposite sign and is not used",
    scope="left wrist camera, 320x240, vertical top grasp near the board. Translation gain is from pre-grasp height. Yaw targets use the fresh initial top observation; wrist edge slope is diagnostic only, not an extrinsic calibration or a world-yaw measurement",
    object_colour="Lab a* >= 140 (the red development block); checked against edges read by eye on six stored frames (2 .. 4 px)",
    jaw_reference="inner edges of the two jaw tips on the bottom image row against a plain bright background (folded pose): 64 / 246 px at gripper tick 3366 in three stored frames",
    aligned_slope_deg=4.7, slope_per_jaw_deg=0.87,
    aligned_basis="PROVISIONAL, vertical (tilt 0) top grasp only. Run 10 of 2026-10-08: far top edge at 4.5 .. 4.9 deg in three looks (pre-grasp, pre-grasp, above the block) with the pads coming down beside the block and the slope unchanged (0.35 deg). "
                  "Run 13: the top depth gave the block's yaw as -12.5 deg, the wrist followed it (jaw axis 90.8 -> 77.5 deg by measured joints), the edge stood at -6.9 deg and the block was turned at the first stop (to -3.4 deg; afterwards 5 mm / 4 deg off in the top view). "
                  "Response 0.87 deg of slope per deg of jaw yaw from those two runs, which assumes the block stood the same in both; the response measured in the session replaces it. This is a provisional current-image alignment reference, not a physical yaw measurement; perspective and hand pose can change the edge slope between heights",
    slope_tolerance_deg=3.0, slope_basis="far top edge of the block in the image: 9.3 and 10.9 deg in two pre-grasp frames of the same block (1.6 deg apart); after the descent that left a pad on the block it was -0.3 deg (11 deg change)",
    width_role="the block's width in the image is an AUXILIARY sign of approach only (it must grow when the hand comes down). It is not a height: the 201 .. 207 px of the two contacts and the 228 .. 230 px of the person-placed grasp are not targets",
    lateral_error_seen_mm=8.5, lateral_basis="largest offset of the block from the jaw midline at the pre-grasp pose so far: 4.2 mm (PICK01), 7.1 and 8.5 mm (PICK02 runs 6 and 7, 2026-10-08): how far the hand has had to be shifted along the jaw axis",
    measure_px=4, measure_basis="automatic edges against edges read by eye: 2 .. 4 px")
WRIST["stored_jaw_references"] = [                                               # measured midlines (camera fixed to the gripper: they depend on the opening only)
    dict(gripper_tick=3366.0, inner_px=[64, 246], midline_px=155.0, source="PICK01 2026-10-08, three stored frames"),
    dict(gripper_tick=3433.0, inner_px=[57, 250], midline_px=153.5, source="2026-10-18 camera-only check against a bright background")]
WRIST_LEFT = WRIST
STORED_REF_TICK_TOL, STORED_REF_PX_TOL = 5.0, 8.0                              # same opening; a live read further than this from the stored midline is not trusted


def jaw_ok(scene):
    """observe: the jaw reference for the session arm, live or the stored one at the same opening (recorded in the scene)."""
    w = scene.get("wrist") or {}; tick = ((scene.get("ticks") or {}).get(SIDE) or {}).get("gripper")
    if "jaw_reference" in w:
        w["jaw_reference"] = jaw_reference_with_stored(w["jaw_reference"], tick); scene["wrist"] = w
    return bool((w.get("jaw_reference") or {}).get("readable"))


def jaw_reference_with_stored(live, tick):
    """The live jaw-tip read when it agrees with a stored measurement at the same opening; else the stored one (10/18: a dark band
    on the bottom image row made the live read 211 px against 161 px measured three times at the same tick). Without a stored
    reference at this opening the live read stands as it is (readable or not)."""
    live = dict(live or {}); refs = [r for r in WRIST.get("stored_jaw_references") or [] if tick is not None and abs(r["gripper_tick"] - tick) <= STORED_REF_TICK_TOL]
    if not refs and tick is not None:
        # 204559_auto (right): the jaws stood fully open at 2990 (they stop at 2998, measured by hand 2026-10-18) and the live read failed on a dim
        # wall; the stored reference was taken fully open too (3014, before the range check). Both at the mechanical end = the same opening.
        try:
            mech = float(json.loads((HERE / "sweepick_start_pose.json").read_text())[SIDE]["gripper_mechanical_open_tick"])
        except Exception:
            mech = None
        if mech is not None and tick >= mech - 20:
            refs = [r for r in WRIST.get("stored_jaw_references") or [] if r["gripper_tick"] >= mech - 20]
    if not refs:
        return live
    ref = min(refs, key=lambda r: abs(r["gripper_tick"] - tick))
    if live.get("readable") and abs(live.get("midline_px", 1e9) - ref["midline_px"]) <= STORED_REF_PX_TOL:
        return dict(live, stored_check=dict(midline_px=ref["midline_px"], agrees=True))
    return dict(readable=True, inner_px=list(ref["inner_px"]), midline_px=float(ref["midline_px"]), gap_px=int(ref["inner_px"][1] - ref["inner_px"][0]),
                source="STORED_REFERENCE_SAME_OPENING", stored=ref, live=live, gripper_tick=tick)
# 2026-10-16: the arm of a session is chosen per block (select_arm). The right arm runs only with ITS OWN measured wrist
# alignment record and gripper device profile; until both files exist it is planned but not executed.
RIGHT_WRIST_RECORD = HERE / "sweepick_wrist_record_right.json"
JAW_PROFILES = {"left": getattr(mv, "JAW_PROFILE", HERE / "sweepick_jaw_profile_left.json"), "right": HERE / "sweepick_jaw_profile_right.json"}
SIDE = "left"


def other(side):
    return "right" if side == "left" else "left"


def executable_arms():
    """The arms whose execution references are measured: the left always (PICK01/PICK02, sweepick_jaw_profile_left.json);
    the right only when its wrist record and jaw profile files exist."""
    return ("left",) + (("right",) if RIGHT_WRIST_RECORD.exists() and JAW_PROFILES["right"].exists() else ())


def set_side(side):
    """Make `side` the session's arm: its wrist record and jaw profile become the ones used. Refuses an arm without its own."""
    global SIDE, WRIST
    if side not in ("left", "right"):
        raise ValueError(f"unknown arm {side!r}")
    if side == "right":
        if "right" not in executable_arms():
            raise RuntimeError(f"RIGHT_ARM_NOT_MEASURED: {RIGHT_NOT_WIRED}")
        WRIST = json.loads(RIGHT_WRIST_RECORD.read_text())
    else:
        WRIST = WRIST_LEFT
    SIDE = side
    return side


def _wrist_red_components(bgr):
    """The existing red-block mask and its connected components."""
    import cv2
    ch = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[..., int(OBJECT_COLOUR_RULE["lab_channel"])].astype(int)          # sweepick_object.json (red: a >= 140, the existing prior)
    mask = ch >= OBJECT_COLOUR_RULE["value"] if OBJECT_COLOUR_RULE["op"] == ">=" else ch <= OBJECT_COLOUR_RULE["value"]
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return (mask,) + cv2.connectedComponentsWithStats(mask)[1:3]


def _contour_runs(points, selected):
    """Contiguous cyclic runs of observed contour pixels matching selected."""
    if not selected.any():
        return []
    if selected.all():
        return [points.tolist()]
    start = (int(np.flatnonzero(~selected)[0]) + 1) % len(points); runs = []; run = []
    for offset in range(len(points)):
        index = (start + offset) % len(points)
        if selected[index]:
            run.append(points[index].tolist())
        elif run:
            runs.append(run); run = []
    if run:
        runs.append(run)
    return runs


WRIST_EDGE_MIN_LEN_PX = 200                                                    # outline length (off the image border) needed for an angle: about one block side at the pre-grasp height


def block_edge_angle(mask, border=3, min_seg=25):
    """Direction of the block's outline in the wrist image, mod 90 deg, in (-45, 45]: 0 = sides along image x (the jaw axis)
    and image y. Only outline segments that are not the image border count (a block cut by the border still has its seen
    sides). Length-weighted mean on the 90-deg circle. 2026-10-18 R1: the head-depth yaw of the same unmoved block read 4..29 deg
    over four observations; this angle gave the same block yaw within +-3 deg in all of them."""
    import cv2
    cnt, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnt:
        return None, 0.0
    poly = cv2.approxPolyDP(max(cnt, key=cv2.contourArea), 3.0, True).reshape(-1, 2).astype(float); H, W = mask.shape
    on_b = lambda r: r[0] <= border or r[1] <= border or r[0] >= W - 1 - border or r[1] >= H - 1 - border
    angs, lens = [], []
    for i in range(len(poly)):
        p_, q_ = poly[i], poly[(i + 1) % len(poly)]
        if on_b(p_) and on_b(q_) and (abs(p_[0] - q_[0]) < 2 or abs(p_[1] - q_[1]) < 2):
            continue
        L = float(np.hypot(*(q_ - p_)))
        if L < min_seg:
            continue
        a = np.degrees(np.arctan2(q_[1] - p_[1], q_[0] - p_[0])) % 90.0; angs.append(a if a <= 45 else a - 90); lens.append(L)
    if not lens:
        return None, 0.0
    th = np.radians(np.array(angs) * 4); w = np.array(lens)
    return float(np.degrees(np.arctan2((w * np.sin(th)).sum(), (w * np.cos(th)).sum())) / 4), float(w.sum())


def wrist_object(bgr, object_width_m=2 * BLOCK_HALF_M):
    """The block in the wrist image: left / right edge on its widest row, centre, width, slope of its far top edge."""
    import cv2
    a, lab, st = _wrist_red_components(bgr); n = len(st)
    if n < 2 or st[1:, cv2.CC_STAT_AREA].max() < 1500:
        return dict(visible=False)
    m = lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])); rows = np.flatnonzero(m.sum(axis=1) >= 10)
    w = {int(y): (int(np.flatnonzero(m[y])[0]), int(np.flatnonzero(m[y])[-1])) for y in rows}
    y = max(w, key=lambda k: w[k][1] - w[k][0]); xl, xr = w[y]
    cols = [x for x in range(m.shape[1]) if m[:, x].sum() >= 10]; mid = cols[len(cols) // 5: len(cols) - len(cols) // 5]
    slope = float(np.degrees(np.arctan(np.polyfit(mid, [int(np.flatnonzero(m[:, x])[0]) for x in mid], 1)[0]))) if len(mid) >= 10 else None
    ea, el = block_edge_angle(m)
    return dict(visible=True, edge_angle_deg=ea, edge_angle_len_px=el, edges_px=[xl, xr], row=y, centre_px=(xl + xr) / 2, width_px=xr - xl, mm_per_px=object_width_m * 1000 / max(xr - xl, 1), far_edge_slope_deg=slope, touches_border=bool(xl <= 1 or xr >= m.shape[1] - 2), border_side=("both" if (xl <= 1 and xr >= m.shape[1] - 2) else "left" if xl <= 1 else "right" if xr >= m.shape[1] - 2 else None))


def _wrist_surface_candidate(bgr, labels, label):
    """Current pixels of one unresolved red component."""
    import cv2
    mask = labels == int(label); height, width = mask.shape
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
    clipped = ((contour[:, 0] == 0) | (contour[:, 0] == width - 1) |
               (contour[:, 1] == 0) | (contour[:, 1] == height - 1))
    visible_runs = _contour_runs(contour, ~clipped); clipped_runs = _contour_runs(contour, clipped)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    interior = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8))
    rgb_edge_mask = (cv2.Canny(gray, 50, 150) != 0) & (interior != 0)
    edge_contours, _ = cv2.findContours(rgb_edge_mask.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    edges = [dict(classification="RGB_EDGE_CANDIDATE", points_px=c.reshape(-1, 2).tolist())
             for c in edge_contours if len(c) >= 10]
    return dict(classification="OBSERVED_RED_COMPONENT_CANDIDATE", component_label=int(label),
                component_area_px=int(mask.sum()), contour_points_px=contour.tolist(),
                visible_edges=[dict(classification="VISIBLE_EXTERIOR_EDGE", points_px=run) for run in visible_runs],
                clipped_boundary=[dict(classification="FRAME_CLIPPED_CONTOUR", points_px=run) for run in clipped_runs],
                interior_edge_candidates=edges, frame_clipped=bool(clipped_runs)), mask, rgb_edge_mask


def wrist_object_surfaces(bgr):
    """Observed 2-D red contour and RGB edges; no metric face or gripper-pad identity is inferred."""
    import cv2
    red, labels, stats = _wrist_red_components(bgr)
    areas = stats[1:, cv2.CC_STAT_AREA] if len(stats) > 1 else np.empty(0, int)
    significant = np.flatnonzero(areas >= 1500) + 1
    components = [dict(label=int(i), area_px=int(stats[i, cv2.CC_STAT_AREA]),
                       bbox_xywh=[int(v) for v in stats[i, :4]]) for i in significant]
    common = dict(source="WRIST_RGB", colour_role="EXISTING_FIXED_DEVELOPMENT_BLOCK_IDENTIFICATION_PRIOR",
                  edge_role="OBSERVED_2D_RED_REGION_BOUNDARY; physical edge/occlusion distinction is UNOBSERVED",
                  component_criterion="Lab a >= 140, 5x5 open, area >= 1500 px (the existing wrist_object development-block prior)",
                  rgb_edge_candidate_criterion="grayscale Canny 50/150 inside a 5x5-eroded red mask; contour length >= 10 px; 2D candidate extraction only, never a physical or 3D gate",
                  significant_components=components, metric_centre="UNOBSERVED", metric_normal="UNOBSERVED",
                  metric_yaw="UNOBSERVED", pad_identity="UNOBSERVED", surface_3d="UNOBSERVED")
    extracted = [_wrist_surface_candidate(bgr, labels, label) for label in significant]
    candidates = [item[0] for item in extracted]; candidate_masks = [item[1] for item in extracted]
    if len(significant) != 1:
        status = "AMBIGUOUS" if len(significant) > 1 else "MISSING"
        return dict(common, status=status, visible=False,
                    missing_reason="multiple significant red components" if len(significant) > 1 else "no significant red component",
                    candidates=candidates, contour_points_px=[], visible_edges=[], clipped_boundary=[],
                    interior_edge_candidates=[], frame_clipped=False, _candidate_masks=candidate_masks,
                    _mask=np.zeros_like(red, bool), _rgb_edge_mask=np.zeros_like(red, bool))

    candidate, mask, rgb_edge_mask = extracted[0]
    return dict(common, status="OK", visible=True, candidates=candidates, **candidate,
                _candidate_masks=candidate_masks, _mask=mask, _rgb_edge_mask=rgb_edge_mask)


def jaw_reference(bgr):
    """Inner edges of the jaw tips on the bottom row; only against a plain bright background (the folded pose)."""
    import cv2
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(int); H, W = g.shape; d = g[H - 2] < 90
    l = int(np.argmax(d[:8])) if d[:8].any() else 0                           # the tip starts within a few pixels of the image border (the last column or two can be a bright sensor edge)
    while l < W // 2 and d[l]: l += 1
    r = W - 1 - int(np.argmax(d[::-1][:8])) if d[-8:].any() else W - 1
    while r > W // 2 and d[r]: r -= 1
    ok = 0 < l < W // 2 and W // 2 < r < W - 1 and int(np.median(g[H - 40:, l + 10:r - 10])) > 150
    return dict(readable=bool(ok), inner_px=[l - 1, r + 1], midline_px=(l - 1 + r + 1) / 2, gap_px=r - l + 2)


def wrist_offset(obj, ref, tips=None, gripper_tick=None):
    """Where the block sits against the jaw midline along the jaw axis (image x). The midline comes from the jaw tips read
    against a plain background at the start pose; it is USED for a frame only when a tip read in that same frame agrees
    with it (the camera is fixed to the gripper, so at one opening the tips do not move in the image - this checks it)."""
    if obj.get("visible") and obj.get("stable", True) and obj.get("touches_border") and obj.get("border_side") in ("left", "right") and ref and ref.get("readable"):
        # 2026-10-18 R1: 2.5 cm off at the pre-grasp height put the block across the left border and the loop gave up. The far
        # edge is still seen: the block's centre lies at least half the VISIBLE width inside it, so the offset is a LOWER BOUND
        # in that direction. It is used for one capped correction and a new look; it can never make the gate pass (judge).
        cap = WRIST.get("border_step_cap_mm", 15.0)
        off = obj["centre_px"] - ref["midline_px"]; mm = float(np.clip(off * obj["mm_per_px"], -cap, cap))
        if (obj["border_side"] == "left" and off >= 0) or (obj["border_side"] == "right" and off <= 0):
            return dict(readable=False, why="the block is cut by the border on the side of the jaw midline: no direction can be read")
        return dict(readable=True, border_bound=True, border_side=obj["border_side"], offset_px=float(off), offset_mm=mm, uncertainty_mm=float(WRIST["measure_px"] * obj["mm_per_px"]),
                    reference_confirmed_in_this_frame=True, confirmed_by=["border step: direction from the far edge; no midline precision is needed for a capped step"], tips_in_this_frame=tips or {}, tips_hidden=None,
                    note=f"LOWER BOUND: the block is cut by the {obj['border_side']} border; the step is capped at {cap:.0f} mm and followed by a new look")
    if not obj.get("visible") or not obj.get("stable", True) or obj.get("touches_border") or not ref or not ref.get("readable"):
        return dict(readable=False, why="the block is not visible / not stable over consecutive frames / cut by the image border, or there is no jaw reference")
    tips = tips or {}; m = WRIST["measure_px"]
    agree = [k for k, i in (("left", 0), ("right", 1)) if tips.get(f"{k}_px") is not None and abs(tips[f"{k}_px"] - ref["inner_px"][i]) <= m]
    hidden = tips.get("left_px") is None and tips.get("right_px") is None
    # the tips move in the image only when the jaws move (the camera is fixed to the gripper): the same gripper reading as when
    # the reference was taken confirms it for this frame whatever the background is. Run 8 of 2026-10-08: over the printed board
    # the dark squares next to the tips moved the image reading by 7 .. 12 px although nothing had moved.
    same_opening = gripper_tick is not None and ref.get("gripper_tick") is not None and abs(gripper_tick - ref["gripper_tick"]) <= 2
    if same_opening:
        agree = agree + ["gripper reading unchanged since the reference (%d vs %d ticks)" % (gripper_tick, ref["gripper_tick"])]
    off = obj["centre_px"] - ref["midline_px"]
    return dict(readable=True, offset_px=float(off), offset_mm=float(off * obj["mm_per_px"]), uncertainty_mm=float(m * obj["mm_per_px"]), reference_confirmed_in_this_frame=bool(agree), confirmed_by=agree,
                tips_in_this_frame=tips, tips_hidden=bool(hidden))


def image_response_prior(obj):
    """px of block motion in the image per mm of tool motion along +jaw axis, before this session has measured it: the image
    scale at the block (pure translation) times the share seen in PICK01 (0.63 .. 1.0 of it; the wrist also turns when the
    arm moves). Positive: the block moves image-RIGHT for a tool move along +jaw (PICK01 record)."""
    return 0.8 / obj["mm_per_px"]


def overlay(scene, bgr, path):
    """Top image with the model's arm geometry (dots), tool points (crosses) and the object centre (circle) projected in."""
    import cv2, mujoco
    m = model(); d = mujoco.MjData(m); sweepick_kin.set_q12(m, d, np.array(scene["q12"])); mujoco.mj_forward(m, d)
    Tcw = np.linalg.inv(np.array(scene["T_world_camera"])); K = np.array(scene["K"]).reshape(3, 3); vis = bgr.copy()
    def px(p):
        c = Tcw @ np.r_[p, 1]; return int(K[0, 0] * c[0] / c[2] + K[0, 2]), int(K[1, 1] * c[1] / c[2] + K[1, 2])
    for g, name in robot_boxes(m, d):
        cv2.circle(vis, px(d.geom_xpos[g]), 4, (0, 255, 0) if name.startswith("left") else (0, 128, 255), -1)
    for s, col in (("left", (0, 0, 255)), ("right", (255, 0, 0))):
        cv2.drawMarker(vis, px(d.site_xpos[m.site(f"{s}_cube_grasp").id]), col, cv2.MARKER_CROSS, 30, 3)
    if scene.get("object"):
        cv2.circle(vis, px(np.array(scene["object"]["centre_world"]) + [0, 0, 0.02]), 14, (0, 255, 255), 2)
    cv2.imwrite(str(path), cv2.resize(vis, (960, 690)))


def look(out, tag):
    out = Path(out); d = out / "looks" / tag; d.mkdir(parents=True, exist_ok=True); m_start = time.monotonic()
    # 203018 timing: each look waited for 30 head FrameSets (~2-3 s). The block is found from the head at observe and checked at the end (05_home);
    # the looks in between only record the scene, so they take 8.
    top = top_frame(d / f"sdk_{int(time.time())}", depth_frames=30 if tag in ("00_observe", "05_home") else LOOK_DEPTH_FRAMES); t_top = time.time(); m_top = time.monotonic()        # a fresh folder: a stop file left by an earlier reader would end this one at once
    arms = read_ticks(); t_q1 = time.time(); m_q1 = time.monotonic()                                    # joints read AFTER the top frames were taken (the reader needs about 13 s to start), not before
    ticks = {s: arms[s]["ticks"] for s in arms}
    scene = analyse(top["bgr"], top["depth_mm"], top["K"], ticks, lite=tag.startswith("01")); scene["_out"] = str(out); scene.update(K=list(top["K"]), tag=tag, arms=arms, stamps=top["stamps"], depth=dict(frames=top["depth_frames"], valid_share=top["valid_share"]), wrist=wrist_frame(d / f"{SIDE}_wrist.jpg"), wrist_side=SIDE)
    m_wrist = time.monotonic()                                                 # wrist frames taken and both views analysed
    arms2 = read_ticks(); t_q2 = time.time(); m_q2 = time.monotonic()                                   # ... and again after the wrist frames: the arm must not have moved in between
    moved = {f"{s_} {n}": arms2[s_]["ticks"][n] - arms[s_]["ticks"][n] for s_ in arms for n in arms[s_]["ticks"] if abs(arms2[s_]["ticks"][n] - arms[s_]["ticks"][n]) > 2}
    scene["joint_reads"] = dict(top_frames_done_host_s=t_top, joints_host_s=t_q1, joints_again_host_s=t_q2, seconds_top_to_joints=t_q1 - t_top, seconds_between_reads=t_q2 - t_q1, moved_more_than_2_ticks=moved, static=not moved,
                                note="the port is opened read-only for each read; nothing reads the joints while the cameras are being read, so this is two snapshots around the images, not a continuous watch")
    if moved:
        scene["wrist"]["object"] = dict(scene["wrist"].get("object") or {}, stable=False, why_unstable=f"the joints changed between the two reads around the images: {moved}")
    st = state_plan(out)
    if st and st.get("plan") and scene["ok"]:
        try:
            scene["support_region"] = support_region(out, st, scene, top)
        except Exception as e:
            scene["support_region"] = dict(state="UNKNOWN (area observation failed)", error=f"{type(e).__name__}: {e}")
    if st:
        scene["tool_minus_plan"] = tool_minus_plan(scene, st)
        if scene["wrist"].get("object"):
            scene["wrist"]["offset"] = wrist_offset(scene["wrist"]["object"], st.get("jaw_reference"), scene["wrist"].get("tips"), gripper_tick=arms2[SIDE]["ticks"].get("gripper"))
    import cv2
    cv2.imwrite(str(d / "top.png"), top["bgr"]); np.save(d / "depth_mm.npy", top["depth_mm"])
    if scene["ok"]:
        overlay(scene, top["bgr"], d / "overlay.png")
    scene["timing_monotonic_s"] = dict(look_start=m_start, top_frames_done=m_top, joints_read=m_q1, wrist_frames_and_analysis_done=m_wrist, joints_read_again=m_q2,
                                       look_done=time.monotonic(), clock="time.monotonic (CLOCK_MONOTONIC, the clock of the recorder's recv_monotonic_s)", role="recording only")
    (d / "SCENE.json").write_text(json.dumps(scene, indent=1, default=str)); return scene


def state_plan(out):
    f = Path(out) / "STATE.json"
    return json.loads(f.read_text()) if f.exists() else None


def tool_minus_plan(scene, state):
    """Where the tool point of the MEASURED joints is, relative to the object centre the plan was made for (model frame)."""
    try:
        o = json.loads((Path(state["scene"]) if Path(state["scene"]).is_absolute() else Path(scene["_out"]) / state["scene"]).read_text())["object"]["centre_world"]
    except Exception:
        return None
    if not scene.get(f"tool_{SIDE}"):
        return None                                                           # no frame chain in this look (board not found): unknown, not zero
    return [float(a - b) for a, b in zip(scene[f"tool_{SIDE}"], o)]


def support_region(out, state, scene, top, radius_m=0.03):
    """The place where the block stood, by the EXISTING area observation (tjj_perception.observe_area through
    sweepick_local_loop.real_area): OCCUPIED / CLEAR / UNKNOWN (not fully observed). A pixel without depth is 'not observed'."""
    import mujoco
    from sweepick_local_loop import real_area
    c = state_object(out, state)[0]; m = model()
    if MOUNTED:                                                                 # real_area works in diag(1,-1,-1) @ inv(T_camera_from_board); hand it the robot frame (floor z = 0) the same way
        floor = (scene.get("desk_z") or {}).get("floor") or {}
        if not floor.get("ok"):
            return dict(state="UNKNOWN (not fully observed)", valid=False, clear=False, reason="no floor plane in the observation", centre_robot_xy=[float(c[0]), float(c[1])], radius_m=radius_m)
        fz = floor_z(floor, float(c[0]), float(c[1])); Tz = np.eye(4); Tz[2, 3] = -fz       # the floor under the place becomes z 0
        F = np.diag([1.0, -1.0, -1.0, 1.0]); T_cb = np.linalg.inv(F @ Tz @ np.array(scene["T_world_camera"]))
        r = real_area(np.asarray(top["depth_mm"], float) * DEPTH_SCALE_MOUNTED, top["K"], T_cb, 0.0, (float(c[0]), float(c[1])), radius_m)
        return dict(r, centre_robot_xy=[float(c[0]), float(c[1])], floor_z_m=fz, radius_m=radius_m, note="mounted: the robot frame lifted so that the observed floor under the place is z 0 (depth scaled 1/1.016); the arm above the place also counts as 'occupied'; used only as CLEAR = validly observed and empty")
    Tb = np.eye(4); Tb[:3, 3] = DATUM_IN_MODEL_BASE; Tw = np.eye(4); Tw[:3, 3] = m.body("left_base").pos
    pb = np.linalg.inv(Tw @ Tb @ T_DATUM_FROM_BOARD) @ np.r_[c[0], c[1], BOARD_Z_IN_WORLD, 1.0]                       # into the printed board's frame
    r = real_area(top["depth_mm"], top["K"], scene["board"]["T_camera_from_board"], 0.0, (float(pb[0]), float(-pb[1])), radius_m)
    return dict(r, centre_board_xy=[float(pb[0]), float(-pb[1])], radius_m=radius_m, note="the arm above the place also counts as 'occupied' here; the result is used only as CLEAR = validly observed and empty")


def observe(out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    scene = look(out, "00_observe"); state = dict(schema="tjj.pick01.v1", scene="looks/00_observe/SCENE.json", stages={}, home_ticks=None, plan=None)
    why = None
    if not scene["ok"]:
        why = scene["why"]
    elif any(v for s in scene["arms"].values() for v in s["torque"].values()):
        why = "torque is already on somewhere: another controller may own an arm"
    elif scene["object"] is None:
        why = "no current supported-block depth observation was found " + (f"on the floor (no {OBJECT['size_m'] * 1000:.0f} mm {OBJECT['colour']} block standing on the fitted floor)" if MOUNTED else "on the desk")
    else:
        if MOUNTED:                                                            # the block may be within either arm's reach: plan both, choose by the same cost
            sel = select_arm(scene); plan = sel["plans"][sel["selected"]] if sel["selected"] else sel["plans"]["left"]; state["arm_selection"] = {k: v for k, v in sel.items() if k != "plans"}
        else:
            plan = plan_pick(scene)
        state["plan"] = plan
        if plan.get("ok") and abs(float(plan.get("object_observed_yaw_rad", 0.0)) - float(scene["object"].get("yaw_rad") or 0.0)) > 1e-9:
            state.setdefault("object_updates", []).append(dict(centre_world=[float(v) for v in scene["object"]["centre_world"]], yaw_rad=float(plan["object_observed_yaw_rad"]),
                                                              previous_yaw_rad=float(scene["object"].get("yaw_rad") or 0.0), source="head top-face rectangle folded to the taught grasp yaw of this arm (or the taught yaw alone)", kind="BLOCK YAW PRIOR (HEAD RECTANGLE + TAUGHT GRASP)", uncertainty_deg=45.0))
        chosen = state["arm_selection"]["selected"] if MOUNTED else "left"
        if MOUNTED and chosen in executable_arms() and chosen != SIDE:         # the selected arm becomes the session arm; its own wrist gives the jaw reference
            set_side(chosen)
            scene["wrist"] = wrist_frame(out / "looks/00_observe" / f"{SIDE}_wrist.jpg"); scene["wrist_side"] = SIDE
            (out / "looks/00_observe/SCENE.json").write_text(json.dumps(scene, indent=1, default=str))
        if MOUNTED and state["arm_selection"]["selected"] is None:
            why = f"neither arm has a pick path to the block at {[round(v, 3) for v in scene['object']['centre_world'][:2]]}: left {state['arm_selection']['plans']['left']['why'][:1]}, right {state['arm_selection']['plans']['right']['why'][:1]}"
        elif MOUNTED and state["arm_selection"]["selected"] not in executable_arms():
            why = f"RIGHT_ARM_SELECTED_NOT_MEASURED: the right arm was selected ({state['arm_selection']['reason']}); the right-arm pick is planned but not executed: {RIGHT_NOT_WIRED}"
        elif not plan["ok"]:
            why = f"the planner found no pick path: {plan['why']}"

        elif plan["model_jaw_gap_now_m"] <= plan["object_width_m"]:
            why = f"the jaws are not open wide enough in the model ({plan['model_jaw_gap_now_m'] * 1000:.1f} mm) for the object"
        elif not jaw_ok(scene):
            why = "the jaw tips cannot be read in the wrist image of the start pose (needs a plain bright background): no reference for the alignment"
        elif plan["room_each_side_m"] * 1000 <= WRIST["measure_px"] * 0.27:
            why = f"the grasp the arm can make here leaves {plan['room_each_side_m'] * 1000:.1f} mm each side (jaw axis leaning {plan['jaw_lean_deg']:.1f} deg), not more than the wrist image can resolve: the block has to stand differently for this arm"
    state.update(object_geometry=(scene.get("object") or {}).get("geometry"), visible_patch_class_match=(scene.get("object") or {}).get("class_match"),
                 jaw_reference=dict((scene.get("wrist") or {}).get("jaw_reference") or {}, gripper_tick=(scene.get("ticks") or {}).get(SIDE, {}).get("gripper")) if scene["ok"] else None, wrist_record=WRIST, side=SIDE, run_name="PICK02", control="SENSOR_PLANNED / SUPERVISED_AUTOMATED (no ACT)", evidence=[],
                 sources={f: hashlib.sha256((HERE / f).read_bytes()).hexdigest() for f in ("sweepick_pick01.py", "sweepick_move_a.py", "sweepick_progress.py", "sweepick_grasp.py", "sweepick_progress_profile.json", "sweepick_jaw_profile_left.json", "sweepick_motion_profile.json")})
    home = scene.get("ticks", {}).get(SIDE) if scene["ok"] else None
    home_command, home_basis = home_command_ticks(home, mapping(), SIDE) if home is not None else (None, None)
    state.update(ready=why is None, why=why, home_ticks=home, home_command_ticks=home_command, home_command_basis=home_basis,
                 right_ticks=scene.get("ticks", {}).get("right") if scene["ok"] else None, other_ticks=scene.get("ticks", {}).get(other(SIDE)) if scene["ok"] else None)
    (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str)); return state


# ---- one stage = one call of the existing executor -------------------------------------------------------------------
_MODEL_RANGE_TICKS = {}


def model_range_ticks(side, joint, mp):
    """The servo ticks of this joint whose model angle lies inside the dual model's joint range (through the candidate mapping)."""
    key = (side, joint)
    if key not in _MODEL_RANGE_TICKS:
        m = mp[side][joint]; t_ = np.arange(float(m["lo"]), float(m["hi"]) + 1.0); q_ = np.array([rc.tick_to_model(joint, m, x) for x in t_])
        ok_ = t_[(q_ >= m["model_range"][0]) & (q_ <= m["model_range"][1])]
        _MODEL_RANGE_TICKS[key] = (float(ok_.min()), float(ok_.max()))
    return _MODEL_RANGE_TICKS[key]


def stage_request(state, name, mp, on_now=None):
    """mode / target / powered joints of a stage. The gripper's close target is the closed end of its calibrated range:
    the end of the commanded path, not an opening for this object; the executor stops when the gripper stops advancing."""
    tk, home = state["plan"]["ticks"], state["home_ticks"]
    home_command, home_basis = home_command_ticks(home, mp, state.get("side", "left"))
    if name == "pregrasp":
        return dict(mode="out", target=tk["pregrasp"], torque_joints=ARM, scene="block")
    if name == "align":
        return dict(mode="next", target=tk[state.get("height") or "pregrasp"], torque_joints=ARM, scene="block")
    if name == "descend_a":
        return dict(mode="next", target=tk["descend_a"], torque_joints=ARM, scene="block")
    if name == "descend_b":
        return dict(mode="next", target=tk["descend_b"], torque_joints=ARM, scene="block")
    if name in ("descend", "descend_settle"):
        return dict(mode="next", target=tk["descend"], torque_joints=ARM, scene="block")
    if name == "close":
        return dict(mode="grip", torque_joints=ALL, scene="object: block on the desk, left pad - block contact allowed")
    if name == "lift":
        return dict(mode="next", target=tk["lift"], torque_joints=ALL, scene="object: block carried by the left hand, left pad - block and block - desk contact allowed")
    if name == "carry":                                                       # 2026-10-18 (user): the held block is folded over the base for the drive (the taught carry pose of this arm)
        f_ = TAUGHT_REFS.get(state.get("side", "left"), Path("/nonexistent")) / "H9_carry" / "ticks.json"
        side_ = state.get("side", "left"); ct = json.loads(f_.read_text())[side_]["ticks"]
        # 211928_auto (right): the taught carry wrist_flex 969 lies 23 ticks (2 deg) outside the model range (992..): REFUSED with the block in the hand.
        # The carry target is the taught pose with each joint brought inside the model range (+5 ticks); the change is recorded.
        tgt_ = {n: float(ct[n]) for n in ARM}; clamped_ = {}
        for n in ARM:
            lo_t, hi_t = model_range_ticks(side_, n, mp)
            v_ = min(max(tgt_[n], lo_t + 5.0), hi_t - 5.0)
            if v_ != tgt_[n]:
                clamped_[n] = dict(taught=tgt_[n], commanded=v_); tgt_[n] = v_
        return dict(mode="next", target=tgt_, torque_joints=ALL, scene="object: block carried by the left hand, left pad - block and block - desk contact allowed", carry_clamped_into_model_range=clamped_ or None)
    if name == "carry_back":
        return dict(mode="next", target=tk["lift"], torque_joints=ALL, scene="object: block carried by the left hand, left pad - block and block - desk contact allowed")
    if name == "lower":
        return dict(mode="next", target=tk["descend"], torque_joints=ALL, scene="object: block carried by the left hand, left pad - block and block - desk contact allowed")
    if name == "open":
        return dict(mode="next", target=dict(gripper=home_command["gripper"]), torque_joints=ALL, scene="object: block on the desk, left pad - block contact allowed", home_command=home_basis)
    if name == "retreat":
        return dict(mode="next", target=tk["pregrasp"], torque_joints=ALL, scene="block")
    if name == "retreat_arm":
        return dict(mode="next", target=tk["pregrasp"], torque_joints=ARM, scene="block")
    if name == "home_arm":
        return dict(mode="return", rest={n: home_command[n] for n in ARM}, torque_joints=ARM, scene="block", home_command=home_basis)
    if name == "abort_home":                                                  # from wherever a stage ended holding: back to the start pose with the joints that are on, then release
        on = [n for n in ALL if on_now[n]]
        return dict(mode="return", rest={n: home_command[n] for n in on}, torque_joints=on, scene="block", home_command=home_basis)
    return dict(mode="return", rest={n: home_command[n] for n in ALL}, torque_joints=ALL, scene="block", home_command=home_basis)


def certify(traj_file, out_file, scene_file):
    command = [sys.executable, str(HERE / "sweepick_collision_cert.py"), str(traj_file), str(CAND), str(CAL), str(ASM), str(out_file), str(scene_file)]
    deferred = EPISODE is not None
    r = subprocess.run(command + (["--nominal-only"] if deferred else []), capture_output=True, text=True, env=dict(__import__("os").environ, MUJOCO_GL="egl"))
    if not Path(out_file).exists():
        raise RuntimeError(f"collision check did not run: {r.stderr[-400:]}")
    result = json.loads(Path(out_file).read_text())
    if deferred and "tube_diagnostic" in result:
        diagnostic = Path(out_file).with_suffix(f".DIAGNOSTIC{len(DEFERRED_COLLISION_DIAGNOSTICS) + 1}.json")
        snapshot = diagnostic.with_suffix(".TRAJECTORY.json")
        snapshot.write_bytes(Path(traj_file).read_bytes())
        command[2], command[6] = str(snapshot), str(diagnostic)
        result["tube_diagnostic"]["result_file"] = str(diagnostic)
        Path(out_file).write_text(json.dumps(result, indent=1))
        DEFERRED_COLLISION_DIAGNOSTICS.append(command)
    return result


def finish_collision_diagnostics():
    # Uncalibrated offset diagnostics do not delay motion; every original evaluation runs after device owners close.
    commands = list(DEFERRED_COLLISION_DIAGNOSTICS); DEFERRED_COLLISION_DIAGNOSTICS.clear()
    for command in commands:
        r = subprocess.run(command, capture_output=True, text=True, env=dict(__import__("os").environ, MUJOCO_GL="egl"))
        if r.returncode != 0 or not Path(command[6]).exists():
            RECORDING_ERRORS.append(dict(kind="collision_diagnostic", file=command[6], error=r.stderr[-400:]))


def defer_collision_diagnostics(out):
    """Console-17: a limited session ends holding the object, so the post-session tube diagnostics are not run inline.
    Their exact commands and trajectory snapshots are kept for offline review; the nominal checks already gated every stage."""
    commands = list(DEFERRED_COLLISION_DIAGNOSTICS); DEFERRED_COLLISION_DIAGNOSTICS.clear()
    if commands:
        (Path(out) / "COLLISION_DIAGNOSTICS_PENDING.json").write_text(json.dumps(dict(
            schema="sweepick.collision-diagnostics-pending.v1", count=len(commands), commands=commands, env=dict(MUJOCO_GL="egl"),
            role="post-session diagnostic only; not a gate. Run offline with the same argv; each writes its DIAGNOSTIC*.json"), indent=1))
    return len(commands)


def object_check(doc, block, *, carried, right_ticks, grasp_arm_ticks=None, all_contacts=False):
    """Collision result of a commanded path WITH the object in the scene, by the SIM teacher's own check: robot self
    contact, robot - desk and any robot part on the block are contacts; only left pad - block and block - desk are allowed.
    carried: the block moves rigidly with the left tool point from where it was at `grasp_arm_ticks`."""
    import mujoco, hashlib
    import sweepick_trajectory as tt
    from sim_data_factory.teacher import HandoverTeacher
    mp = mapping(); m = model(block); t = HandoverTeacher(m); d = mujoco.MjData(m); traj = doc["trajectory"]
    adr = t.cm.block_qpos; block_q = m.qpos0[adr:adr + 7].copy()
    def ctrl(arm):                                                            # `right_ticks` = the OTHER arm's ticks (name kept)
        q = rc.ticks_to_q12(mp, {SIDE: arm, other(SIDE): right_ticks}); sweepick_kin.set_q12(m, d, q); return d.qpos.copy(), d.qpos[t.plan.adr].copy()
    rel = None
    if carried:
        base, c = ctrl(grasp_arm_ticks); t.plan.pose(base, c, block_q); ps, rot = t.plan.site_frame(SIDE)
        rb = np.empty(9); mujoco.mju_quat2Mat(rb, block_q[3:]); rel = np.eye(4); rel[:3, :3] = rot.T @ rb.reshape(3, 3); rel[:3, 3] = rot.T @ (block_q[:3] - ps)
    first = None; steps = traj["steps"]; contact_steps = []; contact_pairs = []
    entry = None
    if traj.get("command_start") is not None:
        entry = tt.make(traj["start"], {n: v for n, v in traj["command_start"].items() if v != traj["start"][n]}, doc.get("profile") or tt.load_profile())
    paths = ([dict(steps=1, goals={n: [traj["start"][n]] for n in rc.JOINTS}), entry] if entry else []) + [traj]
    checked_steps = 0
    for left, phase in (({n: float(path["goals"][n][k]) for n in rc.JOINTS}, "range_entry" if path is not traj else "commanded") for path in paths for k in range(path["steps"])):
        base, c = ctrl(left)
        bq = t.plan.carried_block(base, c, (SIDE, rel)) if carried else block_q
        bad = t.plan.collisions(base, c, bq, allow_pad_block=(SIDE,), clearance=0.0)
        if bad:
            if first is None:
                first = dict(step=checked_steps, phase=phase, pairs=[list(map(str, b)) for b in bad[:4]])
            if not all_contacts:
                break
            contact_steps.append(checked_steps); contact_pairs.append(sorted({tuple(sorted(map(str, b_[:2]))) for b_ in bad}))   # every pair of every contact sample (review 2026-10-19)
        checked_steps += 1
    msha = hashlib.sha256(CAND.read_bytes()).hexdigest()
    return dict(schema="tjj.object-collision-result.v1", identity=tt.identity(doc["side"], traj, msha), nominal_verdict="CLEAR" if first is None else "CONTACT", contact_steps=contact_steps if all_contacts else None, contact_pairs=[[list(p_) for p_ in ps_] for ps_ in contact_pairs] if all_contacts else None, first_nominal_contact=first, nominal_steps_checked=checked_steps,
                range_entry_checked=entry is not None and first is None, range_entry_identity=tt.identity(doc["side"], traj, msha) if entry is not None else None,
                range_entry_actual_start=traj["start"] if entry is not None else None, range_entry_command_seed=traj.get("command_start"), range_entry_samples=0 if entry is None else entry["steps"]+1,
                scene="assembled dual model, desk at the real height, block " + (f"carried by the {SIDE} tool point" if carried else "on the desk"), allowed=[f"{SIDE} pad - block", "block - desk"], not_allowed="every other pair, housings and arm links on the block included",
                check="sim_data_factory.teacher.Planner.collisions (unchanged)", mapping_status="CANDIDATE (verified=false)")


def sweep_doc(arm_ticks, grip_from, grip_to):
    """The whole possible closing sweep of the gripper at one arm pose, as a path the object check can read."""
    n = int(abs(grip_from - grip_to)) + 1
    goals = {j: [float(arm_ticks[j])] * n for j in ARM}; goals["gripper"] = [float(v) for v in np.linspace(grip_from, grip_to, n)]
    return dict(side=SIDE, trajectory=dict(start=dict(arm_ticks, gripper=grip_from), target=dict(gripper=grip_to), moving=["gripper"], dt=0.02, steps=n, goals=goals))


TAUGHT_REFS = {"right": Path.home() / "sweepick_261018_right_r0/refs", "left": Path.home() / "sweepick_261018_right_r0/refs_left"}
TAUGHT_PREGRASP_CAMERA_DISTANCE_M = 0.19                                       # camera to block at the taught pre-grasp view: from the two taught views (area 12654 -> 15743 px for about 2 cm lower), PROVISIONAL
HEIGHT_TOL_RATIO = 0.15                                                        # image size within +-15 % of the taught pre-grasp view (2026-10-18 175957: +-8 % kept firing as the wrist turned)
ALIGN_MAX_LOOKS = 8                                                            # looks per height before the loop gives up (planned way back)
ACROSS_TOL_MM = 5.0                                                            # across the jaw axis the pads (about 20 mm wide) still take the block this far off its centre


def red_blob(img):
    """Centre and area (px, 320x240) of the largest red region of a wrist image."""
    import cv2
    if img is None:
        return None
    if img.shape[1] != 320:
        img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
    _, lab, st = _wrist_red_components(img)
    if len(st) < 2:
        return None
    k = 1 + int(np.argmax(st[1:, 4])); ys, xs = np.nonzero(lab == k); ang, alen = block_edge_angle(lab == k); coh = edge_coherence(lab == k)
    return dict(cx=float(xs.mean()), cy=float(ys.mean()), area=int(len(xs)), top=int(ys.min()), bottom=int(ys.max()), left=int(xs.min()), right=int(xs.max()), angle=ang, angle_len=alen, angle_coherence=coh)


def edge_coherence(mask, border=6):
    """How square the seen outline is, 0..1: |sum w e^(4i theta)| / sum w over the mask's edge gradients, image border excluded.
    A clean rectangle gives a high value; the outline of a block cut by the image border (diagonal cut corners) a low one.
    2026-10-18: 175957 look 0 (a false 31 deg on a cut block) 0.40; 182311 look 0 (a real ~14 deg, cut at top and right) 0.50;
    the taught views 0.54..0.59. A gate for using the outline angle of a CUT block, not a measurement."""
    import cv2
    m = cv2.GaussianBlur(mask.astype(np.float32), (7, 7), 0); gx = cv2.Sobel(m, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(m, cv2.CV_32F, 0, 1, ksize=3); w = np.hypot(gx, gy)
    w[:border] = 0; w[-border:] = 0; w[:, :border] = 0; w[:, -border:] = 0
    return float(abs((w * np.exp(4j * np.arctan2(gy, gx))).sum()) / max(float(w.sum()), 1e-9))


EDGE_COHERENCE_MIN = 0.45
# image-angle change per wrist-yaw change (deg/deg), the sign of the image-Jacobian's yaw entry before a session measures it.
# right: MEASURED 2026-10-18 183317_auto look 0->1, the wrist turned -9.9 deg (model FK of read joints) and the block's outline angle
# went -14.2 -> -24.5 deg (+1.05; the -1 prior turned the wrist the wrong way). left: +1, the sign of the earlier left loop
# (visual_align: block yaw := yaw - edge angle, 'image angle grew with the jaw axis', four sessions); not measured in this loop yet.
IMAGE_ANGLE_PER_YAW = {"right": 1.05, "left": 1.0}
SIDEWAYS_BEFORE_DOWN_MM, SIDEWAYS_NEEDS_RISE_MM, RISE_BEFORE_SIDEWAYS_MM = 5.0, 8.0, 20.0   # 190907_auto: the block pushed by a sideways move near it (see visual_align_je)


def taught_view(side=None):
    """The wrist view taught by hand with the jaws centred on the block (2026-10-18 dashboard refs): at the pre-grasp height
    and just above the block. None when not taught."""
    import cv2
    s_ = side or SIDE; out = {}
    for k, d in (("pregrasp", "H6_pregrasp"), ("above", "H2_above_c")):
        f = TAUGHT_REFS[s_] / d / f"{s_}_wrist_rgb.jpg"
        im = cv2.imread(str(f)) if f.exists() else None
        if im is None or im.shape[1] != 320:                                    # only the recorder's 320x240 mode (640x480 has another field of view: sizes differ by about 1.7x)
            return None
        b = red_blob(im)
        if b is None:
            return None
        out[k] = b
    return out


def taught_row(view, area, key="cy"):
    """The image row of the block's centre (key 'cy') or of its far / near edge ('top' / 'bottom') when the jaws are centred,
    at the height its image size says: between the two taught views, linear in area (clamped)."""
    H = 240; ok_ = {k: not ((key == "top" and view[k]["top"] <= 1) or (key == "bottom" and view[k]["bottom"] >= H - 2)) for k in ("pregrasp", "above")}
    if not ok_["above"]:                                                       # that edge is cut in the taught 'above' view: the pre-grasp view alone
        return view["pregrasp"][key], 0.0
    if not ok_["pregrasp"]:
        return view["above"][key], 1.0
    a0, a1 = view["pregrasp"]["area"], view["above"]["area"]
    f = 0.0 if a1 == a0 else float(np.clip((area - a0) / (a1 - a0), 0.0, 1.0))
    return view["pregrasp"][key] + f * (view["above"][key] - view["pregrasp"][key]), f


def across_feature(blob, view):
    """Which row tells the across-jaw error: the centre when the block is whole in the image, else the edge that is NOT cut
    (2026-10-18 173148_auto: the far edge was cut at every look, the across error was never measured, the pads then landed on
    the top face). The taught view must show the same edge uncut. None when both edges are cut (too close: height first)."""
    H = 240; top_cut, bot_cut = blob["top"] <= 1, blob["bottom"] >= H - 2
    for key, usable in (("cy", not top_cut and not bot_cut), ("bottom", top_cut and not bot_cut), ("top", bot_cut and not top_cut)):
        if usable and any((view[k]["top"] > 1 if key == "top" else view[k]["bottom"] < H - 2 if key == "bottom" else True) for k in ("pregrasp", "above")):
            return key
    return None


def current_wrist_alignment(plan, scene):
    """Keep the observed-yaw target; a camera edge slope is not a world-yaw error."""
    w = scene.get("wrist") or {}; obj = w.get("object") or {}; off = w.get("offset") or {}
    slope = obj.get("far_edge_slope_deg"); target = WRIST.get("aligned_slope_deg")
    edge_on_border = obj.get("row") is not None and int(obj["row"]) <= 0          # the far edge read on the top image row is the border, not the block's edge (2026-10-18 R1 #3: slope 0.0 there)
    known = (plan.get("tilt_deg") == 0.0 and obj.get("visible") and obj.get("stable")
             and not obj.get("touches_border") and not edge_on_border and off.get("readable")
             and off.get("reference_confirmed_in_this_frame")
             and slope is not None and target is not None and np.isfinite([slope, target]).all())
    error = float(target - slope) if known else None
    axis = scene.get("jaw_axis_xy"); yaw = plan.get("object_observed_yaw_rad")
    jaw_deg = None if axis is None else float(np.degrees(np.arctan2(axis[1], axis[0])))
    target_deg = None if yaw is None else float(np.degrees(yaw) + (90.0 if plan.get("jaw") == "y" else 0.0))
    rec = dict(ok=bool(known),
               edge_slope_deg=slope, square_at_deg=target, error_deg=error,
               tolerance_deg=WRIST["slope_tolerance_deg"], classification="PROVISIONAL_TOP_OBSERVATION_YAW",
               image_error_role="DIAGNOSTIC_ONLY; not a yaw command or angular gate",
               camera_frame=f"{SIDE}_wrist; rigidly attached to the rotating gripper",
               target_jaw_axis_deg=target_deg, measured_jaw_axis_deg=jaw_deg,
               model_yaw_error_deg=None if target_deg is None or jaw_deg is None else (target_deg - jaw_deg + 90.0) % 180.0 - 90.0,
               physical_rotation_verified=False, basis="fresh initial top yaw through the candidate frame/mapping; current wrist image confirms visibility and centring")
    ea, el = obj.get("edge_angle_deg"), obj.get("edge_angle_len_px") or 0.0
    if ea is not None and el >= WRIST_EDGE_MIN_LEN_PX and obj.get("visible") and obj.get("stable"):
        # 2026-10-18 R1 #4: the jaws must be square to the block's sides IN the wrist image (5-6 cm away, read the same within
        # +-3 deg on an unmoved block); it is the direct measurement of what the grasp needs
        # 215335_auto (right, 45 deg block): down beside the block, everything else ready, the edge read -3.7 deg against 0 +-3 -> not closed. The
        # hand-taught views of this arm read -1.2 .. -1.9 deg when the person squared the jaws: that is the reference when they exist.
        tv_ = taught_view(); ref_ = float(np.median([tv_[k]["angle"] for k in tv_ if tv_[k].get("angle") is not None])) if tv_ else 0.0
        rec.update(ok=bool(abs(ea - ref_) <= WRIST["slope_tolerance_deg"]), check="WRIST_EDGE_ANGLE", wrist_edge_angle_deg=ea, wrist_edge_len_px=el,
                   edge_reference_deg=ref_, edge_reference_source="hand-taught wrist views of this arm" if tv_ else "image axes (no taught view)")
        rec.pop("unknown", None)
        return rec
    if not known:
        # 2026-10-18 R1 #3: a tilted grasp (planner chose 10 deg) or a far edge on the image border has no image slope profile.
        # The purpose stays: the jaw axis must follow the block's observed yaw. Then it is checked in the model frame: the
        # top-observed yaw against the jaw axis of the MEASURED joints, same tolerance; the block must still be seen and centred
        # with a confirmed jaw reference in this frame (the centring itself is the separate 'centred' check).
        myaw = rec["model_yaw_error_deg"]
        seen = bool(obj.get("visible") and obj.get("stable") and not obj.get("touches_border") and off.get("readable") and off.get("reference_confirmed_in_this_frame"))
        if myaw is not None and seen:
            rec.update(ok=bool(abs(myaw) <= WRIST["slope_tolerance_deg"]), check="MODEL_FRAME_YAW",
                       basis_model_frame="no image edge profile here (tilt %s deg%s): top-observed block yaw vs the jaw axis of the measured joints, within %.1f deg" % (plan.get("tilt_deg"), ", far edge on the border" if edge_on_border else "", WRIST["slope_tolerance_deg"]))
        else:
            rec["unknown"] = "current stable edge / confirmed jaw reference / vertical top-grasp profile unavailable, and no model-frame yaw with the block seen and the reference confirmed"
    return rec


def arm5_within(reached, tol_ticks):
    """review 2026-10-19: all five arm joints must be present with a finite shortfall within tol (a gripper-only or one-joint record is not
    'the arm is at the target'; all([]) would say True)."""
    try:
        return bool(reached) and all(n in reached and reached[n].get("minus_target") is not None and np.isfinite(float(reached[n]["minus_target"]))
                                     and abs(float(reached[n]["minus_target"])) <= tol_ticks for n in ARM)
    except (TypeError, ValueError, AttributeError):
        return False


def judge(out, state, what, scene):
    """Observation gates. Each returns (ok, record); the record goes into STATE['evidence'] with the numbers it used."""
    import sweepick_grasp as tg
    plan = state["plan"]; w = scene.get("wrist") or {}; obj = w.get("object") or {}; off = w.get("offset") or {}
    tol = plan["room_each_side_m"] * 1000 - (off.get("uncertainty_mm") or 1.0)
    rec = dict(what=what, look=scene.get("tag"), wrist_object=obj, wrist_offset=off, room_each_side_mm=plan["room_each_side_m"] * 1000, tolerance_mm=tol)
    grasp = plan.get("placement_at_grasp") or {}; unc = off.get("uncertainty_mm") or 1.0
    target_mm = float(grasp.get("tool_along_jaw_mm") or 0.0)                    # the hand is aimed this far along +jaw from the block centre (visual offset + re-centring); image x runs along +jaw (PICK01 record), so the block should sit this far from the midline
    if off.get("readable"):
        rec["lateral_error_mm"] = off["offset_mm"] - target_mm; rec["lateral_error_px"] = rec["lateral_error_mm"] / (obj.get("mm_per_px") or 1.0); rec["lateral_target_mm"] = target_mm
    def by_joints():
        """Pad placement of the MEASURED joints, from the pad meshes: vertically against the observed block, laterally against where
        this session's wrist-image corrections put the block relative to the hand. The same candidate mapping the plan used:
        it shows what the arm actually did against its plan; it is not an independent measurement of the block."""
        import mujoco
        if not scene.get("q12"):
            return None
        c, yaw = state_object(out, state); m = model((c, yaw)); d = mujoco.MjData(m); sweepick_kin.set_q12(m, d, np.array(scene["q12"])); mujoco.mj_forward(m, d)
        v = tool_offset(state); return pad_placement(m, d, c + np.array([v[0], v[1], 0.0]), yaw, side=SIDE)      # laterally against where the image has put the block relative to the hand so far (the corrections of this session)
    pm = by_joints(); rec["pads_by_measured_joints"] = pm
    def centred(need_confirmed=True):
        if not off.get("readable"):
            return dict(ok=False, unknown=off.get("why", "not readable"))
        if off.get("border_bound"):
            return dict(ok=False, error_mm=rec["lateral_error_mm"], allowed_mm=tol, basis="the block is cut by the image border: the error is only a lower bound; a capped correction and a new look follow, the gate cannot pass on it")
        if need_confirmed and not off.get("reference_confirmed_in_this_frame"):
            return dict(ok=False, unknown="no jaw tip readable in this frame agrees with the jaw reference: the midline is not confirmed for this image", tips=off.get("tips_in_this_frame"))
        return dict(ok=bool(tol > 0 and abs(rec["lateral_error_mm"]) <= tol), error_mm=rec["lateral_error_mm"], allowed_mm=tol, basis="room of this grasp from the pad meshes, less the image uncertainty of this frame")
    def pads_at_grasp():
        if pm is None or not grasp.get("pads"):
            return dict(ok=False, unknown="no joint reading / no planned pad placement")
        e = pm["tool_along_jaw_mm"] - grasp["tool_along_jaw_mm"]                # where the tool really is along the jaw axis against where the grasp was planned
        g = [grasp["pads"][n]["gap_mm"] + grasp["pads"][n]["side"] * e for n in pad_geoms()]
        return dict(ok=bool(min(g) >= unc), tool_error_along_jaw_mm=e, predicted_gaps_mm=g, needed_mm=unc, basis="the planned pad gaps at the grasp pose shifted by the lateral error of the measured joints; each gap must exceed what the image can resolve")
    def pads_above_top():
        if pm is None:
            return dict(ok=False, unknown="no joint reading")
        low = min(pm["pads"][n]["lowest_above_top_mm"] for n in pad_geoms())
        return dict(ok=bool(low > 0), lowest_pad_vertex_above_top_mm=low, basis="pad meshes at the measured joints: no pad vertex at or below the block's top face while the hand is still above it")
    if what == "ready_to_descend":
        checks = dict(centred=centred(), pads_at_grasp_by_measured_joints=pads_at_grasp(), current_alignment=current_wrist_alignment(plan, scene)); ok = all(c["ok"] for c in checks.values())
        rec.update(checks=checks, correctable=bool(not ok and "unknown" not in checks["centred"] and "unknown" not in checks["pads_at_grasp_by_measured_joints"] and not checks["centred"]["ok"]))
        rec["why"] = "the block sits between the jaws within the room the pad meshes leave, in the image and by the measured joints" if ok else "; ".join(f"{k}: {c.get('unknown') or ({a_: (round(b_, 2) if isinstance(b_, float) else b_) for a_, b_ in c.items() if a_ not in ('basis', 'ok')})}" for k, c in checks.items() if not c["ok"])
    elif what == "ready_for_final_descent":
        pre = next((e for e in reversed(state.get("evidence", [])) if e["what"] in ("ready_to_descend", "ready_for_final_descent") and e["ok"]), None)
        checks = dict(centred=centred(), pads_at_grasp_by_measured_joints=pads_at_grasp(), pads_above_the_top_face=pads_above_top(), current_alignment=current_wrist_alignment(plan, scene))
        if obj.get("visible") and pre and pre["wrist_object"].get("visible"):
            po = pre["wrist_object"]; ds = None if obj.get("far_edge_slope_deg") is None or po.get("far_edge_slope_deg") is None else abs(obj["far_edge_slope_deg"] - po["far_edge_slope_deg"])
            rec["cross_height_edge_change"] = dict(slope_change_deg=ds, classification="DIAGNOSTIC", physical_rotation_verified=False, basis="different hand pose / perspective; not evidence that the block physically turned")
            checks["approached"] = dict(width_px=[po["width_px"], obj["width_px"]], ok=bool(obj["width_px"] >= po["width_px"] - 2 * WRIST["measure_px"]), basis="auxiliary: the block must not look smaller than in the last accepted image (it grows as the hand comes down; a second look at the same height shows the same size); the size itself is not used as a height")
        else:
            checks["comparable_image"] = dict(ok=False, unknown="no accepted earlier image to compare with, or the block is not visible")
        ok = all(c["ok"] for c in checks.values()); bad = [k for k, c in checks.items() if not c["ok"]]
        rec.update(checks=checks, correctable=bool(bad and set(bad) <= {"centred", "pads_at_grasp_by_measured_joints"} and "unknown" not in checks["centred"] and not checks["centred"]["ok"]),
                   why="current image aligned and centred, pads above the top face by measured joints (candidate model)" if ok else f"not ready: {bad}")
    elif what == "ready_to_close":
        pre = next((e for e in reversed(state.get("evidence", [])) if e["what"] == "ready_for_final_descent" and e["ok"]), None); checks = {}; unknown = []; ds = None
        if not (obj.get("visible") and obj.get("stable", True) and off.get("readable") and pre and pre["wrist_object"].get("visible")) or obj.get("touches_border"):
            unknown.append("the block is not fully readable in the wrist image after the descent (or no accepted image from before it)")
        else:
            po, pe = pre["wrist_object"], pre
            arrived = pm is not None and None not in pm["gaps_mm"]
            checks["pads_beside_the_block_by_measured_joints"] = dict(gaps_mm=None if pm is None else pm["gaps_mm"], lowest_above_top_mm=None if pm is None else [pm["pads"][n]["lowest_above_top_mm"] for n in pad_geoms()], ok=bool(arrived and min(pm["gaps_mm"]) >= 0), basis="pad meshes at the measured joints: both pads reach below the top face and neither is over the block (candidate model)")
            shift = rec["lateral_error_mm"] - pe.get("lateral_error_mm", 0.0); lim = unc + ((pe.get("wrist_offset") or {}).get("uncertainty_mm") or 1.0)
            rec["offset_change_since_the_last_stop"] = dict(change_mm=shift, resolvable_mm=lim, note="diagnostic: different hand poses can change image offset and slope; current alignment and centring are evaluated separately")
            ds = None if obj.get("far_edge_slope_deg") is None or po.get("far_edge_slope_deg") is None else abs(obj["far_edge_slope_deg"] - po["far_edge_slope_deg"])
            rec["cross_height_edge_change"] = dict(slope_change_deg=ds, classification="DIAGNOSTIC", physical_rotation_verified=False, basis="different hand pose / perspective; not evidence that the block physically turned")
            checks["current_alignment"] = current_wrist_alignment(plan, scene)
            # 211241_auto (right): down at the grasp, both pads beside the block and 15-17 mm below its top face, the block between the jaws in the image,
            # 6.1 mm off the midline against 4.1 allowed -> home. With the pads already down beside it, closing pushes the block to the middle; the
            # limit there is the pad room plus 2 mm (the block slides that far on the floor), not the room less the image uncertainty.
            # 222416_auto (right): down at the grasp (joints within 20 ticks), both pads 12-19 mm BELOW the block's top face, the block square
            # between the jaws in the image - but the model put one pad 3.2 mm over the block (its block position is the head + corrections, cm-level).
            # A pad that is below the top face cannot be over the block unless it pushed it: with the joints at the grasp target and the block
            # where it was in the image (centred within the close limit below), the pads are beside it; the model gap is then a diagnostic.
            pb_ = checks["pads_beside_the_block_by_measured_joints"]
            dsc0_ = ((state["stages"].get("descend_settle") or {}).get("reached") or (state["stages"].get("descend") or {}).get("reached") or {})
            at_target_ = arm5_within(dsc0_, 20.0)
            below_top_ = bool(pb_.get("lowest_above_top_mm")) and all(v_ is not None and v_ < -2.0 for v_ in pb_["lowest_above_top_mm"])
            lat_ = rec.get("lateral_error_mm")                                  # review: an exact 0.0 is a valid error, only None is missing
            if not pb_["ok"] and at_target_ and below_top_ and lat_ is not None and abs(lat_) <= float(plan["room_each_side_m"]) * 1000.0 + CLOSE_CENTRING_EXTRA_MM:
                pb_.update(ok=True, by="joints at the grasp target, both pads below the top face, block centred in the image (the model gap is a diagnostic: its block position is cm-level)")
            pads_down_ = checks["pads_beside_the_block_by_measured_joints"]["ok"]
            # Squeeze grasp (Goldberg 1993, 'Orienting polygonal parts without sensors'): two flat parallel jaws closing on a square part turn it until
            # a side lies flat on a jaw. With the pads already down beside the block, a small yaw between block and jaws is squared by the closing
            # itself; the angle is then a diagnostic within SQUEEZE_SQUARES_DEG (kept well below 45 deg, where the closing may turn either way).
            ca_ = checks["current_alignment"]
            if pads_down_ and not ca_.get("ok") and ca_.get("wrist_edge_angle_deg") is not None:
                rel_ = abs(float(ca_["wrist_edge_angle_deg"]) - float(ca_.get("edge_reference_deg") or 0.0))
                if rel_ <= SQUEEZE_SQUARES_DEG:
                    ca_.update(ok=True, squeeze=dict(relative_deg=rel_, limit_deg=SQUEEZE_SQUARES_DEG, basis="pads down beside the block: the parallel jaws square the block as they close"))
            tol_close = max(tol, float(plan["room_each_side_m"]) * 1000.0 + CLOSE_CENTRING_EXTRA_MM) if pads_down_ else tol
            checks["centred"] = dict(error_mm=rec["lateral_error_mm"], allowed_mm=tol_close, ok=bool(tol_close > 0 and abs(rec["lateral_error_mm"]) <= tol_close), reference="jaw midline confirmed at the previous height; at the grasp height the tips are behind the block",
                                     allowed_basis="pads down beside the block: pad room + %.0f mm (closing centres the block)" % CLOSE_CENTRING_EXTRA_MM if pads_down_ else "pad room less the image uncertainty")
            # 202048_auto: the hand was down (every arm joint within 16 ticks of the grasp target) with the block between the jaw tips, but the block
            # was cut by the image bottom / the jaws there, its width read 231 px as above it (215 after the settle) -> 'not approached', home.
            # The width is auxiliary: the descent counts as made when the measured arm joints ended within 20 ticks of the grasp target.
            dsc_ = ((state["stages"].get("descend_settle") or {}).get("reached") or (state["stages"].get("descend") or {}).get("reached") or {})
            down_ = arm5_within(dsc_, 20.0)
            checks["approached"] = dict(width_px=[po["width_px"], obj["width_px"]], joints_at_grasp_within_20_ticks=down_, ok=bool(obj["width_px"] > po["width_px"] or down_), basis="auxiliary only: larger than at the last stop above the block, or the measured arm joints at the grasp target")
        ok = not unknown and all(c["ok"] for c in checks.values())
        # Preserve the stricter existing settle-retry heuristic; current alignment does not authorize a retry.
        rec["block_undisturbed_in_the_image"] = bool(obj.get("visible") and obj.get("stable", True) and not obj.get("touches_border") and ds is not None and ds <= WRIST["slope_tolerance_deg"])
        rec["settle_retry_evidence_limit"] = "small image change only; neither physical immobility nor absence of contact is proven"
        rec.update(checks=checks, unknown=unknown, placement="PADS_BESIDE_THE_BLOCK (image and measured joints agree)" if ok else ("NOT_DISTINGUISHABLE" if unknown else "NOT_CONFIRMED"),
                   why="current wrist image aligned and centred; measured joints put both pads beside the block in the candidate model" if ok else
                       (unknown[0] if unknown else "; ".join(f"{k}: {({a_: (round(b_, 2) if isinstance(b_, float) else b_) for a_, b_ in c.items() if a_ not in ('basis', 'ok', 'reference')})}" for k, c in checks.items() if not c["ok"])),
                   limit="one wrist camera and the candidate joint model. A pad that rests on the top face without moving the block, while the joint readings still say 'beside', is not seen by this; the two stops above the block are what keep the pads off it")
    elif what == "ready_to_lift":
        g = (state["stages"].get("close") or {}).get("grip") or {}
        at_tool = bool(obj.get("visible")) and bool(off.get("readable")) and abs(off["offset_mm"]) <= (plan["model_jaw_gap_now_m"] - plan["object_width_m"]) / 2 * 1000 + 2 * (off.get("uncertainty_mm") or 1.0)
        label = tg.fuse(g.get("state"), object_at_tool=at_tool if obj else None, evidence_valid=at_tool, object_on_support=None, hand_raised=False)
        ok = label == tg.GRASP_CANDIDATE
        rec.update(jaw_state=g.get("state"), label=label, why="contact candidate of the jaws and the block seen at the tool point" if ok else f"label {label}: a stop of the jaws alone does not open the lift")
    elif what == "held_after_lift":
        g = (state["stages"].get("close") or {}).get("grip") or {}; lift = state["stages"].get("lift") or {}
        before = next((e for e in reversed(state.get("evidence", [])) if e["what"] == "ready_to_lift"), None)
        same = bool(obj.get("visible")) and before is not None and before["wrist_object"].get("visible") and abs(obj["centre_px"] - before["wrist_object"]["centre_px"]) <= 3 * WRIST["measure_px"] and abs(obj["width_px"] - before["wrist_object"]["width_px"]) <= 0.15 * before["wrist_object"]["width_px"]
        seen_there = scene.get("object") is not None and bool(scene["object"].get("class_match")) and float(np.hypot(*(np.array(scene["object"]["centre_xy"]) - np.array(state_object(out, state)[0][:2])))) < 0.03
        region = (scene.get("support_region") or {}).get("state", "UNKNOWN (no area observation)")
        on_desk = True if seen_there else (False if region == "CLEAR" else None)      # True: the block is seen there. False: the area is validly observed and clear. None: not observed (sparse depth, hidden by the arm) - not an absence
        jaw = lift.get("jaw_during") or {}
        label = tg.fuse(tg.CONTACT_CANDIDATE if jaw.get("loaded_throughout") else tg.NO_CONTACT, object_at_tool=bool(same), evidence_valid=bool(same), object_on_support=on_desk, co_motion_observed=bool(same), hand_raised_since_contact=True, was_held=True)
        ok = label == tg.HELD_CONFIRMED
        used = ["wrist image: the block at the same place and size between the jaw tips before and after the lift" if same else None, "jaws loaded through the lift and the hold (port owner's rows)" if jaw.get("loaded_throughout") else None,
                "top depth: the place where the block stood is validly observed and clear" if on_desk is False else None]
        rec.update(label=label, wrist_same_place_as_before_the_lift=bool(same), support_region=dict(state=region, block_seen_there=bool(seen_there), on_support=on_desk, detail=scene.get("support_region")), jaw_during_lift=jaw,
                   evidence_used=[u for u in used if u], evidence_not_available=([] if on_desk is not None else ["top depth of the place where the block stood: not observed, so its absence there is NOT part of this result"]),
                   why=("; ".join(u for u in used if u)) if ok else f"label {label}",
                   limit="the height itself is the model's (candidate FK); this is 'moved with the hand', not a measured height")
    rec["ok"] = bool(ok); state.setdefault("evidence", []).append(rec); (Path(out) / "STATE.json").write_text(json.dumps(state, indent=1, default=str)); return bool(ok), rec


def jaw_during(log, jaw_floor_ticks):
    """The jaws while another stage ran, from the port owner's own rows: residual = measured - held command."""
    rows = [r for r in log.get("rows", []) if "goal" in r and "present" in r]
    if not rows:
        return dict(rows=0, loaded_throughout=False)
    res = [r["present"]["gripper"] - r["goal"]["gripper"] for r in rows]
    return dict(rows=len(rows), seconds=rows[-1]["t"] - rows[0]["t"], residual_ticks=dict(min=float(min(res)), median=float(np.median(res)), max=float(max(res))), floor_ticks=jaw_floor_ticks, loaded_throughout=bool(min(res) > jaw_floor_ticks),
                load_raw=dict(min=float(min(r["load"]["gripper"] for r in rows)), max=float(max(r["load"]["gripper"] for r in rows))), present=dict(min=float(min(r["present"]["gripper"] for r in rows)), max=float(max(r["present"]["gripper"] for r in rows))))


FAST_STAGES = ("pregrasp", "carry", "carry_back", "retreat", "home", "home_arm", "abort_home")   # 203018_auto (user): carry 35 s each way at the near profile was too slow; the grip held
GRASP_ABOVE_TAUGHT_M = 0.004   # 203018_auto: at the taught grasp height the wrist still stood 15 ticks short with load 72 (pads on the floor) and the hand slid forward; 4 mm higher on a 40 mm block
CARRY_HOLD_S = 5.0
GRIP_WATCH_MIN_VALID, GRIP_WATCH_MAX_GAP_S = 5, 1.0                           # a hold is HELD only with this many valid reads and no longer gap (review 2026-10-19)
HEAD_RECT_YAW_TRUST_DEG = 12.0                                                 # head top-face rectangle yaw used only when it differs more than this from the taught yaw
YAW_STEP_WHOLE_DEG = 30.0                                                      # largest wrist turn per look when the block outline is whole (the image-Jacobian loop)
SQUEEZE_SQUARES_DEG = 15.0                                                     # ready_to_close with the pads down: the closing squares this much block yaw (squeeze grasp)
CLOSE_CENTRING_EXTRA_MM = 2.0                                                  # ready_to_close with the pads already down beside the block (211241_auto)
SKIP_FAR_ARM_M = 0.05                                                          # the farther arm is not planned when it is this much farther and the nearer one has a path
LOOK_DEPTH_FRAMES = 8                                                          # head FrameSets per look between observe and the end (timing, 2026-10-18)
LIFT_HOLD_S = 2.0                                                              # the held lift is watched this long (was 5 s; the carry hold watches 5 s more)                                                             # seconds held in the carry pose (the base would drive there), jaws watched
MEDIUM_STAGES = ("align", "descend_a")                                         # 210347 timing: a 16 mm align move took 10 s at 0.15 rad/s. Empty hand above the block only: not the final descent, the grip,
                                                                               # or the lift / lower with the block (a P-only servo lags more at speed under load: the shoulder already stood 45..53 ticks behind)
MEDIUM_V, MEDIUM_A = 0.3, 1.5
FAST_V, FAST_A = 0.6, 3.0                                                      # rad/s, rad/s^2 for the moves away from the block (near it the profile file's 0.15 / 1.5 stays)


def fast_profile(v=None, a=None):
    import hashlib as _h, sweepick_trajectory as tt
    p_ = dict(tt.load_profile(), v=FAST_V if v is None else v, a=FAST_A if a is None else a); p_["sha256"] = _h.sha256(json.dumps({k: p_[k] for k in ("v", "a", "j", "dt")}, sort_keys=True).encode()).hexdigest(); return p_


def stage(out, name, *, execute=False, run=mv.run, grip=mv.grip, cert=certify, cameras=None, run_kw=None, torque_now=None):
    import mujoco
    out = Path(out); state = json.loads((out / "STATE.json").read_text()); mp = mapping(); run_kw = dict(run_kw or {})
    set_side(state.get("side", "left")); other_ticks = state.get("other_ticks") or state["right_ticks"]
    torque_now = torque_now or (lambda: read_ticks()[SIDE]["torque"])
    rec = dict(stage=name, execute=bool(execute), t=time.time())
    def done(result, **kw):
        rec.update(result=result, **kw)
        wrote = any((kw.get("writes") or {}).values())
        if execute and result == "RAN" and kw.get("controller_result") == "REFUSED" and not wrote:
            state.setdefault("refusals", []).append(dict(rec)); (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str))      # requested and refused before any write: kept as a record, it does not close the ways back
        elif execute and result == "RAN":                                     # only a stage that reached the executor and wrote is part of the session's history
            state.setdefault("order", []).append(name)
            if kw.get("controller_result") == "HOLDING" or (name in SHORT_OK and kw.get("controller_result") == "TARGET_NOT_REACHED_HOLDING"):
                state["height"] = {"pregrasp": "pregrasp", "descend_a": "descend_a", "descend_b": "descend_b", "descend": "descend", "descend_settle": "descend", "retreat_arm": "pregrasp", "retreat": "pregrasp", "lift": "lift", "carry": "carry", "carry_back": "lift", "lower": "descend"}.get(name, state.get("height"))
                if name == "align":
                    state["pending_align"] = False
            state["stages"][name] = rec; (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str))
        (out / f"stage_{name}.json").write_text(json.dumps(rec, indent=1, default=str)); return rec
    if not state.get("ready"):
        return done("NOT_RUN", why=f"observe did not end ready: {state.get('why')}")
    changed = [f for f, h in (state.get("sources") or {}).items() if hashlib.sha256((HERE / f).read_bytes()).hexdigest() != h]
    if changed and execute and name != "abort_home":                          # the collision-checked way home stays available (213827_auto: the source changed under a running session and the arm stood above the block)
        return done("NOT_RUN", why=f"source / profile files changed since this session's observe: {changed}. A session runs on the files it started with")
    ok_now, basis = allowed(state, name)
    if not ok_now:
        return done("NOT_RUN", why=f"not allowed now: {basis}; stages so far: {[(n, (state['stages'].get(n) or {}).get('controller_result')) for n in state.get('order', [])]}")
    rec["allowed_because"] = basis
    if name in GATE and not any(e["what"] == GATE[name] and e["ok"] for e in state.get("evidence", [])[-1:]):
        return done("NOT_RUN", why=f"the observation '{GATE[name]}' is not there (last evidence: {(state.get('evidence') or [dict(what=None)])[-1].get('what')}, ok={(state.get('evidence') or [dict(ok=None)])[-1].get('ok')}): the stage is not opened by the previous controller result alone")
    if name in state["stages"] and execute and name != "align":            # an alignment move belongs to its correction; every other stage runs once
        return done("NOT_RUN", why="this stage was already run in this session; nothing is repeated")
    rq = stage_request(state, name, mp, torque_now() if name == "abort_home" else None); sdir = out / (name if name != "align" else f"align{len(state.get('correction') or [])}"); rec["request"] = {k: v for k, v in rq.items()}
    obj = state_object(out, state); with_object = rq["scene"].startswith("object")
    scene_file = out / f"scene_block{len(state.get('object_updates') or [])}.mjb"      # one scene per OBSERVED block pose; hand corrections do not make a new one
    if not with_object and not scene_file.exists():
        mujoco.mj_saveModel(model(obj), str(scene_file))
    grasp_ticks = ((state["stages"].get("descend") or {}).get("end") or {}).get("ticks")
    def check(doc, folder, tag):
        """collision result of one commanded path for THIS stage's scene (also used for a trim of this stage)"""
        folder.mkdir(parents=True, exist_ok=True)
        if with_object:
            c_ = object_check(doc, obj, carried=name in ("lift", "carry", "carry_back", "lower"), right_ticks=doc.get("other_arm_ticks") or other_ticks, grasp_arm_ticks=None if grasp_ticks is None else {n: grasp_ticks[n] for n in ALL})
        else:
            (folder / f"{tag}.TRAJECTORY.json").write_text(json.dumps(doc)); c_ = cert(folder / f"{tag}.TRAJECTORY.json", folder / f"{tag}.CERT.json", scene_file)
        (folder / f"{tag}.CHECK.json").write_text(json.dumps(c_, indent=1, default=str)); return c_
    kw = dict(level="model", torque_joints=rq["torque_joints"], **run_kw)
    if name in FAST_STAGES and "profile" not in kw:                           # away from the block: faster (user 2026-10-18)
        kw["profile"] = fast_profile()
    elif name in MEDIUM_STAGES and "profile" not in kw:
        kw["profile"] = fast_profile(MEDIUM_V, MEDIUM_A)
    recs = []
    def record():
        if cameras is not None:
            from sweepick_contact01 import FrameRecorder
            try:
                for cam in cameras():
                    recs.append((cam, FrameRecorder(cam, sdir / "frames", [name])))
            except Exception as e:                                             # recording is monitoring only: without it the stage still runs (its gate was already passed, and a way back must not depend on a camera)
                rec["recording_error"] = f"{type(e).__name__}: {e}"
            if recs and EPISODE is None:
                time.sleep(1.0)                                                # let the camera stream start before the motor bus is used: on 2026-10-08 a status packet was corrupted in the same second the stream opened (cameras and motor adapters share one USB hub)
    def stop_recording():
        frames = [f for cam, r in recs for f in r.close()]
        for cam, r in recs:
            cam.close()
        if frames:
            (sdir / "frames/INDEX.json").write_text(json.dumps(frames))
        return frames
    if rq["mode"] == "grip":
        jd = json.loads(JAW_PROFILES[SIDE].read_text()); arm = {n: float(v) for n, v in read_now(run_kw)["ticks"].items()}; lo = float(mp[SIDE]["gripper"]["lo"])
        def closing_check(actual, tag):
            end_tick = lo + float(jd["end_stop_ticks_above_closed"]["value"])
            doc = sweep_doc(actual, actual["gripper"], end_tick)
            checked = check(doc, sdir, tag)
            checked.update(arm_ticks={n: actual[n] for n in ARM}, sweep=[actual["gripper"], end_tick])
            (sdir / f"{tag}.CHECK.json").write_text(json.dumps(checked, indent=1, default=str))
            return checked
        c = closing_check(arm, "sweep")
        rec["collision"] = [dict(verdict=c["nominal_verdict"], first_contact=c.get("first_nominal_contact"), steps=c["nominal_steps_checked"], scene=c["scene"], allowed=c["allowed"])]
        if c["nominal_verdict"] != "CLEAR":
            return done("NOT_RUN", why=f"the closing sweep touches something that is not the intended contact: {c.get('first_nominal_contact')}", controller_result="PLANNED")
        if not execute:
            return done("PLANNED", controller_result="PLANNED")
        record()
        try:
            code, log = grip(SIDE, sdir / "run1", torque_joints=ALL, cert=c, level="model", execute=True, stop_requested=Path(str(out) + ".STOP").exists, **({} if SIDE == "left" else dict(jaw_profile=JAW_PROFILES[SIDE])),
                             start_recheck=lambda actual: closing_check(actual, "sweep_recheck"), **run_kw)
        finally:
            frames = stop_recording()
        return done("RAN", controller_result=log["result"], exit_code=code, outcome=log.get("outcome"), stopped_by=log.get("stopped_by"), refused=log.get("refused"), writes=log.get("writes"), grip=log.get("grip"), end=log.get("end"),
                    needs_a_person=log.get("needs_a_person"), frames=len(frames), owner="sweepick_grasp.Jaw", meaning="a contact candidate of the jaws; whether it is the block is decided by the observation that follows")
    kw.update(other_arm_ticks=other_ticks)
    if name == "pregrasp":
        kw["range_entry"] = True
    if rq["mode"] == "next" and execute:
        here = read_now(run_kw)["ticks"]
        if all(round(here[n]) == round(v) for n, v in rq["target"].items()):   # e.g. a correction smaller than one tick: the arm already stands there, nothing is sent
            return done("RAN", controller_result="HOLDING", exit_code=0, writes=dict(Goal_Position=0, Torque_Enable=0), outcome=dict(command_completed=True, target_reached=True, torque_released=False), end=dict(ticks=here), note="already at the target ticks; nothing was written")
    kw.update(dict(rest=rq["rest"]) if rq["mode"] == "return" else dict(target=rq["target"]))
    trim = ARM if name in TRIM else []
    tries = []
    for attempt in (1,):                                                      # the executor rechecks the actual path; a refused stage ends this authorized attempt
        code, plan = run("plan" if rq["mode"] != "return" else "return", SIDE, sdir / f"plan{attempt}", **kw)
        if plan["result"] != "PLANNED":
            return done("NOT_RUN", why=f"planning read refused: {plan.get('refused')}", controller_result=plan["result"])
        c = check(json.loads((sdir / f"plan{attempt}/TRAJECTORY.json").read_text()), sdir / f"plan{attempt}", "path")
        tries.append(dict(attempt=attempt, identity=c.get("identity"), verdict=c.get("nominal_verdict"), first_contact=c.get("first_nominal_contact"), steps=plan["trajectory"]["steps"], duration_s=plan["trajectory"]["duration_s"], travel_ticks=plan["trajectory"]["travel_ticks"],
                          scene=c.get("scene"), allowed=c.get("allowed", "no new contact at all")))
        rec["collision"] = tries
        if c.get("nominal_verdict") != "CLEAR":
            return done("NOT_RUN", why=f"the commanded path touches something that is not an intended contact: {c.get('first_nominal_contact')}", controller_result="PLANNED")
        if not execute:
            return done("PLANNED", controller_result="PLANNED")
        record(); n_trim = [0]
        def trim_check(doc):
            n_trim[0] += 1; return check(doc, sdir / f"run{attempt}", f"trim{n_trim[0]}")
        def current_right_ticks():                                            # the OTHER arm's current ticks (the name is kept from the left-only executor)
            from sweepick_local_loop import open_readonly_bus
            import contextlib
            with EPISODE.lease(other(SIDE)) if EPISODE is not None else contextlib.nullcontext():
                b = open_readonly_bus(other(SIDE))
                try:
                    return {n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}
                finally:
                    b.disconnect()
        def missing_injected_other_actual():
            raise ValueError("an injected bus requires an explicit other_arm_read; no real device is opened")
        kw.setdefault("other_arm_read", missing_injected_other_actual if "open_bus" in run_kw else current_right_ticks)
        try:
            code, log = run(rq["mode"], SIDE, sdir / f"run{attempt}", execute=True, cert=c, settle_trim=trim, trim_check=trim_check, start_recheck=lambda doc: check(doc, sdir / f"run{attempt}", "start"), hold_s=LIFT_HOLD_S if name == "lift" else 0.0, stop_requested=Path(str(out) + ".STOP").exists, **kw)
        finally:
            frames = stop_recording(); recs.clear()
    rec["monitor"] = motion_monitor(sdir / "frames")
    if name in ("lift", "carry", "carry_back", "lower"):
        rec["jaw_during"] = jaw_during(log, float(json.loads(JAW_PROFILES[SIDE].read_text())["residual_floor_ticks"]["value"]))
    g = (log.get("reached") or {})
    return done("RAN", controller_result=log["result"], exit_code=code, outcome=log.get("outcome"), stopped_by=log.get("stopped_by"), refused=log.get("refused"), writes=log.get("writes"), command_waits=log.get("command_waits"),
                reached={n: dict(tick=v["tick"], minus_target=v["minus_target"], load=v.get("load"), current=v.get("current")) for n, v in g.items()}, settle_trim=log.get("settle_trim"), end=log.get("end"), needs_a_person=log.get("needs_a_person"), frames=len(frames),
                meaning=None)


def motion_monitor(folder):
    """What the wrist frames RECORDED WHILE THE STAGE MOVED show (analysed after the move; it does not change the command
    while the arm is moving): frame counter gaps, receive times, and where the block was seen."""
    import cv2
    idx = Path(folder) / "INDEX.json"
    if not idx.exists():
        return dict(role="MONITOR_ONLY", frames=0, note="no frames recorded for this stage")
    rows = json.loads(idx.read_text()); seen = []
    for r in rows:
        im = cv2.imread(str(Path(folder) / r["file"])); o = wrist_object(im) if im is not None else dict(visible=False)
        seen.append(dict(n=r["n"], t=r["recv_host_s"], visible=bool(o.get("visible")), centre_px=o.get("centre_px"), width_px=o.get("width_px")))
    ns = [x["n"] for x in seen]; ts = [x["t"] for x in seen]; vis = [x for x in seen if x["visible"]]
    return dict(role="MONITOR_ONLY (recorded during the motion, read afterwards)", frames=len(seen), distinct_frames=len(set(ns)), counter_went_back=bool(any(b <= a for a, b in zip(ns, ns[1:]))), largest_gap_s=float(max((b - a for a, b in zip(ts, ts[1:])), default=0.0)),
                block_visible_in=len(vis), first_visible_t=vis[0]["t"] if vis else None, visible_in_the_last_frame=bool(seen and seen[-1]["visible"]),
                centre_px=dict(first=vis[0]["centre_px"], last=vis[-1]["centre_px"]) if vis else None, width_px=dict(first=vis[0]["width_px"], last=vis[-1]["width_px"]) if vis else None)


def read_now(run_kw=None):
    """Fresh read-only joint read of the session arm (the fake of a test supplies its own)."""
    if run_kw and "open_bus" in run_kw:
        b = run_kw["open_bus"](f"/dev/dapier/{SIDE}_arm"); return dict(ticks={n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()})
    return read_ticks()[SIDE]


def state_object(out, state):
    """The block where it was OBSERVED (top depth of this session's observe). Hand corrections never move it; it changes only
    through an entry in state['object_updates'] that names the observation it comes from."""
    s = json.loads((Path(out) / state["scene"]).read_text()); o = s["object"]; c = np.array(o["centre_world"], float); yaw = o["yaw_rad"] or 0.0
    for u in state.get("object_updates") or []:                                # each: dict(centre_world, yaw_rad, source=<look tag / frames>, uncertainty_m)
        c, yaw = np.array(u["centre_world"], float), u.get("yaw_rad", yaw)
    return (c, yaw)


def tool_offset(state):
    """Sum of the hand corrections of this session (world, m): where the hand is aimed relative to the observed block."""
    return sum((np.array(k["tool_shift_world_m"]) for k in state.get("correction") or []), np.zeros(3))


def correct(out, along_jaw_mm, basis, assisted=False, axis_xy=None, above_b=None, yaw_rad=None, across_mm=0.0, up_mm=0.0):
    """A correction of where the HAND is aimed, along the jaw axis, from the wrist image; and a new plan from the pose the arm
    holds now. The observed block pose, the collision scene and the place where the block stands are NOT changed by it.
    axis_xy: the jaw axis the image error was measured against (from the look); the same axis is used for the shift and,
    after the move, for the tool displacement. Reads only; the move itself is the stage 'align'."""
    out = Path(out); state = json.loads((out / "STATE.json").read_text())
    if above_b is not None:
        state["above_b_m"] = float(above_b)
    if state.get("height") not in ("pregrasp", "descend_a", "descend_b") or "close" in state["stages"]:
        return dict(plan_ok=False, why="a correction is made while the pads are still above the block (pre-grasp or intermediate height)")
    arms = read_ticks(); ticks = {s_: arms[s_]["ticks"] for s_ in arms}; mp = mapping(); q12 = rc.ticks_to_q12(mp, ticks)
    if axis_xy is None and assisted:                                           # a person's correction from the command line: the jaw axis of the pose held now
        import mujoco
        m_ = model(); d_ = mujoco.MjData(m_); sweepick_kin.set_q12(m_, d_, q12); mujoco.mj_forward(m_, d_); axis_xy = tool_axis(m_, d_, SIDE)["jaw_axis_xy"]
    if axis_xy is None:
        return dict(plan_ok=False, why="no jaw axis from the look that measured the error")
    jaw = np.array([axis_xy[0], axis_xy[1], 0.0]); shift = along_jaw_mm / 1000.0 * jaw
    if across_mm:                                                              # 2026-10-18: across the jaw axis too, + = away from this arm's base (forward in the wrist image)
        import mujoco
        m_ = model(); d_ = mujoco.MjData(m_); sweepick_kin.set_q12(m_, d_, q12); mujoco.mj_forward(m_, d_)
        perp = np.array([-jaw[1], jaw[0], 0.0]); radial = d_.site_xpos[m_.site(f"{SIDE}_cube_grasp").id] - d_.xpos[m_.body(f"{SIDE}_base").id]; radial[2] = 0.0
        if float(np.dot(perp, radial)) < 0:
            perp = -perp
        shift = shift + across_mm / 1000.0 * perp
    if up_mm:                                                                  # the approach heights only (pre-grasp, above the block); the grasp height is not moved by it
        key_dz = "approach_dz_m" if state.get("height") == "pregrasp" else "above_dz_m"   # 200250_auto: a rise at the pre-grasp no longer lifts the stop above the block
        state[key_dz] = float(state.get(key_dz, 0.0)) + up_mm / 1000.0
    total = tool_offset(state) + shift
    if yaw_rad is not None:                                                     # the block's yaw seen in the wrist image replaces the head-depth one (an observation; the centre is kept)
        c_prev, y_prev = state_object(out, state)
        state.setdefault("object_updates", []).append(dict(centre_world=[float(v) for v in c_prev], yaw_rad=float(yaw_rad), previous_yaw_rad=float(y_prev), source=f"wrist image: {basis}", kind="BLOCK YAW FROM THE WRIST IMAGE", uncertainty_deg=3.0))
    scene = json.loads((out / state["scene"]).read_text()); c0, yaw = state_object(out, state)
    scene = dict(scene, q12=[float(v) for v in q12], object=dict(scene["object"], centre_world=[float(v) for v in c0], yaw_rad=float(yaw)))
    plan = plan_pick(scene, tool_offset=total, above_b=state.get("above_b_m"), keep=dict(tilt_deg=state["plan"]["tilt_deg"], jaw=state["plan"]["jaw"]), side=SIDE, approach_dz=float(state.get("approach_dz_m", 0.0)), above_dz=float(state.get("above_dz_m", 0.0)))
    # 2026-10-18 171138_auto: a 2 cm 'up' correction re-planned with the wrist turned 42 deg (another IK branch / the block's 90-deg
    # symmetry), the wrist stalled. A position correction keeps the wrist: of the block's symmetric yaws the plan whose wrist
    # roll is nearest to where the wrist stands is taken, and more than 20 deg of roll is refused.
    if yaw_rad is None:
        here_roll = float(ticks[SIDE]["wrist_roll"]); h_ = state.get("height") or "pregrasp"
        def roll_change(pl):
            return abs(float(pl["ticks"].get(h_, pl["ticks"]["pregrasp"])["wrist_roll"]) - here_roll) if pl.get("ok") else 1e9
        best = plan
        for dy in ((np.pi / 2, -np.pi / 2, np.pi) if roll_change(plan) > 3.0 / 360.0 * 4096 else ()):   # the same yaw already keeps the wrist within 3 deg: the symmetric ones are not re-planned (timing)
            sc2 = dict(scene, object=dict(scene["object"], yaw_rad=float(yaw + dy)))
            alt = plan_pick(sc2, tool_offset=total, above_b=state.get("above_b_m"), keep=dict(tilt_deg=state["plan"]["tilt_deg"], jaw=state["plan"]["jaw"]), side=SIDE, approach_dz=float(state.get("approach_dz_m", 0.0)), above_dz=float(state.get("above_dz_m", 0.0)))
            if roll_change(alt) < roll_change(best):
                best = alt
        plan = best
        if not plan.get("ok"):
            # 2026-10-18 182311_auto: at the far edge of the left arm's reach a 1 cm shift made the kept tilt (10 deg) unreachable at the
            # grasp (IK axis error 9.5 deg) while other tilts of the same jaw were feasible. The jaw direction is kept, the tilt may change;
            # the same wrist-roll rule (nearest symmetric yaw, at most 20 deg) decides.
            for dy in (0.0, np.pi / 2, -np.pi / 2, np.pi):
                sc2 = dict(scene, object=dict(scene["object"], yaw_rad=float(yaw + dy)))
                alt = plan_pick(sc2, tool_offset=total, above_b=state.get("above_b_m"), keep=dict(tilt_deg=None, jaw=state["plan"]["jaw"]), side=SIDE, approach_dz=float(state.get("approach_dz_m", 0.0)), above_dz=float(state.get("above_dz_m", 0.0)))
                if roll_change(alt) < roll_change(plan):
                    plan = alt
            if not plan.get("ok"):
                plan = best
        if plan.get("ok") and roll_change(plan) > 20.0 / 360.0 * 4096:
            plan = dict(ok=False, why=[f"a position correction would turn the wrist by {roll_change(plan) * 360 / 4096:.0f} deg (more than 20): refused"])
    if not plan.get("ok") and yaw_rad is not None:                             # a yaw step at the edge of reach (183317_auto: tilt 0 failed at the grasp): the same jaw at another tilt
        alt = plan_pick(scene, tool_offset=total, above_b=state.get("above_b_m"), keep=dict(tilt_deg=None, jaw=state["plan"]["jaw"]), side=SIDE, approach_dz=float(state.get("approach_dz_m", 0.0)), above_dz=float(state.get("above_dz_m", 0.0)))
        if alt.get("ok"):
            plan = alt
    rec = dict(source="ASSISTED: amount given by a person" if assisted else "wrist image of this session", kind="HAND TARGET OFFSET (not an object measurement)", along_jaw_mm=along_jaw_mm, across_mm=across_mm, up_mm=up_mm, axis_xy=[float(axis_xy[0]), float(axis_xy[1])],
               tool_shift_world_m=[float(v) for v in shift], tool_offset_total_world_m=[float(v) for v in total], object_observed_world_m=[float(v) for v in c0], basis=basis, measured_ticks=ticks[SIDE], plan_ok=plan["ok"], previous_ticks=state["plan"]["ticks"])
    if not plan["ok"]:
        rec["why"] = plan["why"]; (out / "correction_refused.json").write_text(json.dumps(rec, indent=1, default=str)); return rec
    if along_jaw_mm == 0.0 and above_b is not None:                             # only the second stop was added: the poses already reached stay exactly as they are (no move at this height)
        for h in ("pregrasp", "descend_a"):
            plan["ticks"][h] = dict(state["plan"]["ticks"][h])
        rec["kind"] = "SECOND STOP ADDED (no hand shift)"
    state.setdefault("correction", []).append(rec); state["plan"] = plan; state["pending_align"] = bool(along_jaw_mm != 0.0 or yaw_rad is not None or across_mm or up_mm)
    (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str)); return dict(rec, new_ticks=plan["ticks"], tilt_deg=plan["tilt_deg"])


def snapshot(out, scene, evidence):
    """State at the lift for the right-arm RECEIVE preparation, from what this session already recorded. SNAPSHOT_ONLY: by
    the time it is read the block is back on the desk and the arm is home; a receive needs fresh inputs and one owner."""
    out = Path(out); st = json.loads((out / "STATE.json").read_text()); lift = st["stages"].get("lift") or {}; close = st["stages"].get("close") or {}; end = lift.get("end") or {}
    mp = mapping(); goal = end.get("goal_register"); right = (scene.get("arms") or {}).get("right", {}).get("ticks")
    doc = dict(schema="tjj.handoff-snapshot.v1", kind="SNAPSHOT_ONLY (not a state that is being held)", run=st.get("run_name"), control=st.get("control"), command_owner_at_the_lift="sweepick_move_a.run stage 'lift' (arm), sweepick_grasp.Jaw hold command (gripper)",
               q12_measured=scene.get("q12"), ticks_measured=scene.get("ticks"), joint_reads=scene.get("joint_reads"), top_stamps=scene.get("stamps"), wrist_frames=(scene.get("wrist") or {}).get("frames"),
               last_applied=dict(left_goal_ticks=goal, left_goal_q=None if not goal or not right else [float(v) for v in rc.ticks_to_q12(mp, dict(left=goal, right=right))[:6]], source="Goal_Position registers read by the executor at the end of the lift stage",
                                 jaw_hold_command_tick=(close.get("grip") or {}).get("hold_command_tick"), right_arm="not commanded in this session"),
               object=dict(observed_on_the_desk_world_m=[float(v) for v in state_object(out, st)[0]], yaw_rad=float(state_object(out, st)[1]), size_m=[2 * BLOCK_HALF_M] * 3, kind=f"{OBJECT['colour']} {OBJECT['size_m'] * 1000:.0f} mm development block ({OBJECT['name']})", frame="assembled dual model world (left base at its model pose), through the candidate chain camera <- board <- ruler datum",
                           size_source="MODEL_PRIOR: existing A block model; not measured hidden dimensions",
                           geometry=st.get("object_geometry"),
                           hand_target_offset_world_m=[float(v) for v in tool_offset(st)], in_hand="not measured as a pose: at the tool point of the left gripper by the wrist image; the tool point comes from the candidate joint mapping",
                           uncertainty="top-depth centre a few mm (depth reads about 2 % long); candidate q12 and chain unverified"),
               tool_left_world_m=scene.get("tool_left"), tool_session_arm_world_m=scene.get(f"tool_{SIDE}"), session_arm=SIDE, jaw_axis_xy=scene.get("jaw_axis_xy"), grasp_evidence=dict(jaw=close.get("grip"), jaw_during_lift=lift.get("jaw_during"), held_after_lift=evidence),
               identifiers=dict(sources=st.get("sources"), mapping="CANDIDATE_Q12_CONFIG.json sha256 " + hashlib.sha256(CAND.read_bytes()).hexdigest(), scene="dual_scene_desk_both.mjb sha256 " + hashlib.sha256(SCENE.read_bytes()).hexdigest(), motion_profile="sweepick_motion_profile.json", jaw_profile="sweepick_jaw_profile_left.json"))
    (out / "HANDOFF_SNAPSHOT.json").write_text(json.dumps(doc, indent=1, default=str)); return doc


def wrist_cams():
    if EPISODE is not None and f"{SIDE}_wrist" in EPISODE.wrists:
        return [EPISODE.lend(f"{SIDE}_wrist")]
    from sweepick_local_loop import WristCam
    return [WristCam(f"{SIDE}_wrist", f"/dev/dapier/{SIDE}_wrist_rgb")]


SKILL = dict(pregrasp="START", align="START", descend_a="START", descend_b="START", descend="START", descend_settle="START", close="START", lift="START",
             lower="PUT_BACK", open="PUT_BACK", retreat="PUT_BACK", home="PUT_BACK", retreat_arm="PUT_BACK", home_arm="PUT_BACK", abort_home="PUT_BACK")


RECORDING_ERRORS = []      # failures of the auxiliary record made on the control path in this process; reported as data quality, never raised into the motion's flow


def _rec(fn, what):
    try:
        return fn()
    except Exception as e:
        RECORDING_ERRORS.append(f"{what}: {type(e).__name__}: {e}")


def recorded_stage(stage_fn):
    """The same stage function, with the session's recorder told what runs and made to let go of the left arm's port for it.
    Without a recorder this is the stage function itself."""
    def run(o, n, execute=False):
        ep = EPISODE
        if ep is None or not execute:
            return stage_fn(o, n, execute=execute)
        _rec(lambda: ep.mark(n, skill=SKILL.get(n), owner=f"sweepick_move_a ({SIDE} arm)" if n not in ("close", "open") else f"sweepick_grasp.Jaw through sweepick_move_a.grip ({SIDE} gripper)"), f"mark {n}")
        with ep.lease("left", "right"):                                         # The writer owns current reads of both arms; pause duplicate background polling.
            r = stage_fn(o, n, execute=execute)
        # the stage has ended and its result is what the session acts on; nothing below may replace or lose it
        def after():
            try:
                folder = n if n != "align" else f"align{len(json.loads((Path(o) / 'STATE.json').read_text()).get('correction') or [])}"
            except Exception:
                folder = n
            ep.stage(n, Path(o) / folder, SIDE, r.get("controller_result") or r.get("result")); ep.mark(f"after_{n}", owner=None)
        _rec(after, f"stage record {n}")
        return r
    return run


def put_back_evidence(state, scene):
    """Revision 2 of the put-back check (2026-10-08), next to the original one, which is kept as it is. Run 14 put the block
    back and the original check said 'not seen' because the top depth read it 29.2 mm wide (class threshold 30 mm). The
    threshold is not changed and 'at the same place' is not a success by itself. This uses what the session already has:
    the jaws were opened, the arm went home and is released, and the place is validly observed and occupied by something
    of the block's height. Sparse depth or no detection is UNKNOWN."""
    st = state.get("stages") or {}; op, hm = st.get("open") or {}, st.get("home") or {}; o = scene.get("object"); reg = scene.get("support_region") or {}
    tq = ((scene.get("arms") or {}).get(SIDE) or {}).get("torque") or {}
    ev = dict(jaws_opened=bool(op.get("controller_result") == "HOLDING" and (op.get("reached") or {}).get("gripper") is not None),
              arm_home_and_released=bool(hm.get("controller_result") == "RETURNED_RELEASED" and tq and not any(tq.values())),
              place_validly_observed=bool(reg.get("valid")) if reg else None, place_occupied=(reg.get("state") == "OCCUPIED") if reg.get("valid") else None,
              object_seen=o is not None, object_top_height_m=None if o is None else o.get("top_height_above_depth_desk_m"), size_class=None if o is None else bool(o.get("class_match")),
              block_in_the_wrist_view=((scene.get("wrist") or {}).get("object") or {}).get("visible"))
    h = ev["object_top_height_m"]; ev["height_is_the_blocks"] = None if h is None else bool(abs(h - 2 * BLOCK_HALF_M) <= BLOCK_HALF_M / 2)       # the class check's own tolerance (a quarter of the size), on the height
    need = ("jaws_opened", "arm_home_and_released", "place_validly_observed", "place_occupied", "object_seen", "height_is_the_blocks")
    if any(ev[k] is None for k in need):
        ev["verdict"] = "UNKNOWN"
    elif all(ev[k] for k in need):
        ev["verdict"] = "ON_THE_DESK_AT_THE_PLACE_JAWS_OPEN_ARM_AWAY" + ("" if ev["size_class"] else " (size class NOT confirmed by the top depth)")
    else:
        ev["verdict"] = "NOT_CONFIRMED"
    ev["revision"] = "put_back_evidence.v2, 2026-10-08; the original 'placed_back' (size class at the final look) is reported unchanged next to it"
    return ev


LEFT_TO_LIFT = "LEFT_START_TO_LIFT"                                             # Console-17 limited execution scope: the same session up to held_after_lift, then nothing more


VSERVO_SHADOW_SCHEMA = "sweepick.vservo-shadow-input.v1"


def vservo_shadow_input(out, row, scene):
    """Shadow-only interface for B18/B19 (2026-10-15): one line per visual-loop look with the inputs A used and the
    correction A computed, plus the stored frame/scene paths. Nothing reads this file on the control path; a proposal
    computed from it is never executable (B18 contract: executable=false, motor_write_authorized=false)."""
    tag = scene.get("tag"); folder = Path(out) / "looks" / str(tag)
    line = dict(schema=VSERVO_SHADOW_SCHEMA, observation_identity=dict(session_out=str(out), look_tag=tag),
                files=dict(scene="looks/%s/SCENE.json" % tag, left_wrist="looks/%s/left_wrist.jpg" % tag, top="looks/%s/top.png" % tag,
                           present={n: (folder / n).exists() for n in ("SCENE.json", "left_wrist.jpg", "top.png")}),
                height=row.get("height"), k=row.get("k"), gate=row.get("gate"), ok=row.get("ok"), q12=scene.get("q12"), joint_reads=scene.get("joint_reads"),
                a_feature=dict(offset_px=row.get("offset_px"), offset_mm=row.get("offset_mm"), lateral_error_mm=row.get("lateral_error_mm"),
                               lateral_target_mm=row.get("lateral_target_mm"), uncertainty_mm=row.get("uncertainty_mm"), reference_confirmed=row.get("reference_confirmed"),
                               tool_xy_m=row.get("tool_xy_m"), axis=row.get("axis")),
                a_response=row.get("response"), a_correction=row.get("correction"), a_plan_ok=row.get("plan_ok"), a_end=row.get("end"),
                timing_monotonic_s=row.get("timing_monotonic_s"), look_timing_monotonic_s=row.get("look_timing_monotonic_s"),
                previous_align_stage_monotonic_s=row.get("previous_align_stage_monotonic_s"),
                role="SHADOW_INPUT_ONLY", executable=False, motor_write_authorized=False)
    with open(Path(out) / "VSERVO_SHADOW_INPUT.jsonl", "a") as stream:
        stream.write(json.dumps(line, default=str) + "\n")
    return line


def limited_lift_end(out, state, scene, evidence, held, trail):
    """Console-17: the explicit boundary after held_after_lift. Nothing below this is commanded in LEFT_START_TO_LIFT: no
    CARRY/hand-over, no lower/open/retreat/home, no Torque_Enable write. The left arm stays powered and holding as the lift
    left it; the process ends after the recorder closes. Ending the process is NOT a motor stop or a torque-off."""
    lift = (state.get("stages") or {}).get("lift") or {}
    end = lift.get("end") or {}
    arms = ((scene.get("arms") or {}).get(SIDE) or {})
    terminal = f"{SIDE.upper()}_LIFT_HELD_CONFIRMED" if held else f"{SIDE.upper()}_LIFT_HOLD_UNKNOWN"
    return dict(result=f"{terminal}_HOLDING_NEEDS_A_PERSON", mode=LEFT_TO_LIFT, limited_terminal=terminal, held_after_lift=held,
                boundary=dict(after="held_after_lift", next_stage_commanded=None, handoff_entered=False, put_back_entered=False,
                              torque_writes_after_lift=0),
                last_lift=dict(controller_result=lift.get("controller_result"), exit_code=lift.get("exit_code"), stopped_by=lift.get("stopped_by"),
                               writes=lift.get("writes"), end=end, jaw_during=lift.get("jaw_during"), recorded_at_host_s=lift.get("t")),
                last_look=dict(tag=scene.get("tag"), ok=scene.get("ok"), arms_left=arms, arm_side=SIDE, joint_reads=scene.get("joint_reads")),
                hold_evidence=dict(gate="held_after_lift", ok=held, why=evidence.get("why"), label=evidence.get("label"),
                                   basis="the existing held_after_lift observation; not a force/slip measurement"),
                normal_home=False, torque_off_confirmed=False, safe_shutdown="NOT_PERFORMED",
                operator_required=True, operator_action=f"support the object/arm; the {SIDE} arm is still powered at its lift goal. Recover through the existing manual recovery confirmation",
                trail=trail)


def limited_terminal(r, state, stop_requested=False):
    """Console-17: one terminal class for a LEFT_START_TO_LIFT session result, from the session's own result and the
    stage records (controller results). Unknown is not promoted to success."""
    res = (r or {}).get("result")
    if (r or {}).get("limited_terminal"):
        return r["limited_terminal"]
    stages = (state or {}).get("stages") or {}
    order = (state or {}).get("order") or list(stages)
    last = stages.get(order[-1]) if order else None
    for rec in [stages[n] for n in order if n in stages]:
        cr, gate = rec.get("controller_result"), (rec.get("stopped_by") or {}).get("gate")
        if cr in ("BUS_LOST", "FAULT_HOLDING", "FAULT_JOINTS_RELEASED", "EXECUTOR_EXCEPTION") or gate in ("bus_lost", "servo_fault"):
            return "BUS_OR_SERVO_FAULT"
    if r is None:
        return "BUS_OR_SERVO_FAULT"                                           # ended by an exception: not known to be healthy
    if stop_requested or any((rec.get("stopped_by") or {}).get("gate") == "operator_stop" for rec in stages.values()):
        return "OPERATOR_STOPPED"
    if (stages.get("close") or {}).get("controller_result") == "GRIP_UNAVAILABLE_HOLDING":
        return f"{(state or {}).get('side', 'left').upper()}_CLOSE_UNAVAILABLE"
    if "lift" in stages:
        return f"{(state or {}).get('side', 'left').upper()}_LIFT_FAILED"
    return "NOT_STARTED" if res in ("NOT_STARTED", "PLANNED") else "ENDED_BEFORE_LIFT"


T_SESSION0 = time.monotonic()


def session(out, *, execute=False, stage_fn=None, look_fn=None, observe_fn=None, second_stop=False, handoff_fn=None, limit=None):
    """PICK02 in one go: observe -> pre-grasp -> visual alignment loop -> first part of the descent (pads above the block) ->
    visual alignment loop at close range -> last part of the descent -> placement check -> close (Jaw) -> lift -> lower ->
    open -> retreat -> home. The visual loop is STEPWISE (stop, look, correct through the existing planner and executor, look
    again); frames recorded while a stage moves are read afterwards (monitoring), they do not change a command in motion. A stage is opened by its controller result AND its observation. When an observation says 'not ready'
    the arm goes back the planned way (open if the jaws were closed, retreat, home); when a controller ends in any other
    state (fault, bus loss, operator stop, no progress) nothing further is commanded."""
    stage_fn, look_raw, observe_fn = recorded_stage(stage_fn or (lambda o, n, execute=False: stage(o, n, execute=execute, cameras=wrist_cams))), look_fn or look, observe_fn or observe
    out = Path(out); trail = []; rec_errors = RECORDING_ERRORS
    def look_fn(o, tag):
        """a look that cannot be made (camera gone, reader error) is an observation failure, not a crash: the gates then say 'not ready' and the planned way back is taken"""
        try:
            return look_raw(o, tag)
        except Exception as e:
            trail.append(dict(step="look_failed", tag=tag, error=f"{type(e).__name__}: {e}")); (Path(o) / "SESSION.json").write_text(json.dumps(dict(run="PICK02", trail=trail), indent=1, default=str))
            return dict(tag=tag, ok=False, why=f"the look could not be made: {type(e).__name__}: {e}", wrist={}, arms=dict(left=dict(torque={})), object=None)
    def note(**k):
        print(f"[step +{time.monotonic() - T_SESSION0:.0f}s] " + " ".join(f"{a}={k[a]}" for a in ("step", "height", "k", "result", "controller_result", "judge_ok", "plan_ok", "end", "after") if k.get(a) is not None)
              + (f" why={str(k['why'])[:160]}" if k.get("why") else ""), flush=True)
        trail.append(k); (out / "SESSION.json").write_text(json.dumps(dict(run="PICK02", trail=trail), indent=1, default=str))
        if EPISODE is not None:
            try:
                EPISODE.event("trail", **{a: b for a, b in k.items() if a in ("step", "gate", "ok", "result", "controller_result", "label", "placement", "height", "k", "end", "why")})
            except Exception as e:                                             # the record's quality; the session's own trail is already written
                rec_errors.append(f"trail event: {type(e).__name__}: {e}")
    try:
        with EPISODE.lease("left", "right") if EPISODE is not None else contextlib.nullcontext():
            st = observe_fn(out)
    except Exception as e:                                                     # e.g. a camera that does not open: nothing has been commanded yet, so this is simply 'not started'
        import traceback
        out.mkdir(parents=True, exist_ok=True); note(step="observe", ready=False, why=f"initial observation or pick planning failed: {type(e).__name__}: {e}", traceback=traceback.format_exc())
        return dict(result="NOT_STARTED", why=f"{type(e).__name__}: {e}", trail=trail)
    note(step="observe", ready=st["ready"], why=st["why"], plan=None if not st.get("plan") else {k: st["plan"].get(k) for k in ("tilt_deg", "jaw", "jaw_lean_deg", "room_each_side_m", "considered")})
    if not st["ready"] or not execute:
        return dict(result="NOT_STARTED" if not st["ready"] else "PLANNED", trail=trail)
    S = lambda: json.loads((out / "STATE.json").read_text())
    if limit not in (None, LEFT_TO_LIFT) or (limit and handoff_fn is not None):
        raise ValueError(f"unsupported session limit {limit!r} (a limited session has no hand-over callback)")
    if handoff_fn is not None:
        # FULL-CHAIN mode: everything the hand-over needs is checked BEFORE the first motor command. A session whose right-arm
        # side is not ready is not started in this mode (the baseline mode is the one to run then).
        try:
            rd = handoff_fn.ready(out, st)
        except Exception as e:
            rd = dict(ready=False, missing=[f"the readiness check failed: {type(e).__name__}: {e}"])
        note(step="full_chain_readiness", ready=bool(rd.get("ready")), missing=rd.get("missing"), items=rd.get("items"))
        if not rd.get("ready"):
            return dict(result="NOT_STARTED_FULL_CHAIN_NOT_READY", mode="FULL_CHAIN", missing=rd.get("missing"), trail=trail)
    def run_stage(name, ok=HOLDING_OK):
        ok = ok + (("TARGET_NOT_REACHED_HOLDING",) if name in SHORT_OK else ())    # completed, settled and holding a little short of the joint target: go on to the observation that judges it
        r = stage_fn(out, name, execute=True); note(step=name, result=r["result"], controller_result=r.get("controller_result"), exit_code=r.get("exit_code"), why=r.get("why"))
        return r["result"] == "RAN" and r.get("controller_result") in ok
    def back(names):
        for n in names:
            if not run_stage(n, HOLDING_OK + ("RETURNED_RELEASED",)):
                # 184014_auto: 'retreat_arm' stalled (NO_PROGRESS_HOLDING) above the open block and the arm stayed there. A stalled way back
                # with the jaws not closed on anything goes home by the collision-checked 'abort_home'; a fault / bus loss stays with a person.
                last = (S()["stages"].get(n) or {}).get("controller_result")
                if last in ("NO_PROGRESS_HOLDING", "TARGET_NOT_REACHED_HOLDING") and "close" not in S()["stages"] and n != "abort_home":
                    r_ = stage_fn(out, "abort_home", execute=execute); note(step="abort_home", result=r_.get("result"), controller_result=r_.get("controller_result"), why=r_.get("why"), after=f"{n}: {last}")
                    if r_.get("controller_result") == "RETURNED_RELEASED":
                        return None
                return dict(result=f"ENDED_IN_{n.upper()}_NEEDS_A_PERSON", trail=trail)
        return None
    def end(result, names):
        return back(names) or dict(result=result, trail=trail)
    if not run_stage("pregrasp"):
        return dict(result="STOPPED_AT_PREGRASP_HOLDING_NEEDS_A_PERSON", trail=trail)

    last_look = {}

    def visual_align(prefix, gate):
        """STEPWISE visual loop at one height: look -> image error -> short corrected joint target through the existing
        planner / trajectory / collision checks -> move -> look again. The image response (px per mm of tool motion along the
        jaw axis) is MEASURED from each move of this session and replaces the PICK01 prior; a response of the wrong sign, an
        error that does not shrink by more than the image can resolve, or an unreadable image ends the loop. Returns
        'ALIGNED', 'NOT_ALIGNED' (go back the planned way) or 'STOPPED' (a controller ended abnormally)."""
        prev, gain, src, k = None, None, "prior", 0
        yaw_steps = [0]
        view = taught_view(); y_sign = [1.0]; prev_ey = [None]; prev_pure_across = [False]   # the hand-taught centred view: image rows -> across-jaw error (2026-10-18)
        while True:
            if k >= ALIGN_MAX_LOOKS:
                note(step="visual", height=prefix, k=k, end=f"{ALIGN_MAX_LOOKS} looks at this height without the gate passing"); return "NOT_ALIGNED"
            m = dict(look_called=time.monotonic())                            # 2026-10-15 latency record: recording only, nothing below reads it
            sc = look_fn(out, f"{prefix}_{k}"); m["look_returned"] = time.monotonic()
            ok, ev = judge(out, S(), gate, sc); off = ev.get("wrist_offset") or {}; m["judged"] = time.monotonic()
            err_px, err_mm = ev.get("lateral_error_px"), ev.get("lateral_error_mm")
            row = dict(step="visual", loop="STEPWISE_VISUAL_LOOP", height=prefix, k=k, gate=gate, ok=ok, why=ev["why"], frames=(sc.get("wrist") or {}).get("frames"), offset_px=off.get("offset_px"), offset_mm=off.get("offset_mm"), lateral_error_mm=err_mm, lateral_target_mm=ev.get("lateral_target_mm"), uncertainty_mm=off.get("uncertainty_mm"),
                       checks={k_: (c_.get("ok"), c_.get("unknown")) for k_, c_ in (ev.get("checks") or {}).items()},
                       allowed_mm=ev.get("tolerance_mm"), reference_confirmed=off.get("reference_confirmed_in_this_frame"), tool_xy_m=(sc.get(f"tool_{SIDE}") or [None, None])[:2], joints_read=sc.get("joint_reads"), not_controlled=["image y (along the pads' width)", "yaw of the block against the jaws", "height (checked, not servoed)"])
            row["timing_monotonic_s"] = m; row["look_timing_monotonic_s"] = sc.get("timing_monotonic_s")
            if prev is not None:
                row["previous_align_stage_monotonic_s"] = prev.get("stage_timing")
            tool_now = sc.get(f"tool_{SIDE}"); axis_now = sc.get("jaw_axis_xy")
            if prev is not None and off.get("readable") and tool_now is not None and axis_now is not None and not off.get("border_bound") and not prev["off"].get("border_bound"):   # a border lower bound is not a position: no response is measured from it
                dmm = 1000.0 * float(np.dot(np.array(tool_now[:2]) - np.array(prev["tool"][:2]), np.array(prev["axis"])))     # the tool's displacement projected on the ONE axis the correction was made along
                turn = float(np.degrees(np.arccos(np.clip(np.dot(np.array(axis_now), np.array(prev["axis"])), -1.0, 1.0))))
                dpx = err_px - prev["err_px"]; row["axis"] = dict(used=prev["axis"], now=axis_now, turned_deg=turn)
                if turn > WRIST["slope_tolerance_deg"]:
                    gain, src = None, "prior (the jaw axis turned between the two looks: the measured response is not carried over)"
                row["response"] = dict(tool_moved_mm=dmm, block_moved_px=dpx, commanded_mm=prev["commanded"], px_per_mm=(dpx / dmm if abs(dmm) > 1e-6 else None), expected_sign="+ (PICK01 record)", frame="world xy projected on the axis the correction used")
                if abs(dpx) > WRIST["measure_px"] and dpx * dmm < 0:
                    row["end"] = "the image moved the opposite way to the record for this move: the image-to-motion relation is not identified here"; note(**row); return "NOT_ALIGNED"
                if abs(dmm) > 0.5 and abs(dpx) > WRIST["measure_px"] and turn <= WRIST["slope_tolerance_deg"]:
                    gain, src = dpx / dmm, f"measured in this session (move {k - 1} at this height)"
            last_look.update(scene=sc, evidence=ev)
            alignment = current_wrist_alignment(S()["plan"], sc)
            row["square"] = alignment
            ey_mm = None; up_move = 0.0; height_bad = False
            if view is not None:
                import cv2
                b_ = red_blob(cv2.imread(str(out / "looks" / str(sc.get("tag")) / f"{SIDE}_wrist.jpg")))
                if b_ is not None and prefix == "01_pregrasp":                 # height from the image size against the taught pre-grasp view (a block cut by the border gives a lower bound)
                    r_ = (b_["area"] / view["pregrasp"]["area"]) ** 0.5; height_bad = abs(r_ - 1.0) > HEIGHT_TOL_RATIO
                    up_move = float(np.clip(TAUGHT_PREGRASP_CAMERA_DISTANCE_M * (1.0 - 1.0 / r_) * 1000.0, -20.0, 20.0)) if height_bad else 0.0
                    row["height"] = dict(area=b_["area"], taught_area=view["pregrasp"]["area"], size_ratio=r_, cut_by_border=b_["top"] <= 1, up_mm=up_move)
                key_ = across_feature(b_, view) if b_ is not None else None
                if b_ is not None and key_ is None:
                    row["across"] = dict(unknown="both the far and the near edge of the block are cut: too close to read the across error")
                if key_ is not None:
                    want, frac = taught_row(view, b_["area"], key_); mmpp = 40.0 / max(b_["area"] ** 0.5, 1.0)
                    ey_mm = float((b_[key_] - want) * mmpp)
                    row["across"] = dict(feature=key_, row=b_[key_], taught_row=want, between_taught=frac, mm_per_px=mmpp, error_mm=ey_mm, sign=y_sign[0])
                    if prev_pure_across[0] and prev_ey[0] is not None and abs(prev_ey[0]) > ACROSS_TOL_MM and abs(ey_mm) > abs(prev_ey[0]) + 1.0:   # judged only after a move that was across only
                        if y_sign[0] > 0:
                            y_sign[0] = -1.0; row["across"]["sign_flipped"] = "the across error grew after the last move: the image-row to motion sign is the other one"
                        else:
                            row["end"] = "the across error grew with both signs"; note(**row); return "NOT_ALIGNED"
            across_bad = (ey_mm is not None and abs(ey_mm) > ACROSS_TOL_MM) or height_bad
            wo = sc.get("wrist", {}).get("object") or {}
            if (prefix == "01_pregrasp" and yaw_steps[0] < 2 and alignment.get("check") == "WRIST_EDGE_ANGLE" and not alignment["ok"]
                    and not wo.get("touches_border")                           # an outline cut by a side border gives a wrong angle (175957: 30.7 deg then -11.9): centre first
                    and tool_now is not None and axis_now is not None):
                # square the jaws to the block first (R1 #4): the block's yaw becomes (current target) - (angle in the image); the
                # sign is the one seen in four sessions (image angle grew with the jaw axis). Then a new look; the lateral loop restarts.
                yaw_now = float(S()["plan"]["object_observed_yaw_rad"]); new = yaw_now - np.radians(wo["edge_angle_deg"])
                row["yaw_correction"] = dict(edge_angle_deg=wo["edge_angle_deg"], edge_len_px=wo.get("edge_angle_len_px"), yaw_from_deg=float(np.degrees(yaw_now)), yaw_to_deg=float(np.degrees(new)))
                c = correct(out, 0.0, f"look {sc.get('tag')}: block sides at {wo['edge_angle_deg']:+.1f} deg to the jaw axis", axis_xy=axis_now, yaw_rad=new); row["plan_ok"] = c.get("plan_ok"); note(**row)
                if not c.get("plan_ok"):
                    return "NOT_ALIGNED"
                if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):
                    return "STOPPED"
                yaw_steps[0] += 1; prev = None; k += 1; continue
            if "unknown" in alignment and not off.get("border_bound"):         # a block across the border has no edge to read yet: the capped step comes first, then a new look
                row["end"] = alignment["unknown"]; note(**row); return "NOT_ALIGNED"
            if ok and not across_bad:
                note(**row); _rec(lambda: vservo_shadow_input(out, row, sc), "vservo shadow input"); return "ALIGNED"
            if not ev.get("correctable") and not (across_bad and off.get("readable")):
                row["end"] = "not a lateral offset that a correction along the jaw axis can remove (or not readable)"; note(**row); return "NOT_ALIGNED"
            if prev is not None and not across_bad and abs(err_mm) > abs(prev["err_mm"]) - off["uncertainty_mm"]:
                row["end"] = f"the error did not shrink by more than the image resolves ({prev['err_mm']:+.1f} -> {err_mm:+.1f} mm)"; note(**row); return "NOT_ALIGNED"
            g = gain if gain else image_response_prior(ev["wrist_object"]); move = -err_px / g if (err_px is not None and not ok) else 0.0
            if off.get("border_bound"):                                       # a lower bound across the border: one capped step, then a new look
                cap = WRIST.get("border_step_cap_mm", 15.0); move = float(np.clip(move, -cap, cap))
            row["correction"] = dict(tool_move_along_jaw_mm=move, px_per_mm=g, relation=src, basis="local 1-axis image response; axis = the jaw axis (image x). Sign from the PICK01 record, checked against every move")
            if tool_now is None or axis_now is None:
                row["end"] = "the tool position / jaw axis of this look is unknown (no frame chain): no correction is computed"; note(**row); return "NOT_ALIGNED"
            m["correction_computed"] = time.monotonic()
            # all axes in ONE move (175957: one axis at a time took six looks and the along error never got its turn)
            move = float(np.clip(move, -25.0, 25.0))
            across_move = float(np.clip(-y_sign[0] * ey_mm, -20.0, 20.0)) if (ey_mm is not None and abs(ey_mm) > ACROSS_TOL_MM) else 0.0     # block higher in the image than taught = further forward: move forward
            prev_ey[0] = ey_mm; prev_pure_across[0] = bool(across_move and not move and not up_move)
            row["correction"]["tool_move_across_mm"] = across_move; row["correction"]["tool_move_up_mm"] = up_move
            c = correct(out, move, f"wrist image {sc.get('tag')}: block {err_px or 0:+.1f} px along / {ey_mm or 0:+.1f} mm across / up {up_move:+.1f} mm against the taught centred view; {g:.2f} px/mm ({src})", axis_xy=axis_now, across_mm=across_move, up_mm=up_move); row["plan_ok"] = c.get("plan_ok"); m["planned"] = time.monotonic(); note(**row)
            _rec(lambda: vservo_shadow_input(out, row, sc), "vservo shadow input")
            if not c.get("plan_ok"):
                return "NOT_ALIGNED"
            stage_timing = dict(stage_called=time.monotonic())
            if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):     # short of the joint target but completed and holding: the next look measures where the hand really is
                return "STOPPED"
            stage_timing["stage_returned"] = time.monotonic()
            prev = dict(off=off, err_px=err_px, err_mm=err_mm, tool=list(tool_now), axis=list(axis_now), commanded=move, stage_timing=stage_timing); k += 1

    def visual_align_je(prefix, gate):
        """IMAGE-JACOBIAN alignment (2026-10-18): features s = [block centre x, across row (uncut edge or centre), log area,
        outline angle] against the hand-taught view at the same height s*; one move per look for all four tool motions
        v = [along jaw, across (forward), up, wrist yaw]:  v = -lambda (J'WJ + mu)^-1 J'W (s - s*). J starts from physics
        (block size in px, camera distance, yaw -1), is corrected after every move by Broyden from the MEASURED tool motion
        (model FK of the read joints) and the measured image change, and is kept per arm and height for the next session."""
        import cv2, mujoco
        jf = HERE / f"sweepick_image_jacobian_{SIDE}.json"
        store = json.loads(jf.read_text()) if jf.exists() else {}
        key_h = "pregrasp" if prefix == "01_pregrasp" else "above"; tv = view_[key_h]
        mm_ = model(); dd_ = mujoco.MjData(mm_); mujoco.mj_forward(mm_, dd_); base_xy = dd_.xpos[mm_.body(f"{SIDE}_base").id][:2].copy()
        LAM, CAPS = 0.7, np.array([25.0, 20.0, 20.0, 10.0]); FEATS = ("cx", "row", "logA", "ang")
        # which tool motion can change which feature (rows cx,row,logA,ang x cols along,across,up,yaw). A Broyden step after a combined
        # move spreads the error over every column (183317: the angle row picked up along/across/up terms); only these entries are learnt.
        MASK = np.array([[1, 0, 0, 1], [0, 1, 1, 1], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        def prior(area):
            g = (area ** 0.5) / 40.0; dist = TAUGHT_PREGRASP_CAMERA_DISTANCE_M * 1000.0 * (1.0 if key_h == "pregrasp" else (view_["pregrasp"]["area"] / view_["above"]["area"]) ** 0.5)
            return np.array([[g, 0, 0, 0], [0, g, 0, 0], [0, 0, -2.0 / dist, 0], [0, 0, 0, IMAGE_ANGLE_PER_YAW[SIDE]]], float)
        J = np.array(store[key_h]["J"], float) if key_h in store else None; src_j = "stored" if J is not None else "prior"
        prev = None; worse = 0; k = 0
        while True:
            if k >= ALIGN_MAX_LOOKS:
                note(step="visual", height=prefix, k=k, end=f"{ALIGN_MAX_LOOKS} looks without the gate passing"); return "NOT_ALIGNED"
            sc = look_fn(out, f"{prefix}_{k}"); ok, ev = judge(out, S(), gate, sc)
            last_look.update(scene=sc, evidence=ev)                            # 185636_auto: the step after the loop reads the last look (KeyError 'evidence' with the arm above the block)
            b = red_blob(cv2.imread(str(out / "looks" / str(sc.get("tag")) / f"{SIDE}_wrist.jpg")))
            tool_now, axis_now = sc.get(f"tool_{SIDE}"), sc.get("jaw_axis_xy")
            row = dict(step="visual", loop="IMAGE_JACOBIAN", height=prefix, k=k, gate=gate, judge_ok=ok, why=ev.get("why"), lateral_error_mm=ev.get("lateral_error_mm"), jacobian_source=src_j)
            if b is None or tool_now is None or axis_now is None:
                row["end"] = "no block in the wrist image or no tool pose"; note(**row); return "NOT_ALIGNED"
            if J is None:
                J = prior(b["area"])
            # 191754_auto: a learnt along gain (stored from earlier sessions) of about a third of the true one turned a 26 px error into a 21 mm move and an 86 px
            # overshoot. The sideways gains are not learnt: they are the block's own size in the image, px per mm at its depth (40 mm block), every look
            # (measured there: 86 px for 20.7 mm = 4.2 px/mm, the block size gave 4.2).
            J[0], J[1] = prior(b["area"])[0], prior(b["area"])[1]
            f, t = {}, {}
            l_cut, r_cut = b["left"] <= 1, b["right"] >= 318                  # along the jaw: the centre when whole, else the side edge that is NOT cut (a cut side would stop the along correction)
            ck = "cx" if not (l_cut or r_cut) else ("right" if l_cut and not r_cut else ("left" if r_cut and not l_cut else None))
            if ck:
                f["cx"] = float(b[ck]); t["cx"] = taught_row(view_, b["area"], ck)[0]; row_ck = ck
            rk = across_feature(b, view_)
            if rk:
                f["row"] = float(b[rk]); t["row"] = taught_row(view_, b["area"], rk)[0]
            if key_h == "pregrasp":                                             # above the block the height is the plan's (pads above the top face by the joints), not the image size (184014_auto)
                f["logA"] = float(np.log(b["area"])); t["logA"] = float(np.log(tv["area"]))
            # 2026-10-18 182311_auto: the block turned ~14 deg in the wrist image but cut at top and right -> no yaw feature, the wrist never turned.
            # A cut outline's angle is used when the seen outline is square enough (edge coherence), with half the weight of an uncut one.
            cut_ = b["left"] <= 1 or b["right"] >= 318 or b["top"] <= 1 or b["bottom"] >= 238
            if b.get("angle") is not None and b.get("angle_len", 0) >= WRIST_EDGE_MIN_LEN_PX and tv.get("angle") is not None and (not cut_ or (b.get("angle_coherence") or 0.0) >= EDGE_COHERENCE_MIN):
                f["ang"] = float(b["angle"]); t["ang"] = float(tv["angle"]); row["angle_from_cut_outline"] = bool(cut_)
            e = {q: f[q] - t[q] for q in f}; g = (b["area"] ** 0.5) / 40.0
            scale = dict(cx=2.0 * g, row=3.0 * g, logA=0.05, ang=4.0 if row.get("angle_from_cut_outline") else 2.0)
            enorm = float(np.sqrt(sum((e[q] / scale[q]) ** 2 for q in e)))
            row.update(features=f, targets=t, errors=e, error_norm=enorm, row_feature=rk, col_feature=ck)
            # 184014_auto: above the block the judge passed at the first look (centred, aligned, pads above the top face by the joints) but this
            # loop's own size target (+20 % area) sent the hand UP, after which the judge's 'approached' check failed for 7 looks. The judge is the
            # gate for the jaw placement along the jaw and for the height; the image adds what the judge does not see: the across row, the angle,
            # an uncut block, and (pre-grasp only) a size that is not grossly off.
            # 193016_auto: looks 6/7 matched the taught view (block centred within 3 mm, size +3 %, angle 0.5 deg) but the judge's own centring
            # (jaw-midline reference) and the model's pad check did not pass, and 2 mm moves are below what the arm executes: 8 looks, home.
            # With a taught view the gate is that view: the block where it was when the person centred the jaws, within the pad room
            # (less 0.5 mm), across within 5 mm, angle within the slope tolerance, block uncut; above the block also the pads above the top face.
            room_mm = float(S()["plan"].get("room_each_side_m") or 0.003) * 1000.0
            # the pre-grasp stop is coarse (the stop above the block is aligned again, closer): 5 mm there; above the block the pad room less 0.3 mm
            cx_tol_mm = ACROSS_TOL_MM if key_h == "pregrasp" else max(1.5, room_mm - 0.3)
            # review 2026-10-19: a feature that was not measured is UNKNOWN, never a zero error. The across row is required; the angle is required
            # unless the judge's own alignment check passed on this look (203018 above: no outline angle, judge alignment OK); above the block the
            # pads-above-the-top evidence must be present and True.
            ca_now = ((ev.get("checks") or {}).get("current_alignment") or {})
            row_ok = "row" in e and abs(e["row"]) / g <= ACROSS_TOL_MM
            ang_ok = (abs(e["ang"]) <= WRIST["slope_tolerance_deg"]) if "ang" in e else (ca_now.get("ok") is True)
            fine = (row_ok and ang_ok and ck == "cx" and "cx" in e and abs(e["cx"]) / g <= cx_tol_mm
                    and (key_h != "pregrasp" or ("logA" in e and abs(e["logA"]) <= np.log(1.35))))
            above_ok = (((ev.get("checks") or {}).get("pads_above_the_top_face") or {}).get("ok") is True) if gate == "ready_for_final_descent" else True
            row["taught_view_gate"] = dict(fine=bool(fine), cx_mm=float(abs(e["cx"]) / g) if "cx" in e else None, allowed_mm=cx_tol_mm, pads_above_top=above_ok, judge_ok=ok,
                                           row="UNKNOWN" if "row" not in e else bool(row_ok), angle=("judge alignment" if "ang" not in e else "taught view") + (" OK" if ang_ok else " not OK"))
            if fine and above_ok:
                if not ok:                                                      # the descent stages read the last ACCEPTED evidence: record this acceptance as such
                    st_ = S(); st_.setdefault("evidence", []).append(dict(ev, what=gate, ok=True, accepted_by="taught-view gate (image Jacobian loop)", judge_ok=False)); (out / "STATE.json").write_text(json.dumps(st_, indent=1, default=str))
                note(**row); return "ALIGNED"
            # Broyden: the image change against the tool motion actually made (read joints through the model)
            if prev is not None:
                ax0 = np.array(prev["axis"]); perp = np.array([-ax0[1], ax0[0]]); rad = np.array(prev["tool"][:2]) - base_xy
                if float(np.dot(perp, rad)) < 0: perp = -perp
                dtool = np.array(tool_now) - np.array(prev["tool"])
                dyaw = float(np.degrees(np.arctan2(axis_now[1], axis_now[0]) - np.arctan2(ax0[1], ax0[0]))); dyaw = (dyaw + 90.0) % 180.0 - 90.0
                dv = np.array([float(np.dot(dtool[:2], ax0)) * 1000, float(np.dot(dtool[:2], perp)) * 1000, float(dtool[2]) * 1000, dyaw])
                if float(np.dot(dv, dv)) > 1.0:
                    for i_, q in enumerate(FEATS):
                        if q in ("cx", "row"):
                            continue                                           # sideways gains come from the block size every look (191754_auto)
                        if q in f and q in prev["f"] and (q != "cx" or prev.get("ck") == ck) and (q != "row" or prev.get("rk") == rk):   # the same feature definition at both looks
                            ds = f[q] - prev["f"][q]
                            if q == "ang":                                     # the image angle answers to the wrist yaw alone: its gain is the measured ratio (sign included) when the wrist turned enough
                                if abs(dv[3]) >= 3.0:
                                    J[3] = np.array([0.0, 0.0, 0.0, float(np.clip(ds / dv[3], -2.0, 2.0))]); row["measured_angle_per_yaw"] = float(ds / dv[3])
                                continue
                            # 184014_auto: 1..3 mm moves (the arm's own settle error is ~1..2 mm) flipped the left along->cx gain from +2.6 to -7.7, the loop then
                            # moved the wrong way and crept. Only motions of at least 4 mm / 3 deg teach a column, and the physical entries (along->cx,
                            # across->row, up->logA) keep the sign and stay within 0.3..3x of the optics prior.
                            dm = dv * MASK[i_] * (np.abs(dv) >= np.array([4.0, 4.0, 4.0, 3.0]))
                            if float(dm @ dm) > 1e-9:
                                J[i_] = J[i_] + (ds - float(J[i_] @ dv)) * dm / float(dm @ dm)
                                p0 = prior(b["area"])[i_, i_]
                                if i_ < 3 and p0 != 0:
                                    J[i_, i_] = float(np.sign(p0) * np.clip(abs(J[i_, i_]), 0.3 * abs(p0), 3.0 * abs(p0))) if np.sign(J[i_, i_]) == np.sign(p0) else p0
                    src_j = "updated in this session"; row["measured_tool_motion"] = dv.tolist()
                    store[key_h] = dict(J=J.tolist(), t=time.strftime("%Y-%m-%dT%H:%M:%S"), note="image-Jacobian rows cx,row,logA,ang x cols along,across,up,yaw (mm, deg)")
                    _rec(lambda: jf.write_text(json.dumps(store, indent=1)), "image jacobian store")
                # 195343_auto: the angle feature appeared at the second look (43 deg) and the norm 'grew' although every error that was there before
                # shrank: the comparison is made on the features both looks have, with the same definitions
                # 214724_auto (right, 45 deg): the sideways error fell (cx -31 -> -2 px) but the deliberate rises before sideways moves made the block look
                # smaller (log area -0.54) and that alone read as 'grew twice'. Height is not a growth criterion here: only the sideways and angle features.
                common = [q for q in e if q in prev["e"] and q != "logA" and (q != "cx" or prev.get("ck") == ck) and (q != "row" or prev.get("rk") == rk)]
                n_now = float(np.sqrt(sum((e[q] / scale[q]) ** 2 for q in common))); n_prev = float(np.sqrt(sum((prev["e"][q] / prev["scale"][q]) ** 2 for q in common)))
                row["error_norm_common"] = dict(features=common, now=n_now, before=n_prev)
                worse = worse + 1 if (common and n_now > n_prev * 1.05) else 0
                if worse >= 2:
                    row["end"] = "the image error grew twice in a row"; note(**row); return "NOT_ALIGNED"
            idx = [i_ for i_, q in enumerate(FEATS) if q in e]
            Js = J[idx]; ev_ = np.array([e[FEATS[i_]] for i_ in idx]); Wd = np.diag([1.0 / scale[FEATS[i_]] ** 2 for i_ in idx])
            A = Js.T @ Wd @ Js + 1e-3 * np.diag(1.0 / CAPS ** 2) * 100.0
            # 2026-10-18 (user): the 10 deg yaw step was a guard from before the per-arm sign was measured (183317). With the sign measured and the
            # outline whole (not cut), a turned block (45 deg) is corrected in one or two looks: up to 30 deg; a cut outline keeps 10 deg.
            caps_ = CAPS.copy(); caps_[3] = YAW_STEP_WHOLE_DEG if (row.get("angle_from_cut_outline") is False and "ang" in e) else CAPS[3]
            v = -LAM * np.linalg.solve(A, Js.T @ Wd @ ev_); v = np.clip(v, -caps_, caps_)
            row["correction"] = dict(tool_move_along_jaw_mm=float(v[0]), tool_move_across_mm=float(v[1]), tool_move_up_mm=float(v[2]), yaw_deg=float(v[3]), jacobian=J.tolist())
            yaw_new = float(S()["plan"]["object_observed_yaw_rad"] + np.radians(v[3])) if abs(v[3]) >= 1.0 else None
            # 190907_auto: a 25 mm along + 20 mm across move at the pre-grasp height (model: pads 4 cm above the top face) pushed the block ~10 cm away
            # and turned it ~30 deg. Whatever the vertical error of the model there, a hand that still has to go sideways near the block goes UP first:
            # no downward step while the sideways error is more than 5 mm, and a sideways step of more than 8 mm is made only after a separate 20 mm rise.
            lat = float(np.hypot(v[0], v[1]))
            if v[2] < 0 and lat > SIDEWAYS_BEFORE_DOWN_MM:
                row["down_held"] = dict(up_mm_computed=float(v[2]), sideways_mm=lat); v[2] = 0.0
            if lat > SIDEWAYS_NEEDS_RISE_MM:
                c0 = correct(out, 0.0, f"rise before a {lat:.0f} mm sideways move (look {sc.get('tag')})", axis_xy=axis_now, up_mm=RISE_BEFORE_SIDEWAYS_MM)
                row["rise_first"] = dict(up_mm=RISE_BEFORE_SIDEWAYS_MM, plan_ok=c0.get("plan_ok"))
                if not c0.get("plan_ok"):
                    row["plan_ok"] = False; note(**row); return "NOT_ALIGNED"
                if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):
                    note(**row); return "STOPPED"
                v[2] = max(float(v[2]) - RISE_BEFORE_SIDEWAYS_MM, 0.0)
            c = correct(out, float(v[0]), f"image Jacobian look {sc.get('tag')}: |e| {enorm:.2f}", axis_xy=axis_now, across_mm=float(v[1]), up_mm=float(v[2]), yaw_rad=yaw_new)
            row["plan_ok"] = c.get("plan_ok"); note(**row)
            if not c.get("plan_ok"):
                return "NOT_ALIGNED"
            if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):
                return "STOPPED"
            prev = dict(f=f, e=dict(e), scale=dict(scale), enorm=enorm, tool=list(tool_now), axis=list(axis_now), ck=ck, rk=rk); k += 1

    view_ = taught_view()
    align = visual_align_je if view_ is not None else visual_align

    def watch_grip(seconds):
        """Hold in the carry pose and watch the jaws (read-only): a block slipping out lets the held gripper close further and its load fall.
        review 2026-10-19: the result is a STATUS - HELD (enough valid reads, no slip), DROPPED (two slip reads in a row), FAULT (gripper torque
        off or a servo status bit), STOP (operator stop) or UNKNOWN (no reference read, too few valid reads, or a gap without reads longer than
        GRIP_WATCH_MAX_GAP_S). 'Not dropped' is never inferred from reads that failed."""
        from sweepick_local_loop import open_readonly_bus
        rows, bad, start, valid, last_ok = [], 0, None, 0, time.monotonic(); t0 = time.monotonic(); t_end = t0 + seconds; gap_max = 0.0
        stop_f = Path(str(out) + ".STOP")
        def res(status, **kw):
            return dict(status=status, dropped=status == "DROPPED", start=start, rows=rows, valid_reads=valid, error_reads=len(rows) - valid, longest_gap_s=gap_max, **kw)
        while time.monotonic() < t_end:
            if stop_f.exists():
                return res("STOP")
            try:
                with EPISODE.lease("left", "right") if EPISODE is not None else contextlib.nullcontext():
                    b_ = open_readonly_bus(SIDE)
                    try:
                        pos = float(b_.sync_read("Present_Position", normalize=False)["gripper"]); load = float(b_.sync_read("Present_Load", normalize=False)["gripper"])
                        tq = int(b_.sync_read("Torque_Enable", normalize=False)["gripper"]); stv = int(b_.sync_read("Status", normalize=False)["gripper"])
                    finally:
                        b_.disconnect()
            except Exception as e:                                             # a lost read is not a drop and not a hold: counted apart
                rows.append(dict(t=time.time(), error=f"{type(e).__name__}: {e}")); gap_max = max(gap_max, time.monotonic() - last_ok); time.sleep(0.2); continue
            gap_max = max(gap_max, time.monotonic() - last_ok); last_ok = time.monotonic(); valid += 1
            rows.append(dict(t=time.time(), pos=pos, load=load, torque=tq, status=stv))
            if tq != 1 or stv != 0:
                return res("FAULT", why=f"gripper torque {tq}, status {stv}")
            start = start or dict(pos=pos, load=abs(load) or 1.0)
            slip = pos < start["pos"] - 25 or abs(load) < 0.4 * start["load"]; bad = bad + 1 if slip else 0
            if bad >= 2:
                return res("DROPPED")
            time.sleep(0.2)
        gap_max = max(gap_max, time.monotonic() - last_ok)
        if start is None or valid < GRIP_WATCH_MIN_VALID or gap_max > GRIP_WATCH_MAX_GAP_S:
            return res("UNKNOWN", why=f"valid reads {valid}, longest gap {gap_max:.2f} s")
        return res("HELD")

    def stalled_home(label):
        """2026-10-18 (user): an arm move that stalled ABOVE the block with the jaws never closed (NO_PROGRESS / short of
        target, holding) goes home by itself: the manual 'abort_home' path (collision-checked) and release. A fault or a
        bus loss still leaves everything to a person."""
        last = (S()["stages"].get((S().get("order") or [None])[-1]) or {}).get("controller_result")
        if last in ("NO_PROGRESS_HOLDING", "TARGET_NOT_REACHED_HOLDING") and "close" not in S()["stages"]:
            r_ = stage_fn(out, "abort_home", execute=execute); note(step="abort_home", result=r_.get("result"), controller_result=r_.get("controller_result"), why=r_.get("why"), after=last)
            if r_.get("controller_result") in ("RETURNED_RELEASED",):
                return dict(result=f"{label}_STALLED_RETURNED_HOME", trail=trail)
        return dict(result=f"STOPPED_IN_{label}_HOLDING_NEEDS_A_PERSON", trail=trail)

    r1 = align("01_pregrasp", "ready_to_descend")
    if r1 != "ALIGNED":
        return end("NOT_ALIGNED_RETURNED_HOME", ["home_arm"]) if r1 == "NOT_ALIGNED" else stalled_home("ALIGN")
    if not run_stage("descend_a"):
        return stalled_home("DESCEND_A")
    r2 = align("01a_above_block", "ready_for_final_descent")
    if r2 != "ALIGNED":
        return end("NOT_READY_ABOVE_THE_BLOCK_RETURNED_HOME", ["retreat_arm", "home_arm"]) if r2 == "NOT_ALIGNED" else stalled_home("ALIGN_ABOVE")
    # a second stop closer to the top face, as close as what was MEASURED at the first stop allows: how far the tool really stood from where that stop was commanded (vertically), plus 1 mm for the block's own height
    pm, pa = (last_look["evidence"].get("pads_by_measured_joints") or {}), (S()["plan"].get("placement_above_block") or {})
    # second_stop is OFF by default: on the real arm a few-millimetre move of loaded joints does not start reliably (runs 10 and 11), and the first stop
    # already has the pads above the top face with the block centred; the last descent then starts from an accepted observation
    if second_stop and pm and pa and last_look["scene"].get("jaw_axis_xy") is not None:
        seen_err = abs(pm["tool_above_top_mm"] - pa["tool_above_top_mm"]); now_clear = min(pm["pads"][n]["lowest_above_top_mm"] for n in pad_geoms()); want = seen_err + 1.0
        note(step="next_descent", measured_vertical_error_mm=seen_err, pads_above_top_now_mm=now_clear, next_stop_pads_above_top_mm=want, basis="the vertical error of the measured joints at this stop against its commanded pose, plus 1 mm; not a fixed distance")
        if want < now_clear - 1.0:                                             # otherwise the first stop is already as close as that uncertainty allows
            c = correct(out, 0.0, f"second stop above the block: pads {want:.1f} mm above the top face (measured vertical error {seen_err:.1f} mm + 1 mm)", axis_xy=last_look["scene"]["jaw_axis_xy"], above_b=want / 1000.0)
            note(step="plan_second_stop", plan_ok=c.get("plan_ok"), why=c.get("why"))
            if not c.get("plan_ok"):
                return end("SECOND_STOP_NOT_PLANNABLE_RETURNED_HOME", ["retreat_arm", "home_arm"])
            # the first stop's pose is unchanged by this (no move here); the observation just accepted at it still stands for the next stage
            if not run_stage("descend_b"):
                return dict(result="STOPPED_IN_DESCEND_B_HOLDING_NEEDS_A_PERSON", trail=trail)
            r3 = visual_align("01b_just_above_block", "ready_for_final_descent")
            if r3 != "ALIGNED":
                return end("NOT_READY_JUST_ABOVE_THE_BLOCK_RETURNED_HOME", ["retreat_arm", "home_arm"]) if r3 == "NOT_ALIGNED" else dict(result="STOPPED_IN_ALIGN_HOLDING_NEEDS_A_PERSON", trail=trail)
    if not run_stage("descend"):
        return dict(result="STOPPED_IN_DESCEND_HOLDING_NEEDS_A_PERSON", trail=trail)
    sc = look_fn(out, "02_descended"); ok, ev = judge(out, S(), "ready_to_close", sc); note(step="look", gate="ready_to_close", ok=ok, why=ev["why"], placement=ev.get("placement"), checks=ev.get("checks"), unknown=ev.get("unknown"))
    if not ok and ev.get("block_undisturbed_in_the_image") and (S()["stages"].get("descend") or {}).get("controller_result") == "TARGET_NOT_REACHED_HOLDING":
        # the descent stopped short of its joint target (loaded joints do) and the image shows the block where it was, not turned: the shortfall is the arm's own.
        # The same target once more, this time with the checked settle trim; then judged again.
        st_ = S(); st_.setdefault("evidence", []).append(dict(what="block_undisturbed_after_descent", ok=True, look=sc.get("tag"), wrist_object=ev.get("wrist_object"), basis="wrist image after the descent: block visible, the same in consecutive frames, not turned")); (out / "STATE.json").write_text(json.dumps(st_, indent=1, default=str))
        if not run_stage("descend_settle"):
            return dict(result="STOPPED_IN_DESCEND_SETTLE_HOLDING_NEEDS_A_PERSON", trail=trail)
        sc = look_fn(out, "02b_descended_settled"); ok, ev = judge(out, S(), "ready_to_close", sc); note(step="look", gate="ready_to_close", ok=ok, why=ev["why"], placement=ev.get("placement"), checks=ev.get("checks"), unknown=ev.get("unknown"))
    if not ok:
        return end("PAD_PLACEMENT_NOT_CONFIRMED_RETURNED_HOME", ["retreat_arm", "home_arm"])
    if not run_stage("close", CLOSE_OK):
        r = S()["stages"].get("close") or {}
        return end("NO_CONTACT_RETURNED_HOME", ["open", "retreat", "home"]) if r.get("controller_result") == "GRIP_NO_CONTACT_HOLDING" else dict(result="STOPPED_IN_CLOSE_HOLDING_NEEDS_A_PERSON", trail=trail)
    sc = look_fn(out, "03_closed"); ok, ev = judge(out, S(), "ready_to_lift", sc); note(step="look", gate="ready_to_lift", ok=ok, why=ev["why"], label=ev.get("label"))
    if not ok:
        return end("CONTACT_WITHOUT_THE_BLOCK_SEEN_RETURNED_HOME", ["open", "retreat", "home"])
    if not run_stage("lift"):
        return dict(result="STOPPED_IN_LIFT_HOLDING_NEEDS_A_PERSON", trail=trail)
    sc = look_fn(out, "04_lifted"); held, ev = judge(out, S(), "held_after_lift", sc); note(step="look", gate="held_after_lift", ok=held, why=ev["why"], label=ev.get("label"))
    try:
        snapshot(out, sc, ev)
        note(step="snapshot", file="HANDOFF_SNAPSHOT.json", kind="SNAPSHOT_ONLY")
    except Exception as e:
        note(step="snapshot", error=f"{type(e).__name__}: {e}")
    if limit == LEFT_TO_LIFT:
        note(step="limited_scope_terminal", result="LEFT_LIFT_HELD_CONFIRMED" if held else "LEFT_LIFT_HOLD_UNKNOWN", gate="held_after_lift", ok=held,
             why="LEFT_START_TO_LIFT ends here: no CARRY/hand-over, no automatic lower/open/retreat/home/OFF")
        return limited_lift_end(out, S(), sc, ev, held, trail)
    if handoff_fn is not None and held:
        # FULL-CHAIN mode: the same process (this one) keeps the left hold and hands the session to the callback. The callback
        # says what it commanded and who has the block. Only when it commanded NOTHING and the left hand still holds as lifted
        # is the planned put-back still the right way back; after anything else the old lower / open is not run by itself.
        if EPISODE is not None:
            _rec(lambda: EPISODE.mark("handoff", skill="CARRY", owner="handoff callback (same process; left Jaw hold kept)"), "mark handoff")
        try:
            h = handoff_fn(dict(out=out, snapshot=json.loads((out / "HANDOFF_SNAPSHOT.json").read_text()) if (out / "HANDOFF_SNAPSHOT.json").exists() else None, scene=sc, evidence=ev, state=S, look=look_fn, stage=stage_fn, note=note, episode=EPISODE))
        except Exception as e:
            h = dict(result="FAILED", commands_sent=None, object_owner="UNKNOWN", why=f"{type(e).__name__}: {e}")
        note(step="handoff", result=h.get("result"), commands_sent=h.get("commands_sent"), object_owner=h.get("object_owner"), why=h.get("why"), stages=h.get("stages"), missing=h.get("missing"), preparation=h.get("preparation"), recording_errors=h.get("recording_errors"))
        if h.get("result") == "DONE":
            # The final area check and the clean-resume request belong to ONE manager: the hand-over's task manager made them
            # (REOBSERVE_CLEAR -> CLEAN_RESUME_REQUEST). They are reported here as it made them; this session does not look again
            # and does not overrule them.
            req = bool(h.get("clean_resume_request_emitted"))
            return dict(result="FULL_CHAIN_DONE" if req else "FULL_CHAIN_RAN_NO_CLEAN_RESUME_REQUEST", mode="FULL_CHAIN", held_after_lift=held, handoff=h, clean_resume_request=req, cleaning_succeeded=False,
                        outcomes=dict(manipulation=h.get("result"), task_manager=(h.get("task_manager") or {}).get("state") if isinstance(h.get("task_manager"), dict) else None, clean_resume_request="EMITTED by the task manager" if req else "NOT EMITTED", cleaning="NOT RUN (no cleaning device in this session)"),
                        clean_resume_note="a REQUEST to resume cleaning made by the task manager after it observed the source area; it is not a cleaning result", human_assistance="none recorded by the tool; a person's report overrides this", trail=trail)
        approached = (h.get("scope") == "RECEIVE_APPROACH" and h.get("result") == "RECEIVE_APPROACH_OBSERVED"
                      and h.get("left_at_lift") is True and h.get("object_owner") == "LEFT"
                      and (h.get("real_arm_motion") or {}).get("status") == "OBSERVED"
                      and (h.get("end") or {}).get("right_released") is True and (h.get("end") or {}).get("returned") is True
                      and (h.get("end") or {}).get("right_jaw_torque_unchanged") is True
                      and (h.get("end") or {}).get("right_jaw_goal_unchanged") is True
                      and (h.get("end") or {}).get("left_jaw_hold_kept") is True)
        contacted = (h.get("scope") == "RECEIVE_CONTACT" and h.get("result") in ("RECEIVE_CONTACT_COLLECTED", "RECEIVE_CONTACT_NO_CONTACT", "RECEIVE_CONTACT_NOT_READY_RETURNED")
                     and h.get("left_at_lift") is True and h.get("object_owner") == "LEFT"
                     and (h.get("real_arm_motion") or {}).get("status") == "OBSERVED"
                     and (h.get("end") or {}).get("right_released") is True and (h.get("end") or {}).get("returned") is True
                     and (h.get("end") or {}).get("right_jaw_torque_unchanged") is True
                     and (h.get("end") or {}).get("right_jaw_goal_unchanged") is True
                     and (h.get("end") or {}).get("left_jaw_hold_kept") is True)
        observed = h.get("result") == "OBSERVED" and h.get("left_at_lift") is True and h.get("object_owner") == "LEFT" and (h.get("end") or {}).get("right_released") is True and (h.get("end") or {}).get("returned") is True
        if approached:
            note(step="receive_approach_observed", why=h.get("why"), end=h.get("end"), real_arm_motion=h.get("real_arm_motion"))
        if contacted:
            note(step="receive_contact_finished", result=h.get("result"), why=h.get("why"), contact_candidate=h.get("contact_candidate"), insertion=h.get("insertion"), end=h.get("end"))
        if observed:
            # OBSERVE scope reached its planned end: the right arm is home and released, the left hand never let go and stands at the pose it lifted to. The planned put-back is this scope's planned way home.
            note(step="handoff_observed", why=h.get("why"), end=h.get("end"), observation_file=h.get("observation_file"))
        elif not approached and not contacted and not (h.get("result") == "NOT_READY" and h.get("commands_sent") is False and h.get("object_owner") == "LEFT"):
            return dict(result=f"HANDOFF_{h.get('result') or 'FAILED'}_HOLDING_NEEDS_A_PERSON", mode="FULL_CHAIN", held_after_lift=held, handoff=h, trail=trail,
                        note="the hand-over ended after it had commanded something (or it is not known whether it had): the put-back planned before the hand-over is NOT run by itself")
        if not observed and not approached and not contacted:
            note(step="handoff_not_entered", why="the callback commanded nothing and the left hand holds as lifted: the planned put-back is still valid")
        try:
            ab = getattr(handoff_fn, "abandon", None); ab_r = ab() if callable(ab) and not observed and not approached and not contacted else None
            if ab_r is not None:
                note(step="handoff_abandon", result=ab_r.get("result"))
        except Exception as e:
            note(step="handoff_abandon", error=f"{type(e).__name__}: {e}"); return dict(result="HANDOFF_ABANDON_FAILED_HOLDING_NEEDS_A_PERSON", mode="FULL_CHAIN", held_after_lift=held, handoff=h, trail=trail)
    # 2026-10-18 (user, 200250_auto): after the lift the held block goes to the taught carry pose (folded over the base, where the base would
    # drive), is held there with the jaws watched, and comes back over the place before the put-back. Only with a taught carry pose and a held block.
    carry_f = TAUGHT_REFS.get(SIDE, Path("/nonexistent")) / "H9_carry" / "ticks.json"
    if held and carry_f.exists() and handoff_fn is None and limit is None:
        if not run_stage("carry"):
            if (S()["stages"].get("carry") or {}).get("controller_result") == "REFUSED" or "carry" not in (S().get("order") or []):
                # refused before any write: the hand still holds the block at the lift pose -> the planned put-back, not a stop in the air (211928_auto)
                note(step="carry_skipped", why="the carry move was refused before any write; the block is put back from the lift pose")
                done_ = back(["lower", "open", "retreat", "home"])
                return done_ or dict(result="PICKED_AND_PUT_BACK_WITHOUT_CARRY", trail=trail)
            return stalled_home("CARRY") if "close" not in S()["stages"] else dict(result="STOPPED_IN_CARRY_HOLDING_NEEDS_A_PERSON", trail=trail)
        w = watch_grip(CARRY_HOLD_S); note(step="carry_hold", result=w["status"], start=w["start"], valid_reads=w["valid_reads"], error_reads=w["error_reads"], longest_gap_s=w["longest_gap_s"], why=w.get("why"), last=w["rows"][-1] if w["rows"] else None)
        if w["status"] != "HELD":                                              # DROPPED / UNKNOWN / FAULT / STOP: the carried-object plan does not go on by itself
            return dict(result=f"CARRY_HOLD_{w['status']}_NEEDS_A_PERSON", trail=trail)
        if not run_stage("carry_back"):
            return dict(result="STOPPED_IN_CARRY_BACK_HOLDING_NEEDS_A_PERSON", trail=trail)
    done_ = back(["lower", "open", "retreat", "home"])                         # as planned, whatever the snapshot is for: the block is not kept in the air
    if done_:
        return done_
    sc = look_fn(out, "05_home"); placed = sc.get("object") is not None and bool(sc["object"].get("class_match")); note(step="look", gate="placed_back", ok=placed, object=sc.get("object") and sc["object"].get("centre_xy"), torque=((sc.get("arms") or {}).get("left") or {}).get("torque"))
    try:
        pb = put_back_evidence(S(), sc)
    except Exception as e:
        pb = dict(verdict="UNKNOWN", error=f"{type(e).__name__}: {e}")
    note(step="put_back_evidence", **pb)
    was_observation = bool(handoff_fn is not None and held and locals().get("observed"))
    was_approach = bool(handoff_fn is not None and held and locals().get("approached"))
    was_contact = bool(handoff_fn is not None and held and locals().get("contacted"))
    scope_note = dict(scope="SUPPORT_OBSERVATION", full_task=False, release_allowed=False, support_confirmed=False, left_released_in_the_hand_over=False, handoff=h,
                      support_observation="DONE: SUPPORT_OBSERVATION.json; the left hand was not released; this is a partial run, not a hand-over and not a full task") if was_observation else {}
    if was_approach:
        scope_note = dict(scope="RECEIVE_APPROACH", full_task=False, release_allowed=False, support_confirmed=False,
                          left_released_in_the_hand_over=False, handoff=h,
                          receive_approach="DONE: actual right ARM5 motion was observed; right ARM5 returned/released and the left hand returned to its lift pose before this planned put-back")
    if was_contact:
        scope_note = dict(scope="RECEIVE_CONTACT", full_task=False, release_allowed=False, support_confirmed=False, grasp_confirmed=False,
                          left_released_in_the_hand_over=False, handoff=h,
                          receive_contact="DONE: the finite first-contact scope ended normally (candidate, free range, or missing-pose no-close); right returned/released and left returned to lift before this planned put-back")
    base = "PICKED_LIFTED_AND_PUT_BACK" if held and placed else ("LIFT_NOT_CONFIRMED_RETURNED_HOME" if not held else "PUT_BACK_NOT_SEEN_RETURNED_HOME")
    scoped = (("SUPPORT_OBSERVATION_DONE_" if was_observation else "RECEIVE_CONTACT_DONE_" if was_contact else "RECEIVE_APPROACH_DONE_") + ("PUT_BACK" if placed else "PUT_BACK_NOT_SEEN")) if was_observation or was_approach or was_contact else base
    return dict(result=scoped, **scope_note, held_after_lift=held, placed_back=placed, put_back_evidence=pb, human_assistance="none recorded by the tool; a person's report overrides this", trail=trail)


def write_limited_terminal(out, r, recorder=None, pending_diagnostics=0):
    """Console-17: LIMITED_SCOPE_TERMINAL.json, written after the recorder released its readers and polling. It records what
    was read; process exit afterwards is not a motor stop and not a torque-off."""
    out = Path(out)
    try:
        state = json.loads((out / "STATE.json").read_text())
    except (OSError, ValueError):
        state = {}
    stop = Path(str(out) + ".STOP").exists()
    terminal = limited_terminal(r, state, stop_requested=stop)
    if r is not None:
        r["limited_terminal"] = terminal
    stages = state.get("stages") or {}
    order = state.get("order") or []
    last_name = order[-1] if order else None
    last = stages.get(last_name) or {}
    end = last.get("end") or {}
    # only the existing pre-lift way back (home) can end released; read from that stage's own end registers, never inferred
    released = bool(last.get("controller_result") == "RETURNED_RELEASED" and isinstance(end.get("torque"), dict) and end["torque"] and not any(end["torque"].values()))
    doc = dict(schema="sweepick.limited-scope-terminal.v1", scope=LEFT_TO_LIFT, terminal=terminal, session_result=(r or {}).get("result"),
               ended_by_exception=r is None, stop_latch_present=stop,
               last_stage=dict(name=last_name, controller_result=last.get("controller_result"), exit_code=last.get("exit_code"), stopped_by=last.get("stopped_by"), recorded_at_host_s=last.get("t")),
               last_actual=dict(ticks=end.get("ticks"), goal_register=end.get("goal_register"), torque=end.get("torque"), status=end.get("status"), moving=end.get("moving"),
                                source=f"{last_name} stage executor end read (move.json end)", read_error=end.get("error")),
               last_goal_ack=dict(note="per-write ACK/readback is in the stage's run*/move.json (last_acknowledged_goal, last_goal_write_attempt, read_after_loss)",
                                  move_files=sorted(str(p.relative_to(out)) for p in (out / str(last_name)).glob("run*/move.json")) if last_name else []),
               hold_evidence=(r or {}).get("hold_evidence"), held_after_lift=(r or {}).get("held_after_lift"),
               stages_after_lift_commanded=[n for n in order[order.index("lift") + 1:]] if "lift" in order else [],
               owner=dict(process="this launcher process (single owner); it ends after this file", recorder=recorder, port_lease="released with the recorder; executor buses are closed per stage"),
               left_returned_released=released, normal_home=released, torque_off_confirmed=released, safe_shutdown="LEFT_HOME_RELEASED_BY_EXISTING_RETURN" if released else "NOT_PERFORMED",
               motion_stop_confirmed="NO_MOTION_COMMANDED" if not order else "UNKNOWN",
               operator_required=not released and bool(order), next_run_allowed=False,
               collision_diagnostics=dict(nominal_checks="ran before every stage write (unchanged)", post_session_tube_diagnostics="DEFERRED_OFFLINE",
                                          pending=pending_diagnostics, file="COLLISION_DIAGNOSTICS_PENDING.json" if pending_diagnostics else None),
               meaning="process exit is not a motor stop or torque-off; HOLDING != RETURNED_RELEASED != safe_shutdown != torque_off_confirmed")
    (out / "LIMITED_SCOPE_TERMINAL.json").write_text(json.dumps(doc, indent=1, default=str))
    return doc


def run_session(out, *, execute, record=False, full_chain=False, episode_fn=None, handoff=None, session_kw=None, scope="full", right_lift_m=None, observation_plan=None, contact_plan=None, limit=None, prepare_arms=None):
    """The session with its optional recorder and its optional full-chain callback, in this one process. The recorder is
    started before the first observation and closed after the last one; if it cannot start, a session that asked for it is
    not started (a record that silently is not there would be found out too late)."""
    global EPISODE
    if MOUNTED and full_chain:
        raise RuntimeError("MOUNTED_FLOOR setup: the full chain (hand-over, receive, place) still uses the desk frame chain; only the single-arm floor pick runs here")
    out = Path(out); out.mkdir(parents=True, exist_ok=True); ep = None; observing = scope in ("observe", "SUPPORT_OBSERVATION"); approaching = scope in ("approach", "RECEIVE_APPROACH"); contacting = scope in ("contact", "RECEIVE_CONTACT")
    mode = ("SUPPORT_OBSERVATION" if observing else ("RECEIVE_APPROACH" if approaching else "RECEIVE_CONTACT" if contacting else "FULL_TASK")) if full_chain else "BASELINE_PICK_AND_PUT_BACK"
    if limit is not None:                                                      # Console-17: a limited scope is its own mode; never combined with the full chain or a hand-over
        if limit != LEFT_TO_LIFT or full_chain or handoff is not None or not record:
            raise ValueError(f"limit {limit!r} needs full_chain=False, no hand-over and record=True")
        mode = LEFT_TO_LIFT
    record = record or full_chain                                              # the full chain runs inside the one record of the session (its hand-over is bound to the recorder's id and lease)
    del RECORDING_ERRORS[:]
    if record:
        try:
            import sweepick_episode
            ep = (episode_fn or (lambda: sweepick_episode.real(out, mode=mode, identifiers=dict(sources={f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(HERE.glob('sweepick_*.py')) + sorted(HERE.glob('sweepick_*profile*.json'))}, mapping=hashlib.sha256(CAND.read_bytes()).hexdigest(), scene=hashlib.sha256(SCENE.read_bytes()).hexdigest()))))()
        except Exception as e:
            r = dict(result="NOT_STARTED", why=f"the recorder could not start: {type(e).__name__}: {e}", trail=[]); (out / "SESSION.json").write_text(json.dumps(dict(run="PICK02", **r), indent=1)); print(json.dumps(r, indent=1)); return 3
    EPISODE = ep; r = None
    if prepare_arms is None:                                                   # the real devices: yes; an injected recorder or stage function (tests) opens no device
        prepare_arms = episode_fn is None and not (session_kw or {}).get("stage_fn")
    if execute and prepare_arms:                                               # both arms to the start pose before observe (user 2026-10-18: nobody sets the arms by hand)
        def leased_run(*a_, **k_):
            with ep.lease("left", "right") if ep is not None else contextlib.nullcontext():
                return mv.run(*a_, **k_)
        for s_ in ("left", "right"):
            try:
                pr = prepare(Path(out) / f"prepare_{s_}", s_, execute=True, run=leased_run, stop_file=Path(str(out) + ".STOP"))
            except Exception as e:
                pr = dict(result="NOT_RUN", why=f"{type(e).__name__}: {e}")
            off_ = (pr.get("start_pose_check") or {}).get("arm_off_ticks") or {}; after_ = (pr.get("start_pose_after") or {}).get("arm_off_ticks")
            print(f"[prepare {s_}] {pr['result']}: off before {off_ or 'none'} -> after {after_ if after_ is not None else '-'}; gripper {(pr.get('start_pose_check') or {}).get('gripper_tick')}"
                  + (f"; {pr.get('why') or pr.get('needs_a_person')}" if (pr.get('why') or pr.get('needs_a_person')) else ""), flush=True)   # the dashboard log shows stdout (user 2026-10-18: prepare not in the log)
            if pr["result"] != "ALREADY_AT_START" and not (pr["result"] == "RAN" and pr.get("at_start_pose")):
                r = dict(result="NOT_STARTED", why=f"prepare {s_}: {pr['result']} {pr.get('why') or pr.get('needs_a_person') or ''}", trail=[])
                Path(out).mkdir(parents=True, exist_ok=True); (Path(out) / "SESSION.json").write_text(json.dumps(dict(run="PICK02", **r), indent=1))
                if ep is not None:
                    _rec(lambda: sweepick_episode.release(ep, task_outcome="NOT_STARTED: prepare"), "release after prepare")
                EPISODE = None; print(json.dumps(r, indent=1)); return 3
    try:
        if full_chain and handoff is None:
            import sweepick_full_chain
            handoff, _owner = sweepick_full_chain.build(out, ep, scope=scope, right_lift_m=right_lift_m, observation_plan=observation_plan, contact_plan=contact_plan)
            (out / "ACTIVE_SELECTION.json").write_text(json.dumps(sweepick_full_chain.selection(scope), indent=1))      # which native files and which pin THIS process loaded, with their hashes                 # Codex's FullEpisodeHandoff with this process's owner (executor, readers, field records) injected
        try:
            r = session(out, execute=execute, handoff_fn=handoff, **dict(session_kw or {}, **(dict(limit=limit) if limit else {})))
        except Exception as exc:
            # 185636_auto: an exception in the session left the arm holding above the block. With the jaws never closed, the arm goes home by
            # the collision-checked 'abort_home' before the error is raised again; after a close it stays with a person.
            try:
                st_ = json.loads((Path(out) / "STATE.json").read_text()) if (Path(out) / "STATE.json").exists() else {}
                if execute and st_.get("stages") and "close" not in st_["stages"]:
                    r_ = stage(out, "abort_home", execute=True, cameras=wrist_cams)
                    print(json.dumps(dict(after_exception=f"{type(exc).__name__}: {exc}", abort_home=r_.get("controller_result")), indent=1))
            except Exception as exc2:
                print(f"abort_home after the exception failed: {type(exc2).__name__}: {exc2}")
            raise
    finally:
        EPISODE = None
        if ep is not None:
            import sweepick_episode
            res = (r or {}).get("result"); full_done = res == "FULL_CHAIN_DONE" and not observing
            doc = sweepick_episode.release(ep, task_outcome=("FULL_TASK_DONE" if full_done else "PARTIAL_OR_DIAGNOSTIC: " + str(res)) if r else "ENDED_BY_AN_EXCEPTION",
                                           stage_outcomes=[dict(step=t.get("step"), controller_result=t.get("controller_result"), gate=t.get("gate"), ok=t.get("ok")) for t in (r or {}).get("trail", []) if t.get("controller_result") or t.get("gate")],
                                           notes=[f"mode {mode}", "a baseline run is START / partial, not a full-task success" if not full_done else "full chain"])
            if r is not None:
                r["episode"] = dict(session_id=doc["session_id"], cameras=doc["cameras"], joints=doc["joints"], data_quality=doc["data_quality"])
        if limit is None:
            finish_collision_diagnostics()
        else:
            pending_diagnostics = len(DEFERRED_COLLISION_DIAGNOSTICS)
        if r is not None:
            r["recording_errors"] = list(RECORDING_ERRORS)                       # the record's quality, next to (not instead of) what the robot did
        if limit is not None:
            _rec(lambda: write_limited_terminal(out, r, pending_diagnostics=pending_diagnostics, recorder=None if ep is None else dict(released=True, session_id=doc.get("session_id"), readers_closed=doc.get("readers_closed"), task_outcome=doc.get("task_outcome"))), "limited terminal")
            _rec(lambda: defer_collision_diagnostics(out), "collision diagnostics queue")   # after the terminal record, also on an exception: kept for offline review, never run here
    if limit is not None:
        print(json.dumps(r, indent=1, default=str))
        return 0 if (r or {}).get("limited_terminal") == "LEFT_LIFT_HELD_CONFIRMED" and not RECORDING_ERRORS else 3
    print(json.dumps(r, indent=1, default=str)); return 0 if r["result"] in ("PICKED_LIFTED_AND_PUT_BACK", "PLANNED", "FULL_CHAIN_DONE", "SUPPORT_OBSERVATION_DONE_PUT_BACK", "RECEIVE_APPROACH_DONE_PUT_BACK", "RECEIVE_CONTACT_DONE_PUT_BACK") else 3


# ---- start pose (2026-10-18): a session starts from the folded rest with the gripper open; nobody has to set it by hand ----
START_POSE = HERE / "sweepick_start_pose.json"


def start_pose_targets(side, ticks, doc=None):
    """The joints of `side` that are off the start pose and where they go: the five arm joints together when any of them is
    further than the tolerance from the recorded folded rest; the gripper alone when it is not at an opening with a stored
    jaw reference. Empty = already at the start pose."""
    doc = doc or json.loads(START_POSE.read_text()); sp = doc[side]; tol = float(doc["arm_tolerance_ticks"])
    off = {n: float(ticks[n] - sp["arm"][n]) for n in ARM if abs(ticks[n] - sp["arm"][n]) > tol}
    refs = [float(r["gripper_tick"]) for r in (WRIST.get("stored_jaw_references") or [])] or [float(sp["gripper_open_tick"])]
    at_ref = any(abs(ticks["gripper"] - r) <= STORED_REF_TICK_TOL for r in refs) or abs(ticks["gripper"] - float(sp["gripper_open_tick"])) <= 20.0
    target = {n: float(sp["arm"][n]) for n in ARM} if off else {}
    if not at_ref:
        target["gripper"] = float(sp["gripper_open_tick"])
    return target, dict(arm_off_ticks=off, arm_tolerance_ticks=tol, gripper_tick=float(ticks["gripper"]), gripper_reference_ticks=refs, gripper_at_reference=at_ref)


def prepare(out, side, *, execute=False, run=mv.run, cert=certify, reads=None, scene_of=None, stop_file=None):
    """Bring `side` to the start pose before observe: plan (default) or execute. The move is the executor's own 'out'
    (only the joints that go somewhere, plus the other arm joints when the arm moves, so it does not sag), checked for
    collisions in a scene with the block where the head camera sees it now, then a 'return' to the reached pose that
    releases the torque (confirmed by a read). Nothing else is commanded; the other arm is not opened."""
    import mujoco
    out = Path(out)
    stop_ = Path(stop_file) if stop_file is not None else Path(str(out) + ".STOP")   # review 2026-10-19: the Task STOP is <RUN>.STOP; a prepare inside a run listens to that one
    out.mkdir(parents=True, exist_ok=True); set_side(side)
    rec = dict(schema="sweepick.prepare-start.v1", side=side, execute=bool(execute), t=time.time(), start_pose_file=START_POSE.name)
    def done(result, **kw):
        rec.update(result=result, **kw); (out / "PREPARE.json").write_text(json.dumps(rec, indent=1, default=str)); return rec
    now = (reads or read_ticks)()
    if any(v for s_ in now.values() for v in s_["torque"].values()):
        return done("NOT_RUN", why="torque is already on somewhere: another controller may own an arm")
    if any(v for s_ in now.values() for v in (s_.get("status") or {}).values()):
        return done("NOT_RUN", why=f"a servo reports a fault: {({s_: v.get('status') for s_, v in now.items()})}")
    target, check_ = start_pose_targets(side, now[side]["ticks"]); rec["start_pose_check"] = check_; rec["target"] = target
    if not target:
        return done("ALREADY_AT_START")
    if scene_of is None:
        def scene_of(folder, ticks_now):
            fs = top_frame(str(folder)); sc = analyse(fs["bgr"], fs["depth_mm"], fs["K"], {s_: ticks_now[s_]["ticks"] for s_ in ("left", "right")})
            (Path(folder) / "SCENE_prepare.json").write_text(json.dumps(sc, default=lambda o: o.tolist() if hasattr(o, "tolist") else str)); return sc
    sc = scene_of(out, now); o = (sc or {}).get("object")
    if not (sc or {}).get("ok"):
        return done("NOT_RUN", why=f"no observation for the collision scene: {(sc or {}).get('why')}")
    block = None if o is None else (np.array(o["centre_world"], float), float(o.get("yaw_rad") or 0.0)); rec["block_in_scene"] = None if o is None else o["centre_world"]
    in_process = cert is certify and block is not None
    scene_file = out / "scene_prepare.mjb"
    if not in_process:
        mujoco.mj_saveModel(model(block), str(scene_file))                         # 2026-10-18: the same in-process scene check the stages use (no checker process per move: start-up time)
    def check(doc, folder, tag):
        folder.mkdir(parents=True, exist_ok=True); (folder / f"{tag}.TRAJECTORY.json").write_text(json.dumps(doc))
        if in_process:
            c_ = object_check(doc, block, carried=False, right_ticks=now[other(side)]["ticks"])
            fc_ = c_.get("first_nominal_contact") or {}
            if c_.get("nominal_verdict") != "CLEAR" and fc_.get("step") == 0 and all(all(str(x).startswith(side) for x in pr) for pr in (fc_.get("pairs") or [])):
                # 221010_auto (right): after a power cycle the arm rested folded a little further than the start pose and the MODEL put the shoulder
                # and the lower arm into each other at that measured pose (step 0) - the arm is physically there. Contacts only on a run of the
                # first samples, all inside this arm, and clear after it: the path only leaves a model overlap of where the arm already is.
                ca_ = object_check(doc, block, carried=False, right_ticks=now[other(side)]["ticks"], all_contacts=True); cs_ = ca_.get("contact_steps") or []
                # review 2026-10-19: every contact sample must hold ONLY pairs that were already there at the measured start (step 0), all inside this
                # arm; a new pair (object, floor, body, other arm, another own pair) anywhere refuses the exception. Penetration depth is not available
                # from this checker: a deepening overlap is not detected (recorded as a limit).
                cp_ = [{tuple(p_) for p_ in ps_} for ps_ in (ca_.get("contact_pairs") or [])]
                init_ = cp_[0] if cp_ else set()
                own_only_ = bool(init_) and all(all(str(x).startswith(side) for x in p_) for p_ in init_)
                no_new_ = all(ps_ <= init_ for ps_ in cp_)
                if cs_ and own_only_ and no_new_ and cs_ == list(range(len(cs_))) and len(cs_) < (ca_.get("nominal_steps_checked") or 0):
                    c_ = dict(c_, nominal_verdict="CLEAR", nominal_verdict_raw="CONTACT", range_entry_checked=c_.get("range_entry_identity") is not None, start_overlap_left=dict(contact_steps=len(cs_), of=ca_.get("nominal_steps_checked"), pairs=sorted(map(list, init_)), depth_checked=False,
                                                                                   basis="the measured start pose overlaps in the model (own links only); the path leaves it and stays clear"))
        else:
            c_ = cert(folder / f"{tag}.TRAJECTORY.json", folder / f"{tag}.CERT.json", scene_file)
        (folder / f"{tag}.CHECK.json").write_text(json.dumps(c_, indent=1, default=str)); return c_
    powered = list(ARM) + ["gripper"] if any(n in ARM for n in target) else ["gripper"]
    kw = dict(level="model", torque_joints=powered, other_arm_ticks=now[other(side)]["ticks"], profile=fast_profile())     # away from the block: the fast profile
    code, plan = run("plan", side, out / "plan1", target=target, **kw)
    if plan["result"] != "PLANNED":
        return done("NOT_RUN", why=f"planning read refused: {plan.get('refused')}", controller_result=plan["result"])
    c = check(json.loads((out / "plan1/TRAJECTORY.json").read_text()), out / "plan1", "path")
    rec["collision"] = dict(verdict=c.get("nominal_verdict"), first_contact=c.get("first_nominal_contact"), steps=plan["trajectory"]["steps"], duration_s=plan["trajectory"]["duration_s"])
    if c.get("nominal_verdict") != "CLEAR":
        return done("NOT_RUN", why=f"the path to the start pose touches something: {c.get('first_nominal_contact')}", controller_result="PLANNED")
    if not execute:
        return done("PLANNED", controller_result="PLANNED")
    n_trim = [0]
    def trim_check(doc):
        n_trim[0] += 1; return check(doc, out / "run_out", f"trim{n_trim[0]}")
    code, log = run("out", side, out / "run_out", target=target, execute=True, cert=c, start_recheck=lambda doc: check(doc, out / "run_out", "start"), stop_requested=stop_.exists,
                    settle_trim=[n for n in ARM if n in target], trim_check=trim_check, **kw)     # a loaded joint that settles short is trimmed onto the start pose
    rec["out"] = dict(controller_result=log["result"], writes=log.get("writes"), reached=log.get("reached"), stopped_by=log.get("stopped_by"), refused=log.get("refused"), needs_a_person=log.get("needs_a_person"))
    if log["result"] == "REFUSED" and not any((log.get("writes") or {}).values()):
        return done("NOT_RUN", why=f"the executor refused before any write: {log.get('refused')}", controller_result="REFUSED")
    if log["result"] not in ("HOLDING", "TARGET_NOT_REACHED_HOLDING"):
        return done("RAN", controller_result=log["result"], needs_a_person=log.get("needs_a_person") or ("refused before any write" if not any((log.get("writes") or {}).values()) else "see run_out/move.json"))
    here = {n: float(v) for n, v in (reads or read_ticks)()[side]["ticks"].items()}
    rkw = dict(rest={n: here[n] for n in powered}, torque_joints=powered, level="model", other_arm_ticks=now[other(side)]["ticks"], profile=fast_profile())
    _, plan2 = run("return", side, out / "plan_release", **rkw)                 # where it stands now -> the same pose: release (a few ticks of settling at most)
    traj2 = out / "plan_release/TRAJECTORY.json"
    c2 = check(json.loads(traj2.read_text()), out / "plan_release", "path") if traj2.exists() else None
    if c2 is not None and c2.get("nominal_verdict") != "CLEAR":
        return done("RAN", controller_result="HOLDING", needs_a_person=f"the arm holds at the start pose; the release path touches something: {c2.get('first_nominal_contact')}")
    code2, log2 = run("return", side, out / "run_release", execute=True, cert=c2, start_recheck=lambda doc: check(doc, out / "run_release", "start"), stop_requested=stop_.exists, **rkw)
    after = (reads or read_ticks)()[side]
    doc_ = json.loads(START_POSE.read_text()); sag = float(doc_.get("release_sag_allowance_ticks", 0))
    loose = dict(doc_, arm_tolerance_ticks=float(doc_["arm_tolerance_ticks"]) + sag)        # released: the arm settles under its own weight
    left_off, check_after = start_pose_targets(side, after["ticks"], loose)
    return done("RAN", controller_result=log2["result"], release=log2.get("release"), torque_after=after["torque"], ticks_after=after["ticks"], start_pose_after=check_after, at_start_pose=not left_off,
                why=None if not left_off else f"after the release still off the start pose: {check_after['arm_off_ticks']} (allowed {check_after['arm_tolerance_ticks']:.0f} incl. {sag:.0f} for settling)",
                needs_a_person=log2.get("needs_a_person"))


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=("observe", "look", "stage", "correct", "session", "prepare")); ap.add_argument("--along-jaw-mm", type=float); ap.add_argument("--basis", default=""); ap.add_argument("out"); ap.add_argument("name", nargs="?", help="look: a tag; stage: one of " + ", ".join(STAGES)); ap.add_argument("--execute", action="store_true")
    ap.add_argument("--record", action="store_true", help="session: one continuous 3-view + both-arm record under one session id (sweepick_episode)"); ap.add_argument("--full-chain", action="store_true", help="session: after the lift hand the session to the RECEIVE -> RELEASE -> PLACE -> STOW callback (sweepick_full_chain); not started unless everything it needs is ready")
    ap.add_argument("--full-chain-scope", choices=("full", "observe", "approach", "contact"), default="full", help="full: whole task. observe: contact/load-transfer observation. approach: ARM5 prefix and return. contact: observed insert plus finite right-ID6 first-contact collection and planned return")
    ap.add_argument("--execution-scope", choices=("FULL_TASK", "SUPPORT_OBSERVATION", "RECEIVE_APPROACH", "RECEIVE_CONTACT"), help="the scope by its external name")
    ap.add_argument("--support-observation-plan", help="SUPPORT_OBSERVATION: the approved plan file (source, right_lift_m, object_support_plan, end_state=HOLD_BOTH_NO_RELEASE at the observation point)")
    ap.add_argument("--receive-contact-plan", help="RECEIVE_CONTACT: plan file (source, baseline_file, close_target_tick, observe_s, object_support_plan, end_state=LEFT_HOLD_THEN_PLANNED_RETURN)")
    a = ap.parse_args(argv)
    global KEEP_TOP
    KEEP_TOP = a.mode in ("session", "prepare", "observe")                    # one head reader per process
    if a.mode == "prepare":
        r = prepare(a.out, a.name or "left", execute=a.execute); print(json.dumps({k: r.get(k) for k in ("result", "why", "start_pose_check", "target", "collision", "controller_result", "at_start_pose", "needs_a_person")}, indent=1, default=str)); return 0 if r["result"] in ("ALREADY_AT_START", "PLANNED") or (r["result"] == "RAN" and r.get("at_start_pose")) else 2
    if a.mode == "observe":
        s = observe(a.out); print(json.dumps({k: s[k] for k in ("ready", "why", "plan", "home_ticks")}, indent=1, default=str)); return 0 if s["ready"] else 2
    if a.mode == "session":
        scope = a.execution_scope or a.full_chain_scope; plan = json.loads(Path(a.support_observation_plan).read_text()) if a.support_observation_plan else None
        contact_plan = json.loads(Path(a.receive_contact_plan).read_text()) if a.receive_contact_plan else None
        return run_session(a.out, execute=a.execute, record=a.record, full_chain=a.full_chain, scope=scope, observation_plan=plan, contact_plan=contact_plan)
    if a.mode == "correct":
        print(json.dumps(correct(a.out, a.along_jaw_mm, a.basis, assisted=True), indent=1, default=str)); return 0
    if a.mode == "look":
        st_path = Path(a.out) / "STATE.json"                                     # the session's arm (observe chose it); a separate look process starts as LEFT otherwise (10/18 R0)
        if st_path.exists():
            set_side(json.loads(st_path.read_text()).get("side", "left"))
        s = look(a.out, a.name); print(json.dumps({k: s.get(k) for k in ("ok", "why", "arms_on_model", "object", "tool_left", "tool_minus_plan", "wrist", "desk_z")}, indent=1, default=str)); return 0
    r = stage(a.out, a.name, execute=a.execute, cameras=wrist_cams)
    print(json.dumps({k: v for k, v in r.items() if k != "end"}, indent=1, default=str)); return 0 if r["result"] in ("RAN", "PLANNED") and r.get("exit_code", 0) == 0 else (r.get("exit_code") or 2)


if __name__ == "__main__":
    sys.exit(main())
