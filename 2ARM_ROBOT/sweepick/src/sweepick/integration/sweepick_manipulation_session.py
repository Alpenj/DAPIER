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

usage: python -m sweepick.integration.sweepick_manipulation_session session OUT [--execute]   the whole PICK02 session (without --execute: observe and plan only)
       python -m sweepick.integration.sweepick_manipulation_session observe OUT | look OUT TAG | stage OUT NAME [--execute] | correct OUT --along-jaw-mm X (ASSISTED)
"""
from sweepick.integration.sweepick_resource_paths import source_path, source_files
import argparse, hashlib, json, subprocess, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
RT = Path.home() / "DAPIER/tjj-runtime/v1"
sys.path[:0] = [p for p in (str(RT / "factory/v1"), str(RT / "source/production_source_g000"), str(RT / "source/turtlebot3_waffle_pi")) if p not in sys.path]
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_joint_kinematics as sweepick_kin
from sweepick.control import sweepick_trajectory_executor as mv

IN = Path.home() / "sweepick_261007_commission/inputs"
CAND, CAL, ASM = IN / "CANDIDATE_Q12_CONFIG.json", Path.home() / ".config/dapier/lerobot-calibration", IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"
SCENE = RT / "model/dual_scene_desk_both.mjb"
# the arm datum on the printed board: operator ruler record of 2026-09-23 (registered-capture-07-board-input.json), axes assumed parallel
T_DATUM_FROM_BOARD = np.array([[0, -1, 0, 0.201], [-1, 0, 0, -0.034], [0, 0, -1, -0.04], [0, 0, 0, 1.0]])
DATUM_IN_MODEL_BASE = np.array([0.0388353, 0.0, 0.0254])                       # motor-1 lower shaft face in the model base (motor1-cad-datum-audit.json)
BOARD_Z_IN_WORLD = float(DATUM_IN_MODEL_BASE[2] + T_DATUM_FROM_BOARD[2, 3])    # the real desk top in the model's world: -14.6 mm
ARM = list(rc.JOINTS[:5]); ALL = list(rc.JOINTS)
STAGES = ("pregrasp", "align", "descend_a", "descend_b", "descend", "descend_settle", "close", "lift", "lower", "open", "retreat", "home", "retreat_arm", "home_arm", "abort_home")
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
    if name == "lower":
        return last == "lift", "the lift pose is held"
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


SHORT_OK = ("pregrasp", "align", "descend_a", "descend_b", "descend", "descend_settle", "lift", "lower", "retreat", "retreat_arm")     # arm positioning stages; never the gripper, and not the final return (that one must be confirmed before torque is released)
HOLDING_OK = ("HOLDING",)
CLOSE_OK = ("GRIP_CONTACT_HOLDING",)                                          # the existing Jaw primitive ended holding a contact candidate
GATE = {"descend_settle": "block_undisturbed_after_descent", "descend_a": "ready_to_descend", "descend_b": "ready_for_final_descent", "descend": "ready_for_final_descent", "close": "ready_to_close", "lift": "ready_to_lift"}   # observation needed before the stage, besides the previous controller result
TRIM = ("pregrasp", "align", "descend_a", "descend_settle", "lift", "retreat", "home", "retreat_arm", "home_arm", "abort_home")   # descend_a: the pads are above the block by the vertical uncertainty, so a shortfall there is the arm's own; never descend_b / descend / lower or the gripper           # loaded free moves only; never where a shortfall may be a contact (descend, lower) or on the gripper


def mapping():
    return rc.load_mapping(CAND, CAL, ASM)


def model(block=None, table_z=BOARD_Z_IN_WORLD):
    """The assembled dual model with the desk top at the real height and the block at `block` (centre, yaw) or far away."""
    import mujoco
    m = mujoco.MjModel.from_binary_path(str(SCENE))
    m.geom_pos[m.geom("table").id][2] += table_z
    adr = m.jnt_qposadr[m.body_jntadr[m.body("red_block").id]]
    if block is None:
        m.qpos0[adr:adr + 3] = [2.0, 0.0, 0.02]
    else:
        (x, y, z), yaw = block
        m.qpos0[adr:adr + 7] = [x, y, z, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    return m


def world_from_camera(T_camera_from_board, m):
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


def analyse(bgr, depth_mm, K, ticks, *, object_half=0.02):
    """Scene from one top FrameSet and the joint readings: frame chain, how well the depth points of the arms fall on the
    model arms (the joint mapping and the chain are both in this number), and the object's top face."""
    import mujoco, tjj_perception as tp
    from sweepick.perception.sweepick_wrist_capture import board_pose
    K = np.asarray(K, float).reshape(3, 3); bp = board_pose(bgr, K)
    if bp is None:
        return dict(ok=False, why="the printed board was not found in the top image: no frame chain")
    mp = mapping(); m = model(); d = mujoco.MjData(m)
    q12 = rc.ticks_to_q12(mp, ticks); sweepick_kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    Twc = world_from_camera(bp["T"], m); Kd = K.copy(); Kd[:2] /= 2
    pts = tp.back_project(np.asarray(depth_mm, float) / 1000.0, Kd, Twc)
    zt = float(np.median(pts[(np.abs(pts[:, 2] - BOARD_Z_IN_WORLD) < 0.03) & ~inside(m, d, pts, "", 0.03)][:, 2])) if len(pts) else None    # desk height as the DEPTH sees it
    arms = {}
    for side in ("left", "right"):
        near = inside(m, d, pts, side, 0.06) & (pts[:, 2] > zt + 0.03)
        arms[side] = dict(points=int(near.sum()), **{f"share_within_{int(k * 1000)}mm": (float(inside(m, d, pts[near], side, k).mean()) if near.sum() else None) for k in (0.0, 0.006, 0.012, 0.025)})
    free = ~inside(m, d, pts, "", 0.02) & (pts[:, 0] > 0.05) & (pts[:, 0] < 0.45) & (np.abs(pts[:, 1]) < 0.25)
    top = pts[free & (pts[:, 2] > zt + 2 * object_half - 0.01) & (pts[:, 2] < zt + 2 * object_half + 0.01)]
    obj = None
    if len(top) >= 50:
        c = np.median(top, 0); top = top[np.hypot(top[:, 0] - c[0], top[:, 1] - c[1]) < 3 * object_half]
        ex = [float(np.percentile(top[:, i], 98) - np.percentile(top[:, i], 2)) for i in (0, 1)]
        yaw, n = tp.top_face_yaw(np.asarray(depth_mm, float) / 1000.0, Kd, Twc, np.median(top, 0), surface_z=zt, robot_mask=lambda p: inside(m, d, p, "", 0.02))
        obj = dict(top_points=int(len(top)), centre_xy=[float(v) for v in np.median(top, 0)[:2]], extent_xy_m=ex, top_height_above_depth_desk_m=float(np.median(top[:, 2]) - zt), yaw_rad=None if yaw is None else float(yaw), yaw_points=int(n),
                   centre_world=[float(np.median(top, 0)[0]), float(np.median(top, 0)[1]), BOARD_Z_IN_WORLD + object_half],
                   class_match=bool(all(abs(e - 2 * object_half) <= object_half / 2 for e in ex)))
    return dict(ok=True, board=dict(markers=bp["markers"], rms_px=bp["rms_px"], T_camera_from_board=bp["T"]), T_world_camera=Twc.tolist(), camera_in_world=Twc[:3, 3].tolist(),
                desk_z=dict(by_chain=BOARD_Z_IN_WORLD, by_depth=zt, note="the depth reads about 2 % long (READ02); the chain value is used for heights"), q12=[float(v) for v in q12], ticks=ticks, arms_on_model=arms, object=obj,
                tool_left=[float(v) for v in d.site_xpos[m.site("left_cube_grasp").id]], **tool_axis(m, d))


def tool_axis(m, d):
    rot = d.site_xmat[m.site("left_cube_grasp").id].reshape(3, 3); jaw = rot[:, 2].copy(); jaw[2] = 0.0; jaw /= np.linalg.norm(jaw)
    return dict(jaw_axis_xy=[float(jaw[0]), float(jaw[1])], tool_along_jaw_mm=float(1000 * np.dot(d.site_xpos[m.site("left_cube_grasp").id][:2], jaw[:2])))


PAD_GEOMS = ("left_pgripper_pad_1", "left_pgripper_pad_2")


def geom_vertices(m, d, name):
    """World vertices of one mesh geom of the model at the pose in d (the real shape, not a box around it)."""
    import mujoco
    g = m.geom(name).id; k = m.geom_dataid[g]; R = d.geom_xmat[g].reshape(3, 3)
    return d.geom_xpos[g] + m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]] @ R.T


def pad_placement(m, d, centre, yaw, half=0.02):
    """Where the two pads of the left gripper are against the block, from the pad meshes at the pose in d, in one frame:
      vertical_mm   lowest pad vertex that is over the block's top face (or within 3 mm of its edge), above the top face
      gaps_mm       per pad: distance along the jaw axis from the pad's vertices at or below the top face to the block's side
                    face (None while the pad is still above the top face); negative = that pad is over / inside the block
      tool_minus_pad_low_mm   how far the lowest pad vertex is below the tool point (the model's own number)
    PICK01 / PICK02 run 5 were planned with 'pad 5 mm below the tool point'; the meshes say about 16 .. 18 mm."""
    top = centre[2] + half; rot = d.site_xmat[m.site("left_cube_grasp").id].reshape(3, 3); jaw = rot[:, 2].copy(); jaw[2] = 0.0; jaw /= np.linalg.norm(jaw)
    tool = d.site_xpos[m.site("left_cube_grasp").id]; Rb = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    rel = np.arctan2(jaw[1], jaw[0]) - yaw; half_along = half * (abs(np.cos(rel)) + abs(np.sin(rel)))
    out = dict(jaw_axis_xy=[float(jaw[0]), float(jaw[1])], block_half_along_jaw_mm=float(1000 * half_along), tool_above_top_mm=float(1000 * (tool[2] - top)), tool_along_jaw_mm=float(1000 * np.dot(tool[:2] - centre[:2], jaw[:2])), pads={})
    low_over, low_all = [], []
    for n in PAD_GEOMS:
        v = geom_vertices(m, d, n); loc = (v[:, :2] - centre[:2]) @ Rb.T; near = (np.abs(loc) <= half + 0.003).all(axis=1)
        s_ = (v[:, :2] - centre[:2]) @ jaw[:2]; side = float(np.sign(s_.mean())); band = v[:, 2] <= top + 0.002
        low_all.append(v[:, 2].min()); low_over += [v[near, 2].min()] if near.any() else []
        out["pads"][n] = dict(side=int(side), lowest_above_top_mm=float(1000 * (v[:, 2].min() - top)), gap_mm=float(1000 * ((side * s_[band]).min() - half_along)) if band.any() else None)
    out["vertical_mm"] = float(1000 * (min(low_over) - top)) if low_over else None
    out["tool_minus_pad_low_mm"] = float(1000 * (tool[2] - min(low_all)))
    g = [out["pads"][n]["gap_mm"] for n in PAD_GEOMS]
    out["gaps_mm"] = g; out["room_each_side_mm"] = None if None in g else float(sum(g) / 2)
    out["recentre_along_jaw_mm"] = None if None in g else float(out["pads"][PAD_GEOMS[0]]["side"] * (g[1] - g[0]) / 2)     # tool shift along +jaw that makes the two gaps equal: a pad on side s gains s * shift
    return out


def plan_pick(scene, cfg=None, tool_offset=None, above_b=None, keep=None):
    """Pre-grasp / intermediate / grasp / lift joint targets of the left arm from the SIM teacher's planner (IK + its path
    collision check), at the measured start pose.
    tool_offset (world, m): where the HAND is aimed relative to the observed block centre (visual correction). The block stays
    at its OBSERVED pose in the scene: a hand correction is not a measurement of the block.
    The grasp is chosen from every tilt / jaw direction / lift height the arm can do, by the room the PAD MESHES leave on
    both sides of the block against how far a joint residual moves the pads along the jaw axis; the hand is re-centred so
    that the two pad gaps are equal (a leaning jaw puts its lower pad nearer the block than the tool point suggests)."""
    import mujoco
    from sweepick.control import sweepick_trajectory_profile as tt
    from sim_data_factory.teacher import HandoverTeacher, TeacherFailure, UP
    o = scene["object"]; yaw = o["yaw_rad"] or 0.0; centre = np.array(o["centre_world"]); width = 0.04
    m = model((centre, yaw)); t = HandoverTeacher(m); cfg = t.cfg; d = mujoco.MjData(m); q12 = np.array(scene["q12"]); sweepick_kin.set_q12(m, d, q12); mujoco.mj_forward(m, d)
    start_gear = d.qpos[t.plan.adr].copy(); t.cmd = start_gear.copy(); mp = mapping(); prof = tt.load_profile(); pg = __import__("sweepick.control.sweepick_motion_progress", fromlist=["*"]).load()
    user = np.zeros(3) if tool_offset is None else np.asarray(tool_offset, float)
    seen = centre + np.array([user[0], user[1], 0.0])                          # where the wrist image puts the block relative to the hand (lateral only); used for the pad gaps, never for the collision scene
    rb = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]]); jaws = dict(x=rb[:, 0], y=rb[:, 1])
    t.plan.pose(d.qpos, t.cmd); rl = t.plan.radial("left", centre); reasons, cands = [], []
    here = {n: float(v) for n, v in rc.q12_to_ticks(mp, q12)["left"].items()}; rng = {n: (mp["left"][n]["lo"], mp["left"][n]["hi"]) for n in ARM}
    def ticks(w):
        q = t.to_canonical(np.array(w)); q[5] = q12[5]; q[6:] = q12[6:]
        tk = rc.q12_to_ticks(mp, q)["left"]; return {n: float(round(tk[n])) for n in ARM}
    def pose_of(tk_):
        dd = mujoco.MjData(m); sweepick_kin.set_q12(m, dd, rc.ticks_to_q12(mp, dict(left=dict(here, **tk_), right={n: float(v) for n, v in rc.q12_to_ticks(mp, q12)["right"].items()}))); mujoco.mj_forward(m, dd); return dd
    def solve(start, pos, ap, jaw, label, allow=()):
        dd = mujoco.MjData(m); dd.qpos[:] = d.qpos; dd.qpos[t.plan.adr] = start; t.cmd = np.array(start, float).copy()
        w, info = t.plan_move(dd, "left", pos, ap, [jaw], allow=allow, label=label)
        if len(w) != 1:
            raise TeacherFailure(f"{label}: the planner needs a via point; this session runs single joint-space segments only")
        return w[0], info
    for tilt in np.deg2rad(cfg.pick_tilt_deg):
        ap = np.cos(tilt) * -UP + np.sin(tilt) * rl
        for jname, jaw in jaws.items():
            tag = f"tilt {np.degrees(tilt):.0f} jaw {jname}"
            if keep is not None and (abs(np.degrees(tilt) - keep["tilt_deg"]) > 1e-6 or jname != keep["jaw"]):
                continue                                                       # a re-plan inside a session keeps the grasp it started with: a correction is a small shift, never another approach or jaw direction
            try:
                aim = centre + user
                for _ in range(2):                                             # second pass: the hand re-centred so that both pad gaps are equal
                    pre, info = solve(start_gear, aim + [0, 0, cfg.pregrasp_height_m], ap, jaw, "left pregrasp")
                    des, _ = solve(pre, aim + [0, 0, cfg.left_grasp_offset_m], ap, jaw, "left grasp", allow=("left",))
                    pl = pad_placement(m, pose_of(ticks(des)), seen, yaw)
                    if pl["room_each_side_mm"] is None:
                        raise TeacherFailure("left grasp: a pad does not reach below the block's top face at the grasp pose")
                    shift = pl["recentre_along_jaw_mm"] / 1000.0 * np.array([pl["jaw_axis_xy"][0], pl["jaw_axis_xy"][1], 0.0])
                    if abs(pl["recentre_along_jaw_mm"]) < 0.1:
                        break
                    aim = aim + shift
                # how far joint residuals of the size the executor accepts as 'reached' move the pads: along the jaw axis and vertically
                tkd = ticks(des); base = pose_of(tkd); p0 = base.site_xpos[m.site("left_cube_grasp").id].copy(); jx = np.array(pl["jaw_axis_xy"]); u_lat = u_ver = 0.0
                for n in ARM:
                    pp = pose_of(dict(tkd, **{n: tkd[n] + pg["reach"]})).site_xpos[m.site("left_cube_grasp").id]
                    u_lat += abs(float(np.dot(pp[:2] - p0[:2], jx))); u_ver += abs(float(pp[2] - p0[2]))
                # intermediate height: the lowest pad vertex stays above the top face by the vertical uncertainty (and 1 mm for the block's own height)
                h_a = 0.02 + pl["tool_minus_pad_low_mm"] / 1000.0 + u_ver + 0.001
                if h_a >= cfg.pregrasp_height_m:
                    raise TeacherFailure(f"the pads would already be within the vertical uncertainty ({u_ver * 1000:.1f} mm) of the top face at the pre-grasp height")
                mid, _ = solve(pre, aim + [0, 0, h_a], ap, jaw, "left above block")
                mid_b = None
                if above_b is not None:                                        # a second stop: pads this far above the top face (from the residual MEASURED at the first stop)
                    mid_b, _ = solve(mid, aim + [0, 0, 0.02 + pl["tool_minus_pad_low_mm"] / 1000.0 + above_b], ap, jaw, "left just above block")
                des2, _ = solve(mid if mid_b is None else mid_b, aim + [0, 0, cfg.left_grasp_offset_m], ap, jaw, "left grasp from above", allow=("left",))
                pl_mid = pad_placement(m, pose_of(ticks(mid)), seen, yaw); pl = pad_placement(m, pose_of(ticks(des2)), seen, yaw)
                lift = None
                for lh in (cfg.lift_height_m, 0.045, 0.03):                    # the lift only has to take the block clear of the desk; a lower lift may be within the joint ranges where the full one is not
                    try:
                        l_, _ = solve(des2, aim + [0, 0, cfg.left_grasp_offset_m + lh], ap, jaw, "left lift", allow=("left",))
                    except TeacherFailure:
                        continue
                    tk = dict(pregrasp=ticks(pre), descend_a=ticks(mid), descend=ticks(des2), lift=ticks(l_)); bad = []
                    if mid_b is not None:
                        tk["descend_b"] = ticks(mid_b)
                    for a_, b_ in (("start", "pregrasp"), ("pregrasp", "descend_a")) + ((("descend_a", "descend_b"), ("descend_b", "descend")) if mid_b is not None else (("descend_a", "descend"),)) + (("descend", "lift"),):          # the executor's own range gates on each segment, before anything moves
                        seg = tt.make(dict(here, **({} if a_ == "start" else tk[a_])), tk[b_], prof)
                        if not seg["moving"]:
                            continue
                        v = tt.validate("left", mp["left"], seg, prof, mapping_sha="plan", level="model")
                        bad += [f"{b_}: {g_['gate']} {g_.get('joint', '')}" for g_ in v["hard"] + v["provisional"] if g_["gate"] in ("servo_range", "model_joint_range_candidate")]
                    if not bad:
                        lift = (lh, tk); break
                    last_bad = bad
                if lift is None:
                    raise TeacherFailure(f"no lift height within the joint ranges ({locals().get('last_bad')})")
            except TeacherFailure as e:
                reasons.append(f"{tag}: {str(e)[:150]}"); continue
            lh, tk = lift; room = pl["room_each_side_mm"]
            cand = dict(ok=True, tilt_deg=float(np.degrees(tilt)), jaw=jname, ik=info if not isinstance(info, dict) else {k: float(v) for k, v in info.items()}, ticks=tk, object_width_m=width, lift_height_m=float(lh),
                        room_each_side_m=float(room / 1000.0), pad_gaps_mm=pl["gaps_mm"], joint_residual_moves_pads_mm=dict(along_jaw=float(u_lat * 1000), vertical=float(u_ver * 1000), residual_ticks=pg["reach"],
                        basis="each arm joint off by the executor's reach tolerance, effects added: the worst case the executor would still call 'reached'"),
                        room_over_residual=float(room / max(u_lat * 1000, 1e-6)), jaw_lean_deg=float(np.degrees(np.arcsin(abs(float(pose_of(tk["descend"]).site_xmat[m.site("left_cube_grasp").id].reshape(3, 3)[2, 2]))))),
                        placement_at_grasp=pl, placement_above_block=pl_mid, placement_just_above_block=None if mid_b is None else pad_placement(m, pose_of(ticks(mid_b)), seen, yaw), above_b_m=above_b, model_jaw_gap_now_m=float(width + 2 * room / 1000.0), pad_overlap_m=float(-pl["pads"][PAD_GEOMS[0]]["lowest_above_top_mm"] / 1000.0),
                        descend_a=dict(tool_above_centre_m=float(h_a), pads_above_top_mm=pl_mid["vertical_mm"], lowest_pad_vertex_above_top_mm=min(pl_mid["pads"][n]["lowest_above_top_mm"] for n in PAD_GEOMS), fraction=None,
                                       basis="IK pose straight above the grasp: the lowest vertex of the pad meshes stays above the block's top face by what joint residuals can move it vertically, plus 1 mm"),
                        geometry=dict(pregrasp_height_m=cfg.pregrasp_height_m, grasp_offset_m=cfg.left_grasp_offset_m, lift_height_m=float(lh), source="sim_data_factory.teacher.TeacherConfig; lift height lowered only when the full one is outside a joint range"),
                        object_observed_world_m=[float(v) for v in centre], tool_offset_world_m=[float(v) for v in (aim - centre)], visual_offset_world_m=[float(v) for v in user], recentring_world_m=[float(v) for v in (aim - centre - user)],
                        room_note="from the pad meshes at the grasp pose, after the hand is re-centred between the pads; candidate model")
            reasons.append(f"{tag}: feasible, room {room:.1f} mm each side, a joint residual moves the pads {u_lat * 1000:.1f} mm along the jaw axis (ratio {cand['room_over_residual']:.2f}), lift {lh * 1000:.0f} mm")
            cands.append(cand)
    if not cands:
        return dict(ok=False, why=reasons)
    best = max(cands, key=lambda c: c["room_over_residual"])                  # most room against what joint residuals do to the pads along the jaw axis
    best["considered"] = reasons
    return best


# ---- reading the cell (no write) ---------------------------------------------------------------------------------------
EPISODE = None        # the session's continuous recorder (sweepick_episode.Episode) when one runs: it owns the camera readers and lends them; it lets go of an arm's port for every read or stage here


def read_ticks():
    if EPISODE is not None:
        with EPISODE.lease("left", "right"):
            return _read_ticks()
    return _read_ticks()


def _read_ticks():
    from sweepick.integration.sweepick_observation_session import open_readonly_bus
    out = {}
    for side in ("left", "right"):
        b = open_readonly_bus(side)
        try:
            out[side] = dict(ticks={n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}, torque={n: int(v) for n, v in b.sync_read("Torque_Enable", normalize=False).items()},
                             status={n: int(v) for n, v in b.sync_read("Status", normalize=False).items()})
        finally:
            b.disconnect()
    return out


def top_frame(workdir, wait_s=30.0, depth_frames=30):
    """One image and a depth map from the SDK reader. The depth is the per-pixel median of the valid readings of
    `depth_frames` consecutive FrameSets of the static scene (a single FrameSet leaves holes on plain surfaces)."""
    from sweepick.integration.sweepick_observation_session import SdkTop
    resident = EPISODE is not None and EPISODE.top is not None                  # the session's one SDK reader: only FrameSets received AFTER this call are used
    s = EPISODE.top if resident else SdkTop(workdir, depth_frames + 40); t0 = time.monotonic(); fs, stack = None, []
    first = s.get(); seen = first["n"] if resident and first is not None else 0
    try:
        while time.monotonic() - t0 < wait_s and len(stack) < depth_frames:
            cur = s.get()
            if cur is not None and cur["n"] != seen and cur["n"] >= 5 and cur["bgr"] is not None:
                seen, fs = cur["n"], cur; stack.append(cur["zd"].astype(np.float32))
            time.sleep(0.01)
        K = s.K
    finally:
        closed = dict(resident_reader="kept open by the session's recorder", framesets_received=s.count, reader_error=s.error) if resident else s.close()
    if fs is None:
        raise RuntimeError(f"no FrameSet from the top camera within {wait_s} s ({s.error})")
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
    from sweepick.integration.sweepick_observation_session import WristCam
    resident = EPISODE is not None and "left_wrist" in EPISODE.wrists
    c = EPISODE.lend("left_wrist") if resident else WristCam("left_wrist", "/dev/dapier/left_wrist_rgb"); got, t0 = [], time.monotonic()
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
    scope="left wrist camera, 320x240, pre-grasp height of a top grasp near the board. Not an extrinsic calibration; other poses are not covered",
    object_colour="Lab a* >= 140 (the red development block); checked against edges read by eye on six stored frames (2 .. 4 px)",
    jaw_reference="inner edges of the two jaw tips on the bottom image row against a plain bright background (folded pose): 64 / 246 px at gripper tick 3366 in three stored frames",
    aligned_slope_deg=4.7, slope_per_jaw_deg=0.87,
    aligned_basis="PROVISIONAL, vertical (tilt 0) top grasp only. Run 10 of 2026-10-08: far top edge at 4.5 .. 4.9 deg in three looks (pre-grasp, pre-grasp, above the block) with the pads coming down beside the block and the slope unchanged (0.35 deg). "
                  "Run 13: the top depth gave the block's yaw as -12.5 deg, the wrist followed it (jaw axis 90.8 -> 77.5 deg by measured joints), the edge stood at -6.9 deg and the block was turned at the first stop (to -3.4 deg; afterwards 5 mm / 4 deg off in the top view). "
                  "Response 0.87 deg of slope per deg of jaw yaw from those two runs, which assumes the block stood the same in both; the response measured in the session replaces it. The camera is fixed to the hand, so the slope of a block square to the jaws does not depend on where the hand is",
    slope_tolerance_deg=3.0, slope_basis="far top edge of the block in the image: 9.3 and 10.9 deg in two pre-grasp frames of the same block (1.6 deg apart); after the descent that left a pad on the block it was -0.3 deg (11 deg change)",
    width_role="the block's width in the image is an AUXILIARY sign of approach only (it must grow when the hand comes down). It is not a height: the 201 .. 207 px of the two contacts and the 228 .. 230 px of the person-placed grasp are not targets",
    lateral_error_seen_mm=8.5, lateral_basis="largest offset of the block from the jaw midline at the pre-grasp pose so far: 4.2 mm (PICK01), 7.1 and 8.5 mm (PICK02 runs 6 and 7, 2026-10-08): how far the hand has had to be shifted along the jaw axis",
    measure_px=4, measure_basis="automatic edges against edges read by eye: 2 .. 4 px")


def wrist_object(bgr, object_width_m=0.04):
    """The block in the wrist image: left / right edge on its widest row, centre, width, slope of its far top edge."""
    import cv2
    a = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[..., 1].astype(int) >= 140
    a = cv2.morphologyEx(a.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(a)
    if n < 2 or st[1:, cv2.CC_STAT_AREA].max() < 1500:
        return dict(visible=False)
    m = lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])); rows = np.flatnonzero(m.sum(axis=1) >= 10)
    w = {int(y): (int(np.flatnonzero(m[y])[0]), int(np.flatnonzero(m[y])[-1])) for y in rows}
    y = max(w, key=lambda k: w[k][1] - w[k][0]); xl, xr = w[y]
    cols = [x for x in range(m.shape[1]) if m[:, x].sum() >= 10]; mid = cols[len(cols) // 5: len(cols) - len(cols) // 5]
    slope = float(np.degrees(np.arctan(np.polyfit(mid, [int(np.flatnonzero(m[:, x])[0]) for x in mid], 1)[0]))) if len(mid) >= 10 else None
    return dict(visible=True, edges_px=[xl, xr], row=y, centre_px=(xl + xr) / 2, width_px=xr - xl, mm_per_px=object_width_m * 1000 / max(xr - xl, 1), far_edge_slope_deg=slope, touches_border=bool(xl <= 1 or xr >= m.shape[1] - 2))


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
    out = Path(out); d = out / "looks" / tag; d.mkdir(parents=True, exist_ok=True)
    top = top_frame(d / f"sdk_{int(time.time())}"); t_top = time.time()        # a fresh folder: a stop file left by an earlier reader would end this one at once
    arms = read_ticks(); t_q1 = time.time()                                    # joints read AFTER the top frames were taken (the reader needs about 13 s to start), not before
    ticks = {s: arms[s]["ticks"] for s in arms}
    scene = analyse(top["bgr"], top["depth_mm"], top["K"], ticks); scene["_out"] = str(out); scene.update(K=list(top["K"]), tag=tag, arms=arms, stamps=top["stamps"], depth=dict(frames=top["depth_frames"], valid_share=top["valid_share"]), wrist=wrist_frame(d / "left_wrist.jpg"))
    arms2 = read_ticks(); t_q2 = time.time()                                   # ... and again after the wrist frames: the arm must not have moved in between
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
            scene["wrist"]["offset"] = wrist_offset(scene["wrist"]["object"], st.get("jaw_reference"), scene["wrist"].get("tips"), gripper_tick=arms2["left"]["ticks"].get("gripper"))
    import cv2
    cv2.imwrite(str(d / "top.png"), top["bgr"]); np.save(d / "depth_mm.npy", top["depth_mm"])
    if scene["ok"]:
        overlay(scene, top["bgr"], d / "overlay.png")
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
    if not scene.get("tool_left"):
        return None                                                           # no frame chain in this look (board not found): unknown, not zero
    return [float(a - b) for a, b in zip(scene["tool_left"], o)]


def support_region(out, state, scene, top, radius_m=0.03):
    """The place where the block stood, by the EXISTING area observation (tjj_perception.observe_area through
    sweepick_local_loop.real_area): OCCUPIED / CLEAR / UNKNOWN (not fully observed). A pixel without depth is 'not observed'."""
    import mujoco
    from sweepick.integration.sweepick_observation_session import real_area
    c = state_object(out, state)[0]; m = model(); Tb = np.eye(4); Tb[:3, 3] = DATUM_IN_MODEL_BASE; Tw = np.eye(4); Tw[:3, 3] = m.body("left_base").pos
    pb = np.linalg.inv(Tw @ Tb @ T_DATUM_FROM_BOARD) @ np.r_[c[0], c[1], BOARD_Z_IN_WORLD, 1.0]                       # into the printed board's frame
    r = real_area(top["depth_mm"], top["K"], scene["board"]["T_camera_from_board"], 0.0, (float(pb[0]), float(-pb[1])), radius_m)
    return dict(r, centre_board_xy=[float(pb[0]), float(-pb[1])], radius_m=radius_m, note="the arm above the place also counts as 'occupied' here; the result is used only as CLEAR = validly observed and empty")


def correction_headroom(scene, plan):
    """Does the chosen grasp stay plannable when the hand has to be shifted along the jaw axis by as much as the wrist image has
    asked for so far (plus what the image resolves)? Run 7 of 2026-10-08 approached a block that needed a 10.6 mm shift and
    had no plannable grasp left for it."""
    need = WRIST["lateral_error_seen_mm"] + WRIST["measure_px"] * 0.27; ax = np.array(list(plan["placement_at_grasp"]["jaw_axis_xy"]) + [0.0]); keep = dict(tilt_deg=plan["tilt_deg"], jaw=plan["jaw"]); res = {}
    for sgn in (-1.0, 1.0):
        q = plan_pick(scene, tool_offset=sgn * need / 1000.0 * ax, keep=keep); res[f"{sgn * need:+.1f} mm"] = bool(q.get("ok"))
    return dict(shift_mm=need, basis=WRIST["lateral_basis"], plannable=res)


def observe(out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    scene = look(out, "00_observe"); state = dict(schema="tjj.pick01.v1", scene="looks/00_observe/SCENE.json", stages={}, home_ticks=None, plan=None)
    why = None
    if not scene["ok"]:
        why = scene["why"]
    elif any(v for s in scene["arms"].values() for v in s["torque"].values()):
        why = "torque is already on somewhere: another controller may own an arm"
    elif scene["object"] is None or not scene["object"]["class_match"]:
        why = f"no object of the expected class (40 mm cube) was found on the desk: {scene['object']}"
    else:
        plan = plan_pick(scene); state["plan"] = plan
        if not plan["ok"]:
            why = f"the planner found no pick path: {plan['why']}"
        elif plan["model_jaw_gap_now_m"] <= plan["object_width_m"]:
            why = f"the jaws are not open wide enough in the model ({plan['model_jaw_gap_now_m'] * 1000:.1f} mm) for the object"
        elif (lambda h: h is not None and state.update(correction_headroom=h) is None and not all(h["plannable"].values()))(correction_headroom(scene, plan)):
            why = f"the grasp found here cannot be shifted by the correction the wrist image has asked for so far ({state['correction_headroom']}): the block stands at the edge of where this grasp works"
        elif not (scene.get("wrist") or {}).get("jaw_reference", {}).get("readable"):
            why = "the jaw tips cannot be read in the wrist image of the start pose (needs a plain bright background): no reference for the alignment"
        elif plan["room_each_side_m"] * 1000 <= WRIST["measure_px"] * 0.27:
            why = f"the grasp the arm can make here leaves {plan['room_each_side_m'] * 1000:.1f} mm each side (jaw axis leaning {plan['jaw_lean_deg']:.1f} deg), not more than the wrist image can resolve: the block has to stand differently for this arm"
    state.update(jaw_reference=dict((scene.get("wrist") or {}).get("jaw_reference") or {}, gripper_tick=(scene.get("ticks") or {}).get("left", {}).get("gripper")) if scene["ok"] else None, wrist_record=WRIST, run_name="PICK02", control="SENSOR_PLANNED / SUPERVISED_AUTOMATED (no ACT)", evidence=[],
                 sources={f: hashlib.sha256(source_path(f).read_bytes()).hexdigest() for f in ("sweepick_pick01.py", "sweepick_move_a.py", "sweepick_progress.py", "sweepick_grasp.py", "sweepick_progress_profile.json", "sweepick_jaw_profile_left.json", "sweepick_motion_profile.json")})
    state.update(ready=why is None, why=why, home_ticks=scene.get("ticks", {}).get("left") if scene["ok"] else None, right_ticks=scene.get("ticks", {}).get("right") if scene["ok"] else None)
    (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str)); return state


# ---- one stage = one call of the existing executor -------------------------------------------------------------------
def stage_request(state, name, mp, on_now=None):
    """mode / target / powered joints of a stage. The gripper's close target is the closed end of its calibrated range:
    the end of the commanded path, not an opening for this object; the executor stops when the gripper stops advancing."""
    tk, home = state["plan"]["ticks"], state["home_ticks"]
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
    if name == "lower":
        return dict(mode="next", target=tk["descend"], torque_joints=ALL, scene="object: block carried by the left hand, left pad - block and block - desk contact allowed")
    if name == "open":
        return dict(mode="next", target=dict(gripper=home["gripper"]), torque_joints=ALL, scene="object: block on the desk, left pad - block contact allowed")
    if name == "retreat":
        return dict(mode="next", target=tk["pregrasp"], torque_joints=ALL, scene="block")
    if name == "retreat_arm":
        return dict(mode="next", target=tk["pregrasp"], torque_joints=ARM, scene="block")
    if name == "home_arm":
        return dict(mode="return", rest={n: home[n] for n in ARM}, torque_joints=ARM, scene="block")
    if name == "abort_home":                                                  # from wherever a stage ended holding: back to the start pose with the joints that are on, then release
        on = [n for n in ALL if on_now[n]]
        return dict(mode="return", rest={n: home[n] for n in on}, torque_joints=on, scene="block")
    return dict(mode="return", rest={n: home[n] for n in ALL}, torque_joints=ALL, scene="block")


def certify(traj_file, out_file, scene_file):
    r = subprocess.run([sys.executable, "-m", "sweepick.control.sweepick_collision_preview", str(traj_file), str(CAND), str(CAL), str(ASM), str(out_file), str(scene_file)], capture_output=True, text=True, env=dict(__import__("os").environ, MUJOCO_GL="egl"))
    if not Path(out_file).exists():
        raise RuntimeError(f"collision check did not run: {r.stderr[-400:]}")
    return json.loads(Path(out_file).read_text())


def object_check(doc, block, *, carried, right_ticks, grasp_arm_ticks=None):
    """Collision result of a commanded path WITH the object in the scene, by the SIM teacher's own check: robot self
    contact, robot - desk and any robot part on the block are contacts; only left pad - block and block - desk are allowed.
    carried: the block moves rigidly with the left tool point from where it was at `grasp_arm_ticks`."""
    import mujoco, hashlib
    from sweepick.control import sweepick_trajectory_profile as tt
    from sim_data_factory.teacher import HandoverTeacher
    mp = mapping(); m = model(block); t = HandoverTeacher(m); d = mujoco.MjData(m); traj = doc["trajectory"]
    adr = t.cm.block_qpos; block_q = m.qpos0[adr:adr + 7].copy()
    def ctrl(left):
        q = rc.ticks_to_q12(mp, dict(left=left, right=right_ticks)); sweepick_kin.set_q12(m, d, q); return d.qpos.copy(), d.qpos[t.plan.adr].copy()
    rel = None
    if carried:
        base, c = ctrl(grasp_arm_ticks); t.plan.pose(base, c, block_q); ps, rot = t.plan.site_frame("left")
        rb = np.empty(9); mujoco.mju_quat2Mat(rb, block_q[3:]); rel = np.eye(4); rel[:3, :3] = rot.T @ rb.reshape(3, 3); rel[:3, 3] = rot.T @ (block_q[:3] - ps)
    first = None; steps = traj["steps"]
    for k in range(steps):
        left = {n: float(traj["goals"][n][k]) for n in rc.JOINTS}
        base, c = ctrl(left)
        bq = t.plan.carried_block(base, c, ("left", rel)) if carried else block_q
        bad = t.plan.collisions(base, c, bq, allow_pad_block=("left",), clearance=0.0)
        if bad:
            first = dict(step=k, pairs=[list(map(str, b)) for b in bad[:4]]); break
    msha = hashlib.sha256(CAND.read_bytes()).hexdigest()
    return dict(schema="tjj.object-collision-result.v1", identity=tt.identity(doc["side"], traj, msha), nominal_verdict="CLEAR" if first is None else "CONTACT", first_nominal_contact=first, nominal_steps_checked=steps,
                scene="assembled dual model, desk at the real height, block " + ("carried by the left tool point" if carried else "on the desk"), allowed=["left pad - block", "block - desk"], not_allowed="every other pair, housings and arm links on the block included",
                check="sim_data_factory.teacher.Planner.collisions (unchanged)", mapping_status="CANDIDATE (verified=false)")


def sweep_doc(arm_ticks, grip_from, grip_to):
    """The whole possible closing sweep of the gripper at one arm pose, as a path the object check can read."""
    n = int(abs(grip_from - grip_to)) + 1
    goals = {j: [float(arm_ticks[j])] * n for j in ARM}; goals["gripper"] = [float(v) for v in np.linspace(grip_from, grip_to, n)]
    return dict(side="left", trajectory=dict(start=dict(arm_ticks, gripper=grip_from), target=dict(gripper=grip_to), moving=["gripper"], dt=0.02, steps=n, goals=goals))


def judge(out, state, what, scene):
    """Observation gates. Each returns (ok, record); the record goes into STATE['evidence'] with the numbers it used."""
    from sweepick.control import sweepick_gripper_contact_hold as tg
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
        v = tool_offset(state); return pad_placement(m, d, c + np.array([v[0], v[1], 0.0]), yaw)      # laterally against where the image has put the block relative to the hand so far (the corrections of this session)
    pm = by_joints(); rec["pads_by_measured_joints"] = pm
    def centred(need_confirmed=True):
        if not off.get("readable"):
            return dict(ok=False, unknown=off.get("why", "not readable"))
        if need_confirmed and not off.get("reference_confirmed_in_this_frame"):
            return dict(ok=False, unknown="no jaw tip readable in this frame agrees with the jaw reference: the midline is not confirmed for this image", tips=off.get("tips_in_this_frame"))
        return dict(ok=bool(tol > 0 and abs(rec["lateral_error_mm"]) <= tol), error_mm=rec["lateral_error_mm"], allowed_mm=tol, basis="room of this grasp from the pad meshes, less the image uncertainty of this frame")
    def pads_at_grasp():
        if pm is None or not grasp.get("pads"):
            return dict(ok=False, unknown="no joint reading / no planned pad placement")
        e = pm["tool_along_jaw_mm"] - grasp["tool_along_jaw_mm"]                # where the tool really is along the jaw axis against where the grasp was planned
        g = [grasp["pads"][n]["gap_mm"] + grasp["pads"][n]["side"] * e for n in PAD_GEOMS]
        return dict(ok=bool(min(g) >= unc), tool_error_along_jaw_mm=e, predicted_gaps_mm=g, needed_mm=unc, basis="the planned pad gaps at the grasp pose shifted by the lateral error of the measured joints; each gap must exceed what the image can resolve")
    def pads_above_top():
        if pm is None:
            return dict(ok=False, unknown="no joint reading")
        low = min(pm["pads"][n]["lowest_above_top_mm"] for n in PAD_GEOMS)
        return dict(ok=bool(low > 0), lowest_pad_vertex_above_top_mm=low, basis="pad meshes at the measured joints: no pad vertex at or below the block's top face while the hand is still above it")
    if what == "ready_to_descend":
        checks = dict(centred=centred(), pads_at_grasp_by_measured_joints=pads_at_grasp()); ok = all(c["ok"] for c in checks.values())
        rec.update(checks=checks, correctable=bool(not ok and "unknown" not in checks["centred"] and "unknown" not in checks["pads_at_grasp_by_measured_joints"] and not checks["centred"]["ok"]))
        rec["why"] = "the block sits between the jaws within the room the pad meshes leave, in the image and by the measured joints" if ok else "; ".join(f"{k}: {c.get('unknown') or ({a_: (round(b_, 2) if isinstance(b_, float) else b_) for a_, b_ in c.items() if a_ not in ('basis', 'ok')})}" for k, c in checks.items() if not c["ok"])
    elif what == "ready_for_final_descent":
        pre = next((e for e in reversed(state.get("evidence", [])) if e["what"] in ("ready_to_descend", "ready_for_final_descent") and e["ok"]), None)
        checks = dict(centred=centred(), pads_at_grasp_by_measured_joints=pads_at_grasp(), pads_above_the_top_face=pads_above_top())
        if obj.get("visible") and pre and pre["wrist_object"].get("visible"):
            po = pre["wrist_object"]; ds = None if obj.get("far_edge_slope_deg") is None or po.get("far_edge_slope_deg") is None else abs(obj["far_edge_slope_deg"] - po["far_edge_slope_deg"])
            checks["not_turned"] = dict(slope_change_deg=ds, ok=bool(ds is not None and ds <= WRIST["slope_tolerance_deg"]), basis="a changed edge slope shows the block was turned; an unchanged slope alone shows nothing else")
            checks["approached"] = dict(width_px=[po["width_px"], obj["width_px"]], ok=bool(obj["width_px"] >= po["width_px"] - 2 * WRIST["measure_px"]), basis="auxiliary: the block must not look smaller than in the last accepted image (it grows as the hand comes down; a second look at the same height shows the same size); the size itself is not used as a height")
        else:
            checks["comparable_image"] = dict(ok=False, unknown="no accepted earlier image to compare with, or the block is not visible")
        ok = all(c["ok"] for c in checks.values()); bad = [k for k, c in checks.items() if not c["ok"]]
        rec.update(checks=checks, correctable=bool(bad and set(bad) <= {"centred", "pads_at_grasp_by_measured_joints"} and "unknown" not in checks["centred"] and not checks["centred"]["ok"]),
                   why="between the jaws in the image and by the measured joints, pads above the top face, block not turned" if ok else f"not ready: {bad}")
    elif what == "ready_to_close":
        pre = next((e for e in reversed(state.get("evidence", [])) if e["what"] == "ready_for_final_descent" and e["ok"]), None); checks = {}; unknown = []
        if not (obj.get("visible") and obj.get("stable", True) and off.get("readable") and pre and pre["wrist_object"].get("visible")) or obj.get("touches_border"):
            unknown.append("the block is not fully readable in the wrist image after the descent (or no accepted image from before it)")
        else:
            po, pe = pre["wrist_object"], pre
            arrived = pm is not None and None not in pm["gaps_mm"]
            checks["pads_beside_the_block_by_measured_joints"] = dict(gaps_mm=None if pm is None else pm["gaps_mm"], lowest_above_top_mm=None if pm is None else [pm["pads"][n]["lowest_above_top_mm"] for n in PAD_GEOMS], ok=bool(arrived and min(pm["gaps_mm"]) >= 0), basis="pad meshes at the measured joints: both pads reach below the top face and neither is over the block (candidate model)")
            shift = rec["lateral_error_mm"] - pe.get("lateral_error_mm", 0.0); lim = unc + ((pe.get("wrist_offset") or {}).get("uncertainty_mm") or 1.0)
            rec["offset_change_since_the_last_stop"] = dict(change_mm=shift, resolvable_mm=lim, note="recorded, not a check: the offset from the jaw midline changes when the HAND settles differently at the two heights, so it does not say the block moved. Where the block is against the pads is the 'centred' check; whether it was turned is 'not_turned'")
            ds = None if obj.get("far_edge_slope_deg") is None or po.get("far_edge_slope_deg") is None else abs(obj["far_edge_slope_deg"] - po["far_edge_slope_deg"])
            checks["not_turned"] = dict(slope_change_deg=ds, ok=bool(ds is not None and ds <= WRIST["slope_tolerance_deg"]), basis="a changed edge slope shows the block was turned; an unchanged slope alone shows nothing else")
            checks["centred"] = dict(error_mm=rec["lateral_error_mm"], allowed_mm=tol, ok=bool(tol > 0 and abs(rec["lateral_error_mm"]) <= tol), reference="jaw midline confirmed at the previous height; at the grasp height the tips are behind the block")
            checks["approached"] = dict(width_px=[po["width_px"], obj["width_px"]], ok=bool(obj["width_px"] > po["width_px"]), basis="auxiliary only: larger than at the last stop above the block")
        ok = not unknown and all(c["ok"] for c in checks.values())
        # 'not turned' is independent of where the hand stands; the offset from the midline is not (a hand that stopped short shows as a shift even when the block has not moved).
        # Both contacts so far turned the block by about 10 degrees; a push that does not turn it is not seen by this.
        rec["block_undisturbed_in_the_image"] = bool(obj.get("visible") and obj.get("stable", True) and not obj.get("touches_border") and checks.get("not_turned", {}).get("ok"))
        rec.update(checks=checks, unknown=unknown, placement="PADS_BESIDE_THE_BLOCK (image and measured joints agree)" if ok else ("NOT_DISTINGUISHABLE" if unknown else "NOT_CONFIRMED"),
                   why="the block was not moved or turned by the last descent and the measured joints put both pads beside it" if ok else
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


def stage(out, name, *, execute=False, run=mv.run, grip=mv.grip, cert=certify, cameras=None, run_kw=None, torque_now=lambda: read_ticks()["left"]["torque"]):
    import mujoco
    out = Path(out); state = json.loads((out / "STATE.json").read_text()); mp = mapping(); run_kw = dict(run_kw or {})
    rec = dict(stage=name, execute=bool(execute), t=time.time())
    def done(result, **kw):
        rec.update(result=result, **kw)
        wrote = any((kw.get("writes") or {}).values())
        if execute and result == "RAN" and kw.get("controller_result") == "REFUSED" and not wrote:
            state.setdefault("refusals", []).append(dict(rec)); (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str))      # requested and refused before any write: kept as a record, it does not close the ways back
        elif execute and result == "RAN":                                     # only a stage that reached the executor and wrote is part of the session's history
            state.setdefault("order", []).append(name)
            if kw.get("controller_result") == "HOLDING" or (name in SHORT_OK and kw.get("controller_result") == "TARGET_NOT_REACHED_HOLDING"):
                state["height"] = {"pregrasp": "pregrasp", "descend_a": "descend_a", "descend_b": "descend_b", "descend": "descend", "descend_settle": "descend", "retreat_arm": "pregrasp", "retreat": "pregrasp", "lift": "lift", "lower": "descend"}.get(name, state.get("height"))
                if name == "align":
                    state["pending_align"] = False
            state["stages"][name] = rec; (out / "STATE.json").write_text(json.dumps(state, indent=1, default=str))
        (out / f"stage_{name}.json").write_text(json.dumps(rec, indent=1, default=str)); return rec
    if not state.get("ready"):
        return done("NOT_RUN", why=f"observe did not end ready: {state.get('why')}")
    changed = [f for f, h in (state.get("sources") or {}).items() if hashlib.sha256(source_path(f).read_bytes()).hexdigest() != h]
    if changed and execute:
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
            c_ = object_check(doc, obj, carried=name in ("lift", "lower"), right_ticks=state["right_ticks"], grasp_arm_ticks=None if grasp_ticks is None else {n: grasp_ticks[n] for n in ALL})
        else:
            (folder / f"{tag}.TRAJECTORY.json").write_text(json.dumps(doc)); c_ = cert(folder / f"{tag}.TRAJECTORY.json", folder / f"{tag}.CERT.json", scene_file)
        (folder / f"{tag}.CHECK.json").write_text(json.dumps(c_, indent=1, default=str)); return c_
    kw = dict(level="model", torque_joints=rq["torque_joints"], **run_kw)
    recs = []
    def record():
        if cameras is not None:
            from sweepick.control.sweepick_gripper_contact_inspection import FrameRecorder
            try:
                for cam in cameras():
                    recs.append((cam, FrameRecorder(cam, sdir / "frames", [name])))
            except Exception as e:                                             # recording is monitoring only: without it the stage still runs (its gate was already passed, and a way back must not depend on a camera)
                rec["recording_error"] = f"{type(e).__name__}: {e}"
            if recs:
                time.sleep(1.0)                                                # let the camera stream start before the motor bus is used: on 2026-10-08 a status packet was corrupted in the same second the stream opened (cameras and motor adapters share one USB hub)
    def stop_recording():
        frames = [f for cam, r in recs for f in r.close()]
        for cam, r in recs:
            cam.close()
        if frames:
            (sdir / "frames/INDEX.json").write_text(json.dumps(frames))
        return frames
    if rq["mode"] == "grip":
        jd = json.loads(mv.JAW_PROFILE.read_text()); arm = {n: float(v) for n, v in read_now(run_kw)["ticks"].items()}; lo = float(mp["left"]["gripper"]["lo"])
        doc = sweep_doc(arm, arm["gripper"], lo + float(jd["end_stop_ticks_above_closed"]["value"]))
        c = check(doc, sdir, "sweep"); c.update(arm_ticks={n: arm[n] for n in ARM}, sweep=[arm["gripper"], lo + float(jd["end_stop_ticks_above_closed"]["value"])])
        rec["collision"] = [dict(verdict=c["nominal_verdict"], first_contact=c.get("first_nominal_contact"), steps=c["nominal_steps_checked"], scene=c["scene"], allowed=c["allowed"])]
        if c["nominal_verdict"] != "CLEAR":
            return done("NOT_RUN", why=f"the closing sweep touches something that is not the intended contact: {c.get('first_nominal_contact')}", controller_result="PLANNED")
        if not execute:
            return done("PLANNED", controller_result="PLANNED")
        record()
        try:
            code, log = grip("left", sdir / "run1", torque_joints=ALL, cert=c, level="model", execute=True, stop_requested=Path(str(out) + ".STOP").exists, **run_kw)
        finally:
            frames = stop_recording()
        return done("RAN", controller_result=log["result"], exit_code=code, outcome=log.get("outcome"), stopped_by=log.get("stopped_by"), refused=log.get("refused"), writes=log.get("writes"), grip=log.get("grip"), end=log.get("end"),
                    needs_a_person=log.get("needs_a_person"), frames=len(frames), owner="sweepick_grasp.Jaw", meaning="a contact candidate of the jaws; whether it is the block is decided by the observation that follows")
    kw.update(other_arm_ticks=state["right_ticks"])
    if rq["mode"] == "next" and execute:
        here = read_now(run_kw)["ticks"]
        if all(round(here[n]) == round(v) for n, v in rq["target"].items()):   # e.g. a correction smaller than one tick: the arm already stands there, nothing is sent
            return done("RAN", controller_result="HOLDING", exit_code=0, writes=dict(Goal_Position=0, Torque_Enable=0), outcome=dict(command_completed=True, target_reached=True, torque_released=False), end=dict(ticks=here), note="already at the target ticks; nothing was written")
    kw.update(dict(rest=rq["rest"]) if rq["mode"] == "return" else dict(target=rq["target"]))
    trim = ARM if name in TRIM else []
    tries = []
    for attempt in (1, 2):                                                    # the second pass only re-plans when the arm reading changed between plan and start (nothing written)
        code, plan = run("plan" if rq["mode"] != "return" else "return", "left", sdir / f"plan{attempt}", **kw)
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
        try:
            code, log = run(rq["mode"], "left", sdir / f"run{attempt}", execute=True, cert=c, settle_trim=trim, trim_check=trim_check, start_recheck=lambda doc: check(doc, sdir / f"run{attempt}", "start"), hold_s=5.0 if name == "lift" else 0.0, stop_requested=Path(str(out) + ".STOP").exists, **kw)
        finally:
            frames = stop_recording(); recs.clear()
        rebind = log["result"] == "REFUSED" and (log.get("refused") or {}).get("gate") == "certificate_binding" and log["writes"] == dict(Goal_Position=0, Torque_Enable=0)
        if not (rebind and attempt == 1):
            break
    rec["monitor"] = motion_monitor(sdir / "frames")
    if name in ("lift", "lower"):
        rec["jaw_during"] = jaw_during(log, float(json.loads(mv.JAW_PROFILE.read_text())["residual_floor_ticks"]["value"]))
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
    """Fresh read-only joint read of the left arm (the fake of a test supplies its own)."""
    if run_kw and "open_bus" in run_kw:
        b = run_kw["open_bus"]("/dev/dapier/left_arm"); return dict(ticks={n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()})
    return read_ticks()["left"]


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


def correct(out, along_jaw_mm, basis, assisted=False, axis_xy=None, above_b=None, yaw_deg=None, yaw_source=None):
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
        m_ = model(); d_ = mujoco.MjData(m_); sweepick_kin.set_q12(m_, d_, q12); mujoco.mj_forward(m_, d_); axis_xy = tool_axis(m_, d_)["jaw_axis_xy"]
    if axis_xy is None:
        return dict(plan_ok=False, why="no jaw axis from the look that measured the error")
    jaw = np.array([axis_xy[0], axis_xy[1], 0.0]); shift = along_jaw_mm / 1000.0 * jaw; total = tool_offset(state) + shift
    if yaw_deg is not None:                                                    # the block's yaw as the WRIST image shows it against the jaws: an observation of the block, recorded as one (the place it stands is not changed)
        c_, y_ = state_object(out, state)
        state.setdefault("object_updates", []).append(dict(centre_world=[float(v) for v in c_], yaw_rad=float(y_ + np.radians(yaw_deg)), yaw_change_deg=float(yaw_deg), source=yaw_source, uncertainty_deg=WRIST["slope_tolerance_deg"], kind="YAW FROM THE WRIST IMAGE (centre unchanged)"))
    scene = json.loads((out / state["scene"]).read_text()); c0, yaw = state_object(out, state)
    scene = dict(scene, q12=[float(v) for v in q12], object=dict(scene["object"], centre_world=[float(v) for v in c0], yaw_rad=float(yaw)))
    plan = plan_pick(scene, tool_offset=total, above_b=state.get("above_b_m"), keep=dict(tilt_deg=state["plan"]["tilt_deg"], jaw=state["plan"]["jaw"]))
    rec = dict(source="ASSISTED: amount given by a person" if assisted else "wrist image of this session", kind="HAND TARGET OFFSET (not an object measurement)", along_jaw_mm=along_jaw_mm, axis_xy=[float(axis_xy[0]), float(axis_xy[1])],
               tool_shift_world_m=[float(v) for v in shift], tool_offset_total_world_m=[float(v) for v in total], object_observed_world_m=[float(v) for v in c0], basis=basis, measured_ticks=ticks["left"], plan_ok=plan["ok"], previous_ticks=state["plan"]["ticks"])
    if not plan["ok"]:
        rec["why"] = plan["why"]; (out / "correction_refused.json").write_text(json.dumps(rec, indent=1, default=str)); return rec
    if along_jaw_mm == 0.0 and above_b is not None:                             # only the second stop was added: the poses already reached stay exactly as they are (no move at this height)
        for h in ("pregrasp", "descend_a"):
            plan["ticks"][h] = dict(state["plan"]["ticks"][h])
        rec["kind"] = "SECOND STOP ADDED (no hand shift)"
    state.setdefault("correction", []).append(rec); state["plan"] = plan; state["pending_align"] = bool(along_jaw_mm != 0.0 or yaw_deg is not None)
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
               object=dict(observed_on_the_desk_world_m=[float(v) for v in state_object(out, st)[0]], yaw_rad=float(state_object(out, st)[1]), size_m=[0.04, 0.04, 0.04], kind="red 40 mm development block", frame="assembled dual model world (left base at its model pose), through the candidate chain camera <- board <- ruler datum",
                           hand_target_offset_world_m=[float(v) for v in tool_offset(st)], in_hand="not measured as a pose: at the tool point of the left gripper by the wrist image; the tool point comes from the candidate joint mapping",
                           uncertainty="top-depth centre a few mm (depth reads about 2 % long); candidate q12 and chain unverified"),
               tool_left_world_m=scene.get("tool_left"), jaw_axis_xy=scene.get("jaw_axis_xy"), grasp_evidence=dict(jaw=close.get("grip"), jaw_during_lift=lift.get("jaw_during"), held_after_lift=evidence),
               identifiers=dict(sources=st.get("sources"), mapping="CANDIDATE_Q12_CONFIG.json sha256 " + hashlib.sha256(CAND.read_bytes()).hexdigest(), scene="dual_scene_desk_both.mjb sha256 " + hashlib.sha256(SCENE.read_bytes()).hexdigest(), motion_profile="sweepick_motion_profile.json", jaw_profile="sweepick_jaw_profile_left.json"))
    (out / "HANDOFF_SNAPSHOT.json").write_text(json.dumps(doc, indent=1, default=str)); return doc


def wrist_cams():
    if EPISODE is not None and "left_wrist" in EPISODE.wrists:
        return [EPISODE.lend("left_wrist")]
    from sweepick.integration.sweepick_observation_session import WristCam
    return [WristCam("left_wrist", "/dev/dapier/left_wrist_rgb")]


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
        _rec(lambda: ep.mark(n, skill=SKILL.get(n), owner="sweepick_move_a (left arm)" if n not in ("close", "open") else "sweepick_grasp.Jaw through sweepick_move_a.grip (left gripper)"), f"mark {n}")
        with ep.lease("left"):                                                   # the lease is about the port, not the record: if the recorder does not let go, the stage is not started (nothing was sent)
            r = stage_fn(o, n, execute=execute)
        # the stage has ended and its result is what the session acts on; nothing below may replace or lose it
        def after():
            try:
                folder = n if n != "align" else f"align{len(json.loads((Path(o) / 'STATE.json').read_text()).get('correction') or [])}"
            except Exception:
                folder = n
            ep.stage(n, Path(o) / folder, "left", r.get("controller_result") or r.get("result")); ep.mark(f"after_{n}", owner=None)
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
    tq = ((scene.get("arms") or {}).get("left") or {}).get("torque") or {}
    ev = dict(jaws_opened=bool(op.get("controller_result") == "HOLDING" and (op.get("reached") or {}).get("gripper") is not None),
              arm_home_and_released=bool(hm.get("controller_result") == "RETURNED_RELEASED" and tq and not any(tq.values())),
              place_validly_observed=bool(reg.get("valid")) if reg else None, place_occupied=(reg.get("state") == "OCCUPIED") if reg.get("valid") else None,
              object_seen=o is not None, object_top_height_m=None if o is None else o.get("top_height_above_depth_desk_m"), size_class=None if o is None else bool(o.get("class_match")),
              block_in_the_wrist_view=((scene.get("wrist") or {}).get("object") or {}).get("visible"))
    h = ev["object_top_height_m"]; ev["height_is_the_blocks"] = None if h is None else bool(abs(h - 0.04) <= 0.01)       # the class check's own tolerance (a quarter of the size), on the height
    need = ("jaws_opened", "arm_home_and_released", "place_validly_observed", "place_occupied", "object_seen", "height_is_the_blocks")
    if any(ev[k] is None for k in need):
        ev["verdict"] = "UNKNOWN"
    elif all(ev[k] for k in need):
        ev["verdict"] = "ON_THE_DESK_AT_THE_PLACE_JAWS_OPEN_ARM_AWAY" + ("" if ev["size_class"] else " (size class NOT confirmed by the top depth)")
    else:
        ev["verdict"] = "NOT_CONFIRMED"
    ev["revision"] = "put_back_evidence.v2, 2026-10-08; the original 'placed_back' (size class at the final look) is reported unchanged next to it"
    return ev


def session(out, *, execute=False, stage_fn=None, look_fn=None, observe_fn=None, second_stop=False, handoff_fn=None):
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
        trail.append(k); (out / "SESSION.json").write_text(json.dumps(dict(run="PICK02", trail=trail), indent=1, default=str))
        if EPISODE is not None:
            try:
                EPISODE.event("trail", **{a: b for a, b in k.items() if a in ("step", "gate", "ok", "result", "controller_result", "label", "placement", "height", "k", "end", "why")})
            except Exception as e:                                             # the record's quality; the session's own trail is already written
                rec_errors.append(f"trail event: {type(e).__name__}: {e}")
    try:
        st = observe_fn(out)
    except Exception as e:                                                     # e.g. a camera that does not open: nothing has been commanded yet, so this is simply 'not started'
        out.mkdir(parents=True, exist_ok=True); note(step="observe", ready=False, why=f"the observation could not be made: {type(e).__name__}: {e}")
        return dict(result="NOT_STARTED", why=f"{type(e).__name__}: {e}", trail=trail)
    note(step="observe", ready=st["ready"], why=st["why"], plan=None if not st.get("plan") else {k: st["plan"].get(k) for k in ("tilt_deg", "jaw", "jaw_lean_deg", "room_each_side_m", "considered")})
    if not st["ready"] or not execute:
        return dict(result="NOT_STARTED" if not st["ready"] else "PLANNED", trail=trail)
    S = lambda: json.loads((out / "STATE.json").read_text())
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
        yprev, ygain, ysrc = None, None, "prior (runs 10 and 13)"
        while True:
            sc = look_fn(out, f"{prefix}_{k}"); ok, ev = judge(out, S(), gate, sc); off = ev.get("wrist_offset") or {}
            err_px, err_mm = ev.get("lateral_error_px"), ev.get("lateral_error_mm")
            row = dict(step="visual", loop="STEPWISE_VISUAL_LOOP", height=prefix, k=k, gate=gate, ok=ok, why=ev["why"], frames=(sc.get("wrist") or {}).get("frames"), offset_px=off.get("offset_px"), offset_mm=off.get("offset_mm"), lateral_error_mm=err_mm, lateral_target_mm=ev.get("lateral_target_mm"), uncertainty_mm=off.get("uncertainty_mm"),
                       checks={k_: (c_.get("ok"), c_.get("unknown")) for k_, c_ in (ev.get("checks") or {}).items()},
                       allowed_mm=ev.get("tolerance_mm"), reference_confirmed=off.get("reference_confirmed_in_this_frame"), tool_xy_m=(sc.get("tool_left") or [None, None])[:2], joints_read=sc.get("joint_reads"), not_controlled=["image y (along the pads' width)", "yaw of the block against the jaws", "height (checked, not servoed)"])
            tool_now = sc.get("tool_left"); axis_now = sc.get("jaw_axis_xy")
            if prev is not None and off.get("readable") and tool_now is not None and axis_now is not None:
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
            slope = (ev.get("wrist_object") or {}).get("far_edge_slope_deg"); jaw_now = None if axis_now is None else float(np.degrees(np.arctan2(axis_now[1], axis_now[0])))
            if prefix == "01_pregrasp" and S()["plan"]["tilt_deg"] == 0.0 and slope is not None and jaw_now is not None:
                # the jaws are turned to the block's yaw from the WRIST image: the top depth's yaw was 10 deg off in run 13 and the pads then met the block's corners
                serr = WRIST["aligned_slope_deg"] - slope; row["square"] = dict(edge_slope_deg=slope, square_at_deg=WRIST["aligned_slope_deg"], error_deg=serr, tolerance_deg=WRIST["slope_tolerance_deg"], jaw_axis_deg=jaw_now, basis=WRIST["aligned_basis"])
                if yprev is not None:
                    dj = (jaw_now - yprev["jaw"] + 180.0) % 360.0 - 180.0; ds_ = slope - yprev["slope"]; row["square"]["response"] = dict(jaw_turned_deg=dj, slope_changed_deg=ds_, slope_per_jaw_deg=(ds_ / dj if abs(dj) > 0.5 else None))
                    if abs(dj) > 0.5 and abs(ds_) > 1.6 and ds_ * dj < 0:
                        row["end"] = "the edge slope changed the opposite way to the record when the wrist turned: not identified here"; note(**row); return "NOT_ALIGNED"
                    if abs(serr) > WRIST["slope_tolerance_deg"] and abs(serr) > abs(yprev["serr"]) - 1.6:
                        row["end"] = f"turning the wrist did not bring the jaws square to the block ({yprev['serr']:+.1f} -> {serr:+.1f} deg)"; note(**row); return "NOT_ALIGNED"
                    if abs(dj) > 2.0 and abs(ds_) > 1.6:
                        ygain, ysrc = ds_ / dj, "measured in this session"
                if abs(serr) > WRIST["slope_tolerance_deg"]:
                    turn_by = serr / (ygain or WRIST["slope_per_jaw_deg"]); row["square"]["correction"] = dict(turn_jaws_by_deg=turn_by, slope_per_jaw_deg=ygain or WRIST["slope_per_jaw_deg"], relation=ysrc)
                    c = correct(out, 0.0, f"wrist image {sc.get('tag')}: far top edge at {slope:+.1f} deg, square to the jaws at {WRIST['aligned_slope_deg']:+.1f} deg", axis_xy=axis_now, yaw_deg=turn_by, yaw_source=f"{sc.get('tag')} frames {(sc.get('wrist') or {}).get('frames')}")
                    row["plan_ok"] = c.get("plan_ok"); note(**row)
                    if not c.get("plan_ok"):
                        return "NOT_ALIGNED"
                    if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):
                        return "STOPPED"
                    yprev = dict(jaw=jaw_now, slope=slope, serr=serr); prev = None; k += 1; continue     # the lateral offset is measured afresh with the jaws turned
            if ok:
                note(**row); return "ALIGNED"
            if not ev.get("correctable"):
                row["end"] = "not a lateral offset that a correction along the jaw axis can remove (or not readable)"; note(**row); return "NOT_ALIGNED"
            if prev is not None and abs(err_mm) > abs(prev["err_mm"]) - off["uncertainty_mm"]:
                row["end"] = f"the error did not shrink by more than the image resolves ({prev['err_mm']:+.1f} -> {err_mm:+.1f} mm)"; note(**row); return "NOT_ALIGNED"
            g = gain if gain else image_response_prior(ev["wrist_object"]); move = -err_px / g
            row["correction"] = dict(tool_move_along_jaw_mm=move, px_per_mm=g, relation=src, basis="local 1-axis image response; axis = the jaw axis (image x). Sign from the PICK01 record, checked against every move")
            if tool_now is None or axis_now is None:
                row["end"] = "the tool position / jaw axis of this look is unknown (no frame chain): no correction is computed"; note(**row); return "NOT_ALIGNED"
            c = correct(out, move, f"wrist image {sc.get('tag')}: block {err_px:+.1f} px from where the plan puts it against the confirmed jaw midline; {g:.2f} px/mm ({src})", axis_xy=axis_now); row["plan_ok"] = c.get("plan_ok"); note(**row)
            if not c.get("plan_ok"):
                return "NOT_ALIGNED"
            if not run_stage("align", HOLDING_OK + ("TARGET_NOT_REACHED_HOLDING",)):     # short of the joint target but completed and holding: the next look measures where the hand really is
                return "STOPPED"
            prev = dict(off=off, err_px=err_px, err_mm=err_mm, tool=list(tool_now), axis=list(axis_now), commanded=move); k += 1

    r1 = visual_align("01_pregrasp", "ready_to_descend")
    if r1 != "ALIGNED":
        return end("NOT_ALIGNED_RETURNED_HOME", ["home_arm"]) if r1 == "NOT_ALIGNED" else dict(result="STOPPED_IN_ALIGN_HOLDING_NEEDS_A_PERSON", trail=trail)
    if not run_stage("descend_a"):
        return dict(result="STOPPED_IN_DESCEND_A_HOLDING_NEEDS_A_PERSON", trail=trail)
    r2 = visual_align("01a_above_block", "ready_for_final_descent")
    if r2 != "ALIGNED":
        return end("NOT_READY_ABOVE_THE_BLOCK_RETURNED_HOME", ["retreat_arm", "home_arm"]) if r2 == "NOT_ALIGNED" else dict(result="STOPPED_IN_ALIGN_HOLDING_NEEDS_A_PERSON", trail=trail)
    # a second stop closer to the top face, as close as what was MEASURED at the first stop allows: how far the tool really stood from where that stop was commanded (vertically), plus 1 mm for the block's own height
    pm, pa = (last_look["evidence"].get("pads_by_measured_joints") or {}), (S()["plan"].get("placement_above_block") or {})
    # second_stop is OFF by default: on the real arm a few-millimetre move of loaded joints does not start reliably (runs 10 and 11), and the first stop
    # already has the pads above the top face with the block centred; the last descent then starts from an accepted observation
    if second_stop and pm and pa and last_look["scene"].get("jaw_axis_xy") is not None:
        seen_err = abs(pm["tool_above_top_mm"] - pa["tool_above_top_mm"]); now_clear = min(pm["pads"][n]["lowest_above_top_mm"] for n in PAD_GEOMS); want = seen_err + 1.0
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
        observed = h.get("result") == "OBSERVED" and h.get("left_at_lift") is True and h.get("object_owner") == "LEFT"
        if observed:
            # OBSERVE scope reached its planned end: the right arm is home and released, the left hand never let go and stands at the pose it lifted to. The planned put-back is this scope's planned way home.
            note(step="handoff_observed", why=h.get("why"), end=h.get("end"), observation_file=h.get("observation_file"))
        elif not (h.get("result") == "NOT_READY" and h.get("commands_sent") is False and h.get("object_owner") == "LEFT"):
            return dict(result=f"HANDOFF_{h.get('result') or 'FAILED'}_HOLDING_NEEDS_A_PERSON", mode="FULL_CHAIN", held_after_lift=held, handoff=h, trail=trail,
                        note="the hand-over ended after it had commanded something (or it is not known whether it had): the put-back planned before the hand-over is NOT run by itself")
        if not observed:
            note(step="handoff_not_entered", why="the callback commanded nothing and the left hand holds as lifted: the planned put-back is still valid")
        try:
            ab = getattr(handoff_fn, "abandon", None); ab_r = ab() if callable(ab) and not observed else None
            if ab_r is not None:
                note(step="handoff_abandon", result=ab_r.get("result"))
        except Exception as e:
            note(step="handoff_abandon", error=f"{type(e).__name__}: {e}"); return dict(result="HANDOFF_ABANDON_FAILED_HOLDING_NEEDS_A_PERSON", mode="FULL_CHAIN", held_after_lift=held, handoff=h, trail=trail)
    done_ = back(["lower", "open", "retreat", "home"])                         # as planned, whatever the snapshot is for: the block is not kept in the air
    if done_:
        return done_
    sc = look_fn(out, "05_home"); placed = sc.get("object") is not None and bool(sc["object"].get("class_match")); note(step="look", gate="placed_back", ok=placed, object=sc.get("object") and sc["object"].get("centre_xy"), torque=((sc.get("arms") or {}).get("left") or {}).get("torque"))
    try:
        pb = put_back_evidence(S(), sc)
    except Exception as e:
        pb = dict(verdict="UNKNOWN", error=f"{type(e).__name__}: {e}")
    note(step="put_back_evidence", **pb)
    scope_note = dict(scope="OBSERVE", support_observation="DONE: SUPPORT_OBSERVATION.json; the left hand was not released; this is a partial run, not a hand-over") if (handoff_fn is not None and held and locals().get("observed")) else {}
    return dict(result="PICKED_LIFTED_AND_PUT_BACK" if held and placed else ("LIFT_NOT_CONFIRMED_RETURNED_HOME" if not held else "PUT_BACK_NOT_SEEN_RETURNED_HOME"), **scope_note, held_after_lift=held, placed_back=placed, put_back_evidence=pb, human_assistance="none recorded by the tool; a person's report overrides this", trail=trail)


def run_session(out, *, execute, record=False, full_chain=False, episode_fn=None, handoff=None, session_kw=None, scope="full", right_lift_m=None):
    """The session with its optional recorder and its optional full-chain callback, in this one process. The recorder is
    started before the first observation and closed after the last one; if it cannot start, a session that asked for it is
    not started (a record that silently is not there would be found out too late)."""
    global EPISODE
    out = Path(out); out.mkdir(parents=True, exist_ok=True); ep = None; mode = "FULL_CHAIN" if full_chain else "BASELINE_PICK_AND_PUT_BACK"
    record = record or full_chain                                              # the full chain runs inside the one record of the session (its hand-over is bound to the recorder's id and lease)
    del RECORDING_ERRORS[:]
    if record:
        try:
            from sweepick.recording import sweepick_episode_recorder as sweepick_episode
            ep = (episode_fn or (lambda: sweepick_episode.real(out, mode=mode, identifiers=dict(sources={f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files()}, mapping=hashlib.sha256(CAND.read_bytes()).hexdigest(), scene=hashlib.sha256(SCENE.read_bytes()).hexdigest()))))()
        except Exception as e:
            r = dict(result="NOT_STARTED", why=f"the recorder could not start: {type(e).__name__}: {e}", trail=[]); (out / "SESSION.json").write_text(json.dumps(dict(run="PICK02", **r), indent=1)); print(json.dumps(r, indent=1)); return 3
    EPISODE = ep; r = None
    try:
        if full_chain and handoff is None:
            from sweepick.manipulation import sweepick_bimanual_handover as sweepick_full_chain
            handoff, _owner = sweepick_full_chain.build(out, ep, scope=scope, right_lift_m=right_lift_m)                 # RECEIVE FullEpisodeHandoff with this process's owner (executor, readers, field records) injected
        r = session(out, execute=execute, handoff_fn=handoff, **(session_kw or {}))
    finally:
        EPISODE = None
        if ep is not None:
            from sweepick.recording import sweepick_episode_recorder as sweepick_episode
            res = (r or {}).get("result"); full_done = res == "FULL_CHAIN_DONE"
            doc = sweepick_episode.release(ep, task_outcome=("FULL_TASK_DONE" if full_done else "PARTIAL_OR_DIAGNOSTIC: " + str(res)) if r else "ENDED_BY_AN_EXCEPTION",
                                           stage_outcomes=[dict(step=t.get("step"), controller_result=t.get("controller_result"), gate=t.get("gate"), ok=t.get("ok")) for t in (r or {}).get("trail", []) if t.get("controller_result") or t.get("gate")],
                                           notes=[f"mode {mode}", "a baseline run is START / partial, not a full-task success" if not full_done else "full chain"])
            if r is not None:
                r["episode"] = dict(session_id=doc["session_id"], cameras=doc["cameras"], joints=doc["joints"], data_quality=doc["data_quality"])
        if r is not None:
            r["recording_errors"] = list(RECORDING_ERRORS)                       # the record's quality, next to (not instead of) what the robot did
    print(json.dumps(r, indent=1, default=str)); return 0 if r["result"] in ("PICKED_LIFTED_AND_PUT_BACK", "PLANNED", "FULL_CHAIN_DONE") else 3


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=("observe", "look", "stage", "correct", "session")); ap.add_argument("--along-jaw-mm", type=float); ap.add_argument("--basis", default=""); ap.add_argument("out"); ap.add_argument("name", nargs="?", help="look: a tag; stage: one of " + ", ".join(STAGES)); ap.add_argument("--execute", action="store_true")
    ap.add_argument("--record", action="store_true", help="session: one continuous 3-view + both-arm record under one session id (sweepick_episode)"); ap.add_argument("--full-chain", action="store_true", help="session: after the lift hand the session to the RECEIVE -> RELEASE -> PLACE -> STOW callback (sweepick_full_chain); not started unless everything it needs is ready")
    ap.add_argument("--full-chain-scope", choices=("full", "observe"), default="full", help="full: the whole task. observe: up to the right contact and the load-transfer observation; the left hand is never released; needs neither the support reference nor the place target")
    ap.add_argument("--right-lift-mm", type=float, help="observe scope: the right-hand lift approved for this session (mm)")
    a = ap.parse_args(argv)
    if a.mode == "observe":
        s = observe(a.out); print(json.dumps({k: s[k] for k in ("ready", "why", "plan", "home_ticks")}, indent=1, default=str)); return 0 if s["ready"] else 2
    if a.mode == "session":
        return run_session(a.out, execute=a.execute, record=a.record, full_chain=a.full_chain, scope=a.full_chain_scope, right_lift_m=None if a.right_lift_mm is None else a.right_lift_mm / 1000.0)
    if a.mode == "correct":
        print(json.dumps(correct(a.out, a.along_jaw_mm, a.basis, assisted=True), indent=1, default=str)); return 0
    if a.mode == "look":
        s = look(a.out, a.name); print(json.dumps({k: s.get(k) for k in ("ok", "why", "arms_on_model", "object", "tool_left", "tool_minus_plan", "wrist", "desk_z")}, indent=1, default=str)); return 0
    r = stage(a.out, a.name, execute=a.execute, cameras=wrist_cams)
    print(json.dumps({k: v for k, v in r.items() if k != "end"}, indent=1, default=str)); return 0 if r["result"] in ("RAN", "PLANNED") and r.get("exit_code", 0) == 0 else (r.get("exit_code") or 2)


if __name__ == "__main__":
    sys.exit(main())
