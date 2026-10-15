"""Physics scripted expert: left picks the free block, hands the SAME block to right.

The teacher only emits canonical q_cmd; core.rollout owns ctrl + mj_step. It reads
privileged SIM truth (block pose, pad contact forces) for planning and phase feedback.
Planning/FK/collision previews run on a private MjData copy. No attachment, no qpos
writes to the live episode, no applied forces.
"""
from dataclasses import dataclass, asdict, field, replace
import numpy as np
import mujoco
from physics_ik import solve_bimanual_position_ik, plan_septic_joint_trajectory, MotionLimits
from pgripper import sync_kinematic_jaws, MOTOR_MAX_RAD
from .core import decode_command
from .task_success import ContactModel, geom_distance

SIDE_OFFSET = {'left': 0, 'right': 6}
GRIP = {'left': 5, 'right': 11}
UP = np.array([0., 0., 1.])
PLANNING_SCOPES = frozenset(('FULL_TASK', 'RECEIVE_APPROACH', 'RECEIVE_CONTACT', 'SUPPORT_OBSERVATION'))
RECEIVE_ONLY_SCOPES = PLANNING_SCOPES - {'FULL_TASK'}
PLANNER_CANDIDATE_FIELDS = ('handover_x_m', 'handover_z_m', 'handover_alpha_deg', 'handover_phi_deg')


@dataclass(frozen=True)
class TeacherConfig:
    # Handover point on the circle whose diameter joins the two pan axes: there the
    # left jaw axis (horizontal tangent of a pitched left approach) is the right arm's
    # radial direction, so both 5-DOF arms can hold orthogonal face pairs.
    handover_x_m: tuple = (0.20, 0.18, 0.22, 0.157, 0.24)
    handover_z_m: tuple = (0.12, 0.14, 0.10, 0.16)
    handover_alpha_deg: tuple = (60.0, 45.0, 75.0, 30.0, 85.0)  # left approach tilt from vertical, toward its radial
    handover_phi_deg: tuple = (0.0, 20.0, -20.0, 35.0, -35.0)  # left jaw rotation away from the horizontal tangent
    handover_max_candidates: int = 160
    pregrasp_height_m: float = 0.06            # site above block centre before descent
    pick_tilt_deg: tuple = (0.0, 10.0, 20.0)
    left_grasp_offset_m: float = 0.0075        # site above block centre (pads cover -5..+20 mm)
    lift_height_m: float = 0.06                # site rise after grasp
    right_pregrasp_back_m: float = 0.045       # right site behind block centre along its approach
    right_stage_back_m: float = 0.07
    right_grasp_depths_m: tuple = (0.0, -0.004, 0.004)
    left_retreat_m: float = 0.06
    plan_clearance_m: float = 0.006
    live_clearance_m: float = 0.003
    align_tol_m: float = 0.0007
    align_gain: float = 0.3
    align_timeout_s: float = 3.0
    grip_target_N: float = 2.0                 # per-pad normal force set point (block is 20 g)
    grip_contact_N: float = 0.05
    grip_confirm_s: float = 0.3
    close_rate_rad_s: float = 0.8              # gear rad/s before contact
    force_gain_rad_per_N: float = 1e-3         # per control step; measured ~60 N/rad per pad after contact
    max_squeeze_rad: float = 0.15              # gear command below measured gear
    release_open_fraction_of_travel: float = 0.3
    hold_s: float = 1.5                        # >= checker 1 s window
    motion: MotionLimits = MotionLimits(max_velocity_rad_s=0.8, max_acceleration_rad_s2=2.5, max_jerk_rad_s3=15.0)
    min_segment_s: float = 0.6
    settle_tol_rad: float = 0.03
    settle_timeout_s: float = 2.0
    grip_timeout_s: float = 6.0
    ik_pos_tol_m: float = 0.003
    ik_axis_tol_deg: float = 8.0               # accepted after FK; solver aims for 2 deg


class TeacherFailure(RuntimeError):
    pass


class Planner:
    """FK / IK / collision preview on a private copy. Never touches the live MjData."""
    def __init__(self, model, *, right_roll_seed_rad=None):
        self.m = model
        self.pd = mujoco.MjData(model)
        self.adr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
        self.cm = ContactModel(model)
        self.site = {s: model.site(f'{s}_cube_grasp').id for s in SIDE_OFFSET}
        self.pan = {s: model.joint(f'{s}_shoulder_pan').id for s in SIDE_OFFSET}
        self.gripper_geoms = {s: [model.geom(f'{s}_pgripper_{n}').id for n in ('housing', 'pad_1', 'pad_2')]
                              for s in SIDE_OFFSET}
        self.gripper_clearance_m = 0.003
        # Margin pairs involve a housing. Pads of the two hands pinch orthogonal faces of the
        # 40 mm block with 35 mm pads, so pad-pad gaps are ~2.5 mm by design: contacts only.
        self.clearance_pairs = [(g1, g2) for g1 in self.gripper_geoms['left'] for g2 in self.gripper_geoms['right']
                                if 'housing' in model.geom(g1).name or 'housing' in model.geom(g2).name]
        self._roll_sign = {}
        self._last_roll_choice = {}
        self.right_roll_seed_rad = right_roll_seed_rad

    def pose(self, base_qpos, ctrl, block_qpos=None):
        pd = self.pd
        pd.qpos[:] = base_qpos
        pd.qpos[self.adr] = ctrl
        if block_qpos is not None:
            pd.qpos[self.cm.block_qpos:self.cm.block_qpos + 7] = block_qpos
        pd.qvel[:] = 0
        sync_kinematic_jaws(self.m, pd)
        mujoco.mj_forward(self.m, pd)
        return pd

    def site_frame(self, side):
        return self.pd.site_xpos[self.site[side]].copy(), self.pd.site_xmat[self.site[side]].reshape(3, 3).copy()

    def pan_anchor(self, side):
        return self.pd.xanchor[self.pan[side]].copy()

    def radial(self, side, point):
        v = np.asarray(point)[:2] - self.pan_anchor(side)[:2]
        return np.r_[v / np.linalg.norm(v), 0.]

    def roll_sign(self, base_qpos, ctrl, side):
        if side not in self._roll_sign:
            i = SIDE_OFFSET[side] + 4
            self.pose(base_qpos, ctrl)
            _, r0 = self.site_frame(side)
            c2 = ctrl.copy(); c2[i] += 0.01
            self.pose(base_qpos, c2)
            _, r1 = self.site_frame(side)
            self._roll_sign[side] = float(np.sign(np.dot(np.cross(r0[:, 2], r1[:, 2]), r0[:, 0])))
        return self._roll_sign[side]

    def set_roll(self, base_qpos, ctrl, side, jaw_candidates):
        """Rotate wrist_roll so the jaw axis (site Z) matches the closest candidate direction."""
        i = SIDE_OFFSET[side] + 4
        roll_sign = self.roll_sign(base_qpos, ctrl, side)
        self.pose(base_qpos, ctrl)
        _, rot = self.site_frame(side)
        a, j = rot[:, 0], rot[:, 2]
        lo, hi = self.m.actuator_ctrlrange[i]
        best = None
        for candidate_index, candidate in enumerate(jaw_candidates):
            for jaw_axis_sign in (1, -1):
                c = jaw_axis_sign * np.asarray(candidate, float)
                c = c - np.dot(c, a) * a
                if np.linalg.norm(c) < 1e-6:
                    continue
                c /= np.linalg.norm(c)
                phi = np.arctan2(np.dot(np.cross(j, c), a), np.dot(j, c))
                roll = ctrl[i] + roll_sign * phi
                if lo + 0.05 <= roll <= hi - 0.05 and (best is None or abs(phi) < abs(best[0])):
                    best = (phi, roll, candidate_index, jaw_axis_sign)
        if best is None:
            self._last_roll_choice.pop(side, None)
            return None
        out = ctrl.copy(); out[i] = best[1]
        self._last_roll_choice[side] = dict(jaw_candidate_index=best[2], jaw_axis_sign=best[3],
                                           roll_delta_rad=float(best[0]), wrist_roll_rad=float(best[1]))
        return out

    # Numerical IK seeds (NOT trajectory starts): the actual trajectory always starts at the
    # current command; these only help the local solver find other joint-space branches.
    SEED_ARMS = ((0.0, -1.0, 1.2, 0.5, 0.0), (0.0, 0.3, 0.8, -1.0, 0.0), (0.0, -0.3, 1.4, -1.2, 0.0),
                 (0.0, 0.5, 0.2, 1.2, 0.0))

    def solve(self, base_qpos, seed, side, pos, approach, jaw_candidates, cfg):
        seed = np.array(seed, float)
        best = None
        seeds = [seed.copy()]
        for arm in self.SEED_ARMS:
            s = seed.copy()
            o = SIDE_OFFSET[side]
            s[o:o + 5] = arm
            s[o] = seed[o]
            seeds.append(s)
        if side == 'right' and self.right_roll_seed_rad is not None:
            for s in seeds:
                s[SIDE_OFFSET[side] + 4] = self.right_roll_seed_rad
        for s in seeds:
            ctrl, info = self._solve_one(base_qpos, s, side, pos, approach, jaw_candidates, cfg)
            if ctrl is not None:
                if side == 'right' and self.right_roll_seed_rad is not None:
                    info['numerical_ik_seed'] = dict(
                        source='model actuator lower wrist endpoint',
                        wrist_roll_rad=float(self.right_roll_seed_rad),
                        trajectory_start=False,
                    )
                return ctrl, info
            best = best or info
        return None, best

    def _solve_one(self, base_qpos, seed, side, pos, approach, jaw_candidates, cfg):
        """Position + approach-axis IK (5 DOF), then wrist roll for the jaw axis; verified by FK."""
        ctrl = np.array(seed, float)
        initial_wrist_roll = float(ctrl[SIDE_OFFSET[side] + 4])
        for _ in range(3):
            r = solve_bimanual_position_ik(self.m, ctrl, {side: pos}, site_names={side: f'{side}_cube_grasp'},
                                           tool_axis_targets={side: approach}, max_iterations=300,
                                           tool_axis_tolerance_rad=np.deg2rad(2.0),
                                           axis_formulation='axis_direction')
            ctrl = np.array(r.action_rad)
            rolled = self.set_roll(base_qpos, ctrl, side, jaw_candidates)
            if rolled is None:
                return None, 'roll_range'
            ctrl = rolled
        self.pose(base_qpos, ctrl)
        p, rot = self.site_frame(side)
        perr = float(np.linalg.norm(p - pos))
        aerr = float(np.degrees(np.arccos(np.clip(np.dot(rot[:, 0], approach / np.linalg.norm(approach)), -1, 1))))
        jerr = min(float(np.degrees(np.arccos(np.clip(abs(np.dot(rot[:, 2], c / np.linalg.norm(c))), -1, 1))))
                   for c in jaw_candidates)
        if perr > cfg.ik_pos_tol_m or aerr > cfg.ik_axis_tol_deg:
            return None, f'ik pos_err={perr:.4f} axis_err={aerr:.1f}deg'
        info = dict(pos_err_m=perr, axis_err_deg=aerr, jaw_err_deg=jerr)
        if side == 'right' and self.right_roll_seed_rad is not None:
            info.update(wrist_roll_seed_rad=initial_wrist_roll,
                        wrist_roll_choice=dict(self._last_roll_choice[side]))
        return ctrl, info

    def collisions(self, base_qpos, ctrl, block_qpos=None, allow_pad_block=(), clearance=None):
        pd = self.pose(base_qpos, ctrl, block_qpos)
        bad = []
        # Planning margin between the two grippers (contacts alone only report penetration).
        margin = self.gripper_clearance_m if clearance is None else clearance
        if margin > 0:
            for g1, g2 in self.clearance_pairs:
                if True:
                    if geom_distance(self.m, pd, g1, g2) < margin:
                        bad.append((self.m.geom(g1).name, self.m.geom(g2).name, 'clearance'))
            if bad:
                return bad
        for c in pd.contact:
            g1, g2 = int(c.geom1), int(c.geom2)
            if c.dist > 0:
                continue
            pair = {g1, g2}
            if self.cm.block_geom in pair:
                other = (pair - {self.cm.block_geom}).pop()
                if other == self.cm.table:
                    continue
                if other in self.cm.pad_index and self.cm.pad_index[other][0] in allow_pad_block:
                    continue
                if self.cm.side[other] is not None:
                    bad.append(('block', self.m.geom(other).name or self.m.body(int(self.m.geom_bodyid[other])).name))
                continue
            if self.cm.side[g1] is None and self.cm.side[g2] is None:
                continue
            bad.append(tuple(self.m.geom(g).name or self.m.body(int(self.m.geom_bodyid[g])).name for g in (g1, g2)))
        return bad

    def carried_block(self, base_qpos, ctrl, carry):
        side, rel = carry
        self.pose(base_qpos, ctrl)
        p, rot = self.site_frame(side)
        quat = np.empty(4); mujoco.mju_mat2Quat(quat, (rot @ rel[:3, :3]).ravel())
        return np.r_[p + rot @ rel[:3, 3], quat]

    def gripper_distance(self, base_qpos, ctrl, block_qpos=None, distmax=0.05):
        pd = self.pose(base_qpos, ctrl, block_qpos)
        return min(min(distmax, geom_distance(self.m, pd, g1, g2)) for g1, g2 in self.clearance_pairs)

    def path_collisions(self, base_qpos, start, goal, carry=None, samples=24, allow_pad_block=(), block_qpos=None):
        """carry=(side, T_site_block (4x4)) moves the block rigidly with that site in the preview."""
        for u in np.linspace(0, 1, samples):
            ctrl = start + (goal - start) * (35*u**4 - 84*u**5 + 70*u**6 - 20*u**7)
            block = block_qpos
            if carry is not None:
                side, rel = carry
                self.pose(base_qpos, ctrl)
                p, rot = self.site_frame(side)
                pos = p + rot @ rel[:3, 3]
                quat = np.empty(4); mujoco.mju_mat2Quat(quat, (rot @ rel[:3, :3]).ravel())
                block = np.r_[pos, quat]
            bad = self.collisions(base_qpos, ctrl, block, allow_pad_block)
            if bad:
                return float(u), bad
        return None


class HandoverTeacher:
    """Stateful phase machine: observe -> left approach -> grasp -> lift -> transfer approach ->
    right grasp -> left release/retreat -> right hold. Phase changes only on measured feedback."""

    def __init__(self, model, config=TeacherConfig(), *, planner_options=None):
        options = {} if planner_options is None else planner_options
        if not isinstance(options, dict):
            raise ValueError('planner_options must be a dict or None')
        if options:
            if set(options) != {'source', 'candidate', 'right_roll_seed'}:
                raise ValueError('planner_options requires source, candidate, and right_roll_seed only')
            if not isinstance(options['source'], str) or not options['source']:
                raise ValueError('planner_options.source must be a nonempty string')
            candidate = options['candidate']
            if not isinstance(candidate, dict) or set(candidate) != set(PLANNER_CANDIDATE_FIELDS):
                raise ValueError(f'planner_options.candidate requires {PLANNER_CANDIDATE_FIELDS}')
            if any(isinstance(candidate[name], bool) or not isinstance(candidate[name], (int, float))
                   or not np.isfinite(candidate[name]) for name in PLANNER_CANDIDATE_FIELDS):
                raise ValueError('planner_options.candidate values must be finite scalars')
            if options['right_roll_seed'] != 'MODEL_LOWER_ENDPOINT':
                raise ValueError('planner_options.right_roll_seed must be MODEL_LOWER_ENDPOINT')
            config = replace(config, **{name: (float(candidate[name]),) for name in PLANNER_CANDIDATE_FIELDS})
            right_roll_seed_rad = float(model.actuator_ctrlrange[SIDE_OFFSET['right'] + 4, 0])
        else:
            right_roll_seed_rad = None
        self.m, self.cfg = model, config
        self.plan = Planner(model, right_roll_seed_rad=right_roll_seed_rad)
        self.cm = self.plan.cm
        # Canonical<->gear map from the contract adapter itself (affine per gripper). The label
        # direction is NOT assumed: physical open is gear = MOTOR_MAX_RAD in this PGripper model.
        base = np.zeros(12)
        g0 = decode_command(base, model)[[5, 11]]
        base[[5, 11]] = 1
        g1 = decode_command(base, model)[[5, 11]]
        self.g_a, self.g_b = g0, g1 - g0
        self.gear_open = MOTOR_MAX_RAD
        self.phase = 'observe'
        self.events = []
        self.failure = None
        self.done = False
        self.cmd = None               # gear-space 12 vector (arm rad + gripper gear rad)
        self.segment = None
        self.grip = {}
        self.phase_t = 0.0
        self.info = {}
        self.planning_scope = 'FULL_TASK'
        if options:
            self.info['planner_options'] = dict(
                source=options['source'], candidate={name: float(options['candidate'][name])
                                                     for name in PLANNER_CANDIDATE_FIELDS},
                right_roll_seed=options['right_roll_seed'],
                resolved_right_roll_seed_rad=right_roll_seed_rad,
                seed_role='numerical IK seed only; never a measured state or trajectory start',
            )

    def set_planning_scope(self, planning_scope):
        if planning_scope not in PLANNING_SCOPES:
            raise ValueError(f'unknown planning_scope {planning_scope!r}')
        self.planning_scope = planning_scope
        self.info['planning_scope'] = planning_scope

    # ---- conversions -------------------------------------------------------------------
    def to_canonical(self, gear_cmd):
        q = np.array(gear_cmd, float)
        q[[5, 11]] = np.clip((q[[5, 11]] - self.g_a) / self.g_b, 0.0, 1.0)
        return q

    def measured_gear(self, data):
        return data.qpos[self.plan.adr].copy()

    # ---- motion primitives -------------------------------------------------------------
    def start_segment(self, goal, sides, min_s=None):
        goal = np.array(goal, float)
        start = self.cmd.copy()
        for s in SIDE_OFFSET:
            if s not in sides:
                goal[SIDE_OFFSET[s]:SIDE_OFFSET[s] + 5] = start[SIDE_OFFSET[s]:SIDE_OFFSET[s] + 5]
        goal[[5, 11]] = start[[5, 11]]
        traj = plan_septic_joint_trajectory(self.m, start, goal, limits=self.cfg.motion,
                                            minimum_duration_s=min_s or self.cfg.min_segment_s)
        self.segment = dict(traj=traj, t=0.0, goal=goal, settle=0.0)

    def step_segment(self, data, dt):
        """Returns True when the arm target is reached and tracking has settled."""
        seg = self.segment
        seg['t'] += dt
        target, _, _, _ = seg['traj'].sample(seg['t'])
        arm = [i for i in range(12) if i not in (5, 11)]
        self.cmd[arm] = target[arm]
        if seg['t'] < seg['traj'].duration_s:
            return False
        err = np.abs(self.measured_gear(data)[arm] - seg['goal'][arm]).max()
        if err < self.cfg.settle_tol_rad:
            return True
        seg['settle'] += dt
        if seg['settle'] > self.cfg.settle_timeout_s:
            raise TeacherFailure(f'tracking did not settle in {self.phase}: max err {err:.3f} rad')
        return False

    def grip_update(self, data, ev, dt):
        for side, g in self.grip.items():
            i = GRIP[side]
            f = np.asarray(ev['pad_force_N'][side])
            meas = self.measured_gear(data)[i]
            if g['mode'] == 'open':
                self.cmd[i] += float(np.clip(g['target'] - self.cmd[i], -1.5 * dt, 1.5 * dt))
            elif g['mode'] == 'close':
                if f.max() >= self.cfg.grip_contact_N:
                    g['mode'] = 'force'
                else:
                    self.cmd[i] = max(0.0, self.cmd[i] - self.cfg.close_rate_rad_s * dt)
            if g['mode'] == 'force':
                err = self.cfg.grip_target_N - f.min()
                self.cmd[i] = self.cmd[i] - self.cfg.force_gain_rad_per_N * err
                self.cmd[i] = float(np.clip(self.cmd[i], max(0.0, meas - self.cfg.max_squeeze_rad),
                                            min(self.gear_open, meas + self.cfg.max_squeeze_rad)))
                ok = f.min() >= 0.5 * self.cfg.grip_target_N
                g['stable'] = g.get('stable', 0.0) + dt if ok else 0.0

    # ---- geometry helpers --------------------------------------------------------------
    def block_pose(self, data):
        b = self.cm.block_body
        return data.xpos[b].copy(), data.xmat[b].reshape(3, 3).copy(), \
            data.qpos[self.cm.block_qpos:self.cm.block_qpos + 7].copy()

    def site_block_rel(self, data, side):
        s = self.plan.site[side]
        p, rot = data.site_xpos[s].copy(), data.site_xmat[s].reshape(3, 3).copy()
        c, rb, _ = self.block_pose(data)
        rel = np.eye(4)
        rel[:3, :3] = rot.T @ rb
        rel[:3, 3] = rot.T @ (c - p)
        return rel

    def plan_move(self, data, side, pos, approach, jaws, carry=None, allow=(), label=''):
        base = data.qpos.copy()
        goal, info = self.plan.solve(base, self.cmd, side, pos, approach, jaws, self.cfg)
        if goal is None:
            raise TeacherFailure(f'{label}: {info}')
        goal[[5, 11]] = self.cmd[[5, 11]]
        hit = self.plan.path_collisions(base, self.cmd, goal, carry=carry, allow_pad_block=allow)
        if hit is not None:
            # One adaptive via-point: same approach, site raised; retried at two heights.
            self.plan.pose(base, self.cmd)
            p0, _ = self.plan.site_frame(side)
            for dz in (0.05, 0.10):
                via_pos = 0.5 * (p0 + np.asarray(pos)) + [0, 0, dz]
                via, _ = self.plan.solve(base, self.cmd, side, via_pos, approach, jaws, self.cfg)
                if via is None:
                    continue
                via[[5, 11]] = self.cmd[[5, 11]]
                if (self.plan.path_collisions(base, self.cmd, via, carry=carry, allow_pad_block=allow) is None
                        and self.plan.path_collisions(base, via, goal, carry=carry, allow_pad_block=allow) is None):
                    return [via, goal], info
            raise TeacherFailure(f'{label}: path collision at u={hit[0]:.2f} {hit[1][:3]}')
        return [goal], info

    def cartesian_hops(self, base, start, side, direction, distance, allow=(), step_m=0.01, block_qpos=None):
        """Straight-line tool motion as short IK hops (keeps approach + jaw axes). Preview only."""
        self.plan.pose(base, start)
        p0, rot = self.plan.site_frame(side)
        seed, out = np.array(start, float), []
        n = max(1, int(np.ceil(distance / step_m)))
        for k in range(1, n + 1):
            goal, info = self.plan._solve_one(base, seed, side, p0 + direction * distance * k / n, rot[:, 0],
                                              [rot[:, 2]], self.cfg)
            if goal is None:
                return None, f'hop {k}/{n} {info}'
            goal[[5, 11]] = start[[5, 11]]
            hit = self.plan.path_collisions(base, seed, goal, samples=6, allow_pad_block=allow, block_qpos=block_qpos)
            if hit is not None:
                return None, f'hop {k}/{n} collision {hit[1][:2]}'
            out.append(goal)
            seed = goal
        return out, None

    def retreat_directions(self, base, ctrl, a_r):
        """Pad-sliding retreat directions for the opened left hand, most direct first."""
        self.plan.pose(base, ctrl)
        _, rot = self.plan.site_frame('left')
        j_l = rot[:, 2]
        away = np.zeros(3)
        pd = self.plan.pd
        for g1 in self.plan.gripper_geoms['left']:
            for g2 in self.plan.gripper_geoms['right']:
                if geom_distance(self.m, pd, g1, g2) < 0.03:
                    v = pd.geom_xpos[g1] - pd.geom_xpos[g2]
                    away += v / max(np.linalg.norm(v), 1e-6)
        cands = [-rot[:, 0], -rot[:, 0] + UP, UP]
        if np.linalg.norm(away) > 1e-6:
            away_slide = away - np.dot(away, j_l) * j_l      # never push a pad into the block
            if np.linalg.norm(away_slide) > 1e-6:
                away_slide /= np.linalg.norm(away_slide)
                cands += [-rot[:, 0] + away_slide, away_slide + UP, away_slide]
        return [c / np.linalg.norm(c) for c in cands]

    # ---- main callback -----------------------------------------------------------------
    def next_command(self, context):
        m, d = context['model'], context['data']
        dt = float(context['control_dt'])
        ev = self.cm.evidence(d)
        if self.cmd is None:
            self.cmd = d.ctrl.copy()
            self.q0_gear = self.cmd.copy()
        prev = self.phase
        try:
            if not self.done and self.failure is None:
                self._advance(d, ev, dt)
        except TeacherFailure as e:
            self.failure = dict(phase=self.phase, reason=str(e), time=float(d.time))
            self.phase = 'failed'
        self.grip_update(d, ev, dt)
        lo, hi = m.actuator_ctrlrange.T
        self.cmd = np.clip(self.cmd, lo, hi)   # teacher-side plan limits; core never clips
        if self.phase != prev:
            self.events.append(dict(time=float(d.time), phase=self.phase))
        self.phase_t = 0.0 if self.phase != prev else self.phase_t + dt
        diag = dict(phase=self.phase, teacher_failure=self.failure, grip={k: dict(v) for k, v in self.grip.items()},
                    evidence={k: ev[k] for k in ('pad_force_N', 'table_support_N', 'block_pos',
                                                 'block_bottom_clearance_m', 'forbidden_contacts',
                                                 'non_pad_block_contacts', 'max_penetration_m')},
                    warnings=int(sum(w.number for w in d.warning)))
        return self.to_canonical(self.cmd), diag

    def _advance(self, d, ev, dt):
        cfg = self.cfg
        ph = self.phase
        if ph == 'observe':
            c, rb, _ = self.block_pose(d)
            if abs(rb[2, 2]) < 0.95:
                raise TeacherFailure('block not upright; pick primitive covers upright block only')
            self.info['block_initial'] = c.tolist()
            jaws = [rb[:, 0], rb[:, 1]]
            self.info['pick_jaws'] = jaws
            # Vertical approach first; if the 5-DOF arm cannot, tilt toward its radial direction
            # (jaw stays a projected face normal; tilt bounded so the pinch stays on the faces).
            self.plan.pose(d.qpos, self.cmd)
            rl = self.plan.radial('left', c)
            waypoints, reasons = None, []
            for tilt in np.deg2rad(cfg.pick_tilt_deg):
                approach = np.cos(tilt) * -UP + np.sin(tilt) * rl
                try:
                    waypoints, ik = self.plan_move(d, 'left', c + [0, 0, cfg.pregrasp_height_m], approach, jaws,
                                                   label='left pregrasp')
                    self.info['pick_approach'] = approach
                    break
                except TeacherFailure as e:
                    reasons.append(str(e)[:100])
            if waypoints is None:
                raise TeacherFailure(f'left pregrasp: {reasons}')
            self.queue = waypoints
            self.grip['left'] = dict(mode='open', target=self.gear_open)
            self.start_segment(self.queue.pop(0), ['left'], 1.5)
            self.phase = 'left_approach'
        elif ph == 'left_approach':
            if self.step_segment(d, dt):
                if self.queue:
                    self.start_segment(self.queue.pop(0), ['left'])
                    return
                c, _, _ = self.block_pose(d)
                goal, _ = self.plan_move(d, 'left', c + [0, 0, cfg.left_grasp_offset_m], self.info['pick_approach'],
                                         self.info['pick_jaws'], allow=('left',), label='left descend')
                self.start_segment(goal[-1], ['left'], 1.2)
                self.phase = 'left_descend'
        elif ph == 'left_descend':
            if self.step_segment(d, dt):
                self.grip['left'] = dict(mode='close')
                self.phase = 'left_grasp'
        elif ph == 'left_grasp':
            if self.grip['left'].get('stable', 0) >= cfg.grip_confirm_s:
                self.plan.pose(d.qpos, self.measured_gear(d))
                p, _ = self.plan.site_frame('left')
                rel = self.site_block_rel(d, 'left')
                _, rot = self.plan.site_frame('left')
                goal, _ = self.plan_move(d, 'left', p + [0, 0, cfg.lift_height_m], self.info['pick_approach'], [rot[:, 2]],
                                         carry=('left', rel), allow=('left',), label='left lift')
                self.start_segment(goal[-1], ['left'], 1.2)
                self.phase = 'left_lift'
            elif self.phase_t > cfg.grip_timeout_s:
                raise TeacherFailure(f'left grasp force not stable: {ev["pad_force_N"]["left"]}')
        elif ph == 'left_lift':
            if np.asarray(ev['pad_force_N']['left']).min() < 0.1:
                raise TeacherFailure('left grip lost during lift')
            if self.step_segment(d, dt):
                if ev['table_support_N'] > 0 or ev['block_bottom_clearance_m'] < 0.02:
                    raise TeacherFailure('block not lifted off table')
                self._plan_transfer(d)
                self.phase = 'transfer_approach'
        elif ph == 'transfer_approach':
            if np.asarray(ev['pad_force_N']['left']).min() < 0.1:
                raise TeacherFailure('left grip lost during transfer')
            if self.step_segment(d, dt):
                if self.queue:
                    side, goal = self.queue.pop(0)
                    self.start_segment(goal, [side], 1.0)
                    return
                self._plan_right_grasp(d)
                self.phase = 'right_approach'
        elif ph == 'right_approach':
            if self.step_segment(d, dt):
                if self.queue:
                    side, goal = self.queue.pop(0)
                    self.start_segment(goal, [side], 1.2)
                    return
                self.align = dict(target=self.info['right_grasp_target'].copy(), ok=0.0)
                self.phase = 'right_align'
        elif ph == 'right_align':
            # Symmetric coupled jaws cannot centre a block that the left hand holds fixed, so the
            # right hand centres itself on the measured block before closing (integral correction
            # of the site target; removes servo sag + IK error).
            c, _, _ = self.block_pose(d)
            s_id = self.plan.site['right']
            site, rot = d.site_xpos[s_id].copy(), d.site_xmat[s_id].reshape(3, 3)
            a_r, n = self.info['right_axes']
            err = (c + self.info['right_depth_m'] * a_r) - site
            local = rot.T @ err
            self.info['right_align_error_mm'] = (local * 1000).round(2).tolist()
            if np.linalg.norm(local[1:]) < cfg.align_tol_m and abs(local[0]) < 2 * cfg.align_tol_m:
                self.align['ok'] += dt
            else:
                self.align['ok'] = 0.0
            if self.align['ok'] >= 0.1:
                self.grip['right'] = dict(mode='close')
                self.phase = 'right_grasp'
                return
            if self.phase_t > cfg.align_timeout_s:
                raise TeacherFailure(f'right hand could not centre on block: local err mm {local * 1000}')
            self.align['target'] = self.align['target'] + cfg.align_gain * err
            goal, _ = self.plan._solve_one(d.qpos.copy(), self.cmd, 'right', self.align['target'], a_r, [n, -n], cfg)
            if goal is not None:
                step = np.clip(goal[6:11] - self.cmd[6:11], -0.01, 0.01)
                self.cmd[6:11] += step
        elif ph == 'right_grasp':
            if self.grip['right'].get('stable', 0) >= cfg.grip_confirm_s:
                travel = self.gear_open - self.measured_gear(d)[GRIP['left']]
                self.grip['left'] = dict(mode='open', target=self.measured_gear(d)[GRIP['left']]
                                         + cfg.release_open_fraction_of_travel * travel)
                self.phase = 'left_release'
            elif self.phase_t > cfg.grip_timeout_s:
                raise TeacherFailure(f'right grasp force not stable: {ev["pad_force_N"]["right"]}')
        elif ph == 'left_release':
            if abs(self.cmd[GRIP['left']] - self.grip['left']['target']) < 1e-6 and self.phase_t > 0.3:
                if np.asarray(ev['pad_force_N']['left']).max() > 0.01:
                    raise TeacherFailure('left pads still loaded after opening')
                if np.asarray(ev['pad_force_N']['right']).min() < 0.3:
                    raise TeacherFailure('right lost block during left release')
                self.plan.pose(d.qpos, self.measured_gear(d))
                p, rot = self.plan.site_frame('left')
                a_r = self.info['right_axes'][0]
                goal, reasons = None, []
                base = d.qpos.copy()
                for direction in self.retreat_directions(base, self.cmd, a_r):
                    goal, why = self.cartesian_hops(base, self.cmd, 'left', direction, cfg.left_retreat_m,
                                                    allow=('right',))
                    if goal is not None:
                        self.info['left_retreat_direction'] = direction.tolist()
                        break
                    reasons.append(why)
                if goal is None:
                    raise TeacherFailure(f'no collision-free left retreat: {reasons}')
                self.queue = [('left', g) for g in goal[1:]]
                self.start_segment(goal[0], ['left'], 0.3)
                self.phase = 'left_retreat'
        elif ph == 'left_retreat':
            if np.asarray(ev['pad_force_N']['right']).min() < 0.3:
                raise TeacherFailure('right lost block during left retreat')
            if self.step_segment(d, dt):
                if self.queue:
                    side, goal = self.queue.pop(0)
                    self.start_segment(goal, [side], 0.3)
                    return
                self.phase = 'right_hold'
        elif ph == 'right_hold':
            if np.asarray(ev['pad_force_N']['right']).min() < 0.3 or ev['table_support_N'] > 0:
                raise TeacherFailure('right hold lost')
            if self.phase_t >= cfg.hold_s:
                self.phase = 'done'
                self.done = True

    def _handover_candidates(self, rel):
        """Analytic pre-filter: for a left handover pose, the right 5-DOF approach must lie in
        its radial vertical plane AND be orthogonal to the free block face it pinches."""
        cfg, plan = self.cfg, self.plan
        out = []
        for x in cfg.handover_x_m:
            for z in cfg.handover_z_m:
                p = np.array([x, 0.0, z])
                rl, rr = plan.radial('left', p), plan.radial('right', p)
                tl = np.cross(UP, rl)
                for alpha in np.deg2rad(cfg.handover_alpha_deg):
                    a_l = np.cos(alpha) * -UP + np.sin(alpha) * rl
                    for phi in np.deg2rad(cfg.handover_phi_deg):
                        j_l = np.cos(phi) * tl + np.sin(phi) * np.cross(a_l, tl)
                        j_l /= np.linalg.norm(j_l)
                        rot = np.c_[a_l, np.cross(j_l, a_l), j_l]
                        rb = rot @ rel[:3, :3]
                        k_left = int(np.argmax(np.abs(rb.T @ j_l)))
                        for k in range(3):
                            if k == k_left:
                                continue
                            n = rb[:, k]
                            beta = np.arctan2(np.dot(rr, n), np.dot(UP, n))
                            beta = (beta + np.pi / 2) % np.pi - np.pi / 2
                            a_r = np.cos(beta) * rr - np.sin(beta) * UP
                            if not np.deg2rad(10) <= beta <= np.deg2rad(65) or np.dot(a_r, a_l) > 0.65:
                                continue
                            # Reach map (right): downward pitches are reachable over the handover region,
                            # upward ones are not. So the right comes from above-behind and the left
                            # presents with a flatter approach (larger alpha) to keep its housing clear.
                            score = abs(np.degrees(beta) - 35) + 0.2 * abs(np.degrees(phi)) - 0.1 * np.degrees(alpha)
                            out.append((score, p, a_l, j_l, rot, a_r, n, float(np.degrees(beta))))
        out.sort(key=lambda c: c[0])
        return out

    def _plan_transfer(self, d):
        cfg, plan = self.cfg, self.plan
        plan.pose(d.qpos, self.measured_gear(d))
        rel = self.site_block_rel(d, 'left')
        base = d.qpos.copy()
        carry = ('left', rel)
        tried = []
        # Global-plan previews use a larger gripper-gripper margin than live checks: execution
        # (servo sag, right-hand centring, grasp slip) moves the hands by a few mm.
        plan.gripper_clearance_m = cfg.plan_clearance_m
        try:
            return self._select_handover(d, rel, base, carry, tried)
        finally:
            plan.gripper_clearance_m = cfg.live_clearance_m

    def _select_handover(self, d, rel, base, carry, tried):
        cfg, plan = self.cfg, self.plan
        # Stage 1: IK both grasp poses per candidate and rank by gripper-gripper clearance.
        ranked, left_cache = [], {}
        for score, p, a_l, j_l, rot, a_r, n, beta in self._handover_candidates(rel)[:cfg.handover_max_candidates]:
            key = (tuple(np.round(p, 4)), tuple(np.round(a_l, 4)), tuple(np.round(j_l, 4)))
            if key not in left_cache:
                left_cache[key] = plan.solve(base, self.cmd, 'left', p - rot @ rel[:3, 3], a_l, [j_l], cfg)
            goal_l, ik = left_cache[key]
            if goal_l is None:
                tried.append(('left_ik', p.tolist(), beta)); continue
            for depth in cfg.right_grasp_depths_m:
                grasp, ik_g = plan.solve(base, goal_l, 'right', p + depth * a_r, a_r, [n], cfg)
                if grasp is None:
                    tried.append(('right_grasp_ik', p.tolist(), beta, depth)); continue
                both = self.cmd.copy(); both[0:5] = goal_l[0:5]; both[6:11] = grasp[6:11]; both[11] = self.gear_open
                clear = plan.gripper_distance(base, both, plan.carried_block(base, both, carry))
                ranked.append((clear, p, a_l, j_l, a_r, n, beta, depth, goal_l, grasp, ik, ik_g))
        ranked.sort(key=lambda r: -r[0])
        self.info['handover_clearance_top_mm'] = [round(r[0] * 1000, 1) for r in ranked[:5]]
        # Stage 2: transfer/insert previews, plus release/retreat when that motion is in scope.
        for clear, p, a_l, j_l, a_r, n, beta, depth, goal_l, grasp, ik, ik_g in ranked:
            if clear < cfg.plan_clearance_m:
                tried.append(('clearance', p.tolist(), beta, round(clear * 1000, 1))); break
            stage, ik_r = plan.solve(base, grasp, 'right', p - cfg.right_stage_back_m * a_r, a_r, [n], cfg)
            pre, ik_pre = plan.solve(base, grasp, 'right', p - cfg.right_pregrasp_back_m * a_r, a_r, [n], cfg)
            if stage is None or pre is None:
                tried.append(('right_stage_ik', p.tolist(), beta)); continue
            left_goal = self.cmd.copy(); left_goal[0:5] = goal_l[0:5]
            right_stage = self.cmd.copy(); right_stage[6:11] = stage[6:11]; right_stage[11] = self.gear_open
            both = np.r_[left_goal[:6], right_stage[6:]]
            pre_c = both.copy(); pre_c[6:11] = pre[6:11]
            grasp_c = both.copy(); grasp_c[6:11] = grasp[6:11]
            if (plan.path_collisions(base, both, pre_c, carry=carry, allow_pad_block=('left',)) is not None or
                    plan.path_collisions(base, pre_c, grasp_c, carry=carry, allow_pad_block=('left',)) is not None):
                tried.append(('right_insert_collision', p.tolist(), beta)); continue
            release_retreat_preview = dict(required=self.planning_scope not in RECEIVE_ONLY_SCOPES)
            if self.planning_scope not in RECEIVE_ONLY_SCOPES:
                released = grasp_c.copy()
                released[5] = self.cmd[5] + cfg.release_open_fraction_of_travel * (self.gear_open - self.cmd[5])
                if plan.path_collisions(base, grasp_c, released, carry=carry, allow_pad_block=('left', 'right'),
                                        samples=4) is not None:
                    tried.append(('left_release_clearance', p.tolist(), beta)); continue
                block_h = plan.carried_block(base, grasp_c, carry)
                ok_retreat = False
                for direction in self.retreat_directions(base, released, a_r):
                    hops, _ = self.cartesian_hops(base, released, 'left', direction, cfg.left_retreat_m,
                                                  allow=('right',), step_m=0.02, block_qpos=block_h)
                    if hops is not None:
                        ok_retreat = True
                        break
                if not ok_retreat:
                    tried.append(('left_retreat_preview', p.tolist(), beta)); continue
                release_retreat_preview['status'] = 'CLEAR'
            else:
                release_retreat_preview.update(status='NOT_REQUIRED_FOR_SCOPE', scope=self.planning_scope)
            lg = left_goal.copy(); lg[11] = self.gear_open
            order = None
            for first, second in ((('right', right_stage), ('left', both)), (('left', lg), ('right', both))):
                if (plan.path_collisions(base, np.r_[self.cmd[:11], self.gear_open], first[1], carry=carry,
                                         allow_pad_block=('left',)) is None and
                        plan.path_collisions(base, first[1], second[1], carry=carry, allow_pad_block=('left',)) is None):
                    order = [first, second]
                    break
            if order is None:
                tried.append(('transfer_collision', p.tolist(), beta)); continue
            self.info.update(handover_point=p.tolist(), handover_left_pitch_deg=float(np.degrees(np.arccos(-a_l[2]))),
                             planned_right_beta_deg=beta, planned_right_depth_m=depth,
                             planned_gripper_clearance_mm=round(clear * 1000, 1), left_handover_ik=ik,
                             right_handover_grasp_ik=ik_g, right_stage_ik=ik_r,
                             right_handover_pregrasp_ik=ik_pre, transfer_order=[s for s, _ in order],
                             handover_candidates_tried=len(tried) + 1,
                             left_release_retreat_preview=release_retreat_preview)
            self.grip['right'] = dict(mode='open', target=self.gear_open)
            side, goal = order.pop(0)
            self.queue = order
            self.start_segment(goal, [side], 1.5)
            return
        raise TeacherFailure(f'no feasible handover candidate ({len(tried)} tried): {tried[:4]}')

    def _plan_right_grasp(self, d):
        cfg, plan = self.cfg, self.plan
        c, rb, _ = self.block_pose(d)
        plan.pose(d.qpos, self.measured_gear(d))
        _, rot_l = plan.site_frame('left')
        a_l, j_l = rot_l[:, 0], rot_l[:, 2]
        axes = [rb[:, k] for k in range(3)]
        k_left = int(np.argmax([abs(np.dot(ax, j_l)) for ax in axes]))
        rr = plan.radial('right', c)
        best, seen = None, []
        planned = np.deg2rad(self.info.get('planned_right_beta_deg', 25.0))
        for k in range(3):
            if k == k_left:
                continue
            n = axes[k]
            # Right approach lies in its radial vertical plane and must be orthogonal to its jaw n.
            b = np.arctan2(np.dot(rr, n), np.dot(UP, n))    # cos b (rr.n) - sin b (up.n) = 0
            b = (b + np.pi / 2) % np.pi - np.pi / 2
            a = np.cos(b) * rr - np.sin(b) * UP
            seen.append((k, round(float(np.degrees(b)), 1), round(float(np.dot(a, a_l)), 2)))
            if not np.deg2rad(-50) <= b <= np.deg2rad(55) or np.dot(a, a_l) > 0.65:
                continue
            if best is None or abs(b - planned) < abs(best[0] - planned):
                best = (b, a, n)
        if best is None:
            raise TeacherFailure(f'no right approach orthogonal to a free block face: (axis, beta, a_r.a_l)={seen}')
        b, a_r, n = best
        base = d.qpos.copy()
        pre, ik1 = plan.solve(base, self.cmd, 'right', c - cfg.right_pregrasp_back_m * a_r, a_r, [n, -n], cfg)
        if pre is None:
            raise TeacherFailure(f'right pregrasp IK: {ik1}')
        rel = self.site_block_rel(d, 'left')
        pre_c = self.cmd.copy(); pre_c[6:11] = pre[6:11]; pre_c[11] = self.gear_open
        hit = plan.path_collisions(base, np.r_[self.cmd[:11], self.gear_open], pre_c, carry=('left', rel),
                                   allow_pad_block=('left',))
        if hit is not None:
            raise TeacherFailure(f'right pregrasp path collision at u={hit[0]:.2f}: {hit[1][:3]}')
        planned = self.info.get('planned_right_depth_m', 0.0)
        reasons, chosen = [], None
        # Live re-plan from the measured block: the planned insertion depth first, then shallower
        # grasps (right housing further from the left gripper).
        for depth in (planned, planned - 0.004, planned - 0.008):
            grasp, ik2 = plan.solve(base, pre, 'right', c + depth * a_r, a_r, [n, -n], cfg)
            if grasp is None:
                reasons.append(f'depth {depth}: {ik2}'); continue
            grasp_c = pre_c.copy(); grasp_c[6:11] = grasp[6:11]
            hit = plan.path_collisions(base, pre_c, grasp_c, carry=('left', rel), allow_pad_block=('left',))
            if hit is not None:
                reasons.append(f'depth {depth}: insert collision {hit[1][:2]}'); continue
            chosen = depth
            break
        if chosen is None:
            raise TeacherFailure(f'right insert: {reasons}')
        pre_c[11] = grasp_c[11] = self.cmd[11]
        self.info.update(right_beta_deg=float(np.degrees(b)), right_pregrasp_ik=ik1, right_grasp_ik=ik2,
                         right_depth_m=chosen, right_grasp_target=c + chosen * a_r, right_axes=(a_r, n))
        self.queue = [('right', grasp_c)]
        self.start_segment(pre_c, ['right'], 1.2)


# ---- episode runner (uses Astra's core.rollout + recorder; no own physics loop) ----------
def _sensor(use_sensor, resolution=(160, 120)):
    if use_sensor:
        from .sensor import SimulatedCameraSensor     # Gemini-owned; imported, never copied
        return SimulatedCameraSensor(width=resolution[0], height=resolution[1])
    from .smoke import NoCamera
    return NoCamera()


def run_episode(output_root, seed, q0, block_xy_yaw, *, max_sim_s=40.0, use_sensor=False,
                sensor_resolution=(160, 120), config=TeacherConfig(), sampling=None, tag='teacher'):
    import hashlib, json, os, uuid
    from datetime import datetime, timezone
    from pathlib import Path
    from .core import REVISION, ACTION_NAMES, UNITS, rollout, source_identity
    from .recorder import EpisodeRecorder
    from .reset_sampler import make_task_episode
    from .task_success import evaluate_states
    output_root = Path(output_root)
    context = make_task_episode(seed, q0, block_xy_yaw)
    m = context['model']
    identity = source_identity()
    model_path = Path(os.environ['DAPIER_SO101_MJCF']).resolve()
    sensor = _sensor(use_sensor, sensor_resolution)
    metadata = dict(contract_revision=REVISION, observation_revision=('os30a-mono-v1' if use_sensor else 'NoCamera'), seed=int(seed), classification='physics_rollout_unchecked',
        model_source=str(model_path), model_source_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        contract_sha256=hashlib.sha256((output_root / 'integration_contract.md').read_bytes()).hexdigest(),
        physics_backend='mujoco_mj_step', task_scope='left_pick_right_handover_same_block',
        hardware_access=False, hardware_qualification='NOT_EVALUATED',
        object_attachment=False, external_forces=False, runtime_qpos_writes=0,
        requested_initial_state=context['reset']['requested_q0'], actual_initial_state=context['initial'],
        core_initial_before_block_placement=context['core_initial_before_block_placement'],
        reset=context['reset'], sampling=sampling,
        scene_config=context['scene_config'], compiled_model_sha256=context['compiled_model_sha256'],
        source_sha256=identity, mujoco_version=mujoco.__version__, physics_dt=float(m.opt.timestep),
        control_dt=context['control_dt'], physics_steps=context['physics_steps'],
        joint_names=ACTION_NAMES, units=UNITS, ctrl_units='arm rad / gripper gear rad',
        integration_state_spec=int(mujoco.mjtState.mjSTATE_INTEGRATION),
        sensor_status=(f'gemini sensor.SimulatedCameraSensor {sensor_resolution[0]}x{sensor_resolution[1]}'
                       if use_sensor else 'NoCamera stub; low-dimensional teacher trace only'),
        teacher='sim_data_factory.teacher.HandoverTeacher', teacher_config=asdict(config),
        student_allowlist=['q_state', 'cameras.rgb', 'cameras.depth_m'],
        privileged_fields=['qpos', 'sim_qvel', 'integration_state', 'diagnostics', 'success',
                           'task_check', 'reset.block_qpos', 'teacher_events'])
    name = datetime.now(timezone.utc).strftime(f'{tag}-s{int(seed)}-%Y%m%dT%H%M%S-') + uuid.uuid4().hex[:6]
    recorder = EpisodeRecorder(output_root / 'datasets/episodes' / name, metadata)
    teacher = HandoverTeacher(m, config)
    context['substep_check'] = teacher.cm.substep_failure
    steps = int(round(max_sim_s / context['control_dt']))
    for _ in range(steps):
        progressed = rollout(context, teacher, sensor, recorder, 1)
        if not progressed:
            teacher.failure = context['terminal_failure'];teacher.phase = 'failed'
        if teacher.done or teacher.failure:
            break
    after = source_identity()
    if any(after.get(k) != v for k, v in identity.items()):
        raise ValueError('source revision changed during rollout')
    lazily_imported = sorted(set(after) - set(identity))
    if not recorder.states:
        failure={**metadata,'source_sha256':after,'complete':False,'training_eligible':False,
                 'task_success':False,'terminal_failure':context.get('terminal_failure'),
                 'terminal_state':context.get('terminal_state')}
        (recorder.path/'failure.json').write_text(json.dumps(failure,default=lambda v: np.asarray(v).tolist()))
        if hasattr(sensor,'close'):sensor.close()
        return recorder.path,dict(stage='none',handover_success=False,pick_lift_success=False,valid=False,
            events={},failures=[teacher.failure],worst={},teacher_failure=teacher.failure,teacher_events=teacher.events,teacher_info={})
    states = np.stack([s['integration_state'] for s in recorder.states])
    qs = np.stack([s['q_state'] for s in recorder.states])
    check = evaluate_states(m, states, qs, control_dt=context['control_dt'])
    rows = check.pop('rows')
    if context.get('terminal_failure'):
        check['valid']=False;check['handover_success']=False;check['pick_lift_success']=False
        check['failures'].append(context['terminal_failure'])
    summary = dict(stage=check['stage'], handover_success=check['handover_success'],
                   pick_lift_success=check['pick_lift_success'], valid=check['valid'],
                   events=check['events'], failures=check['failures'], worst=check['worst'],
                   teacher_failure=teacher.failure, teacher_events=teacher.events,
                   teacher_info=json.loads(json.dumps(teacher.info, default=lambda v: np.asarray(v).tolist())))
    classification = 'physics_demonstration' if check['handover_success'] else (
        'physics_partial_pick_lift' if check['pick_lift_success'] else 'physics_failed_attempt')
    # Recorder copies metadata at construction; the checker verdict is attached before finish().
    final_metadata = json.loads(json.dumps(dict(
        classification=classification, task_success=bool(check['handover_success']),
        task_stage=check['stage'], task_validation='sim_task_success.evaluate_states (replay copy)',
        task_check=summary, checker_config=check['config'], source_sha256=after,
        source_modules_imported_during_rollout=lazily_imported,
        source_identity_skips=__import__('sim_data_factory.core',fromlist=['SOURCE_IDENTITY_SKIPS']).SOURCE_IDENTITY_SKIPS,
        substep_checks=True, terminal_failure=context.get('terminal_failure'),
        terminal_state=context.get('terminal_state')), default=lambda v: np.asarray(v).tolist()))
    path = recorder.finish(final_metadata)
    if hasattr(sensor, 'close'):sensor.close()
    (path / 'task_check_rows.json').write_text(json.dumps(rows))
    return path, summary


def main(argv=None):
    import argparse, json
    from pathlib import Path
    p = argparse.ArgumentParser(description='SIM-only physics handover teacher (no hardware).')
    p.add_argument('--output-root', type=Path, required=True)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--q0-json', type=Path, help='explicit initial_state json {q,names,units}')
    p.add_argument('--block', type=float, nargs=3, default=(0.20, 0.0, 0.0), metavar=('X', 'Y', 'YAW'))
    p.add_argument('--use-sensor', action='store_true')
    p.add_argument('--sensor-res', type=int, nargs=2, default=(160, 120), metavar=('W', 'H'))
    p.add_argument('--tag', default='teacher')
    p.add_argument('--pilot', type=int, default=0, help='randomized q0/block attempts (seed base = --seed)')
    p.add_argument('--workers', type=int, default=8)
    a = p.parse_args(argv)
    if a.pilot:
        path, s = pilot(a.output_root, a.pilot, a.seed, a.workers)
        print(json.dumps({k: v for k, v in s.items() if k != 'rows'}, indent=1), path)
        return
    q0 = json.loads(a.q0_json.read_text())['q']
    path, summary = run_episode(a.output_root, a.seed, q0, a.block, use_sensor=a.use_sensor,
                                sensor_resolution=tuple(a.sensor_res), tag=a.tag)
    print(json.dumps(dict(episode=str(path), **{k: summary[k] for k in
          ('stage', 'handover_success', 'valid', 'events', 'failures', 'teacher_failure')}), indent=1))



# ---- small randomized pilot -------------------------------------------------------------
def _pilot_one(args):
    output_root, seed = args
    from integration_scenes import task_env
    from .reset_sampler import sample_start
    rng = np.random.default_rng(seed)
    model = task_env('desk', grippers='both').model
    try:
        q0, block, rec = sample_start(model, rng)
    except RuntimeError as e:
        return dict(seed=seed, result='reset_exhausted', reason=str(e))
    row = dict(seed=seed, q0=q0.tolist(), block_xy_yaw=rec['block_xy_yaw'], reset_attempts=rec['attempts'],
               reset_rejections=[r['reason'] for r in rec['rejections']])
    try:
        path, summary = run_episode(output_root, seed, q0, rec['block_xy_yaw'], sampling=rec, tag='pilot')
        row.update(episode=str(path), stage=summary['stage'], handover_success=summary['handover_success'],
                   valid=summary['valid'], checker_failures=summary['failures'][:3],
                   teacher_failure=summary['teacher_failure'], events=summary['events'],
                   handover_point=summary['teacher_info'].get('handover_point'), result='ran')
    except Exception as e:           # recorded, never retried silently
        row.update(result='error', reason=f'{type(e).__name__}: {e}'[:400])
    return row


def pilot(output_root, n=32, base_seed=1000, workers=8):
    import json
    from multiprocessing import get_context
    from pathlib import Path
    with get_context('spawn').Pool(workers) as pool:
        rows = pool.map(_pilot_one, [(str(output_root), base_seed + i) for i in range(n)])
    ran = [r for r in rows if r['result'] == 'ran']
    count = lambda st: sum(1 for r in ran if r['valid'] and r['stage'] in st)
    summary = dict(attempted=n, reset_rejections_total=sum(len(r.get('reset_rejections', [])) for r in rows),
                   reset_exhausted=sum(r['result'] == 'reset_exhausted' for r in rows),
                   errors=sum(r['result'] == 'error' for r in rows), episodes_recorded=len(ran),
                   reach=count(('reach', 'pick_lift', 'handover')), pick_lift=count(('pick_lift', 'handover')),
                   full_handover=count(('handover',)), invalid=sum(1 for r in ran if not r['valid']),
                   rows=rows)
    out = Path(output_root) / 'reports' / 'review_artifacts'
    out.mkdir(parents=True, exist_ok=True)
    path = out / f'pilot-seed{base_seed}-n{n}.json'
    path.write_text(json.dumps(summary, indent=1))
    return path, summary


if __name__ == '__main__':
    main()
