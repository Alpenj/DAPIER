"""One kinematic copy of the measured arms in the assembled dual model, used by BOTH the collision certificate and the
execution mirror. Joint positions only; no dynamics, no SIM object or contact truth is taken from it as a controller input.
The PGripper is set as the assembled model defines it: gripper motor angle plus the two jaw slides through the model's own
coupling, (slide - slide0) = c * (gripper - gripper0)."""
import mujoco
import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def _jid(m, n):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    if j < 0:
        raise KeyError(n)
    return j


def set_arm_q(m, d, side, q6):
    """q6: five arm angles [rad] + gripper opening 0..1 (canonical model values)."""
    for n, q in zip(JOINTS[:5], q6[:5]):
        d.qpos[m.jnt_qposadr[_jid(m, f"{side}_{n}")]] = float(q)
    g = _jid(m, f"{side}_gripper")
    th = float(q6[5]) * m.jnt_range[g][1]
    d.qpos[m.jnt_qposadr[g]] = th
    for k in (1, 2):
        e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, f"{side}_pgripper_jaw_{k}_coupling")
        j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
        if j2 != g:
            raise RuntimeError("unexpected jaw coupling in the model")
        dq = th - m.qpos0[m.jnt_qposadr[g]]
        d.qpos[m.jnt_qposadr[j1]] = m.qpos0[m.jnt_qposadr[j1]] + sum(float(c) * dq ** i for i, c in enumerate(m.eq_data[e][:5]))


def set_q12(m, d, q12, t=None):
    d.qpos[:] = m.qpos0
    set_arm_q(m, d, "left", q12[:6]); set_arm_q(m, d, "right", q12[6:])
    if t is not None:
        d.time = float(t)
    mujoco.mj_kinematics(m, d)
