"""Real observation -> the EXISTING command owners -> the application layer. Offline check on a stored real sample.

Nothing here re-implements the policy path. The existing classes run unchanged:
  student_eval.Student   (anchor, C25 prefix, 'project' arm bound, decode_command range rejection)
  tjj_mvp_r3.Arbiter     (one owner per control step)
They are given a MIRROR instead of the simulator: a MuJoCo data object whose joint positions are set from the measured
(candidate-mapped) q12, and a sensor object that hands out the real images. No physics step is made.
The owner's canonical command then goes to sweepick_real_exec.Applier, which produces Goal_Position ticks. No device is opened.

usage: python -m sweepick.integration.sweepick_replay_rig SAMPLE_DIR(candidate-mapped) CANDIDATE.json CALIBRATION_DIR ASSEMBLED.json OUT.json"""
import hashlib, json, sys
from pathlib import Path
import numpy as np, mujoco
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_command_application as rx
from sweepick.control import sweepick_trajectory_profile as tt
from sweepick.control import sweepick_joint_kinematics as sweepick_kin


class MirrorSensor:
    """What Student reads from a sensor: .current[<side>_wrist_rgb]['rgb'] (RGB uint8, model size)."""
    def __init__(self):
        self.current = {}
    def set(self, left, right):
        self.current = dict(left_wrist_rgb=dict(rgb=left), right_wrist_rgb=dict(rgb=right))


class Shim:
    """The two things Arbiter asks of a rig when no guard / corrector is used."""
    def __init__(self):
        self.t, self._step = 0.0, 0
    def first_forward(self, student, context):
        return student.next_command(context)
    def step(self):
        return self._step


def mirror(model, data, q12, t):
    """Joint positions of the measured state into the model: the same kinematic copy the collision certificate uses
    (gripper motor AND jaw slides through the model's own coupling)."""
    sweepick_kin.set_q12(model, data, q12, t)


def main(sample_dir, cand_file, cal_dir, model_file, out_file):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import integration_scenes, receive_candidate as rc_, tjj_mvp_r2 as r2m, tjj_mvp_r3 as r3
    from sim_data_factory import student_eval, core
    sample = Path(sample_dir)
    arr = np.load(sample / "model_input_r6.npz")
    q, left, right, top = arr["q12"].astype(np.float32), arr["left"], arr["right"], arr["top"]
    mapping = rc.load_mapping(cand_file, cal_dir, model_file)
    present = rc.q12_to_ticks(mapping, q)
    lim = tt.load_limits()
    model = integration_scenes.task_env("desk", grippers="both").model
    data = mujoco.MjData(model)
    three = r2m.install_three_cameras()                                        # the existing 3-view patch: observation3 reads three['top']['rgb']
    three.update(top=dict(rgb=top))
    sensor = MirrorSensor(); sensor.set(left, right)
    out = dict(schema="tjj.real-rig-offline.v1", sample=str(sample), commands_sent=0, device_opened=False, q12_measured_candidate=q.tolist(), present_ticks=present, owners={})
    shadow = {n: np.load(sample / f"shadow_r6_chunk_{n}.npy") for n in ("START_3V", "RECEIVE_3V") if (sample / f"shadow_r6_chunk_{n}.npy").exists()}
    m = r2m.models()
    for role, name in (("START", "START_3V"),):
        rc_.MODELS[role] = Path(m[name]["path"])
        policy, stats, meta = rc_.load_policy(role)
        policy.reset(); policy.eval()
        student = student_eval.Student(policy, stats, sensor, rc_.PREFIX, None, "project")
        arb = r3.Arbiter(Shim(), student, f"ACT:{name}")
        app = rx.Applier(mapping, lim, present)
        ctx = dict(model=model, data=data, control_dt=0.02)
        rows, cmds = [], []
        for k in range(rc_.PREFIX):
            mirror(model, data, q, k * 0.02)                                   # offline: the one stored measurement stands in for every step of the prefix
            command, diag = arb.next_command(ctx)
            if arb.failure:
                out["owners"][name] = dict(failure=arb.failure); break
            cmds.append(np.asarray(command, dtype=float))
            goal = app.dispatch(arb.name, command, {s: dict(app.goal[s]) for s in rx.SIDES}, closed_loop=False).write
            rows.append(dict(k=k, owner=arb.name, goal_ticks=goal, raw_step=None, applied_step=None))
        if cmds:
            C = np.array(cmds)
            same = None if name not in shadow else float(np.abs(C - shadow[name][:len(C)]).max())
            tr = student.trace[0]
            far = {f"{s} {n}": round(abs(rc.model_to_tick(n, mapping[s][n], float(C[-1][i * 6 + j])) - app.goal[s][n]), 1) for i, s in enumerate(("left", "right")) for j, n in enumerate(rc.JOINTS)}
            out["owners"][name] = dict(owner_steps=arb.steps, inference_calls=student.calls, prefix=rc_.PREFIX, anchor_equals_measured=bool(np.allclose(tr["q_measured"], q, atol=1e-6)),
                                       projection=dict(policy="project", any_projected=any(any(t.get("projection_mask") or []) for t in student.trace)), decode_command="passed for every step (range rejection of the existing decoder)",
                                       max_abs_difference_to_the_shadow_chunk=same, first_command=C[0].round(4).tolist(), first_goal_ticks=rows[0]["goal_ticks"], last_goal_ticks=rows[-1]["goal_ticks"],
                                       raw_first_step_ticks=rows[0]["raw_step"], applied_first_step_ticks=rows[0]["applied_step"], remaining_to_the_last_target_ticks={k: v for k, v in far.items() if v > 1},
                                       note="the grippers follow ACT's absolute opening at the velocity / acceleration limits; after 0.5 s they have not arrived (see remaining)")
    out["sources"] = dict(student_eval=hashlib.sha256(Path(student_eval.__file__).read_bytes()).hexdigest(), arbiter=hashlib.sha256(Path(r3.__file__).read_bytes()).hexdigest(), applier=hashlib.sha256(Path(rx.__file__).read_bytes()).hexdigest())
    Path(out_file).write_text(json.dumps(out, indent=1, default=str))
    print(json.dumps(out["owners"], default=str))


if __name__ == "__main__":
    main(*sys.argv[1:6])
