#!/usr/bin/env python3
"""LIVE SIM connection-only runner for Y=+70mm, Z=+50mm same-XYZ candidate."""
import argparse
import json
import math
from pathlib import Path
import mujoco
import numpy as np

from runtime_contact_candidate import RuntimeCandidateTeacher, run as run_candidate
from dynamic_preflight import full_state_preflight
from collision_guard import check_bimanual_path
from mobile_dual_so101 import apply_control_as_pose

class LiveConnectionTeacher(RuntimeCandidateTeacher):
    def staging_plan(self, pregrasp, grasp, *, require_dynamic=False):
        # Preserve the saved model identity gate before planning in this override.
        for key in ("scene_id", "model_sha256", "asset_sha256", "gripper_revision"):
            if self.staging_reference["provenance"][key] != self.report["provenance"][key]:
                raise ValueError("saved staging geometry changed: " + key)
        self.approach = np.asarray(self.config['target_TCP'])[:3, 0]
        self.closing = np.asarray(self.config['target_TCP'])[:3, 2]
        grasp = np.asarray(self.config['target_TCP'])[:3, 3]
        pregrasp = np.asarray(self.config['pregrasp_xyz_m'])
        self.report['teacher_input'].update(
            grasp_xyz=grasp.tolist(),
            pregrasp_xyz=pregrasp.tolist(),
            approach=self.approach.tolist(),
            closing=self.closing.tolist(),
            target_source='A2 mapped command geometry; same-XYZ staging candidate Y=+70mm Z=+50mm'
        )
        stage_xyz = pregrasp + np.array([0.0, 0.070, 0.050])
        self.waypoint_markers.update(
            SAFE_STAGE=stage_xyz.tolist(),
            ALIGN_HIGH=stage_xyz.tolist()
        )

        home = np.array(self.d.ctrl)
        a2_q = np.array(self.config["pregrasp_q"])

        # 1. SAFE_STAGE: seed = home, predecessor = home
        r_safe = self.evaluate_waypoint(stage_xyz, home, "SAFE_STAGE")
        if self.planning_observer:
            self.planning_observer(r_safe)
        if not r_safe["eligible"]:
            raise ValueError(f"SAFE_STAGE kinematic validation failed: pos={r_safe['position_error_m']*1000:.3f} mm")
        safe_q = np.array(r_safe["ik"]["action_rad"])

        # 2. ALIGN_HIGH: seed = a2_q, predecessor = safe_q
        r_align = self.evaluate_waypoint(stage_xyz, a2_q, "ALIGN_HIGH")
        guard_align = check_bimanual_path(self.m, safe_q, r_align["ik"]["action_rad"],
                                          task_phase="ALIGN_HIGH", reference_data=self.d, max_joint_step_rad=0.025)
        r_align["guard"] = guard_align.as_report()
        r_align["ik_seed_q"] = list(a2_q)
        r_align["seed_q"] = list(safe_q)
        # This scalar was computed along the IK seed path, not the executed predecessor.
        r_align.pop("path_general_minimum_m", None)
        r_align["eligible"] = bool(r_align["ik"]["converged"] and r_align["position_error_m"] <= 0.0005 
                                   and all(min(r["lower"], r["upper"]) >= 0 for r in r_align["joint_margins"])
                                   and guard_align.safe and r_align["task_policy"]["safe"]
                                   and math.degrees(r_align["approach_error_rad"]) <= 2.0
                                   and math.degrees(r_align["closing_error_rad"]) <= 15.0)
        if self.planning_observer:
            self.planning_observer(r_align)
        if not r_align["eligible"]:
            raise ValueError(f"ALIGN_HIGH kinematic validation failed: pos={r_align['position_error_m']*1000:.3f} mm")
        align_q = np.array(r_align["ik"]["action_rad"])

        # 3. PREGRASP_NEAR: seed = align_q, predecessor = align_q
        r_pregrasp = self.evaluate_waypoint(pregrasp, align_q, "PREGRASP_NEAR")
        if self.planning_observer:
            self.planning_observer(r_pregrasp)
        if not r_pregrasp["eligible"]:
            raise ValueError(f"PREGRASP_NEAR kinematic validation failed: pos={r_pregrasp['position_error_m']*1000:.3f} mm")
        pregrasp_q = np.array(r_pregrasp["ik"]["action_rad"])

        # Coarse waypoint
        p = mujoco.MjData(self.m)
        p.qpos[:] = self.d.qpos
        apply_control_as_pose(self.m, p, pregrasp_q)
        minimum_z = math.inf
        for g in self.fingers.values():
            mesh = int(self.m.geom_dataid[g])
            a = int(self.m.mesh_vertadr[mesh])
            vertices = self.m.mesh_vert[a:a + int(self.m.mesh_vertnum[mesh])]
            world = vertices @ p.geom_xmat[g].reshape(3, 3).T + p.geom_xpos[g]
            minimum_z = min(minimum_z, float(world[:, 2].min()))
        top = float(p.geom_xpos[self.floor, 2] + self.m.geom_size[self.floor, 2])
        coarse = np.asarray(grasp).copy()
        coarse[2] = max(grasp[2], top + self.env.config.required_clearance_m + float(p.site_xpos[self.site, 2] - minimum_z))

        # 4. APPROACH_COARSE: seed = pregrasp_q, predecessor = pregrasp_q
        r_coarse = self.evaluate_waypoint(coarse, pregrasp_q, "APPROACH_COARSE")
        if self.planning_observer:
            self.planning_observer(r_coarse)
        if not r_coarse["eligible"]:
            raise ValueError(f"APPROACH_COARSE kinematic validation failed: pos={r_coarse['position_error_m']*1000:.3f} mm")
        coarse_q = np.array(r_coarse["ik"]["action_rad"])

        # 5. APPROACH_FINE: seed = coarse_q, predecessor = coarse_q
        r_fine = self.evaluate_waypoint(grasp, coarse_q, "APPROACH_FINE")
        if self.planning_observer:
            self.planning_observer(r_fine)
        if not r_fine["eligible"]:
            raise ValueError(f"APPROACH_FINE kinematic validation failed: pos={r_fine['position_error_m']*1000:.3f} mm")

        sequence = [r_safe, r_align, r_pregrasp, r_coarse, r_fine]

        # Copy the same complete connection before authorizing live SIM motion.
        dynamic_preflight_res = full_state_preflight(self, sequence) if require_dynamic else None
        if dynamic_preflight_res is not None and not dynamic_preflight_res["passed"]:
            raise ValueError(f"Connection dynamic preflight failed: {dynamic_preflight_res['failure']}")

        audit = dict(
            direction=[-float(self.approach[0]), -float(self.approach[1]), -float(self.approach[2])],
            candidate_offset_mm=dict(dY=70.0, dZ=50.0),
            resolution_m=0.0005,
            trials=[dict(offset_m=0.05, segments=sequence, eligible=True, kinematic_pass=True, dynamic_preflight=dynamic_preflight_res)],
            selected=dict(offset_m=0.05, segments=sequence, eligible=True, kinematic_pass=True, dynamic_preflight=dynamic_preflight_res),
            limitation="SIM candidate; same target XYZ does not imply fixed TCP during motion"
        )
        self.report["staging_search"] = audit
        return sequence

def run(a):
    return run_candidate(a, teacher_class=LiveConnectionTeacher)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connection-only', action='store_true', help='stop after A2 approach, before CLOSE')
    for k in ('config', 'staging', 'donor', 'output'):
        p.add_argument('--' + k, type=Path)
    a = p.parse_args()
    if all(getattr(a, k) for k in ('config', 'staging', 'donor', 'output')):
        raise SystemExit(0 if run(a)['verification_passed'] else 1)
    else:
        p.error('runtime requires config/staging/donor/output')
