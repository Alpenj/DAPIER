"""Run directly with the existing SIM Python; no physics or hardware."""
import mujoco
import numpy as np
from integration_scenes import build_scene, ARM_MOUNT_X, DESK_EDGE_X, REAR_HOLE_X


def vertices(model, data, meshname):
    mid = model.mesh(meshname).id
    gid = next(i for i in range(model.ngeom)
               if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH and model.geom_dataid[i] == mid)
    v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
    return v @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]


def main():
    layouts = []
    for kind in ("mobile", "desk"):
        m = build_scene(kind).compile()
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        v = vertices(m, d, "left_base_so101_v2")
        base = d.body("left_base").xpos
        local = v - base
        for y in (-.03175, .03175):
            centre = np.array([REAR_HOLE_X, y])
            radius = np.linalg.norm(local[:, :2] - centre, axis=1)
            p = local[(abs(radius - .0025) < 1e-7) & (local[:, 2] < .013), :2]
            assert len(p) >= 20
            fit = np.linalg.lstsq(np.c_[2*p, np.ones(len(p))], (p*p).sum(1), rcond=None)[0][:2]
            assert np.allclose(fit, centre, atol=1e-7)
            print(kind, "rear hole fitted local mm", (fit*1000).tolist())
            if kind == "mobile":
                assert abs(fit[0] + base[0] - d.geom("profile_front").xpos[0]) < 1e-7
        if kind == "desk":
            edge = d.geom("table").xpos[0] - m.geom("table").size[0]
            assert abs(v[:, 0].min() - edge) < 1e-7
            stand = vertices(m, d, "stand_cad_bottom_1")
            assert abs(stand[:, 0].min() - edge) < 1e-7
            assert np.isclose(np.ptp(stand[:, 0]), .109, atol=1e-7)
        layouts.append(d.body("os30a_plate_frame").xpos[:2] - base[:2])
    assert np.allclose(*layouts, atol=1e-9)
    print("PASS: mesh hole fits, rail alignment, shared arm correction, desk edge and109mm footprint")


if __name__ == "__main__":
    main()
