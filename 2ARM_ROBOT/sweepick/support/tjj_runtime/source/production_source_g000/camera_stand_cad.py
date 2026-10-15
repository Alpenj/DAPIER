"""Import the supplied, identity-build 3MF stand meshes without rescaling shape."""
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np

NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}


def meshes(filename):
    path = Path(__file__).with_name("assets") / "camera_stand" / filename
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
    if root.get("unit") != "millimeter":
        raise ValueError("stand CAD must use millimeters")
    items = root.findall("m:build/m:item", NS)
    if any(i.get("transform") is not None for i in items):
        raise ValueError("non-identity build requires explicit transform handling")
    objects = {o.get("id"): o for o in root.findall("m:resources/m:object", NS)}
    result = {}
    for item in items:
        obj = objects[item.get("objectid")]
        if obj.find("m:components", NS) is not None:
            raise ValueError("unexpected component assembly")
        vertices = np.array([[float(p.get(a)) for a in ("x", "y", "z")]
                             for p in obj.findall("m:mesh/m:vertices/m:vertex", NS)])
        faces = np.array([[int(p.get(a)) for a in ("v1", "v2", "v3")]
                          for p in obj.findall("m:mesh/m:triangles/m:triangle", NS)])
        if not np.isfinite(vertices).all() or faces.min() < 0 or faces.max() >= len(vertices):
            raise ValueError("invalid stand mesh")
        result[item.get("objectid")] = (vertices, faces)
    return result


def add_stand(spec, parent, installation_z, camera_x, desk=False):
    bottom = meshes("MOUNTBOTTM2.3mf")
    top = meshes("cam_mount_top2.3mf")
    # Upper print Y is assembly Z. Socket cross section fixes the lower rotation.
    # Inter-part Z overlap is provisional: plate centre is constrained to 380 mm.
    plate = top["3"][0]
    plate_centre = (plate.min(0) + plate.max(0)) / 2
    upper_origin = np.array([-50.417992, plate_centre[1], 15.187])
    lower_origin = np.array([46.81251, 36.974977, 0])
    mast_x = camera_x - (plate_centre[0] - upper_origin[0]) * .001
    for prefix, source in (("bottom", bottom), ("top", top)):
        for key, (v, faces) in source.items():
            if prefix == "bottom":
                v = (v - lower_origin)[:, [1, 0, 2]] * [.001, -.001, .001]
                if desk:
                    # Only the outer 6 mm skirt is shortened to 4.5 mm.
                    # Column and mounting-hole regions remain unchanged.
                    original = v.copy()
                    mask = abs(v[:, 0]) > .050
                    assert np.all(v[mask, 2] <= .015 + 1e-8)
                    v[mask, 0] = np.sign(v[mask, 0]) * (.050 + (abs(v[mask, 0]) - .050) * .75)
                    assert np.array_equal(v[~mask], original[~mask])
                    assert np.isclose(np.ptp(v[:, 0]), .109, atol=1e-6)
                v += [mast_x, 0, installation_z]
            else:
                v = (v - upper_origin)[:, [0, 2, 1]] * [.001, -.001, .001]
                v += [mast_x, 0, installation_z + .38]
                if key == "3":
                    # Preserve the supplied plate-to-upper-column CAD transform.
                    normals = np.cross(v[faces[:, 1]] - v[faces[:, 0]],
                                       v[faces[:, 2]] - v[faces[:, 0]])
                    lengths = np.linalg.norm(normals, axis=1)
                    largest = np.argmax(lengths)
                    tilt = np.rad2deg(np.arccos(abs(normals[largest, 2] / lengths[largest])))
                    assert np.isclose(tilt, 25, atol=.001), tilt
            name = "stand_cad_" + prefix + "_" + key
            spec.add_mesh(name=name, uservert=v.reshape(-1).tolist(),
                          userface=faces.reshape(-1).tolist())
            parent.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH,
                            meshname=name, rgba=[.12, .12, .13, 1])
    return {"mast_x_m": mast_x, "plate_center_x_m": camera_x,
            "plate_height_above_installation_m": .38}
