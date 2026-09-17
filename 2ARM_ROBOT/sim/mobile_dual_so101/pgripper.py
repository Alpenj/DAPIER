"""Pinned NORMA CAD adaptation for simulation, never a hardware calibration."""
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ASSETS = Path(__file__).with_name("assets") / "norma_pgripper"
PARTS = ("Gripper_Base_v1_1", "ST3215_8_v1_1", "Gripper_Gear_v1_1",
         "Gripper_Jaw_01_v1_1", "Gripper_Jaw_02_v1_1")
NAMES = ("pgripper_base", "pgripper_motor", "pgripper_gear", "pgripper_jaw_1", "pgripper_jaw_2")
MOTOR_MAX_RAD = 2.2028
JAW_METRES_PER_RAD = .0115
VARIANTS = {"stock": (), "right": ("right",), "both": ("left", "right")}


def selected_sides(variant):
    if variant not in VARIANTS:
        raise ValueError(f"grippers must be one of {tuple(VARIANTS)}")
    return VARIANTS[variant]


def _vector(element, attribute="xyz"):
    value = np.fromstring(element.get(attribute, "0 0 0"), sep=" ")
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"invalid {attribute} in pinned URDF")
    return value


def _vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count or not count:
        raise ValueError("expected a nonempty binary STL")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3).astype(float) * .001
    if not np.isfinite(vertices).all():
        raise ValueError("non-finite mesh")
    return vertices


def description():
    """Rebase the five gripper links, preserving the upstream CAD transforms."""
    provenance = json.loads((ASSETS / "provenance.json").read_text())
    for name, expected in provenance["sha256"].items():
        if hashlib.sha256((ASSETS / "upstream" / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"PGripper upstream hash mismatch: {name}")
    robot = ET.parse(ASSETS / "upstream/elrobot_follower.urdf").getroot()
    joints = {j.find("child").get("link"): j for j in robot.findall("joint")}
    # Canonical geometry frame: source wrist axis +Z and jaw travel +X.
    # The SO-101 mounting transform is applied separately in replace_gripper.
    z = _vector(joints[PARTS[0]].find("axis")); z /= np.linalg.norm(z)
    x = _vector(joints[PARTS[3]].find("axis")); x -= z * np.dot(x, z); x /= np.linalg.norm(x)
    rotation = np.array([x, np.cross(z, x), z])
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    parts = []
    for original_name, name in zip(PARTS, NAMES):
        link = robot.find(f"link[@name='{original_name}']")
        visual, inertial = link.find("visual"), link.find("inertial")
        pivot = np.zeros(3)
        current = original_name
        while current != PARTS[0]:
            joint = joints[current]
            if not np.allclose(_vector(joint.find("origin"), "rpy"), 0):
                raise ValueError("pinned gripper link rotation changed")
            pivot += _vector(joint.find("origin"))
            current = joint.find("parent").get("link")
        mesh = ASSETS / "upstream" / visual.find("geometry/mesh").get("filename")
        origin = _vector(visual.find("origin"))
        vertices = (_vertices(mesh) + origin + pivot) @ rotation.T
        center = rotation @ (_vector(inertial.find("origin")) + pivot)
        inertia = inertial.find("inertia")
        tensor = np.array([[float(inertia.get("i" + a + b if a <= b else "i" + b + a))
                            for b in "xyz"] for a in "xyz"])
        tensor = rotation @ tensor @ rotation.T
        mass = float(inertial.find("mass").get("value"))
        if name == "pgripper_gear":
            # Upstream rounds this small part's inertia to zero. Use its CAD bounding box and source mass.
            center = (vertices.min(0) + vertices.max(0)) / 2
            extent = np.ptp(vertices, axis=0)
            tensor = np.diag(mass / 12 * (np.dot(extent, extent) - extent ** 2))
        parts.append(dict(name=name, source_name=original_name, mesh=mesh, vertices=vertices,
            center=center, mass=mass, inertia=tensor, pivot=rotation @ pivot,
            mesh_pos=rotation @ (origin + pivot), mesh_quat=quaternion,
            axis=rotation @ _vector(joints[original_name].find("axis"))
                 if joints[original_name].find("axis") is not None else np.zeros(3)))
    return parts, provenance


def _fullinertia(tensor):
    return [tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]]


def replace_gripper(arm):
    """Replace an in-memory arm using the supplied wrist transform and estimated camera rig."""
    parts, provenance = description()
    root = arm.body("gripper")
    jaw = arm.body("moving_jaw_so101_v1")
    if root is None or jaw is None or arm.body("pgripper_gear") is not None:
        raise ValueError("PGripper adaptation requires an unmodified stock SO-101 arm")
    mount = provenance["so101_mount"]
    source_quat, mounted_quat = np.empty(4), np.empty(4)
    # URDF RPY is extrinsic XYZ. Rebase both geometry AND the revolute axis.
    mujoco.mju_euler2Quat(source_quat, np.asarray(mount["rpy_rad"]), "XYZ")
    inverse_basis = parts[0]["mesh_quat"].copy()
    inverse_basis[1:] *= -1
    mujoco.mju_mulQuat(mounted_quat, source_quat, inverse_basis)
    root.pos, root.quat = mount["position_m"], mounted_quat
    basis = np.empty(9)
    mujoco.mju_quat2Mat(basis, parts[0]["mesh_quat"])
    arm.joint("wrist_roll").axis = basis.reshape(3, 3) @ mount["axis_in_source_frame"]
    previous = arm.actuator("gripper")
    actuator_properties = {key: getattr(previous, key) for key in ("gaintype", "biastype", "dyntype")}
    actuator_properties.update({key: getattr(previous, key).copy() for key in ("gainprm", "biasprm", "dynprm")})
    arm.delete(jaw)  # MuJoCo also removes the actuator referring to the deleted joint.
    for geom in list(root.geoms):
        arm.delete(geom)
    for part in parts:
        if part["name"] == "pgripper_base":
            body, offset = root, np.zeros(3)
            body.mass, body.ipos = part["mass"], part["center"]
            body.fullinertia = _fullinertia(part["inertia"])
            body.explicitinertial = True
        else:
            offset = part["center"]
            body = root.add_body(name=part["name"], pos=offset, mass=part["mass"],
                                 ipos=[0, 0, 0], fullinertia=_fullinertia(part["inertia"]),
                                 explicitinertial=True)
        arm.add_mesh(name=part["name"], file=str(part["mesh"]), scale=[.001] * 3)
        body.add_geom(name=part["name"] + "_visual", type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname=part["name"], pos=part["mesh_pos"] - offset, quat=part["mesh_quat"],
            contype=0, conaffinity=0, mass=0, group=2,
            rgba=[.08, .08, .08, 1] if part["name"] == "pgripper_motor" else [.055, .055, .055, 1])
        if part["name"] == "pgripper_gear":
            body.add_joint(name="gripper", type=mujoco.mjtJoint.mjJNT_HINGE,
                pos=part["pivot"] - offset, axis=-part["axis"], range=[0, MOTOR_MAX_RAD],
                ref=MOTOR_MAX_RAD,
                damping=.1, frictionloss=.01, armature=.001)
        elif part["name"].startswith("pgripper_jaw_"):
            index = int(part["name"][-1])
            sign = 1 if index == 1 else -1
            joint_name = part["name"] + "_slide"
            body.add_joint(name=joint_name, type=mujoco.mjtJoint.mjJNT_SLIDE,
                axis=part["axis"], range=[0, .0255] if sign > 0 else [-.0255, 0],
                damping=1, frictionloss=.01, armature=.001)
            arm.add_equality(name=part["name"] + "_coupling", type=mujoco.mjtEq.mjEQ_JOINT,
                name1=joint_name, name2="gripper", objtype=mujoco.mjtObj.mjOBJ_JOINT,
                # Joint equality uses displacement from qpos0, including the motor ref above.
                data=[0, -sign * JAW_METRES_PER_RAD] + [0] * 9,
                solref=[.004, 1], solimp=[.99, .99, .001, .5, 2])
            # ponytail: a convex distal hull preserves taper; full rack/tooth collision is deferred.
            tip = part["vertices"][part["vertices"][:, 2] < -.065]
            lo, hi = tip.min(0), tip.max(0)
            pad_name = f"pgripper_pad_{index}"
            arm.add_mesh(name=pad_name, uservert=(tip - offset).ravel())
            body.add_geom(name=pad_name, type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=pad_name, contype=1, conaffinity=1,
                group=3, rgba=[.1, .7, .8, 0], mass=0, condim=4,
                friction=[1.6, .02, .001], solref=[-200000, -400], solimp=[.95, .99, .001, .5, 2])
            inner = (lo + hi) / 2
            inner[0] = hi[0] if index == 1 else lo[0]
            body.add_site(name=pad_name + "_inner", pos=inner - offset, size=[.001] * 3, group=3)
        elif part["name"] == "pgripper_base":
            lo, hi = part["vertices"].min(0), part["vertices"].max(0)
            body.add_geom(name="pgripper_housing", type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=(lo + hi) / 2, size=(hi - lo) / 2, contype=1, conaffinity=1,
                group=3, rgba=[.1, .7, .8, 0], mass=0)
    arm.add_actuator(**actuator_properties, name="gripper", target="gripper",
        trntype=mujoco.mjtTrn.mjTRN_JOINT, ctrlrange=[0, MOTOR_MAX_RAD], forcerange=[-2.94, 2.94])
    pads = [p["vertices"][p["vertices"][:, 2] < -.065] for p in parts[-2:]]
    # Midpoint of the opposed inner faces stays fixed as the jaws move symmetrically.
    first, second = [(p.min(0) + p.max(0)) / 2 for p in pads]
    first[0], second[0] = pads[0][:, 0].max(), pads[1][:, 0].min()
    midpoint = (first + second) / 2
    # Keep the CAD pinch midpoint, not the stock jaw offset. In this frame
    # site +X is gripper-body -Z (approach), site +Z is body +X (jaw travel).
    arm.site("gripperframe").pos = midpoint
    root.add_site(name="cube_grasp", pos=midpoint, quat=arm.site("gripperframe").quat,
                  size=[.002] * 3, group=3)
    camera = provenance["wrist_camera"]
    # ponytail: photo-estimated seat/optics; tune these persisted values after hand-eye calibration.
    arm.add_mesh(name="pgripper_camera_mount",
        file=str(ASSETS / "upstream/assets/CameraMount_square_27mm.stl"), scale=[.001] * 3)
    root.add_geom(name="pgripper_camera_mount_visual", type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname="pgripper_camera_mount", pos=camera["mesh_position_m"],
        quat=camera["mesh_quaternion_wxyz"], rgba=[.055, .055, .055, 1],
        contype=0, conaffinity=0, mass=0, group=2)
    wrist = root.add_camera(name="wrist_cam", pos=camera["position_m"],
        quat=camera["quaternion_wxyz"], fovy=camera["vertical_fov_deg"])
    optical_rotation = np.empty(9)
    mujoco.mju_quat2Mat(optical_rotation, np.asarray(camera["quaternion_wxyz"]))
    backward = optical_rotation.reshape(3, 3)[:, 2]
    # Visual-only PCB/lens proxies, behind the optical plane so they cannot occlude its image.
    for name, kind, size, offset, color in (
        ("pcb", mujoco.mjtGeom.mjGEOM_BOX, np.asarray(camera["pcb_size_m"]) / 2,
         camera["optical_offset_from_pcb_m"], [.09, .12, .10, 1]),
        ("lens", mujoco.mjtGeom.mjGEOM_CYLINDER, [.005, .0035, 0], .0035, [.025, .025, .025, 1]),
    ):
        root.add_geom(name=f"pgripper_camera_{name}_visual", type=kind, size=size,
            pos=wrist.pos + offset * backward, quat=wrist.quat,
            rgba=color, contype=0, conaffinity=0, mass=0, group=2)
    return parts, provenance


def sync_kinematic_jaws(model, data):
    """Pose initialization/editor only. Dynamic execution uses equality forces, never this helper."""
    for index in range(model.neq):
        if model.eq_type[index] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        joint = int(model.eq_obj1id[index])
        name = model.joint(joint).name
        if not name.endswith(("pgripper_jaw_1_slide", "pgripper_jaw_2_slide")):
            continue
        driver = int(model.eq_obj2id[index])
        target_adr, driver_adr = model.jnt_qposadr[[joint, driver]]
        displacement = data.qpos[driver_adr] - model.qpos0[driver_adr]
        data.qpos[target_adr] = model.qpos0[target_adr] + model.eq_data[index, 1] * displacement


def require_stock_recording(model):
    if isinstance(model, mujoco.MjModel) and any(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_pgripper_gear") >= 0
        for side in ("left", "right")
    ):
        raise ValueError("stock recording/checkpoint cannot be reinterpreted as PGripper data; collect and bind a new gripper calibration/dataset")


def home_action(model, stock_action):
    action = list(stock_action)
    for side in ("left", "right"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_pgripper_gear") >= 0:
            action[5 if side == "left" else 11] = MOTOR_MAX_RAD
    return action


def jaw_gap_m(model, data, side):
    """Projected distance between opposed CAD inner-face datums, not motor travel."""
    axis = data.body(f"{side}_gripper").xmat.reshape(3, 3)[:, 0]
    faces = [data.site(f"{side}_pgripper_pad_{i}_inner").xpos for i in (1, 2)]
    return float(np.dot(faces[1] - faces[0], axis))
