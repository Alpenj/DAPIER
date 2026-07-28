"""Build a Gazebo-stable URDF while preserving the original CAD visuals."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


PHYSICS = {
    'base_sub_assembly': {
        'mass': '0.25',
        'inertia': ('0.00018', '0.00018', '0.00025'),
        'origin': '0 0 0.025',
        'geometry': ('cylinder', {'radius': '0.05', 'length': '0.06'}),
    },
    'shoulder_sub_assembly': {
        'mass': '0.12',
        'inertia': ('0.00008', '0.00008', '0.00005'),
        'origin': '0 0 0.02',
        'geometry': ('box', {'size': '0.07 0.07 0.09'}),
    },
    'arm_1_sub_asssembly': {
        'mass': '0.10',
        'inertia': ('0.00002', '0.00014', '0.00014'),
        'origin': '-0.04 0 0',
        'geometry': ('box', {'size': '0.15 0.05 0.05'}),
    },
    'arm_2_sub_asssembly_copy_1': {
        'mass': '0.08',
        'inertia': ('0.000015', '0.00009', '0.00009'),
        'origin': '-0.04 0 0',
        'geometry': ('box', {'size': '0.15 0.045 0.045'}),
    },
    'end_arm_sub_assembly': {
        'mass': '0.04',
        'inertia': ('0.000008', '0.00002', '0.00002'),
        'origin': '-0.02 0 0',
        'geometry': ('box', {'size': '0.08 0.04 0.04'}),
    },
}

JOINT_NAMES = (
    'base_shoulder',
    'shoulder_arm1',
    'arm1_arm2',
    'arn2_end_arm',
)


def _replace_physics(link, settings):
    """Keep CAD visuals and use stable inertial data for motion inspection."""
    for child in list(link):
        if child.tag in ('collision', 'inertial'):
            link.remove(child)

    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value=settings['mass'])
    ET.SubElement(
        inertial,
        'inertia',
        ixx='0.001', ixy='0', ixz='0',
        iyy='0.001', iyz='0', izz='0.001',
    )


def _add_ros2_control(root, controller_config):
    control = ET.SubElement(
        root, 'ros2_control', name='GazeboSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = (
        'gazebo_ros2_control/GazeboSystem')

    for joint_name in JOINT_NAMES:
        joint = ET.SubElement(control, 'joint', name=joint_name)
        command = ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(command, 'param', name='min').text = '-0.523599'
        ET.SubElement(command, 'param', name='max').text = '0.523599'
        ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='velocity')

    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(
        gazebo,
        'plugin',
        filename='libgazebo_ros2_control.so',
        name='gazebo_ros2_control',
    )
    ET.SubElement(plugin, 'parameters').text = str(controller_config)


def build_gazebo_description(source_urdf, controller_config):
    """Return a hybrid URDF: CAD visuals + simple stable physics."""
    source_urdf = Path(source_urdf)
    root = ET.parse(source_urdf).getroot()
    root.set('name', 'ros_arm_gazebo_hybrid')

    # Gazebo may rewrite package:// URIs to model:// and then require a
    # model.config search path. Absolute file URIs are deterministic for the
    # installed package and keep gzserver startup independent of model paths.
    mesh_directory = (source_urdf.parent.parent / 'meshes').resolve()
    package_prefix = 'package://ros_arm/meshes/'
    for mesh in root.findall('.//visual/geometry/mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith(package_prefix):
            mesh_name = filename[len(package_prefix):]
            mesh.set('filename', (mesh_directory / mesh_name).as_uri())

    links = {link.get('name'): link for link in root.findall('link')}
    for link_name, settings in PHYSICS.items():
        _replace_physics(links[link_name], settings)
        gazebo_reference = ET.SubElement(
            root, 'gazebo', reference=link_name)
        ET.SubElement(gazebo_reference, 'gravity').text = 'false'
        ET.SubElement(gazebo_reference, 'selfCollide').text = 'false'

    for joint in root.findall('joint'):
        if joint.get('name') in JOINT_NAMES:
            dynamics = joint.find('dynamics')
            if dynamics is None:
                dynamics = ET.SubElement(joint, 'dynamics')
            dynamics.set('damping', '0.15')
            dynamics.set('friction', '0.04')

    world = ET.Element('link', name='world')
    root.insert(0, world)
    world_joint = ET.Element('joint', name='world_base', type='fixed')
    ET.SubElement(world_joint, 'parent', link='world')
    ET.SubElement(world_joint, 'child', link='base_sub_assembly')
    root.insert(1, world_joint)

    _add_ros2_control(root, controller_config)
    return ET.tostring(root, encoding='unicode')
