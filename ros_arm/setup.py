from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'ros_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'sequences'), glob('sequences/*.json')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jd',
    maintainer_email='jd@todo.todo',
    description='ROS 2 RViz and serial control for a four-axis Arduino robot arm.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_arm_control = ros_arm.ros_arm_bridge:main',
            'ros_arm_sequence_gui = ros_arm.sequence_gui:main',
            'ros_arm_auto_joint_publisher = ros_arm.auto_joint_publisher:main',
        ],
    },
)
