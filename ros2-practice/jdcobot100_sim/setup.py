from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'jdcobot100_sim'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py')),
        ),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alpenj',
    maintainer_email='29724960+Alpenj@users.noreply.github.com',
    description='ROS 2 RViz and Gazebo simulation for jdcobot100.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'jdcobot100_auto_joint_publisher = '
            'jdcobot100_sim.auto_joint_publisher:main',
        ],
    },
)
