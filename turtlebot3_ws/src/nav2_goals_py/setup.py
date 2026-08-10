from setuptools import find_packages, setup

package_name = 'nav2_goals_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dapier-jhj',
    maintainer_email='29724960+Alpenj@users.noreply.github.com',
    description='Nav2 goal sending examples',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_pose = nav2_goals_py.go_to_pose:main',
            'follow_waypoints = nav2_goals_py.follow_waypoints:main',
        ],
    },
)
