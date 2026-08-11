from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'ros_dd_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🌟 Launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # 🌟 Config 파일 설치 (nav2_params.yaml)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # 🌟 저장된 지도 설치 (navigation.launch.py 기본 map 인자가 참조함)
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jd',
    maintainer_email='jd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
