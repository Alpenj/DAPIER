from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'casino_dealer'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['contracts/*.json']},
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'contracts'),
            glob(os.path.join(package_name, 'contracts', '*.json')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alpenj',
    maintainer_email='29724960+Alpenj@users.noreply.github.com',
    description='Experiment contracts and deterministic planners for the DAPIER card-dealer project.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'casino_blackjack_plan = casino_dealer.cli:main',
            'casino_episode = casino_dealer.episode_cli:main',
            'casino_card_sim = casino_dealer.card_sim_cli:main',
        ],
    },
)
