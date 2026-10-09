import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_coach_lab'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.reference', package_name + '.stub'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'virtual_coach'),
            glob('config/virtual_coach/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description='AutoSDV Lab 2: coach-board tracker, pursuit planner, simulator stand-ins.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coach_tracker = autosdv_coach_lab.coach_tracker_node:main',
            'board_pursuit_planner = autosdv_coach_lab.board_pursuit_planner_node:main',
            'virtual_coach = autosdv_coach_lab.virtual_coach_node:main',
            'drive_gear_hold = autosdv_coach_lab.drive_gear_hold:main',
            'sim_pose_init = autosdv_coach_lab.sim_pose_init:main',
        ],
    },
)
