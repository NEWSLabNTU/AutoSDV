import os
from glob import glob

from setuptools import setup

package_name = 'autosdv_mcl_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoSDV Team',
    maintainer_email='dev@autosdv.org',
    description=(
        'Relays the 2D-MCL particle filter pose output into the '
        'PoseWithCovarianceStamped contract ekf_localizer consumes.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcl_pose_relay = autosdv_mcl_launch.mcl_pose_relay:main',
            'scan_qos_bridge = autosdv_mcl_launch.scan_qos_bridge:main',
            'mcl_wheel_imu_odom = autosdv_mcl_launch.wheel_imu_odom:main',
        ],
    },
)
