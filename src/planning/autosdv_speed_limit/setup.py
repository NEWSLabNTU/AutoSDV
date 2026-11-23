from glob import glob

from setuptools import setup

package_name = 'autosdv_speed_limit'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeon',
    maintainer_email='jerry73204@gmail.com',
    description="The car's overall speed limit from a config file.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed_limit = autosdv_speed_limit.speed_limit:main',
        ],
    },
)
