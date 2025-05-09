import os
from glob import glob
from setuptools import find_packages, setup


package_name = "autosdv_launch"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][ymal]*")),
        ),
        # Include the HTML templates
        (
            os.path.join("share", package_name, "autosdv_launch", "templates"),
            glob(os.path.join("autosdv_launch", "templates", "*.html")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aeon",
    maintainer_email="jerry73204@gmail.com",
    description="AutoSDV launch and utility package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autosdv_monitor = autosdv_launch.autosdv_monitor:main",
        ],
    },
)
