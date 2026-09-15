from glob import glob
import os

from setuptools import find_packages, setup

package_name = "agri_chassis_can"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("lib", package_name), ["scripts/setup_can.sh"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="czb",
    maintainer_email="czb@example.com",
    description="TD48150B-2E SocketCAN chassis control, odometry, and AUTO/MANUAL joystick gate",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_can_node = agri_chassis_can.chassis_can_node:main",
            "chassis_mode_teleop = agri_chassis_can.chassis_mode_teleop:main",
        ],
    },
)
