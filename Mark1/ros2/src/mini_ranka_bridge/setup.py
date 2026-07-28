from glob import glob

from setuptools import find_packages, setup

package_name = "mini_ranka_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="RoastedSalsa",
    maintainer_email="augustaspug@gmail.com",
    description="Serial-JSON to ROS2 telemetry bridge for the Mini_Ranka arm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "telemetry_bridge = mini_ranka_bridge.telemetry_bridge:main",
            "tune = mini_ranka_bridge.tune:main",
            # Digital-twin joint sources — exactly one of these should run.
            "joint_state_bridge = mini_ranka_bridge.joint_state_bridge:main",
            "mock_arm = mini_ranka_bridge.mock_arm:main",
        ],
    },
)
