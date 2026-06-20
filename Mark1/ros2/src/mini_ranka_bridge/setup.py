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
    maintainer_email="augustasger@gmail.com",
    description="Serial-JSON to ROS2 telemetry bridge for the Mini_Ranka arm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "telemetry_bridge = mini_ranka_bridge.telemetry_bridge:main",
            "tune = mini_ranka_bridge.tune:main",
        ],
    },
)
