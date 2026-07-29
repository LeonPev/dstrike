from glob import glob
from setuptools import setup


package_name = "drone_bridge"


setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="dsriker",
    maintainer_email="devnull@example.com",
    description="ROS 2 MAVLink and two-drone formation controller for the Docker-only simulation stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mavlink_bridge = drone_bridge.mavlink_bridge:main",
            "scenario_controller = drone_bridge.scenario_controller:main",
            "target_route_controller = drone_bridge.target_route_controller:main",
        ],
    },
)
