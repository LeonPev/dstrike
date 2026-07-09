from setuptools import setup


package_name = "drone_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dsriker",
    maintainer_email="devnull@example.com",
    description="ROS 2 MAVLink bridge for the Docker-only ArduPilot/MuJoCo stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mavlink_bridge = drone_bridge.mavlink_bridge:main",
        ],
    },
)
