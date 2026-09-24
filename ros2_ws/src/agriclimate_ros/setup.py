from glob import glob

from setuptools import setup

package_name = "agriclimate_ros"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Basheer Al-Tawil",
    maintainer_email="basheeraltaweel@gmail.com",
    description="ROS 2 nodes for fuzzy/AI agricultural climate control",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plant_sim = agriclimate_ros.plant_sim_node:main",
            "climate_controller = agriclimate_ros.climate_controller_node:main",
            "setpoint_scheduler = agriclimate_ros.setpoint_scheduler_node:main",
            "anomaly_monitor = agriclimate_ros.anomaly_monitor_node:main",
            "modbus_bridge = agriclimate_ros.modbus_bridge_node:main",
        ],
    },
)
