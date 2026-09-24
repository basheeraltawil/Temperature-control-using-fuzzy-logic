"""Climate-control stack on real hardware: PLC / remote I/O over Modbus TCP (wall-clock time).

ros2 launch agriclimate_ros hardware.launch.py plc_host:=192.168.0.10 scenario:=tomato_greenhouse_spring
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="greenhouse1"),
        DeclareLaunchArgument("scenario", default_value="tomato_greenhouse_spring"),
        DeclareLaunchArgument("controller", default_value="fuzzy_pi"),
        DeclareLaunchArgument("plc_host", default_value="192.168.0.10"),
        DeclareLaunchArgument("plc_port", default_value="502"),
        DeclareLaunchArgument("ai", default_value="false",
                              description="enable only after training the detector on site data"),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("namespace")),
            Node(package="agriclimate_ros", executable="modbus_bridge", name="modbus_bridge", output="screen",
                 parameters=[{"host": LaunchConfiguration("plc_host"),
                              "port": LaunchConfiguration("plc_port")}]),
            Node(package="agriclimate_ros", executable="setpoint_scheduler", name="setpoint_scheduler",
                 output="screen", parameters=[{"scenario": scenario}]),
            Node(package="agriclimate_ros", executable="climate_controller", name="climate_controller",
                 output="screen", parameters=[{"scenario": scenario,
                                               "controller": LaunchConfiguration("controller")}]),
            Node(package="agriclimate_ros", executable="anomaly_monitor", name="anomaly_monitor",
                 output="screen", parameters=[{"scenario": scenario}],
                 condition=IfCondition(LaunchConfiguration("ai"))),
        ]),
    ])
