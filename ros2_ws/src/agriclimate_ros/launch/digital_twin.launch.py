"""Complete climate-control stack against the digital twin (simulated time).

ros2 launch agriclimate_ros digital_twin.launch.py \
    scenario:=greenhouse_sensor_actuator_faults controller:=fuzzy_pi ai:=true time_scale:=60
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    common = {"scenario": scenario, "use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="greenhouse1"),
        DeclareLaunchArgument("scenario", default_value="tomato_greenhouse_spring"),
        DeclareLaunchArgument("controller", default_value="fuzzy_pi", description="fuzzy_pi | pid | mpc"),
        DeclareLaunchArgument("ai", default_value="true", description="start the AI anomaly monitor"),
        DeclareLaunchArgument("time_scale", default_value="30.0"),
        DeclareLaunchArgument("with_faults", default_value="true"),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("namespace")),
            Node(package="agriclimate_ros", executable="plant_sim", name="plant_sim", output="screen",
                 parameters=[{"scenario": scenario, "time_scale": LaunchConfiguration("time_scale"),
                              "with_faults": LaunchConfiguration("with_faults"), "use_sim_time": False}]),
            Node(package="agriclimate_ros", executable="setpoint_scheduler", name="setpoint_scheduler",
                 output="screen", parameters=[common]),
            Node(package="agriclimate_ros", executable="climate_controller", name="climate_controller",
                 output="screen", parameters=[common, {"controller": LaunchConfiguration("controller")}]),
            Node(package="agriclimate_ros", executable="anomaly_monitor", name="anomaly_monitor",
                 output="screen", parameters=[common], condition=IfCondition(LaunchConfiguration("ai"))),
        ]),
    ])
