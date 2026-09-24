"""Topic names and helpers shared by the agriclimate ROS 2 nodes.

All names are relative, so one climate zone = one namespace, e.g.
``ros2 launch agriclimate_ros digital_twin.launch.py namespace:=greenhouse3``.
"""
from typing import Dict

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

SENSOR_TEMPERATURE = "sensors/temperature_{i}"     # sensor_msgs/Temperature, degC
SENSOR_HUMIDITY = "sensors/humidity"               # sensor_msgs/RelativeHumidity, 0..1
OUTDOOR_TEMPERATURE = "weather/outdoor_temperature"  # sensor_msgs/Temperature
OUTDOOR_HUMIDITY = "weather/outdoor_humidity"      # sensor_msgs/RelativeHumidity
SOLAR = "weather/solar_radiation"                  # std_msgs/Float64, W/m2
SETPOINT = "climate/setpoint"                      # std_msgs/Float64, degC
ACTUATOR = "actuators/{name}"                      # std_msgs/Float64, 0..1 (heater, cooler, vent)
DEMAND = "climate/demand"                          # std_msgs/Float64, -1..1
VALIDATED_TEMPERATURE = "climate/validated_temperature"
EXCLUDED_SENSORS = "anomaly/excluded_sensors"      # std_msgs/Int32MultiArray (0-based)
TRUE_TEMPERATURE = "sim/true_air_temperature"      # ground truth, simulation only
ACTUATORS = ("heater", "cooler", "vent")

LEVELS = {"GOOD": DiagnosticStatus.OK, "DEGRADED": DiagnosticStatus.WARN, "BAD": DiagnosticStatus.ERROR}


def status(name: str, level: int, message: str, values: Dict[str, object], hardware_id: str = "") -> DiagnosticStatus:
    st = DiagnosticStatus()
    st.name, st.level, st.message, st.hardware_id = name, level, message, hardware_id
    st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
    return st


def declare(node, name: str, default):
    """Declare a parameter that accepts int or float from launch files / CLI (e.g. ``time_scale:=200``)."""
    from rcl_interfaces.msg import ParameterDescriptor

    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return node.get_parameter(name).value
