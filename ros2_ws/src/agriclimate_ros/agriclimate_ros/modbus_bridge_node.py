"""Hardware bridge: PLC / remote I/O over Modbus TCP <-> ROS 2 topics.

Drop-in replacement for ``plant_sim`` on the real installation.  The PLC
keeps running its local fuzzy-PI function block whenever this bridge (and
thus the heartbeat) stops.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float64

from agriclimate.io.modbus_io import ModbusClimateIO, RegisterMap
from agriclimate.plant.facility import ActuatorCommand

from . import common as c


class ModbusBridge(Node):
    def __init__(self):
        super().__init__("modbus_bridge")
        self.declare_parameter("host", "192.168.0.10")
        self.declare_parameter("port", 502)
        self.declare_parameter("unit", 1)
        self.declare_parameter("sensor_count", 2)
        c.declare(self, "period", 1.0)
        n = int(self.get_parameter("sensor_count").value)
        self.io = ModbusClimateIO(self.get_parameter("host").value, int(self.get_parameter("port").value),
                                  RegisterMap(n_temperature=n, unit=int(self.get_parameter("unit").value)))
        if not self.io.connect():
            self.get_logger().error("cannot connect to PLC - will retry every cycle")
        self.cmd = ActuatorCommand()
        self.temp_pubs = [self.create_publisher(Temperature, c.SENSOR_TEMPERATURE.format(i=i + 1), 10)
                          for i in range(n)]
        self.rh_pub = self.create_publisher(RelativeHumidity, c.SENSOR_HUMIDITY, 10)
        self.tout_pub = self.create_publisher(Temperature, c.OUTDOOR_TEMPERATURE, 10)
        self.solar_pub = self.create_publisher(Float64, c.SOLAR, 10)
        for a in c.ACTUATORS:
            self.create_subscription(Float64, c.ACTUATOR.format(name=a),
                                     lambda m, a=a: setattr(self.cmd, a, float(m.data)), 10)
        self.create_timer(float(self.get_parameter("period").value), self._tick)

    def _tick(self):
        try:
            d = self.io.read()
            self.io.write(self.cmd)
        except (IOError, OSError) as exc:
            self.get_logger().warn(f"Modbus: {exc}", throttle_duration_sec=10.0)
            self.io.connect()
            return
        stamp = self.get_clock().now().to_msg()
        for i, (v, pub) in enumerate(zip(d["temperatures"], self.temp_pubs)):
            m = Temperature()
            m.header.stamp, m.header.frame_id, m.temperature = stamp, f"sensor_{i + 1}", v
            pub.publish(m)
        rh = RelativeHumidity()
        rh.header.stamp, rh.relative_humidity = stamp, d["rh"] / 100.0
        self.rh_pub.publish(rh)
        to = Temperature()
        to.header.stamp, to.temperature = stamp, d["t_out"]
        self.tout_pub.publish(to)
        self.solar_pub.publish(Float64(data=d["solar"]))


def main():
    rclpy.init()
    node = ModbusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.io.close()
        node.destroy_node()
        rclpy.try_shutdown()
