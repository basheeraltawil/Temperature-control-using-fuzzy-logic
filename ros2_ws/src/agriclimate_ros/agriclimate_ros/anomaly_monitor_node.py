"""AI anomaly monitor node (IsolationForest + learned thermal model).

Publishes the sensors it considers faulty on ``anomaly/excluded_sensors``;
the controller's supervisor then drops them from the sensor fusion.  The
monitor can only *remove* information, never command actuators.
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float64, Int32MultiArray

from agriclimate.sim.runner import build_detector
from agriclimate.sim.scenario import Scenario

from . import common as c


class AnomalyMonitor(Node):
    def __init__(self):
        super().__init__("anomaly_monitor")
        self.declare_parameter("scenario", "tomato_greenhouse_spring")
        self.declare_parameter("sensor_count", 2)
        c.declare(self, "period", 10.0)
        sc = Scenario.load(self.get_parameter("scenario").value)
        self.get_logger().info("training anomaly detector on healthy digital-twin data ...")
        self.detector = build_detector(sc)
        self.get_logger().info(f"detector ready (model: {self.detector.model.metrics})")
        n = int(self.get_parameter("sensor_count").value)
        self.temps = [math.nan] * n
        self.u = {a: 0.0 for a in c.ACTUATORS}
        self.w = {"t_out": 10.0, "rh_out": 60.0, "solar": 0.0}
        for i in range(n):
            self.create_subscription(Temperature, c.SENSOR_TEMPERATURE.format(i=i + 1),
                                     lambda m, i=i: self.temps.__setitem__(i, m.temperature), 10)
        for a in c.ACTUATORS:
            self.create_subscription(Float64, c.ACTUATOR.format(name=a),
                                     lambda m, a=a: self.u.__setitem__(a, m.data), 10)
        self.create_subscription(Temperature, c.OUTDOOR_TEMPERATURE,
                                 lambda m: self.w.__setitem__("t_out", m.temperature), 10)
        self.create_subscription(RelativeHumidity, c.OUTDOOR_HUMIDITY,
                                 lambda m: self.w.__setitem__("rh_out", 100 * m.relative_humidity), 10)
        self.create_subscription(Float64, c.SOLAR, lambda m: self.w.__setitem__("solar", m.data), 10)
        self.pub = self.create_publisher(Int32MultiArray, c.EXCLUDED_SENSORS, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.t0 = None
        self.create_timer(float(self.get_parameter("period").value), self._tick)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        rep = self.detector.update(now - self.t0, list(self.temps), self.u["heater"], self.u["cooler"],
                                   self.u["vent"], self.w["t_out"], self.w["rh_out"], self.w["solar"])
        if rep is None:
            return
        self.pub.publish(Int32MultiArray(data=[int(i) for i in rep.suspect_sensors]))
        level = DiagnosticStatus.WARN if rep.suspect_sensors or rep.actuator_fault else DiagnosticStatus.OK
        msg = "ok"
        if rep.suspect_sensors:
            msg = f"suspect sensors {[i + 1 for i in rep.suspect_sensors]}"
            self.get_logger().warn(msg)
        if rep.actuator_fault:
            msg += f"; {rep.actuator_fault}"
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(c.status(f"{self.get_namespace().strip('/') or 'zone'}/anomaly_monitor", level, msg,
                                   {f"score_sensor_{i + 1}": round(s, 3) for i, s in enumerate(rep.scores)}))
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = AnomalyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
