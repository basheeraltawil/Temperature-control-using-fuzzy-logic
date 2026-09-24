"""Climate controller node: fuzzy-PI / PID / MPC + split-range allocator +
safety supervisor, executed as one deterministic cycle (agriclimate.runtime).

Keeping the supervisor in the same cycle as the controller avoids a race
between two nodes on the actuator topics; the *independent* last line of
defence is the PLC heartbeat watchdog and the hard-wired high-limit
thermostat (see docs/IMPLEMENTATION_GUIDE.md).
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float64, Int32MultiArray

from agriclimate.runtime import ClimateRuntime
from agriclimate.sim.runner import make_controller
from agriclimate.sim.scenario import Scenario

from . import common as c


class ClimateController(Node):
    def __init__(self):
        super().__init__("climate_controller")
        self.declare_parameter("scenario", "tomato_greenhouse_spring")   # limits, allocator, gains
        self.declare_parameter("controller", "fuzzy_pi")                 # fuzzy_pi | pid | mpc
        c.declare(self, "period", 10.0)                           # s (ROS time)
        self.declare_parameter("sensor_count", 2)
        self.declare_parameter("model_path", "")                         # identified model (MPC)
        c.declare(self, "sensor_timeout", 60.0)
        sc = Scenario.load(self.get_parameter("scenario").value)
        name = self.get_parameter("controller").value
        model_path = self.get_parameter("model_path").value
        if name == "mpc" and model_path:
            from agriclimate.ai.sysid import LearnedThermalModel
            from agriclimate.control.mpc import MPCController

            ctrl = MPCController(model=LearnedThermalModel.load(model_path), allocator=sc.allocator())
        else:
            if name == "mpc":
                self.get_logger().warn("no model_path: identifying the MPC model on the digital twin")
            ctrl, _ = make_controller(name, sc)
        self.runtime = ClimateRuntime(ctrl, sc.allocator(), sc.supervisor_config(), supervised=True)
        self.period = float(self.get_parameter("period").value)
        self.timeout = float(self.get_parameter("sensor_timeout").value)
        n = int(self.get_parameter("sensor_count").value)

        self.temps = [(math.nan, -math.inf)] * n
        self.rh, self.t_out, self.rh_out, self.solar = 60.0, 10.0, 60.0, 0.0
        self.setpoint = sc.setpoint(0.0)
        for i in range(n):
            self.create_subscription(Temperature, c.SENSOR_TEMPERATURE.format(i=i + 1),
                                     lambda m, i=i: self._on_temp(i, m), 10)
        self.create_subscription(RelativeHumidity, c.SENSOR_HUMIDITY,
                                 lambda m: setattr(self, "rh", 100.0 * m.relative_humidity), 10)
        self.create_subscription(Temperature, c.OUTDOOR_TEMPERATURE,
                                 lambda m: setattr(self, "t_out", m.temperature), 10)
        self.create_subscription(RelativeHumidity, c.OUTDOOR_HUMIDITY,
                                 lambda m: setattr(self, "rh_out", 100.0 * m.relative_humidity), 10)
        self.create_subscription(Float64, c.SOLAR, lambda m: setattr(self, "solar", m.data), 10)
        self.create_subscription(Float64, c.SETPOINT, lambda m: setattr(self, "setpoint", m.data), 10)
        self.create_subscription(Int32MultiArray, c.EXCLUDED_SENSORS, self._on_excluded, 10)

        self.act_pubs = {a: self.create_publisher(Float64, c.ACTUATOR.format(name=a), 10) for a in c.ACTUATORS}
        self.demand_pub = self.create_publisher(Float64, c.DEMAND, 10)
        self.valid_pub = self.create_publisher(Float64, c.VALIDATED_TEMPERATURE, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.t0 = None
        self.create_timer(self.period, self._cycle)
        self.get_logger().info(f"controller '{name}' on scenario '{sc.name}', period {self.period}s")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_temp(self, i, msg):
        self.temps[i] = (msg.temperature, self._now())

    def _on_excluded(self, msg):
        self.runtime.excluded = list(msg.data)      # AI monitor verdict (advisory input to validation)

    def _cycle(self):
        now = self._now()
        if self.t0 is None:
            self.t0 = now
        # stale sensor values are handed to the supervisor as invalid
        readings = [v if now - ts <= self.timeout else math.nan for v, ts in self.temps]
        out = self.runtime.step(now - self.t0, self.period, readings, self.setpoint, self.rh,
                                self.t_out, self.rh_out, self.solar)
        for a in c.ACTUATORS:
            self.act_pubs[a].publish(Float64(data=float(getattr(out.command, a))))
        self.demand_pub.publish(Float64(data=float(out.demand)))
        if math.isfinite(out.temperature):
            self.valid_pub.publish(Float64(data=float(out.temperature)))
        for a in out.alarms:
            log = self.get_logger().error if a.level == "ALARM" else self.get_logger().warn
            log(f"[{a.code}] {a.message}")
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        level = c.LEVELS.get(out.quality, DiagnosticStatus.ERROR)
        if out.mode not in ("AUTO",):
            level = DiagnosticStatus.ERROR
        arr.status.append(c.status(
            f"{self.get_namespace().strip('/') or 'zone'}/climate_controller", level,
            f"mode={out.mode} quality={out.quality}",
            {"setpoint": round(self.setpoint, 2), "temperature": round(out.temperature, 2),
             "demand": round(out.demand, 3), "heater": round(out.command.heater, 3),
             "cooler": round(out.command.cooler, 3), "vent": round(out.command.vent, 3),
             "excluded_sensors": out.excluded, "active_alarms": sorted(self.runtime.supervisor.active_alarms)}))
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = ClimateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
