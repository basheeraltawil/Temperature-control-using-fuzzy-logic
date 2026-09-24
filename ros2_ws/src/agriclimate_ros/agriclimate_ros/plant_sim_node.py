"""Digital-twin node: simulates a climate zone and publishes /clock.

Replace this node by ``modbus_bridge`` (or a micro-ROS MCU) on real hardware;
every other node stays unchanged.
"""
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float64

from agriclimate.plant.facility import ActuatorCommand, Facility
from agriclimate.plant.sensors import Sensor
from agriclimate.sim.runner import _apply_actuator_faults, _disturbance
from agriclimate.sim.scenario import Scenario

from . import common as c


class PlantSim(Node):
    def __init__(self):
        super().__init__("plant_sim")
        self.declare_parameter("scenario", "tomato_greenhouse_spring")
        c.declare(self, "time_scale", 10.0)      # simulated seconds per wall-clock second
        c.declare(self, "step", 1.0)             # simulated seconds per integration step
        self.declare_parameter("with_faults", True)
        self.sc = Scenario.load(self.get_parameter("scenario").value)
        if not self.get_parameter("with_faults").value:
            self.sc = self.sc.copy(faults={})
        self.dt = float(self.get_parameter("step").value)
        scale = float(self.get_parameter("time_scale").value)
        p = self.sc
        self.plant = Facility(p.facility, p.initial.get("t_air", 18.0), p.initial.get("rh", 70.0))
        self.weather = p.weather()
        cfg = dict(p.sensors_cfg)
        n = int(cfg.pop("count", 2))
        self.sensors = [Sensor(name=f"T{i + 1}", seed=i, faults=p.sensor_faults(i), **cfg) for i in range(n)]
        self.cmd = ActuatorCommand(vent=p.allocator().min_vent)
        self.t = 0.0

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.temp_pubs = [self.create_publisher(Temperature, c.SENSOR_TEMPERATURE.format(i=i + 1), 10)
                          for i in range(n)]
        self.rh_pub = self.create_publisher(RelativeHumidity, c.SENSOR_HUMIDITY, 10)
        self.tout_pub = self.create_publisher(Temperature, c.OUTDOOR_TEMPERATURE, 10)
        self.rhout_pub = self.create_publisher(RelativeHumidity, c.OUTDOOR_HUMIDITY, 10)
        self.solar_pub = self.create_publisher(Float64, c.SOLAR, 10)
        self.truth_pub = self.create_publisher(Float64, c.TRUE_TEMPERATURE, 10)
        for name in c.ACTUATORS:
            self.create_subscription(Float64, c.ACTUATOR.format(name=name),
                                     lambda msg, n=name: self._on_cmd(n, msg), 10)
        self.create_timer(self.dt / scale, self._tick)       # wall-clock timer (this node owns /clock)
        self.get_logger().info(f"simulating '{p.name}' ({p.facility.name}) at {scale}x real time")

    def _on_cmd(self, name, msg):
        setattr(self.cmd, name, min(max(float(msg.data), 0.0), 1.0))

    def _tick(self):
        p, t = self.sc, self.t
        w = self.weather.sample(t)
        _apply_actuator_faults(p, self.plant, t / 3600.0)
        ach, gain = _disturbance(p, t / 3600.0)
        self.plant.step(self.dt, self.cmd, w, ach, gain)
        self.t += self.dt

        clk = Clock()
        clk.clock.sec, clk.clock.nanosec = int(self.t), int((self.t % 1) * 1e9)
        self.clock_pub.publish(clk)
        stamp = clk.clock
        for i, (s, pub) in enumerate(zip(self.sensors, self.temp_pubs)):
            m = Temperature()
            m.header.stamp, m.header.frame_id = stamp, f"sensor_{i + 1}"
            m.temperature, m.variance = s.read(self.plant.state.t_air, self.t, self.dt), s.noise_std ** 2
            pub.publish(m)
        rh = RelativeHumidity()
        rh.header.stamp, rh.relative_humidity = stamp, self.plant.rh / 100.0
        self.rh_pub.publish(rh)
        to = Temperature()
        to.header.stamp, to.temperature = stamp, w.t_out
        self.tout_pub.publish(to)
        rho = RelativeHumidity()
        rho.header.stamp, rho.relative_humidity = stamp, w.rh_out / 100.0
        self.rhout_pub.publish(rho)
        self.solar_pub.publish(Float64(data=w.solar))
        self.truth_pub.publish(Float64(data=self.plant.state.t_air))


def main():
    rclpy.init()
    node = PlantSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
