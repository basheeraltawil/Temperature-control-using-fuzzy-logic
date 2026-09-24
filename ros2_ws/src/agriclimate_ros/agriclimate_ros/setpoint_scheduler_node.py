"""Publishes the climate recipe (setpoint schedule) of a scenario/recipe file."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from agriclimate.sim.scenario import Scenario

from . import common as c


class SetpointScheduler(Node):
    def __init__(self):
        super().__init__("setpoint_scheduler")
        self.declare_parameter("scenario", "tomato_greenhouse_spring")
        c.declare(self, "recipe_offset_h", 0.0)   # start the recipe at a later hour/day
        c.declare(self, "period", 10.0)
        self.schedule = Scenario.load(self.get_parameter("scenario").value).setpoint
        self.offset = float(self.get_parameter("recipe_offset_h").value) * 3600.0
        self.pub = self.create_publisher(Float64, c.SETPOINT, 10)
        self.t0 = None
        self.create_timer(float(self.get_parameter("period").value), self._tick)

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.pub.publish(Float64(data=float(self.schedule(self.offset + now - self.t0))))


def main():
    rclpy.init()
    node = SetpointScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
