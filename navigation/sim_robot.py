"""Simple differential-drive robot simulator.

This node emulates a unicycle / differential-drive robot in 2D. It
subscribes to `cmd_vel` and integrates a kinematic model forward in time
to produce `odom` and a `visualization_msgs/Marker` triangle representing
the robot footprint.

The simulator intentionally ignores dynamics (no acceleration limits,
slip, or noise) so that controller performance is easier to interpret.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class SimRobot(Node):
    """Integrates a unicycle model and publishes odometry and a marker."""

    def __init__(self):
        super().__init__("sim_robot")
        self.declare_parameter("velocity", 0.5)
        self.velocity = float(self.get_parameter("velocity").value)
        self.declare_parameter("robot_size", 0.35)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.marker_pub = self.create_publisher(Marker, "robot_marker", 10)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.callback_cmd_vel, 10)
        self.timer = self.create_timer(0.05, self.update)

        self.v = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def callback_cmd_vel(self, msg: Twist) -> None:
        """Store the latest commanded velocities."""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self) -> None:
        """Integrate the kinematic model and publish odometry + visualization."""
        dt = 0.05
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

        self.marker_pub.publish(self._make_triangle_marker())

    def _make_triangle_marker(self) -> Marker:
        size = float(self.get_parameter("robot_size").value)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sim_robot"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.x)
        marker.pose.position.y = float(self.y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = math.sin(self.theta / 2.0)
        marker.pose.orientation.w = math.cos(self.theta / 2.0)

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Green triangle (3 points) in the robot frame; pose above places/rotates it.
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 1.0

        tip = Point(x=size * 0.6, y=0.0, z=0.0)
        left = Point(x=-size * 0.4, y=+size * 0.35, z=0.0)
        right = Point(x=-size * 0.4, y=-size * 0.35, z=0.0)
        marker.points = [tip, left, right]

        marker.lifetime.sec = 0
        marker.frame_locked = False
        return marker

def main(args=None):
    rclpy.init(args=args)
    sim_robot = SimRobot()
    rclpy.spin(sim_robot)
    sim_robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()