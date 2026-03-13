"""Trajectory tracking controller node.

This node subscribes to:
- `trajectory` (`nav_msgs/Path`): desired time-parameterised trajectory
- `odom` (`nav_msgs/Odometry`): current robot pose in the `map` frame

and publishes:
- `cmd_vel` (`geometry_msgs/Twist`): linear and angular velocity commands

The controller is a lookahead-based tracker conceptually similar to
pure pursuit: it selects a target waypoint some distance ahead along the
trajectory and drives towards it using proportional control on both
distance and heading error, with additional logic to:
- Reset progress when the trajectory changes significantly
- Stop once the final goal is reached
- Rotate in place when the heading error is large.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw angle (z-axis) from a unit quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to the range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Controller(Node):
    """Lookahead-based trajectory tracking controller."""

    def __init__(self):
        super().__init__("controller")
        self.declare_parameter("k_lin", 0.8)
        self.declare_parameter("k_ang", 2.0)
        self.declare_parameter("max_v", 0.6)
        self.declare_parameter("max_w", 1.5)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("rotate_in_place_yaw", 1.4)
        self.declare_parameter("lookahead_points", 5)
        self.declare_parameter("path_change_eps", 1e-3)

        # Subscriptions: desired trajectory and robot odometry.
        self.traj_sub = self.create_subscription(
            Path,
            "trajectory",
            self.callback_trajectory,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.callback_odometry,
            10,
        )
        # Publisher for differential-drive velocity commands.
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Main control loop; period chosen to be reasonably fast yet lightweight.
        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_pose = None
        self.current_trajectory: Path | None = None
        self._target_idx = 0

    def callback_trajectory(self, msg: Path) -> None:
        """Store the latest trajectory, resetting progress only on real changes."""
        # Don't reset progress on every re-publication; only reset when the path changes.
        eps = float(self.get_parameter("path_change_eps").value)
        if self.current_trajectory is None or len(self.current_trajectory.poses) != len(msg.poses):
            self.current_trajectory = msg
            self._target_idx = 0
            return

        if len(msg.poses) == 0:
            self.current_trajectory = msg
            self._target_idx = 0
            return

        old = self.current_trajectory.poses
        new = msg.poses
        dx0 = float(new[0].pose.position.x - old[0].pose.position.x)
        dy0 = float(new[0].pose.position.y - old[0].pose.position.y)
        dx1 = float(new[-1].pose.position.x - old[-1].pose.position.x)
        dy1 = float(new[-1].pose.position.y - old[-1].pose.position.y)
        if (dx0 * dx0 + dy0 * dy0) > eps * eps or (dx1 * dx1 + dy1 * dy1) > eps * eps:
            self._target_idx = 0
        self.current_trajectory = msg

    def callback_odometry(self, msg: Odometry) -> None:
        """Update the current robot pose from odometry."""
        self.current_pose = msg.pose.pose

    def control_loop(self) -> None:
        """Periodic control loop that computes and publishes `cmd_vel`."""
        if self.current_pose is None:
            return

        if self.current_trajectory is None or len(self.current_trajectory.poses) == 0:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        poses = self.current_trajectory.poses
        self._target_idx = min(self._target_idx, len(poses) - 1)

        px = float(self.current_pose.position.x)
        py = float(self.current_pose.position.y)
        q = self.current_pose.orientation
        yaw = _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        goal_tol = float(self.get_parameter("goal_tolerance").value)

        # Advance target if we're close enough.
        while self._target_idx < len(poses) - 1:
            tx = float(poses[self._target_idx].pose.position.x)
            ty = float(poses[self._target_idx].pose.position.y)
            if math.hypot(tx - px, ty - py) > goal_tol:
                break
            self._target_idx += 1

        tx = float(poses[self._target_idx].pose.position.x)
        ty = float(poses[self._target_idx].pose.position.y)
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = _wrap_to_pi(desired_yaw - yaw)

        k_lin = float(self.get_parameter("k_lin").value)
        k_ang = float(self.get_parameter("k_ang").value)
        max_v = float(self.get_parameter("max_v").value)
        max_w = float(self.get_parameter("max_w").value)
        rotate_in_place_yaw = float(self.get_parameter("rotate_in_place_yaw").value)
        lookahead_points = int(self.get_parameter("lookahead_points").value)

        # Re-target to a point ahead of the closest waypoint (within a window),
        # so we track through the trajectory instead of orbiting a single point.
        window_start = max(0, self._target_idx - 10)
        window_end = min(len(poses), self._target_idx + 50)
        best_i = self._target_idx
        best_d2 = float("inf")
        for i in range(window_start, window_end):
            wx = float(poses[i].pose.position.x)
            wy = float(poses[i].pose.position.y)
            d2 = (wx - px) ** 2 + (wy - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        self._target_idx = min(len(poses) - 1, best_i + max(0, lookahead_points))

        cmd = Twist()
        cmd.angular.z = max(-max_w, min(max_w, k_ang * yaw_err))

        # Drive forward toward target. If we're facing mostly away, rotate first.
        v_cmd = max(0.0, min(max_v, k_lin * dist))
        if abs(yaw_err) > rotate_in_place_yaw:
            v_cmd = 0.0
        cmd.linear.x = v_cmd

        # Stop at final goal once we are inside the positional tolerance.
        if self._target_idx == len(poses) - 1 and dist <= goal_tol:
            cmd = Twist()

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()