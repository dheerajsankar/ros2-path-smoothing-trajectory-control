"""Trajectory vs. odometry plotting node.

This node subscribes to:
- `/trajectory` (`nav_msgs/Path`): reference trajectory produced by `TrajectoryGen`.
- `/odom` (`nav_msgs/Odometry`): robot pose from `SimRobot` or a real robot.

When you terminate the node with Ctrl-C, it saves a PNG plot showing
the reference trajectory and the executed path reconstructed from odometry.
"""

from __future__ import annotations

import os
from typing import List

import matplotlib

# Use a non-interactive backend so this works in headless setups.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry


class TrajPlotter(Node):
    """Collects `/trajectory` and `/odom` and generates a 2D plot on shutdown."""

    def __init__(self) -> None:
        super().__init__("traj_plotter")

        self._traj_x: List[float] = []
        self._traj_y: List[float] = []
        self._odom_x: List[float] = []
        self._odom_y: List[float] = []

        self.create_subscription(Path, "trajectory", self._callback_trajectory, 10)
        self.create_subscription(Odometry, "odom", self._callback_odom, 50)

    def _callback_trajectory(self, msg: Path) -> None:
        """Store the latest reference trajectory."""
        self._traj_x = [float(p.pose.position.x) for p in msg.poses]
        self._traj_y = [float(p.pose.position.y) for p in msg.poses]

    def _callback_odom(self, msg: Odometry) -> None:
        """Append the current odometry pose to the executed path trace."""
        self._odom_x.append(float(msg.pose.pose.position.x))
        self._odom_y.append(float(msg.pose.pose.position.y))

    def save_plot(self, filename: str = "trajectory_tracking.png") -> str:
        """Generate and save a trajectory vs. odometry plot.

        Returns the absolute path to the saved PNG file.
        """
        if not self._traj_x or not self._traj_y:
            self.get_logger().warn("No trajectory data received; skipping plot.")
        if not self._odom_x or not self._odom_y:
            self.get_logger().warn("No odometry data received; skipping plot.")

        # Even if one of them is empty, create a plot so the script produces output.
        plt.figure()
        if self._traj_x and self._traj_y:
            plt.plot(self._traj_x, self._traj_y, "b-", label="Reference trajectory")
        if self._odom_x and self._odom_y:
            plt.plot(self._odom_x, self._odom_y, "r--", label="Executed path (odom)")

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Trajectory Tracking")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        out_dir = os.getcwd()
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.abspath(os.path.join(out_dir, filename))
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close()

        self.get_logger().info(f"Saved trajectory tracking plot to: {out_path}")
        return out_path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down traj_plotter, generating plot...")
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

