#!/usr/bin/env python3
"""Path smoothing node.

Takes in a discrete `nav_msgs/Path` on the `waypoints` topic and returns
an up-sampled, smoothed version on `smoothed_path` using a cubic B-spline
(`scipy.interpolate.splprep`/`splev`).

This implements the "Path Smoothing" part of the assignment.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import splprep, splev


class PathSmoothening(Node):
    """Node that smooths a coarse path using cubic B-splines."""

    def __init__(self):
        super().__init__("path_smoothening")
        self.declare_parameter("smoothing", 0.5)
        self.declare_parameter("num_points", 100)

        # Subscribe to coarse waypoints and publish smoothed path.
        self.subscriber_ = self.create_subscription(
            Path,
            "waypoints",
            self.callback_waypoints,
            10,
        )
        self.publisher_ = self.create_publisher(Path, "smoothed_path", 10)

    def callback_waypoints(self, msg: Path) -> None:
        """Callback for incoming coarse waypoints."""
        smoothed = self.smooth_path(
            msg,
            smoothing=float(self.get_parameter("smoothing").value),
            num_points=int(self.get_parameter("num_points").value),
        )
        self.publisher_.publish(smoothed)
        self.get_logger().info(f"Published {len(smoothed.poses)} smoothed waypoints")

    def smooth_path(
        self,
        path_msg: Path,
        *,
        smoothing: float,
        num_points: int,
    ) -> Path:
        """Fit a spline to the input path and return an up-sampled path.

        - If there are fewer than 4 points, the original path is returned
          unchanged because a cubic B-spline cannot be reliably fit.
        - If the spline fit fails (e.g. due to duplicate points), a warning
          is logged and the raw path is passed through.
        """
        poses_in = path_msg.poses
        if len(poses_in) == 0:
            out = Path()
            out.header = path_msg.header
            return out

        if len(poses_in) < 4:
            out = Path()
            out.header = path_msg.header
            out.poses = list(poses_in)
            return out

        xs = np.array([p.pose.position.x for p in poses_in], dtype=float)
        ys = np.array([p.pose.position.y for p in poses_in], dtype=float)

        # SciPy uses a single smoothing factor `s` (bigger => smoother).
        s = max(0.0, float(smoothing))
        try:
            tck, _u = splprep([xs, ys], s=s, k=3)
            n = max(2, int(num_points))
            u_new = np.linspace(0.0, 1.0, n)
            x_new, y_new = splev(u_new, tck)
        except Exception as e:
            # If spline fit fails (e.g., duplicate points), fall back to original path.
            self.get_logger().warn(f"Spline smoothing failed, publishing raw path: {e}")
            out = Path()
            out.header = path_msg.header
            out.poses = list(poses_in)
            return out

        out = Path()
        out.header = path_msg.header
        out.poses = []
        for i in range(len(x_new)):
            pose = PoseStamped()
            # Keep frame consistent; stamps can be filled by upstream or left default.
            pose.header.frame_id = out.header.frame_id
            pose.header.stamp = out.header.stamp
            pose.pose.position.x = float(x_new[i])
            pose.pose.position.y = float(y_new[i])
            pose.pose.orientation.w = 1.0
            out.poses.append(pose)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoothening()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()