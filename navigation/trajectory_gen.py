"""Trajectory generation node.

Consumes a smoothed geometric path (`nav_msgs/Path` on `smoothed_path`)
and generates a time-parameterised trajectory published as another
`nav_msgs/Path` on `trajectory`.

Time parameterisation is based on a constant linear velocity; time is
encoded in the header stamp of each `PoseStamped` along the path.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryGen(Node):
    """Node that assigns timestamps along a smoothed path."""

    def __init__(self):
        super().__init__("trajectory_gen")
        self.declare_parameter("velocity", 0.5)
        self.velocity = float(self.get_parameter("velocity").value)

        self.publisher_ = self.create_publisher(Path, "trajectory", 10)
        self.subscriber_ = self.create_subscription(
            Path,
            "smoothed_path",
            self.callback_smoothed_path,
            10,
        )


    def callback_smoothed_path(self, msg: Path) -> None:
        """Build and publish a time-parameterised trajectory."""
        trajectory = self._build_trajectory(msg)
        self.publisher_.publish(trajectory)
        self.get_logger().info(f"Published {len(trajectory.poses)} trajectory points")

    def _build_trajectory(self, msg: Path) -> Path:
        """Assign timestamps to each pose assuming constant linear velocity.

        The geometric path (sequence of x/y poses) is preserved, while
        time is accumulated based on segment length / configured velocity.
        Each pose's `header.stamp` is offset from a common base time by
        the accumulated travel time.
        """
        trajectory = Path()
        trajectory.header = msg.header
        time_accumulated = 0.0
        base_time = self.get_clock().now()
        prev_x = None
        prev_y = None

        for pose in msg.poses:

            x = pose.pose.position.x
            y = pose.pose.position.y

            if prev_x is not None:
                dx = x - prev_x
                dy = y - prev_y

                distance = math.sqrt(dx**2 + dy**2)
                if self.velocity > 0.0:
                    dt = distance / self.velocity
                    time_accumulated += dt

            traj_pose = PoseStamped()
            traj_pose.header.frame_id = trajectory.header.frame_id or "map"
            # Time stamp encodes when the robot should ideally be at this pose.
            traj_pose.header.stamp = (base_time + Duration(seconds=time_accumulated)).to_msg()
            traj_pose.pose.position.x = x
            traj_pose.pose.position.y = y
            traj_pose.pose.orientation.w = 1.0
            trajectory.poses.append(traj_pose)
            prev_x = x
            prev_y = y
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    trajectory_gen = TrajectoryGen()
    rclpy.spin(trajectory_gen)
    trajectory_gen.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()