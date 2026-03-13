#!/usr/bin/env python3
"""Simple waypoint source node.

This node periodically publishes a small set of 2D waypoints on the
`waypoints` topic as a `nav_msgs/Path`. The rest of the pipeline
(`PathSmoothening`, `TrajectoryGen`, `Controller`) treats these as the
coarse global plan to be smoothed and tracked.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Waypoint(Node):
    """Publisher that streams a fixed list of waypoints."""

    def __init__(self):
        super().__init__("waypoints")
        self.publisher_ = self.create_publisher(Path, "waypoints", 10)
        self.timer_ = self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self) -> None:
        """Publish a small, hard-coded set of waypoints in the `map` frame.

        For a real robot this is where one would:
        - Load waypoints from a configuration file, or
        - Listen to a global planner / mission planner topic.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        waypoints = [(0, 0), (1, 1), (2, 1), (3, 2)]
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()   