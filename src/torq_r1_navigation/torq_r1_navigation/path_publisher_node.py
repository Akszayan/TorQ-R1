#!/usr/bin/env python3
"""
Trajectory Path Publisher for TorQ-R1 SLAM Mapping.
Publishes a clean continuous line (nav_msgs/msg/Path) representing the robot's
traveled trajectory in the 'map' frame without any messy arrows or orientation markers.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')

        self.pub_path = self.create_publisher(Path, '/robot_path', 10)

        # TF listener to track robot position in 'map' frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.last_x = None
        self.last_y = None
        self.min_distance_step = 0.03  # Add point every 3 cm of movement

        # Sample at 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self.update_path)
        self.get_logger().info('Path Publisher Node started. Publishing clean trajectory on /robot_path')

    def update_path(self):
        try:
            # Lookup transform from 'map' to 'base_footprint'
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        # Only add point if moved more than min_distance_step or initial pose
        if self.last_x is None:
            dist = 999.0
        else:
            dist = math.hypot(x - self.last_x, y - self.last_y)

        if dist >= self.min_distance_step:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.02  # slightly elevated above floor
            pose.pose.orientation = t.transform.rotation

            self.path_msg.header.stamp = pose.header.stamp
            self.path_msg.poses.append(pose)

            # Cap max history to 5000 points to keep memory minimal
            if len(self.path_msg.poses) > 5000:
                self.path_msg.poses.pop(0)

            self.last_x = x
            self.last_y = y

        # Always publish latest path so newly joined nodes (RViz) display it immediately
        if len(self.path_msg.poses) > 0:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
