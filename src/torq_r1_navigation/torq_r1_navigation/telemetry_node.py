#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import sys


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_v = 0.0
        self.angular_v = 0.0
        self.min_dist = float('inf')
        self.min_angle = 0.0

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 5 Hz display timer
        self.timer = self.create_timer(0.2, self.display_telemetry)

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.linear_v = msg.twist.twist.linear.x
        self.angular_v = msg.twist.twist.angular.z

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        valid = [(r, i) for i, r in enumerate(msg.ranges) if not math.isnan(r) and not math.isinf(r) and r >= msg.range_min]
        if valid:
            min_r, min_idx = min(valid, key=lambda x: x[0])
            self.min_dist = min_r
            self.min_angle = math.degrees(msg.angle_min + min_idx * msg.angle_increment)

    def display_telemetry(self):
        sector = "Front" if abs(self.min_angle) < 30 else ("Left" if self.min_angle > 0 else "Right")
        sys.stdout.write("\033[2J\033[H")  # Clear screen
        sys.stdout.write(
            "===========================================================\n"
            "             🏎️  TORQ-R1 LIVE TELEMETRY             \n"
            "===========================================================\n"
            f"  📍 Pose (X, Y):        {self.x:+.3f} m,  {self.y:+.3f} m\n"
            f"  🧭 Heading (Yaw):       {math.degrees(self.yaw):+.1f}° ({self.yaw:+.2f} rad)\n"
            f"  ⚡ Linear Speed:       {self.linear_v:+.2f} m/s\n"
            f"  🔄 Angular Speed:      {self.angular_v:+.2f} rad/s\n"
            f"  📡 Nearest Obstacle:    {self.min_dist:.2f} m ({sector} @ {self.min_angle:+.0f}°)\n"
            f"  🗺️  SLAM Status:        MAPPING ACTIVE (Publishing /map)\n"
            "===========================================================\n"
            "  Drive via Teleop Window (Keys: i/j/k/l/, - q/z speed change)\n"
            "===========================================================\n"
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
