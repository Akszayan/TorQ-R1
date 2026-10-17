#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters
        self.declare_parameter('safe_distance', 0.50)
        self.declare_parameter('critical_distance', 0.25)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('turn_speed', 0.75)
        self.declare_parameter('enabled', True)

        # State tracking
        self.state = 'FORWARD'
        self.reverse_counter = 0

        # Subscriptions & Publications
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/obstacle_avoidance/status', 10)

        self.get_logger().info('Obstacle Avoidance Node started. Subscribed to /scan')

    def scan_callback(self, msg: LaserScan):
        if not self.get_parameter('enabled').value:
            return

        safe_dist = self.get_parameter('safe_distance').value
        crit_dist = self.get_parameter('critical_distance').value
        linear_spd = self.get_parameter('linear_speed').value
        turn_spd = self.get_parameter('turn_speed').value

        total_samples = len(msg.ranges)
        if total_samples == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        def get_sector_min(start_deg, end_deg):
            rad_start = math.radians(start_deg)
            rad_end = math.radians(end_deg)
            idx_start = int((rad_start - angle_min) / angle_inc)
            idx_end = int((rad_end - angle_min) / angle_inc)

            idx_low = max(0, min(idx_start, idx_end))
            idx_high = min(total_samples, max(idx_start, idx_end))

            ranges = []
            for r in msg.ranges[idx_low:idx_high]:
                if not math.isnan(r) and not math.isinf(r) and r >= msg.range_min:
                    ranges.append(r)
            return min(ranges) if ranges else msg.range_max

        # Extract sector minimum ranges
        dist_front = get_sector_min(-20, 20)
        dist_front_left = get_sector_min(20, 65)
        dist_front_right = get_sector_min(-65, -20)
        dist_left = get_sector_min(65, 110)
        dist_right = get_sector_min(-110, -65)

        cmd = Twist()

        # Handle reverse escape sequence
        if self.reverse_counter > 0:
            self.reverse_counter -= 1
            cmd.linear.x = -0.15
            cmd.angular.z = turn_spd if dist_right < dist_left else -turn_spd
            self.state = 'ESCAPE_REVERSE'
        elif dist_front < crit_dist or dist_front_left < crit_dist or dist_front_right < crit_dist:
            # Emergency: critically close obstacle ahead
            self.reverse_counter = 8
            cmd.linear.x = -0.15
            cmd.angular.z = turn_spd if dist_right < dist_left else -turn_spd
            self.state = 'EMERGENCY_REVERSE'
        elif dist_front < safe_dist or dist_front_left < safe_dist or dist_front_right < safe_dist:
            # Turn towards the more open side
            cmd.linear.x = 0.05
            if dist_front_left > dist_front_right:
                cmd.angular.z = turn_spd
                self.state = 'AVOID_TURNING_LEFT'
            else:
                cmd.angular.z = -turn_spd
                self.state = 'AVOID_TURNING_RIGHT'
        else:
            # Path is clear
            cmd.linear.x = linear_spd
            # Subtle centering bias if walls on sides
            side_diff = dist_left - dist_right
            cmd.angular.z = float(math.tanh(side_diff * 0.2))
            self.state = 'FORWARD_CRUISE'

        self.pub_cmd_vel.publish(cmd)

        status_msg = String()
        status_msg.data = f"State: {self.state} | Front: {dist_front:.2f}m, FL: {dist_front_left:.2f}m, FR: {dist_front_right:.2f}m"
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.pub_cmd_vel.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
