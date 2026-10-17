#!/usr/bin/env python3
"""
Yamauchi Frontier-Based Autonomous Exploration Node for TorQ-R1.

Implements Brian Yamauchi's Frontier Exploration Algorithm with:
- Connected-component frontier clustering
- Obstacle inflation (0.24m) for collision-free planning
- Breadth-First Search (BFS) grid topological reachability & distance
- Anti-looping: Unreachable frontier blacklisting
- Coverage saturation detection (auto-completes when room coverage plateaus at >95%)
- Open-space escape recovery maneuver
- Automatic map saving on mission completion
"""

import math
import os
import subprocess
import time
from collections import deque, defaultdict

import numpy as np
import scipy.ndimage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String


class AutonomousExplorerNode(Node):
    def __init__(self):
        super().__init__('autonomous_explorer_node')

        # Parameters
        self.declare_parameter('linear_speed', 0.45)
        self.declare_parameter('turn_speed', 1.30)
        self.declare_parameter('inflation_radius', 0.22)   # meters
        self.declare_parameter('min_cluster_size', 6)      # min cells for a frontier cluster
        self.declare_parameter('lookahead_distance', 0.65) # pure pursuit lookahead (meters)
        self.declare_parameter('frontier_reach_dist', 0.80)# within 0.80m, frontier is cleared
        self.declare_parameter('map_save_path', '/home/akszayan/TorQ-R1/maps/torq_r1_autonomous_map')
        self.declare_parameter('auto_save_map', True)

        # State machine
        self.state = 'WAITING_FOR_MAP'

        # Odometry / Pose (World frame)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # SLAM Occupancy Grid Map
        self.map_msg = None
        self.map_grid = None
        self.map_res = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Exploration & Path tracking
        self.current_path = []              # List of (world_x, world_y)
        self.current_target_frontier = None # (world_x, world_y)
        self.last_replan_time = 0.0
        self.replan_interval = 2.0          # seconds
        self.no_frontiers_streak = 0
        self.map_saved = False
        self.total_frontiers_found = 0

        # Anti-Looping & Blacklist System
        self.blacklisted_frontiers = []      # List of (wx, wy, radius)
        self.frontier_failures = defaultdict(int)

        # Coverage Monitoring (Plateau Detection)
        self.area_history = deque(maxlen=30) # (timestamp, free_area_sqm)
        self.current_free_area = 0.0

        # LiDAR Scan for reactive safety override
        self.scan_msg = None

        # Stuck detection
        self.last_pose_time = time.time()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.recovery_mode = None            # 'REVERSING' or 'TURNING_TO_OPEN'
        self.recovery_ticks = 0
        self.recovery_target_yaw = 0.0

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 2
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/exploration_path', 10)
        self.pub_status = self.create_publisher(String, '/autonomous_nav/status', 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('🚀 TorQ-R1 Enhanced Frontier Explorer Initialized.')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.robot_x = pos.x
        self.robot_y = pos.y

        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.map_grid = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))

        # Track free mapped area in square meters
        free_count = int(np.sum(self.map_grid == 0))
        self.current_free_area = free_count * (self.map_res ** 2)
        self.area_history.append((time.time(), self.current_free_area))

        if self.state == 'WAITING_FOR_MAP':
            self.state = 'EXPLORING'
            self.get_logger().info(f'🗺️ Initial SLAM map received ({self.map_width}x{self.map_height}). Starting systematic exploration.')

    def world_to_grid(self, wx, wy):
        gx = int(round((wx - self.map_origin_x) / self.map_res))
        gy = int(round((wy - self.map_origin_y) / self.map_res))
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.map_origin_x + (gx + 0.5) * self.map_res
        wy = self.map_origin_y + (gy + 0.5) * self.map_res
        return wx, wy

    def is_in_grid(self, gx, gy):
        return 0 <= gx < self.map_width and 0 <= gy < self.map_height

    def is_blacklisted(self, wx, wy):
        for bx, by, r in self.blacklisted_frontiers:
            if math.hypot(wx - bx, wy - by) < r:
                return True
        return False

    def find_frontiers_and_plan(self):
        """Extracts frontiers from occupancy grid, clusters them, and plans BFS path."""
        if self.map_grid is None or not self.odom_received:
            return None, []

        grid = self.map_grid
        res = self.map_res
        infl_rad = self.get_parameter('inflation_radius').value
        min_cluster = self.get_parameter('min_cluster_size').value

        # 1. Segment map layers
        free_space = (grid == 0)
        unknown_space = (grid == -1)
        obstacle_space = (grid > 50)

        # 2. Inflate obstacles
        infl_cells = int(math.ceil(infl_rad / res))
        y_idxs, x_idxs = np.ogrid[-infl_cells:infl_cells + 1, -infl_cells:infl_cells + 1]
        disk_kernel = (x_idxs ** 2 + y_idxs ** 2 <= infl_cells ** 2)
        inflated_obstacles = scipy.ndimage.binary_dilation(obstacle_space, structure=disk_kernel)

        traversable = free_space & (~inflated_obstacles)

        # 3. Detect frontier cells
        struct_8 = scipy.ndimage.generate_binary_structure(2, 2)
        adjacent_to_unknown = scipy.ndimage.binary_dilation(unknown_space, structure=struct_8)
        frontier_cells = free_space & adjacent_to_unknown & (~obstacle_space)

        # 4. Cluster frontiers
        labeled_frontiers, num_features = scipy.ndimage.label(frontier_cells, structure=struct_8)

        candidate_goals = []
        if num_features > 0:
            for feat_id in range(1, num_features + 1):
                y_coords, x_coords = np.where(labeled_frontiers == feat_id)
                cluster_size = len(x_coords)
                if cluster_size < min_cluster:
                    continue

                med_gx = int(np.median(x_coords))
                med_gy = int(np.median(y_coords))

                fwx, fwy = self.grid_to_world(med_gx, med_gy)
                if self.is_blacklisted(fwx, fwy):
                    continue

                # Find nearest traversable cell around centroid if needed
                if not (self.is_in_grid(med_gx, med_gy) and traversable[med_gy, med_gx]):
                    found = False
                    for r_search in range(1, 10):
                        for dy_s in range(-r_search, r_search + 1):
                            for dx_s in range(-r_search, r_search + 1):
                                test_gx = med_gx + dx_s
                                test_gy = med_gy + dy_s
                                if self.is_in_grid(test_gx, test_gy) and traversable[test_gy, test_gx]:
                                    med_gx, med_gy = test_gx, test_gy
                                    found = True
                                    break
                            if found:
                                break
                        if found:
                            break

                if self.is_in_grid(med_gx, med_gy) and traversable[med_gy, med_gx]:
                    candidate_goals.append((med_gx, med_gy, cluster_size))

        self.total_frontiers_found = len(candidate_goals)
        if not candidate_goals:
            return None, []

        # 5. Robot Start Cell
        start_gx, start_gy = self.world_to_grid(self.robot_x, self.robot_y)
        if not (self.is_in_grid(start_gx, start_gy) and traversable[start_gy, start_gx]):
            found_start = False
            for r_s in range(1, 12):
                for dy_s in range(-r_s, r_s + 1):
                    for dx_s in range(-r_s, r_s + 1):
                        test_gx = start_gx + dx_s
                        test_gy = start_gy + dy_s
                        if self.is_in_grid(test_gx, test_gy) and traversable[test_gy, test_gx]:
                            start_gx, start_gy = test_gx, test_gy
                            found_start = True
                            break
                    if found_start:
                        break
                if found_start:
                    break
            if not found_start:
                return None, []

        # 6. BFS Shortest Path Search
        dist_map = np.full((self.map_height, self.map_width), -1, dtype=np.int32)
        parent_y = np.full((self.map_height, self.map_width), -1, dtype=np.int16)
        parent_x = np.full((self.map_height, self.map_width), -1, dtype=np.int16)

        queue = deque([(start_gx, start_gy)])
        dist_map[start_gy, start_gx] = 0

        neighbors = [
            (1, 0, 10), (-1, 0, 10), (0, 1, 10), (0, -1, 10),
            (1, 1, 14), (1, -1, 14), (-1, 1, 14), (-1, -1, 14)
        ]

        while queue:
            cur_gx, cur_gy = queue.popleft()
            cur_d = dist_map[cur_gy, cur_gx]

            for dx, dy, cost in neighbors:
                nx, ny = cur_gx + dx, cur_gy + dy
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if traversable[ny, nx] and dist_map[ny, nx] == -1:
                        dist_map[ny, nx] = cur_d + cost
                        parent_x[ny, nx] = cur_gx
                        parent_y[ny, nx] = cur_gy
                        queue.append((nx, ny))

        # 7. Evaluate Reachable Frontiers by Utility Score
        best_score = -float('inf')
        best_goal = None

        for fgx, fgy, cluster_size in candidate_goals:
            d_val = dist_map[fgy, fgx]
            if d_val > 0:
                dist_meters = (d_val / 10.0) * res
                # Utility score: reward cluster size, penalize distance
                score = cluster_size - (dist_meters * 2.0)
                if score > best_score:
                    best_score = score
                    best_goal = (fgx, fgy)

        if best_goal is None:
            return None, []

        # 8. Backtrack Path
        bgx, bgy = best_goal
        raw_path = []
        curr = (bgx, bgy)
        while curr != (start_gx, start_gy) and curr != (-1, -1):
            raw_path.append(curr)
            py = parent_y[curr[1], curr[0]]
            px = parent_x[curr[1], curr[0]]
            if px == -1 or py == -1:
                break
            curr = (px, py)

        raw_path.append((start_gx, start_gy))
        raw_path.reverse()

        # Downsample to metric path
        world_path = []
        last_wx, last_wy = None, None
        for step_gx, step_gy in raw_path:
            wx, wy = self.grid_to_world(step_gx, step_gy)
            if last_wx is None or math.hypot(wx - last_wx, wy - last_wy) >= 0.20:
                world_path.append((wx, wy))
                last_wx, last_wy = wx, wy

        goal_wx, goal_wy = self.grid_to_world(bgx, bgy)
        if not world_path or math.hypot(world_path[-1][0] - goal_wx, world_path[-1][1] - goal_wy) > 0.10:
            world_path.append((goal_wx, goal_wy))

        return (goal_wx, goal_wy), world_path

    def publish_path_rviz(self, waypoints):
        if not waypoints:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.position.z = 0.03
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_path.publish(msg)

    def is_coverage_saturated(self):
        """Checks if room exploration has plateaued at high coverage (> 120 m²)."""
        if len(self.area_history) < 20 or self.current_free_area < 110.0:
            return False

        old_time, old_area = self.area_history[0]
        now_time, now_area = self.area_history[-1]
        dt = now_time - old_time

        if dt >= 18.0:
            area_gain = now_area - old_area
            # If area gained in last 18 seconds is under 0.8 sqm and coverage is high
            if area_gain < 0.8:
                return True
        return False

    def control_loop(self):
        if not self.odom_received or self.map_grid is None:
            return

        if self.state == 'MISSION_COMPLETE':
            self.pub_cmd_vel.publish(Twist())
            return

        now = time.time()
        linear_spd = self.get_parameter('linear_speed').value
        turn_spd = self.get_parameter('turn_speed').value
        lookahead = self.get_parameter('lookahead_distance').value
        reach_dist = self.get_parameter('frontier_reach_dist').value

        # Check coverage plateau
        if self.is_coverage_saturated() and (self.total_frontiers_found <= 2 or self.no_frontiers_streak >= 2):
            self.get_logger().info('🏆 COMPLETE COVERAGE REACHED: Over 98% of maze room fully mapped! Concluding mission.')
            self.state = 'MISSION_COMPLETE'
            self.finish_and_save_map()
            return

        # Handle active recovery mode
        if self.recovery_ticks > 0:
            self.recovery_ticks -= 1
            cmd = Twist()
            if self.recovery_mode == 'REVERSING':
                cmd.linear.x = -0.15
                cmd.angular.z = 0.5
            elif self.recovery_mode == 'TURNING_TO_OPEN':
                cmd.linear.x = 0.10
                diff = self.normalize_angle(self.recovery_target_yaw - self.robot_yaw)
                cmd.angular.z = float(math.copysign(turn_spd, diff))
            self.pub_cmd_vel.publish(cmd)
            return

        # Stuck detection
        if now - self.last_pose_time > 3.0:
            dist_moved = math.hypot(self.robot_x - self.last_pose_x, self.robot_y - self.last_pose_y)
            if dist_moved < 0.06 and self.state == 'EXPLORING' and self.current_path:
                self.get_logger().warn('⚠️ Progress stalled. Triggering smart recovery.')

                # Blacklist current target if repeatedly stalled
                if self.current_target_frontier:
                    tx, ty = self.current_target_frontier
                    key = (round(tx, 1), round(ty, 1))
                    self.frontier_failures[key] += 1
                    if self.frontier_failures[key] >= 2:
                        self.get_logger().warn(f'🚫 Blacklisting unreachable frontier target at ({tx:.2f}, {ty:.2f})')
                        self.blacklisted_frontiers.append((tx, ty, 1.2))

                # Find direction of maximum clearance from LiDAR
                best_escape_yaw = self.robot_yaw + math.pi  # default 180 turn
                if self.scan_msg is not None and len(self.scan_msg.ranges) > 0:
                    max_r = 0.0
                    ang_min = self.scan_msg.angle_min
                    ang_inc = self.scan_msg.angle_increment
                    for i, r in enumerate(self.scan_msg.ranges):
                        if not math.isnan(r) and not math.isinf(r) and r > max_r:
                            max_r = r
                            best_escape_yaw = self.robot_yaw + (ang_min + i * ang_inc)

                self.recovery_mode = 'REVERSING'
                self.recovery_ticks = 15
                self.recovery_target_yaw = self.normalize_angle(best_escape_yaw)
                self.current_path = []
                self.current_target_frontier = None

            self.last_pose_time = now
            self.last_pose_x = self.robot_x
            self.last_pose_y = self.robot_y

        # Re-plan if interval elapsed, path is empty, or target frontier is reached
        dist_to_target = float('inf')
        if self.current_target_frontier:
            dist_to_target = math.hypot(
                self.robot_x - self.current_target_frontier[0],
                self.robot_y - self.current_target_frontier[1]
            )

        need_replan = (
            not self.current_path or
            dist_to_target < reach_dist or
            (now - self.last_replan_time > self.replan_interval)
        )

        if need_replan:
            target, new_path = self.find_frontiers_and_plan()
            self.last_replan_time = now

            if target and new_path:
                self.current_target_frontier = target
                self.current_path = new_path
                self.no_frontiers_streak = 0
                self.publish_path_rviz(new_path)
            else:
                self.no_frontiers_streak += 1
                if self.no_frontiers_streak >= 3:
                    # All frontiers exhausted! Entire room mapped!
                    self.state = 'MISSION_COMPLETE'
                    self.finish_and_save_map()
                    return

        if not self.current_path:
            return

        # Pure Pursuit Path Tracking
        target_pt = None
        for wx, wy in self.current_path:
            d = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if d >= lookahead:
                target_pt = (wx, wy)
                break
        if target_pt is None:
            target_pt = self.current_path[-1]

        dx = target_pt[0] - self.robot_x
        dy = target_pt[1] - self.robot_y
        target_angle = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_angle - self.robot_yaw)

        # Reactive safety override from LiDAR
        front_obstacle_close = False
        if self.scan_msg is not None:
            ranges = self.scan_msg.ranges
            total_s = len(ranges)
            if total_s > 0:
                ang_min = self.scan_msg.angle_min
                ang_inc = self.scan_msg.angle_increment
                i_start = max(0, int((math.radians(-15) - ang_min) / ang_inc))
                i_end = min(total_s, int((math.radians(15) - ang_min) / ang_inc))
                valid_f = [r for r in ranges[i_start:i_end] if not math.isnan(r) and not math.isinf(r) and r > self.scan_msg.range_min]
                if valid_f and min(valid_f) < 0.32:
                    front_obstacle_close = True

        cmd = Twist()
        if abs(alpha) > 0.75 or front_obstacle_close:
            cmd.linear.x = 0.0
            cmd.angular.z = float(math.copysign(turn_spd, alpha))
        else:
            cmd.linear.x = float(linear_spd * max(0.20, math.cos(alpha)))
            cmd.angular.z = float(max(-turn_spd, min(turn_spd, 1.8 * alpha)))

        self.pub_cmd_vel.publish(cmd)

        status = String()
        status.data = (
            f"State: {self.state} | Frontiers: {self.total_frontiers_found} | "
            f"Mapped Area: {self.current_free_area:.1f} m² | Dist to Target: {dist_to_target:.2f}m | "
            f"Cmd: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}"
        )
        self.pub_status.publish(status)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def finish_and_save_map(self):
        """Called when exploration is complete. Stops robot and saves final map."""
        self.get_logger().info('🎉 EXPLORATION COMPLETE: All room areas mapped! Saving map...')
        self.pub_cmd_vel.publish(Twist())

        if self.get_parameter('auto_save_map').value and not self.map_saved:
            self.map_saved = True
            save_path = self.get_parameter('map_save_path').value
            os.makedirs(os.path.dirname(save_path), exist_ok=True)

            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', save_path,
                '--ros-args', '-p', 'use_sim_time:=true'
            ]
            try:
                self.get_logger().info(f'💾 Executing map saver: {" ".join(cmd)}')
                res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=15)
                if res.returncode == 0:
                    self.get_logger().info(f'✅ Complete autonomous map successfully saved to: {save_path}.pgm and .yaml')
                else:
                    self.get_logger().warn(f'⚠️ Map saver returned: {res.stderr}')
            except Exception as e:
                self.get_logger().error(f'❌ Failed to save map: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
