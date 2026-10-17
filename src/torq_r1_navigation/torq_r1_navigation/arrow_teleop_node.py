#!/usr/bin/env python3
"""
High-Speed Differential Drive Teleoperation Node for TorQ-R1.

Supports responsive in-place pivoting and continuous curving:
  - Up Arrow (▲) / 'w' / 'i' : Drive forward
  - Down Arrow (▼) / 's' / ',' : Drive backward
  - Left Arrow (◄) / 'a' / 'j' : Fast differential turn left in place
  - Right Arrow (►) / 'd' / 'l': Fast differential turn right in place
  - 'u' : Curve Forward + Left
  - 'o' : Curve Forward + Right
  - 'm' : Curve Backward + Left
  - '.' : Curve Backward + Right
  - Spacebar / 'k' : Immediate Stop
  - '+' / '=' / ']' / 'e': Increase speed (+0.15 m/s, +0.40 rad/s)
  - '-' / '_' / '['      : Decrease speed (-0.15 m/s, -0.40 rad/s)
  - Preset '1': Standard Speed (0.65 m/s, 3.00 rad/s)
  - Preset '2': Fast Speed     (1.00 m/s, 4.50 rad/s)
  - Preset '3': Turbo Speed    (1.60 m/s, 6.00 rad/s)
  - 'q' : Quit
"""

import sys
import time
import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def safe_addstr(stdscr, y, x, text, attr=0):
    """Safely write to curses window without throwing ERR if window is resized."""
    try:
        max_y, max_x = stdscr.getmaxyx()
        if y < 0 or y >= max_y or x < 0 or x >= max_x:
            return
        max_len = max_x - x - 1
        if max_len <= 0:
            return
        stdscr.addstr(y, x, str(text)[:max_len], attr)
    except curses.error:
        pass


class ArrowTeleopNode(Node):
    def __init__(self):
        super().__init__('arrow_teleop_node')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Balanced, smooth default speeds for controlled mapping
        self.linear_speed = 0.70   # m/s
        self.angular_speed = 1.40  # rad/s (Smooth, controllable differential turning)

        self.speed_step_linear = 0.10
        self.speed_step_angular = 0.20

        self.min_linear = 0.10
        self.max_linear = 2.50
        self.min_angular = 0.30
        self.max_angular = 5.00

        # Independent axis state for combined forward + turn steering
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_linear_time = 0.0
        self.last_angular_time = 0.0
        self.active_action = "IDLE (Stopped)"

        # Timeout for key release detection (tight for crisp stop without lingering turn)
        self.press_timeout = 0.20

    def set_linear(self, val):
        self.current_linear = float(val)
        self.last_linear_time = time.time()

    def set_angular(self, val):
        self.current_angular = float(val)
        self.last_angular_time = time.time()

    def set_action_combo(self, linear, angular):
        self.current_linear = float(linear)
        self.current_angular = float(angular)
        now = time.time()
        self.last_linear_time = now
        self.last_angular_time = now

    def stop_robot(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_linear_time = 0.0
        self.last_angular_time = 0.0
        self.active_action = "IDLE (Stopped)"
        self.publish_twist(0.0, 0.0)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)

    def update_cycle(self):
        now = time.time()
        # Timeout per axis
        if (now - self.last_linear_time) > self.press_timeout:
            self.current_linear = 0.0
        if (now - self.last_angular_time) > self.press_timeout:
            self.current_angular = 0.0

        # Determine human-readable status badge
        lin = self.current_linear
        ang = self.current_angular

        if lin > 0.01 and ang > 0.01:
            self.active_action = "▲◄ CURVE FWD-LEFT"
        elif lin > 0.01 and ang < -0.01:
            self.active_action = "▲► CURVE FWD-RIGHT"
        elif lin < -0.01 and ang > 0.01:
            self.active_action = "▼◄ CURVE BACK-LEFT"
        elif lin < -0.01 and ang < -0.01:
            self.active_action = "▼► CURVE BACK-RIGHT"
        elif lin > 0.01:
            self.active_action = "▲ FORWARD"
        elif lin < -0.01:
            self.active_action = "▼ BACKWARD"
        elif ang > 0.01:
            self.active_action = "◄ PIVOT LEFT (Diff)"
        elif ang < -0.01:
            self.active_action = "► PIVOT RIGHT (Diff)"
        else:
            self.active_action = "IDLE (Stopped)"

        # Continuously publish current velocities
        self.publish_twist(self.current_linear, self.current_angular)


def curses_main(stdscr):
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)  # Non-blocking getch
    try:
        curses.curs_set(0)  # Invisible cursor
    except curses.error:
        pass

    if curses.has_colors():
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_CYAN, -1)     # Header
            curses.init_pair(2, curses.COLOR_GREEN, -1)    # Forward
            curses.init_pair(3, curses.COLOR_YELLOW, -1)   # Turning
            curses.init_pair(4, curses.COLOR_RED, -1)      # Stopped / Reverse
            curses.init_pair(5, curses.COLOR_MAGENTA, -1)  # Speed info
        except curses.error:
            pass

    rclpy.init(args=None)
    node = ArrowTeleopNode()

    KEY_NUMPAD_UP = getattr(curses, 'KEY_A2', 450)
    KEY_NUMPAD_DOWN = getattr(curses, 'KEY_C2', 456)
    KEY_NUMPAD_LEFT = getattr(curses, 'KEY_B1', 452)
    KEY_NUMPAD_RIGHT = getattr(curses, 'KEY_B3', 454)

    running = True
    rate_hz = 30
    dt = 1.0 / rate_hz

    try:
        while running:
            loop_start = time.time()
            rclpy.spin_once(node, timeout_sec=0.0)

            # Read all pending key presses in queue
            key = -1
            while True:
                k = stdscr.getch()
                if k == -1:
                    break
                key = k

            if key != -1:
                # Up / Forward
                if key in (curses.KEY_UP, KEY_NUMPAD_UP, ord('w'), ord('W'), ord('i'), ord('I')):
                    node.set_linear(node.linear_speed)

                # Down / Backward
                elif key in (curses.KEY_DOWN, KEY_NUMPAD_DOWN, ord('s'), ord('S'), ord(',')):
                    node.set_linear(-node.linear_speed)

                # Left / Turn Left
                elif key in (curses.KEY_LEFT, KEY_NUMPAD_LEFT, ord('a'), ord('A'), ord('j'), ord('J')):
                    node.set_angular(node.angular_speed)

                # Right / Turn Right
                elif key in (curses.KEY_RIGHT, KEY_NUMPAD_RIGHT, ord('d'), ord('D'), ord('l'), ord('L')):
                    node.set_angular(-node.angular_speed)

                # Diagonal / Arc curves (curving with gentler angular speed for easy corridor steering)
                elif key in (ord('u'), ord('U')):
                    node.set_action_combo(node.linear_speed, round(node.angular_speed * 0.70, 2))
                elif key in (ord('o'), ord('O')):
                    node.set_action_combo(node.linear_speed, -round(node.angular_speed * 0.70, 2))
                elif key in (ord('m'), ord('M')):
                    node.set_action_combo(-node.linear_speed, round(node.angular_speed * 0.70, 2))
                elif key == ord('.'):
                    node.set_action_combo(-node.linear_speed, -round(node.angular_speed * 0.70, 2))

                # Stop
                elif key in (ord(' '), ord('k'), ord('K')):
                    node.stop_robot()

                # Speed Increase (+, =, ], e)
                elif key in (ord('+'), ord('='), ord(']'), ord('e'), ord('E'), 43, 61):
                    node.linear_speed = round(min(node.linear_speed + node.speed_step_linear, node.max_linear), 2)
                    node.angular_speed = round(min(node.angular_speed + node.speed_step_angular, node.max_angular), 2)

                # Speed Decrease (-, _, [)
                elif key in (ord('-'), ord('_'), ord('['), 45, 95):
                    node.linear_speed = round(max(node.linear_speed - node.speed_step_linear, node.min_linear), 2)
                    node.angular_speed = round(max(node.angular_speed - node.speed_step_angular, node.min_angular), 2)

                # Speed Presets: '1' Precision, '2' Standard Smooth, '3' Quick
                elif key == ord('1'):
                    node.linear_speed = 0.45
                    node.angular_speed = 0.90
                elif key == ord('2'):
                    node.linear_speed = 0.70
                    node.angular_speed = 1.40
                elif key == ord('3'):
                    node.linear_speed = 1.00
                    node.angular_speed = 2.20

                # Quit
                elif key in (ord('q'), ord('Q'), 27):
                    running = False

            # Update timer & dead-man's switch
            node.update_cycle()

            # Render UI
            stdscr.erase()
            color_hdr = curses.color_pair(1) if curses.has_colors() else curses.A_NORMAL
            color_fwd = curses.color_pair(2) if curses.has_colors() else curses.A_NORMAL
            color_turn = curses.color_pair(3) if curses.has_colors() else curses.A_NORMAL
            color_stop = curses.color_pair(4) if curses.has_colors() else curses.A_NORMAL
            color_spd = curses.color_pair(5) if curses.has_colors() else curses.A_NORMAL

            safe_addstr(stdscr, 0, 0, "╔═══════════════════════════════════════════════════════════╗", color_hdr)
            safe_addstr(stdscr, 1, 0, "║     🏎️  TORQ-R1 DIFFERENTIAL-DRIVE TELEOPERATION    ║", color_hdr | curses.A_BOLD)
            safe_addstr(stdscr, 2, 0, "╚═══════════════════════════════════════════════════════════╝", color_hdr)

            # Robot status
            status_color = color_stop
            if "FORWARD" in node.active_action or "FWD" in node.active_action:
                status_color = color_fwd
            elif "PIVOT" in node.active_action or "TURN" in node.active_action:
                status_color = color_turn
            elif "BACKWARD" in node.active_action or "BACK" in node.active_action:
                status_color = color_stop

            safe_addstr(stdscr, 4, 2, "Status: ", curses.A_BOLD)
            safe_addstr(stdscr, 4, 11, f"{node.active_action:<24}", status_color | curses.A_BOLD)

            # Velocity display
            safe_addstr(stdscr, 5, 2, f"Active Cmd : v = {node.current_linear:+.2f} m/s, w = {node.current_angular:+.2f} rad/s")
            safe_addstr(stdscr, 6, 2, f"Current Cap: v = {node.linear_speed:.2f} m/s | w = {node.angular_speed:.2f} rad/s", color_spd | curses.A_BOLD)

            # Controls guide
            safe_addstr(stdscr, 8, 0, "─────────────────────────────────────────────────────────────")
            safe_addstr(stdscr, 9, 2, "CONTROLS (Hold to move, release to idle):", curses.A_BOLD)
            safe_addstr(stdscr, 10, 4, "▲ / ▼ or W / S : Drive Forward / Backward")
            safe_addstr(stdscr, 11, 4, "◄ / ► or A / D : Smooth Pivot Left / Right in Place")
            safe_addstr(stdscr, 12, 4, "U / O          : Smooth Curve Forward-Left / Forward-Right")
            safe_addstr(stdscr, 13, 4, "M / .          : Smooth Curve Back-Left / Back-Right")
            safe_addstr(stdscr, 14, 4, "[SPACE] or K   : Immediate Emergency Stop")
            safe_addstr(stdscr, 15, 4, "+ / = or ] / e : Increase Speed (+0.10 m/s, +0.20 rad/s)")
            safe_addstr(stdscr, 16, 4, "- / _ or [     : Decrease Speed (-0.10 m/s, -0.20 rad/s)")
            safe_addstr(stdscr, 17, 4, "1 / 2 / 3      : Presets (1: Precision 0.9 rad/s | 2: Standard 1.4 rad/s | 3: Quick 2.2 rad/s)")
            safe_addstr(stdscr, 18, 4, "q              : Quit Teleop")
            safe_addstr(stdscr, 19, 0, "─────────────────────────────────────────────────────────────")

            try:
                stdscr.refresh()
            except curses.error:
                pass

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


def main():
    curses.wrapper(curses_main)


if __name__ == '__main__':
    main()
