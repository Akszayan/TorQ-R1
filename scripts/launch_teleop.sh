#!/usr/bin/env bash
# ==============================================================================
# Dedicated Launcher: TorQ-R1 Smooth Arrow Teleoperation Node
# ==============================================================================

WS_DIR="/home/akszayan/TorQ-R1"
source "${WS_DIR}/scripts/setup_workspace.sh"

ros2 run torq_r1_navigation arrow_teleop_node
