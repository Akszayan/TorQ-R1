#!/usr/bin/env bash
# ==============================================================================
# Helper Script: Stop Mapping & Save Final TorQ-R1 Map
# Usage: ./scripts/stop_mapping.sh [map_name]
# ==============================================================================
set -e

WS_DIR="/home/akszayan/TorQ-R1"
source "${WS_DIR}/scripts/setup_workspace.sh"

MAP_NAME="${1:-final_torq_r1_map}"

# First save the map if nodes are running
if pgrep -f "slam_toolbox" > /dev/null; then
    "${WS_DIR}/scripts/save_map.sh" "${MAP_NAME}" || true
fi

echo "Stopping all simulation and mapping processes..."
pkill -9 -f "gz sim|parameter_bridge|slam_toolbox|rviz2|arrow_teleop|path_publisher" 2>/dev/null || true
sleep 1

echo "==================================================================="
echo "  🏁 All TorQ-R1 simulation and mapping processes stopped cleanly."
echo "==================================================================="
