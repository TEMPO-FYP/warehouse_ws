#!/bin/bash
# Wait until a Gazebo Sim world is ready for a robot to be spawned into it.
#
# Same idea as spawn_robot.sh on the transport-robot branch, minus the spawn
# itself (Clearpath's robot_spawn.launch.py does that here):
# 1. Waits for the world's /create service (server ready), with a timeout.
# 2. Waits a fixed GUI settle time. The GUI loads the large warehouse world
#    long after the server is up, and an entity created before it finishes
#    exists on the server but never shows up in the GUI. No reliable
#    "GUI ready" signal was found, so this stays a delay. Set it to 0 for
#    headless runs.
# Exits non-zero on timeout so the launch file can skip the spawn.
#
# Usage: wait_for_gazebo.sh [WORLD_NAME] [TIMEOUT_S] [GUI_SETTLE_S]

WORLD_NAME="${1:-default}"
TIMEOUT_S="${2:-120}"
GUI_SETTLE_S="${3:-30}"

CREATE_SERVICE="/world/${WORLD_NAME}/create"

echo "[wait_for_gazebo] Waiting for $CREATE_SERVICE (timeout ${TIMEOUT_S}s)..."
start=$(date +%s)
until gz service -l 2>/dev/null | grep -qx "$CREATE_SERVICE"; do
    if (( $(date +%s) - start >= TIMEOUT_S )); then
        echo "[wait_for_gazebo] ERROR: $CREATE_SERVICE not available after ${TIMEOUT_S}s."
        exit 1
    fi
    sleep 1
done
echo "[wait_for_gazebo] $CREATE_SERVICE is up after $(( $(date +%s) - start ))s."

# Integer seconds for the comparison (the launch argument may be "30.0")
GUI_SETTLE_S="${GUI_SETTLE_S%%.*}"
if (( ${GUI_SETTLE_S:-0} > 0 )); then
    echo "[wait_for_gazebo] Waiting ${GUI_SETTLE_S}s for the GUI to finish loading the world..."
    sleep "$GUI_SETTLE_S"
fi
echo "[wait_for_gazebo] Ready to spawn."
exit 0
