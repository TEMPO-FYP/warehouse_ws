#!/bin/bash
# Spawn a robot SDF into a running Gazebo Sim world.
#
# 1. Waits for the world's /create service (server ready), with a timeout.
# 2. Waits a fixed GUI settle time. The GUI loads the large warehouse world
#    long after the server is up, and an entity created before it finishes
#    exists on the server but never shows up in the GUI. No reliable
#    "GUI ready" signal was found, so this stays a delay. Set it to 0 for
#    headless runs.
# 3. Spawns, and exits non-zero on failure so the launch file can react.
#
# Usage: spawn_robot.sh [SDF_FILE] [ROBOT_NAME] [WORLD_NAME] [TIMEOUT_S] [GUI_SETTLE_S]

SDF_FILE="${1:-/tmp/robot_abs.sdf}"
ROBOT_NAME="${2:-a200_0001}"
WORLD_NAME="${3:-default}"
TIMEOUT_S="${4:-120}"
GUI_SETTLE_S="${5:-30}"

CREATE_SERVICE="/world/${WORLD_NAME}/create"

if [ ! -s "$SDF_FILE" ]; then
    echo "[spawn_robot] ERROR: SDF file '$SDF_FILE' is missing or empty."
    exit 1
fi

echo "[spawn_robot] Waiting for $CREATE_SERVICE (timeout ${TIMEOUT_S}s)..."
start=$(date +%s)
until gz service -l 2>/dev/null | grep -qx "$CREATE_SERVICE"; do
    if (( $(date +%s) - start >= TIMEOUT_S )); then
        echo "[spawn_robot] ERROR: $CREATE_SERVICE not available after ${TIMEOUT_S}s."
        exit 1
    fi
    sleep 1
done
echo "[spawn_robot] $CREATE_SERVICE is up after $(( $(date +%s) - start ))s."

if (( GUI_SETTLE_S > 0 )); then
    echo "[spawn_robot] Waiting ${GUI_SETTLE_S}s for the GUI to finish loading the world..."
    sleep "$GUI_SETTLE_S"
fi

echo "[spawn_robot] Spawning $ROBOT_NAME..."
response=$(gz service -s "$CREATE_SERVICE" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 10000 \
    --req "sdf_filename: \"$SDF_FILE\", name: \"$ROBOT_NAME\", pose: {position: {x: 0.0, y: -5.0, z: 0.15}}")

if echo "$response" | grep -q "data: true"; then
    echo "[spawn_robot] $ROBOT_NAME spawned."
    exit 0
fi

echo "[spawn_robot] ERROR: spawn request failed. Response: ${response:-<none>}"
exit 1
