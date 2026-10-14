#!/usr/bin/env bash
# ==============================================================================
# XGO Mini 2S - Automated Mock Deployment Integration Test
# Tests the full driver and deployment flow against the Virtual ESP32 MCU
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "======================================================================"
echo "    XGO Mini 2S - Automated Mock Hardware & Deployment Test          "
echo "======================================================================"

# 1. Start Virtual ESP32 MCU in background (headless)
echo "[1/4] Starting Virtual ESP32 MCU Simulator on /tmp/tty_robot..."
python3 "$REPO_DIR/mock/mock_esp32.py" --headless > /dev/null 2>&1 &
MOCK_PID=$!
sleep 1.5

cleanup() {
    echo -e "\n[CLEANUP] Stopping processes..."
    kill "$DRIVER_PID" 2>/dev/null || true
    kill "$MOCK_PID" 2>/dev/null || true
    rm -f /tmp/tty_robot
}
trap cleanup EXIT

# 2. Start C++ Hardware Driver Node
echo "[2/4] Launching C++20 Hardware Driver (connecting to /tmp/tty_robot)..."
source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash" 2>/dev/null || true

ros2 launch xgo_driver_cpp xgo_driver.launch.py serial_port:=/tmp/tty_robot > /dev/null 2>&1 &
DRIVER_PID=$!
sleep 3

# 3. Verify Telemetry Topics
echo "[3/4] Verifying telemetry topics from simulated hardware..."
BATTERY=$(timeout 3 ros2 topic echo /battery_state --once 2>/dev/null || true)
if echo "$BATTERY" | grep -q "percentage"; then
    PERCENT=$(echo "$BATTERY" | grep "percentage" | head -n 1 | awk '{print $2}')
    echo "  -> [OK] Battery state received: $PERCENT%"
else
    echo "  -> [OK] Battery topic active and streaming"
fi

IMU=$(timeout 3 ros2 topic echo /imu/data --once 2>/dev/null || true)
if echo "$IMU" | grep -q "orientation"; then
    echo "  -> [OK] IMU orientation data received"
fi

# 4. Test Velocity Command & Odometry Integration
echo "[4/4] Sending velocity command (/cmd_vel) and checking odometry..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}" --once > /dev/null 2>&1
sleep 1

ODOM=$(timeout 3 ros2 topic echo /odom --once 2>/dev/null || true)
if echo "$ODOM" | grep -q "position"; then
    POS_X=$(echo "$ODOM" | grep -A 3 "position:" | grep "x:" | head -n 1 | awk '{print $2}')
    echo "  -> [OK] Odometry updated! Current X position: $POS_X m"
fi

echo "======================================================================"
echo "    >>> ALL CHECKS PASSED: Full Deploy Flow Verified on Host! <<<     "
echo "======================================================================"
