#!/bin/bash
# Launch multiple RealSense cameras with unique namespaces

set -e

# Check for cameras
CAMERA_COUNT=$(lsusb | grep -c "Intel.*RealSense" || echo "0")

echo "=== Multi-Camera Launcher ==="
echo "Detected $CAMERA_COUNT RealSense camera(s)"
echo

if [ "$CAMERA_COUNT" -eq "0" ]; then
    echo "Error: No RealSense cameras detected"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Launch first camera on default namespace
echo "🎥 Starting Camera 1 (default namespace)..."
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    \&
CAM1_PID=$!

# If second camera exists, launch with different namespace
if [ "$CAMERA_COUNT" -ge "2" ]; then
    echo "🎥 Starting Camera 2 (namespace: camera_2)..."
    sleep 2
    ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera_2 \
        camera_namespace:=camera_2 \
        align_depth.enable:=true \
        serial_no:=$(lsusb -v 2>/dev/null | grep -A5 "RealSense" | grep "iSerial" | sed -n '2p' | awk '{print $2}') \
        \&
    CAM2_PID=$!
fi

echo
echo "Cameras launched!"
echo "  Camera 1: /camera/camera/*"
if [ "$CAMERA_COUNT" -ge "2" ]; then
    echo "  Camera 2: /camera_2/camera/*"
fi
echo
echo "Press Ctrl+C to stop"
echo

# Wait for interrupt
trap "kill $CAM1_PID $CAM2_PID 2>/dev/null; exit" INT
wait
