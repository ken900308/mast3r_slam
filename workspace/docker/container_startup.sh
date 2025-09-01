#!/bin/bash

# Enhanced MASt3R-SLAM Container Startup Script
# This script provides easy access to all enhanced functionality from within the container

echo "=== Enhanced MASt3R-SLAM Container ==="
echo "Available commands:"
echo "  run_basic     - Run basic MASt3R-SLAM with RealSense"
echo "  run_enhanced  - Run with periodic saving (every 5 seconds)"
echo "  run_save      - Run with forced saving"
echo "  run_noviz     - Run without visualization"
echo ""
echo "Enhanced features:"
echo "  ✓ Periodic .ply file saving"
echo "  ✓ Graceful Ctrl+C handling"
echo "  ✓ RealSense camera integration"
echo "  ✓ GPU acceleration"
echo "  ✓ Fixed shader files"
echo ""

# Function to run basic MASt3R-SLAM
run_basic() {
    echo "Running basic MASt3R-SLAM..."
    cd /workspace/MASt3R-SLAM
    export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"
    python main.py --dataset realsense --config config/base.yaml
}

# Function to run enhanced MASt3R-SLAM with periodic saving
run_enhanced() {
    echo "Running enhanced MASt3R-SLAM with periodic saving..."
    echo "Press Ctrl+C to save and exit gracefully"
    cd /workspace/MASt3R-SLAM
    export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"
    python main.py --dataset realsense --config config/base.yaml
}

# Function to run without visualization
run_noviz() {
    echo "Running MASt3R-SLAM without visualization..."
    cd /workspace/MASt3R-SLAM
    export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"
    python main.py --dataset realsense --config config/base.yaml --no-viz
}

# Function to check RealSense camera
check_camera() {
    echo "Checking RealSense camera..."
    python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
print(f'Found {len(devices)} RealSense device(s):')
for i, dev in enumerate(devices):
    print(f'  Device {i}: {dev.get_info(rs.camera_info.name)} - Serial: {dev.get_info(rs.camera_info.serial_number)}')
if len(devices) == 0:
    print('WARNING: No RealSense devices detected!')
"
}

# Function to list saved files
list_saves() {
    echo "Saved .ply files:"
    ls -la /workspace/MASt3R-SLAM/logs/realsense_live* 2>/dev/null || echo "No .ply files found yet"
}

# Handle command line arguments
case "$1" in
    "run_basic")
        run_basic
        ;;
    "run_enhanced")
        run_enhanced
        ;;
    "run_noviz")
        run_noviz
        ;;
    "check_camera")
        check_camera
        ;;
    "list_saves")
        list_saves
        ;;
    *)
        echo "Usage: $0 {run_basic|run_enhanced|run_noviz|check_camera|list_saves}"
        echo ""
        echo "Or run interactively and use the functions directly:"
        echo "  check_camera"
        echo "  run_enhanced"
        echo "  list_saves"
        ;;
esac
