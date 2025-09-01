# MASt3R-SLAM Docker Setup

This repository provides a complete Docker setup for running MASt3R-SLAM with RealSense cameras.

## 🚀 Quick Start

### Prerequisites
- Docker with GPU support (`nvidia-docker2`)
- RealSense camera connected to your system
- X11 server for visualization (Linux) or XServer (Windows/Mac)

### One-Command Setup & Run

```bash
cd docker
./run_mast3r.sh
```

That's it! The script will:
1. Build the Docker image with all dependencies
2. Install MASt3R, lietorch, and all required packages
3. Start MASt3R-SLAM with your RealSense camera
4. Handle GPU acceleration automatically

### Custom Usage

```bash
# Run with TUM dataset instead of RealSense
./run_mast3r.sh datasets/tum/rgbd_dataset_freiburg1_desk

# Run with custom config
./run_mast3r.sh datasets/tum/rgbd_dataset_freiburg1_desk config/calib.yaml

# Run with additional arguments (like --no-viz)
./run_mast3r.sh datasets/tum/rgbd_dataset_freiburg1_desk config/base.yaml "--no-viz"
```

## 🎯 For Development (Persistent Container)

If you're developing and want to avoid reinstalling packages every time:

```bash
# Start persistent container
docker compose up -d

# Run in persistent container (packages stay installed)
./run_realsense.sh
```

## 📋 What's Included

- ✅ CUDA 12.4 support
- ✅ PyTorch 2.5.1 with GPU acceleration
- ✅ RealSense SDK and drivers
- ✅ All MASt3R dependencies (lietorch, mast3r, in3d)
- ✅ GUI support with X11 forwarding
- ✅ Automatic device mounting for cameras

## 🔧 Troubleshooting

**GPU not detected:**
```bash
# Check if nvidia-docker2 is installed
docker run --rm --gpus all nvidia/cuda:12.4-base-ubuntu22.04 nvidia-smi
```

**RealSense not detected:**
```bash
# Check if camera is connected
lsusb | grep Intel
```

**Permission issues with X11:**
```bash
xhost +local:root
```

## 📁 Directory Structure

```
docker/
├── dockerfile              # Main Docker image
├── docker-compose.yml      # Container orchestration
├── run_mast3r.sh           # One-command setup (for sharing)
└── run_realsense.sh        # Quick run (for development)
```
