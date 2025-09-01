# MASt3R-SLAM Docker Setup (Enhanced)

🚀 **Complete Docker solution** for real-time 3D SLAM with RealSense cameras and ROS integration

This repository provides an enhanced Docker setup for running MASt3R-SLAM with RealSense cameras, featuring real-time 3D reconstruction with stable GUI visualization, automatic .ply file generation, and ROS integration capabilities.

## 🚀 Quick Start (Enhanced Workflow)

### Prerequisites
- Docker with GPU support (`nvidia-docker2`)
- RealSense camera connected to your system
- X11 server for visualization (Linux)

### ✨ Key Enhancements

- 🔄 **Automatic file mounting** - No more manual copying required
- 🎮 **GUI crash fix** - Stable visualization with proper shader resources  
- ⚡ **Enhanced signal handling** - Ctrl+C properly terminates processes
- 💾 **Periodic saving** - Reconstructions saved every 30 seconds
- 🔍 **Smart container detection** - Enhanced script finds running containers automatically
- 📊 **Real-time monitoring** - Live camera and processing status
- 🤖 **ROS Integration Ready** - Architecture prepared for PointCloud2 streaming to Unity
- 📚 **Comprehensive Documentation** - Complete setup and troubleshooting guides

### Two-Step Enhanced Setup Process

#### Step 1: Build and Setup Container
```bash
cd docker
./run_mast3r.sh realsense
```

This enhanced command will:
- ✅ Build the Docker image with CUDA 12.4 support
- ✅ Install all dependencies (MASt3R, lietorch, in3d, PyTorch 2.5.1)
- ✅ **Auto-configure GUI resources** for stable visualization
- ✅ Setup RealSense camera drivers with proper USB access
- ✅ **Mount enhanced files automatically** (no manual copying)
- ✅ Keep the container running and ready with all enhancements

**Note:** This enhanced process takes 5-10 minutes and only needs to be done once per container.

#### Step 2: Run Enhanced MASt3R-SLAM (New Terminal)
Open a new terminal and run:
```bash
cd docker
./run_python_enhanced.sh --dataset realsense
```

This will start MASt3R-SLAM with:
- 🎯 **Real-time 3D GUI visualization** 
- 📹 **Live RealSense camera feed**
- 💾 **Automatic .ply saving every 30 seconds**
- 🛑 **Graceful Ctrl+C shutdown**

## 🎛️ Enhanced Features

### Automatic Periodic Saving
- `.ply` files saved every 30 seconds to `logs/realsense_live/`
- Partial saves: `realsense_live_partial_HHMMSS.ply`
- Final save: `realsense_live.ply` (when exiting with Ctrl+C)

### Usage Options
```bash
# Run with GUI visualization (default)
./run_python_enhanced.sh --dataset realsense

# Run in headless mode (no GUI, higher FPS)
./run_python_enhanced.sh --no-viz --dataset realsense

# Use custom config
./run_python_enhanced.sh --dataset realsense --config config/calib.yaml
```

### Graceful Exit
- Press **Ctrl+C** to save final reconstruction and exit cleanly
- No more orphaned processes or continuous saving after exit
- Enhanced signal handling ensures proper cleanup

## 📊 Performance Metrics

- **With GUI**: ~4-5 FPS (includes 3D visualization)
- **Headless**: ~5-6 FPS (maximum processing speed)
- **GPU Utilization**: ~90-95% (CUDA acceleration)
- **Memory Usage**: ~8-10GB GPU memory

## 🔧 Two-Terminal Workflow

Since the container setup keeps running, you'll need two terminals:

**Terminal 1** (Container Setup):
```bash
cd docker
./run_mast3r.sh realsense
# This terminal stays occupied with container management
```

**Terminal 2** (Running SLAM):
```bash
cd docker
./run_python_enhanced.sh --dataset realsense
# Use this terminal for running MASt3R-SLAM
```

## 📁 Output Files

All reconstruction files are automatically saved to:
```
MASt3R-SLAM/logs/realsense_live/
├── realsense_live_partial_103045.ply    # Saved every 30s
├── realsense_live_partial_103115.ply    # Timestamped saves
├── realsense_live_partial_103145.ply
└── realsense_live.ply                   # Final save (Ctrl+C)
```

## 🤖 ROS Integration Architecture

The enhanced MASt3R-SLAM system is designed with ROS integration capabilities for real-time streaming to Unity and other robotics applications.

### ROS PointCloud2 Publisher
```python
# ROS publisher for real-time point cloud streaming
class MASt3RROSPublisher:
    def __init__(self):
        rospy.init_node('mast3r_slam_publisher')
        self.pub = rospy.Publisher('/mast3r/pointcloud', PointCloud2, queue_size=1)
    
    def publish_reconstruction(self, points, colors):
        # Convert MASt3R output to ROS PointCloud2 message
        pointcloud_msg = self.create_pointcloud2_msg(points, colors)
        self.pub.publish(pointcloud_msg)
```

### Unity Integration Workflow
1. **MASt3R-SLAM** → generates 3D reconstruction
2. **ROS Publisher** → streams PointCloud2 messages
3. **Unity ROS Subscriber** → receives real-time 3D data
4. **Unity Visualization** → renders live 3D reconstruction

### Configuration
```yaml
# config/ros_integration.yaml
ros:
  enable_publisher: true
  topic_name: "/mast3r/pointcloud" 
  publish_rate: 10  # Hz
  frame_id: "camera_link"
  point_cloud_quality: "high"  # high/medium/low
```

## 🎯 Alternative Datasets

While optimized for RealSense, you can also use pre-recorded datasets:

```bash
# TUM RGB-D dataset
./run_mast3r.sh datasets/tum/rgbd_dataset_freiburg1_desk
./run_python_enhanced.sh --dataset datasets/tum/rgbd_dataset_freiburg1_desk

# EuRoC dataset  
./run_mast3r.sh datasets/euroc/MH_01_easy
./run_python_enhanced.sh --dataset datasets/euroc/MH_01_easy
```

## 🔧 Troubleshooting

**Container name conflicts:**
```bash
# Stop any existing containers
docker compose down
docker stop $(docker ps -q --filter "name=docker-mast3r-run")
```

**RealSense not detected:**
```bash
# Check camera connection
lsusb | grep Intel
# Should show: Intel Corp. RealSense Camera
```

**GPU not available:**
```bash
# Test NVIDIA Docker support
docker run --rm --gpus all nvidia/cuda:12.4-base-ubuntu22.04 nvidia-smi
```

**GUI not working:**
```bash
# Fix X11 permissions
xhost +local:root
```

**Enhanced script not finding container:**
```bash
# Check running containers
docker ps
# Look for container name starting with "docker-mast3r-run"
```

## 💡 Pro Tips

1. **First Time Setup**: Run `./run_mast3r.sh realsense` and wait for completion message before using enhanced script
2. **Daily Usage**: After initial setup, just use `./run_python_enhanced.sh --dataset realsense`
3. **Performance**: Use `--no-viz` flag for maximum processing speed when GUI not needed
4. **Debugging**: Check container logs with `docker logs CONTAINER_NAME`
5. **Cleanup**: Use `docker compose down` to stop all containers when done

## 📋 What's Included

- ✅ **CUDA 12.4** with PyTorch 2.5.1 GPU acceleration
- ✅ **RealSense SDK** with D435/D455 support
- ✅ **Enhanced MASt3R-SLAM** with periodic saving
- ✅ **GUI visualization** with moderngl and imgui
- ✅ **Automatic dependency management** (no manual installations)
- ✅ **Signal handling** for clean shutdown
- ✅ **Volume mounting** for automatic file sync

## 🚀 Next Steps

Once you have MASt3R-SLAM running, you can:
- Monitor real-time reconstruction quality via GUI
- Analyze saved .ply files in external tools (CloudCompare, MeshLab)
- Integrate with ROS for real-time streaming to other applications
- Experiment with different camera configurations and environments

The enhanced Docker setup provides a robust, production-ready environment for 3D SLAM