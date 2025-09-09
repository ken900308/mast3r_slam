#!/usr/bin/env python3
"""
RealSense D435 Camera Intrinsics Fetcher
========================================

This script connects to a RealSense D435 camera and extracts its intrinsic
calibration parameters for both color and depth cameras.

Usage:
    python fetch_d435_intrinsics.py [--output intrinsics.yaml]

Output:
    - Prints intrinsics to console
    - Optionally saves to YAML file compatible with MASt3R-SLAM
"""

import argparse
import sys
import yaml
from pathlib import Path

try:
    import pyrealsense2 as rs
except ImportError:
    print("❌ Error: pyrealsense2 not installed")
    print("Install with: pip install pyrealsense2")
    sys.exit(1)


def get_camera_intrinsics():
    """
    Connect to RealSense camera and extract intrinsic parameters
    
    Returns:
        dict: Dictionary containing color and depth intrinsics
    """
    print("🔍 Searching for RealSense D435 camera...")
    
    # Create RealSense context
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("❌ No RealSense devices found!")
        print("💡 Make sure your D435 is connected via USB 3.0")
        return None
    
    # Find D435 device
    d435_device = None
    for device in devices:
        name = device.get_info(rs.camera_info.name)
        if "D435" in name or "435" in name:
            d435_device = device
            break
    
    if not d435_device:
        print(f"❌ D435 not found. Available devices:")
        for device in devices:
            print(f"  - {device.get_info(rs.camera_info.name)}")
        return None
    
    print(f"✅ Found: {d435_device.get_info(rs.camera_info.name)}")
    print(f"   Serial: {d435_device.get_info(rs.camera_info.serial_number)}")
    print(f"   Firmware: {d435_device.get_info(rs.camera_info.firmware_version)}")
    
    # Configure pipeline for intrinsics extraction
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable common resolutions for D435
    color_width, color_height = 640, 480
    depth_width, depth_height = 640, 480
    
    config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, 30)
    
    print("🔄 Starting camera pipeline to extract intrinsics...")
    
    try:
        # Start pipeline
        profile = pipeline.start(config)
        
        # Get stream profiles
        color_stream = profile.get_stream(rs.stream.color)
        depth_stream = profile.get_stream(rs.stream.depth)
        
        # Extract intrinsics
        color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        
        # Get extrinsics (transformation between color and depth)
        depth_to_color = depth_stream.get_extrinsics_to(color_stream)
        
        # Stop pipeline
        pipeline.stop()
        
        print("✅ Successfully extracted camera intrinsics!")
        
        # Format intrinsics data
        intrinsics_data = {
            'camera_info': {
                'device_name': d435_device.get_info(rs.camera_info.name),
                'serial_number': d435_device.get_info(rs.camera_info.serial_number),
                'firmware_version': d435_device.get_info(rs.camera_info.firmware_version)
            },
            'color_camera': {
                'resolution': {
                    'width': color_intrinsics.width,
                    'height': color_intrinsics.height
                },
                'intrinsics': {
                    'fx': color_intrinsics.fx,
                    'fy': color_intrinsics.fy,
                    'cx': color_intrinsics.ppx,
                    'cy': color_intrinsics.ppy
                },
                'distortion': {
                    'model': str(color_intrinsics.model).split('.')[-1],
                    'coefficients': list(color_intrinsics.coeffs)
                }
            },
            'depth_camera': {
                'resolution': {
                    'width': depth_intrinsics.width,
                    'height': depth_intrinsics.height
                },
                'intrinsics': {
                    'fx': depth_intrinsics.fx,
                    'fy': depth_intrinsics.fy,
                    'cx': depth_intrinsics.ppx,
                    'cy': depth_intrinsics.ppy
                },
                'distortion': {
                    'model': str(depth_intrinsics.model).split('.')[-1],
                    'coefficients': list(depth_intrinsics.coeffs)
                }
            },
            'extrinsics': {
                'depth_to_color': {
                    'rotation': [list(depth_to_color.rotation[i:i+3]) for i in range(0, 9, 3)],
                    'translation': list(depth_to_color.translation)
                }
            }
        }
        
        return intrinsics_data
        
    except Exception as e:
        print(f"❌ Error extracting intrinsics: {e}")
        pipeline.stop()
        return None


def print_intrinsics(data):
    """Print intrinsics in a readable format"""
    print("\n" + "="*60)
    print("📷 REALSENSE D435 CAMERA INTRINSICS")
    print("="*60)
    
    # Device info
    info = data['camera_info']
    print(f"Device: {info['device_name']}")
    print(f"Serial: {info['serial_number']}")
    print(f"Firmware: {info['firmware_version']}")
    
    # Color camera
    color = data['color_camera']
    print(f"\n🎨 COLOR CAMERA ({color['resolution']['width']}x{color['resolution']['height']}):")
    print(f"  fx: {color['intrinsics']['fx']:.2f}")
    print(f"  fy: {color['intrinsics']['fy']:.2f}")
    print(f"  cx: {color['intrinsics']['cx']:.2f}")
    print(f"  cy: {color['intrinsics']['cy']:.2f}")
    print(f"  Distortion Model: {color['distortion']['model']}")
    print(f"  Distortion Coeffs: {[f'{c:.6f}' for c in color['distortion']['coefficients']]}")
    
    # Depth camera
    depth = data['depth_camera']
    print(f"\n📏 DEPTH CAMERA ({depth['resolution']['width']}x{depth['resolution']['height']}):")
    print(f"  fx: {depth['intrinsics']['fx']:.2f}")
    print(f"  fy: {depth['intrinsics']['fy']:.2f}")
    print(f"  cx: {depth['intrinsics']['cx']:.2f}")
    print(f"  cy: {depth['intrinsics']['cy']:.2f}")
    print(f"  Distortion Model: {depth['distortion']['model']}")
    print(f"  Distortion Coeffs: {[f'{c:.6f}' for c in depth['distortion']['coefficients']]}")
    
    # Extrinsics
    extrinsics = data['extrinsics']['depth_to_color']
    print(f"\n🔄 DEPTH TO COLOR EXTRINSICS:")
    print(f"  Rotation Matrix:")
    for row in extrinsics['rotation']:
        print(f"    [{row[0]:8.6f}, {row[1]:8.6f}, {row[2]:8.6f}]")
    print(f"  Translation Vector: [{extrinsics['translation'][0]:.6f}, {extrinsics['translation'][1]:.6f}, {extrinsics['translation'][2]:.6f}]")
    
    print("="*60)


def save_mast3r_format(data, output_file):
    """Save intrinsics in MASt3R-SLAM compatible format"""
    
    color = data['color_camera']
    depth = data['depth_camera']
    
    # MASt3R-SLAM format
    mast3r_format = {
        'camera_model': 'PINHOLE',
        'color_intrinsics': {
            'width': color['resolution']['width'],
            'height': color['resolution']['height'],
            'fx': float(color['intrinsics']['fx']),
            'fy': float(color['intrinsics']['fy']),
            'cx': float(color['intrinsics']['cx']),
            'cy': float(color['intrinsics']['cy']),
            'k1': float(color['distortion']['coefficients'][0]),
            'k2': float(color['distortion']['coefficients'][1]),
            'p1': float(color['distortion']['coefficients'][2]),
            'p2': float(color['distortion']['coefficients'][3]),
            'k3': float(color['distortion']['coefficients'][4])
        },
        'depth_intrinsics': {
            'width': depth['resolution']['width'],
            'height': depth['resolution']['height'],
            'fx': float(depth['intrinsics']['fx']),
            'fy': float(depth['intrinsics']['fy']),
            'cx': float(depth['intrinsics']['cx']),
            'cy': float(depth['intrinsics']['cy']),
            'k1': float(depth['distortion']['coefficients'][0]),
            'k2': float(depth['distortion']['coefficients'][1]),
            'p1': float(depth['distortion']['coefficients'][2]),
            'p2': float(depth['distortion']['coefficients'][3]),
            'k3': float(depth['distortion']['coefficients'][4])
        },
        'depth_scale': 0.001,  # D435 depth scale (1mm per unit)
        'device_info': data['camera_info']
    }
    
    # Save to file
    with open(output_file, 'w') as f:
        yaml.dump(mast3r_format, f, default_flow_style=False, indent=2)
    
    print(f"💾 Saved MASt3R-SLAM compatible intrinsics to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Fetch RealSense D435 camera intrinsics')
    parser.add_argument('--output', '-o', default='d435_intrinsics.yaml',
                       help='Output YAML file (default: d435_intrinsics.yaml)')
    parser.add_argument('--format', choices=['mast3r', 'full'], default='mast3r',
                       help='Output format: mast3r (MASt3R-SLAM compatible) or full (complete data)')
    
    args = parser.parse_args()
    
    # Extract intrinsics
    intrinsics = get_camera_intrinsics()
    
    if not intrinsics:
        print("❌ Failed to extract camera intrinsics")
        sys.exit(1)
    
    # Print to console
    print_intrinsics(intrinsics)
    
    # Save to file
    output_path = Path(args.output)
    
    if args.format == 'mast3r':
        save_mast3r_format(intrinsics, output_path)
    else:
        with open(output_path, 'w') as f:
            yaml.dump(intrinsics, f, default_flow_style=False, indent=2)
        print(f"💾 Saved full intrinsics data to: {output_path}")
    
    print(f"\n✅ Camera intrinsics extraction complete!")
    print(f"💡 You can now use {output_path} with MASt3R-SLAM")


if __name__ == "__main__":
    main()
