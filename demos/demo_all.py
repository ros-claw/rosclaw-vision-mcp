#!/usr/bin/env python3
"""
ROSClaw Vision MCP Demo Script
演示多相机、目标检测、立体视觉功能
"""

import subprocess
import json
import time
import sys


def mcp_call(tool: str, args: dict = None) -> dict:
    """调用 MCP 工具"""
    cmd = ["mcporter", "call", f"rosclaw-vision.{tool}"]
    if args:
        for k, v in args.items():
            if isinstance(v, bool):
                if v:
                    cmd.append(f"{k}")
            else:
                cmd.append(f"{k}={v}")
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.stdout.strip()


def demo_auto_discovery():
    """Demo 1: 自动发现相机"""
    print("=" * 60)
    print("🎯 Demo 1: 自动发现相机")
    print("=" * 60)
    
    result = mcp_call("discover_cameras")
    print(result)
    print()


def demo_connect_multi():
    """Demo 2: 连接多相机"""
    print("=" * 60)
    print("🎯 Demo 2: 连接所有相机")
    print("=" * 60)
    
    result = mcp_call("connect_multi_camera")
    print(result)
    print()


def demo_camera_status():
    """Demo 3: 查看相机状态"""
    print("=" * 60)
    print("🎯 Demo 3: 相机状态")
    print("=" * 60)
    
    result = mcp_call("get_camera_status")
    print(result)
    print()


def demo_capture():
    """Demo 4: 捕获图像"""
    print("=" * 60)
    print("🎯 Demo 4: 捕获图像")
    print("=" * 60)
    
    # Get list of cameras first
    cameras = ["camera"]  # default
    
    for cam in cameras:
        print(f"\n📷 Capturing from {cam}...")
        result = mcp_call("capture_from_camera", {
            "camera_id": cam,
            "quality": 85
        })
        
        if result.startswith("data:image"):
            print(f"✓ Captured image (Base64 length: {len(result)} chars)")
            # Save preview info
            print(f"  Preview: {result[:100]}...")
        else:
            print(f"✗ {result}")
    print()


def demo_object_detection():
    """Demo 5: 目标检测"""
    print("=" * 60)
    print("🎯 Demo 5: YOLO 目标检测")
    print("=" * 60)
    
    result = mcp_call("detect_objects", {
        "camera_id": "camera",
        "confidence": 0.5
    })
    print(result)
    print()


def demo_stereo():
    """Demo 6: 立体视觉（如果有两个相机）"""
    print("=" * 60)
    print("🎯 Demo 6: 立体视觉捕获")
    print("=" * 60)
    
    result = mcp_call("capture_stereo_image", {
        "left_camera": "camera",
        "right_camera": "camera_2",
        "quality": 85
    })
    
    try:
        data = json.loads(result)
        if "left" in data and "right" in data:
            print("✓ Captured stereo pair")
            print(f"  Left:  {len(data['left'])} chars")
            print(f"  Right: {len(data['right'])} chars")
        else:
            print(result)
    except:
        print(result)
    print()


def main():
    print("""
╔════════════════════════════════════════════════════════════════╗
║           ROSClaw Vision MCP - Demo Suite                      ║
╚════════════════════════════════════════════════════════════════╝
    """)
    
    print("Make sure:")
    print("  1. RealSense cameras are connected")
    print("  2. realsense2_camera is running")
    print("  3. MCP server is running (SSE mode)")
    print()
    
    # Run demos
    demo_auto_discovery()
    time.sleep(1)
    
    demo_connect_multi()
    time.sleep(2)
    
    demo_camera_status()
    time.sleep(1)
    
    demo_capture()
    time.sleep(1)
    
    demo_object_detection()
    time.sleep(1)
    
    demo_stereo()
    
    print("=" * 60)
    print("✅ All demos completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
