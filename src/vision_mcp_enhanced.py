#!/usr/bin/env python3
"""
ROSClaw Vision MCP Server - Enhanced Edition
支持多相机、自动话题发现、YOLO 目标检测
"""

import asyncio
import base64
import io
import json
import os
import re
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from pathlib import Path

try:
    import numpy as np
    from PIL import Image as PILImage
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False
    print("Warning: PIL not available, image encoding disabled")

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False
    print("Warning: ROS2 not available, running in mock mode")

# Optional: YOLO for object detection
try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("rosclaw-vision-enhanced")


# ============ Auto Topic Discovery ============

class TopicDiscovery:
    """自动发现 ROS2 相机话题"""
    
    COMMON_PATTERNS = [
        # Standard RealSense patterns
        r"/camera/color/image_raw",
        r"/camera/camera/color/image_raw",
        r"/camera_(\w+)/color/image_raw",
        r"/(\w+)/camera/color/image_raw",
        # Generic camera patterns
        r"/(\w+)/image_raw",
        r"/camera(\d+)/image_raw",
    ]
    
    @classmethod
    def discover_cameras(cls, timeout: float = 2.0) -> List[Dict[str, str]]:
        """自动发现所有相机话题"""
        cameras = []
        
        try:
            # Use ros2 topic list (不依赖 _HAS_ROS2，直接尝试)
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            if result.returncode != 0:
                print(f"ros2 topic list failed: {result.stderr}")
                return cameras
            
            topics = result.stdout.strip().split("\n")
            
            # Find color image topics (must end with /color/image_raw)
            color_topics = []
            for t in topics:
                # Must be a color image topic, not depth
                if t.endswith("/color/image_raw"):
                    color_topics.append(t)
            
            # Remove duplicates and sort
            color_topics = sorted(set(color_topics))
            
            for color_topic in color_topics:
                # Extract base namespace (everything before /color/image_raw)
                base = color_topic[:-len("/color/image_raw")]
                
                # Camera ID is the last part of the namespace
                if base:
                    camera_id = base.strip("/").replace("/", "_")
                else:
                    camera_id = "camera"
                
                # Derive depth and info topics
                depth_topic = f"{base}/aligned_depth_to_color/image_raw"
                if depth_topic not in topics:
                    # Try non-aligned depth
                    depth_topic = f"{base}/depth/image_rect_raw"
                    if depth_topic not in topics:
                        depth_topic = None
                
                info_topic = f"{base}/color/camera_info"
                
                cameras.append({
                    "id": camera_id,
                    "color": color_topic,
                    "depth": depth_topic if depth_topic and depth_topic in topics else None,
                    "info": info_topic if info_topic in topics else None,
                })
                
        except Exception as e:
            print(f"Topic discovery failed: {e}")
        
        return cameras


# ============ Multi-Camera Support ============

@dataclass
class CameraState:
    """单个相机的状态"""
    camera_id: str
    color_image: Optional[bytes] = None
    depth_image: Optional[bytes] = None
    intrinsics: Optional[Dict] = None
    frame_count: int = 0
    last_update: float = 0.0


class MultiCameraManager:
    """管理多个相机连接"""
    
    def __init__(self):
        self.cameras: Dict[str, CameraState] = {}
        self._bridges: Dict[str, Any] = {}
        self._node: Optional[Node] = None
        
    def initialize_ros(self, node_name: str = "vision_mcp_multi"):
        """初始化 ROS2"""
        if not _HAS_ROS2:
            return False
        
        rclpy.init()
        self._node = Node(node_name)
        return True
    
    def add_camera(self, camera_config: Dict[str, str]):
        """添加一个相机"""
        camera_id = camera_config["id"]
        
        state = CameraState(camera_id=camera_id)
        self.cameras[camera_id] = state
        
        if self._node:
            # Create subscriptions
            color_topic = camera_config.get("color") or f"/{camera_id}/color/image_raw"
            depth_topic = camera_config.get("depth")  # Can be None
            info_topic = camera_config.get("info") or f"/{camera_id}/color/camera_info"
            
            self._node.create_subscription(
                Image, color_topic, 
                lambda msg, cid=camera_id: self._color_callback(cid, msg), 
                10
            )
            
            if depth_topic:
                self._node.create_subscription(
                    Image, depth_topic,
                    lambda msg, cid=camera_id: self._depth_callback(cid, msg),
                    10
                )
            
            self._node.create_subscription(
                CameraInfo, info_topic,
                lambda msg, cid=camera_id: self._info_callback(cid, msg),
                10
            )
        
        return state
    
    def _color_callback(self, camera_id: str, msg: Image):
        """处理彩色图像"""
        if camera_id in self.cameras:
            state = self.cameras[camera_id]
            state.color_image = msg.data
            state.frame_count += 1
            state.last_update = time.time()
    
    def _depth_callback(self, camera_id: str, msg: Image):
        """处理深度图像"""
        if camera_id in self.cameras:
            state = self.cameras[camera_id]
            state.depth_image = msg.data
    
    def _info_callback(self, camera_id: str, msg: CameraInfo):
        """处理相机参数"""
        if camera_id in self.cameras:
            state = self.cameras[camera_id]
            state.intrinsics = {
                "fx": msg.k[0], "fy": msg.k[4],
                "cx": msg.k[2], "cy": msg.k[5],
                "width": msg.width, "height": msg.height,
                "frame_id": msg.header.frame_id
            }
    
    def spin_once(self, timeout_sec: float = 0.01):
        """处理一次 ROS 事件"""
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)


# ============ YOLO Object Detection ============

class ObjectDetector:
    """YOLO 目标检测器"""
    
    def __init__(self, model_name: str = "yolov8n.pt"):
        self.model = None
        self.model_name = model_name
        
        if _HAS_YOLO:
            try:
                # Download model if needed
                self.model = YOLO(model_name)
                print(f"✓ YOLO model loaded: {model_name}")
            except Exception as e:
                print(f"✗ Failed to load YOLO: {e}")
    
    def detect(self, image_array: np.ndarray, confidence: float = 0.5) -> List[Dict]:
        """检测图像中的目标"""
        if self.model is None:
            return []
        
        results = self.model(image_array, conf=confidence)
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = result.names[cls]
                
                detections.append({
                    "class": name,
                    "confidence": round(conf, 3),
                    "bbox": {
                        "x1": int(x1), "y1": int(y1),
                        "x2": int(x2), "y2": int(y2),
                        "center_x": int((x1 + x2) / 2),
                        "center_y": int((y1 + y2) / 2),
                        "width": int(x2 - x1),
                        "height": int(y2 - y1)
                    }
                })
        
        return detections


# ============ Global Instances ============

_manager: Optional[MultiCameraManager] = None
_detector: Optional[ObjectDetector] = None
_recording_process: Optional[subprocess.Popen] = None


# ============ MCP Tools ============

@mcp.tool()
async def discover_cameras() -> str:
    """
    自动发现所有可用的 RealSense 相机。
    扫描 ROS2 话题，返回检测到的相机列表。
    """
    cameras = TopicDiscovery.discover_cameras()
    
    if not cameras:
        return "No cameras discovered. Make sure realsense2_camera is running."
    
    result = [f"Found {len(cameras)} camera(s):"]
    for cam in cameras:
        result.append(f"\n📷 {cam['id']}:")
        result.append(f"  Color: {cam['color']}")
        if cam['depth']:
            result.append(f"  Depth: {cam['depth']}")
        result.append(f"  Info:  {cam['info']}")
    
    return "\n".join(result)


@mcp.tool()
async def connect_multi_camera(camera_configs: Optional[List[Dict]] = None) -> str:
    """
    连接多个相机。
    
    Args:
        camera_configs: 相机配置列表，每个包含 id, color, depth, info
                       如果为 None，自动发现相机
    """
    global _manager
    
    if _manager is None:
        _manager = MultiCameraManager()
        if not _manager.initialize_ros():
            return "Error: ROS2 not available"
    
    # Auto-discover if no configs provided
    if camera_configs is None:
        camera_configs = TopicDiscovery.discover_cameras()
        if not camera_configs:
            return "No cameras found. Please start realsense2_camera."
    
    # Add cameras
    for config in camera_configs:
        _manager.add_camera(config)
    
    # Spin briefly to collect frames
    for _ in range(50):  # 0.5 seconds
        _manager.spin_once(timeout_sec=0.01)
        time.sleep(0.01)
    
    # Report status
    results = [f"Connected {len(camera_configs)} camera(s):"]
    for cam_id, state in _manager.cameras.items():
        status = "✓" if state.frame_count > 0 else "⚠"
        results.append(f"{status} {cam_id}: {state.frame_count} frames")
    
    return "\n".join(results)


@mcp.tool()
async def capture_from_camera(camera_id: str = "camera", quality: int = 85) -> str:
    """
    从指定相机捕获图像。
    
    Args:
        camera_id: 相机 ID (如 'camera', 'camera_1')
        quality: JPEG 质量 (1-95)
    """
    global _manager
    
    if _manager is None or camera_id not in _manager.cameras:
        return f"Error: Camera '{camera_id}' not connected"
    
    state = _manager.cameras[camera_id]
    
    if state.color_image is None:
        return f"Error: No frame from {camera_id}"
    
    # Convert to JPEG Base64
    if not _HAS_PIL:
        return "Error: PIL not available"
    
    # Assume 640x480 RGB8 for now (should get from intrinsics)
    width = state.intrinsics["width"] if state.intrinsics else 640
    height = state.intrinsics["height"] if state.intrinsics else 480
    
    img_array = np.frombuffer(state.color_image, dtype=np.uint8).reshape((height, width, 3))
    pil_img = PILImage.fromarray(img_array)
    
    buffer = io.BytesIO()
    pil_img.save(buffer, format="JPEG", quality=quality)
    img_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
    
    return f"data:image/jpeg;base64,{img_base64}"


@mcp.tool()
async def detect_objects(camera_id: str = "camera", confidence: float = 0.5) -> str:
    """
    在相机视野中检测目标。
    需要 YOLO 支持 (pip install ultralytics)
    
    Args:
        camera_id: 相机 ID
        confidence: 置信度阈值 (0-1)
    """
    global _manager, _detector
    
    if _manager is None or camera_id not in _manager.cameras:
        return f"Error: Camera '{camera_id}' not connected"
    
    if not _HAS_YOLO:
        return "Error: YOLO not installed. Run: pip install ultralytics"
    
    if _detector is None:
        _detector = ObjectDetector()
    
    state = _manager.cameras[camera_id]
    if state.color_image is None:
        return f"Error: No frame from {camera_id}"
    
    # Convert to numpy
    width = state.intrinsics["width"] if state.intrinsics else 640
    height = state.intrinsics["height"] if state.intrinsics else 480
    img_array = np.frombuffer(state.color_image, dtype=np.uint8).reshape((height, width, 3))
    
    # Detect
    detections = _detector.detect(img_array, confidence)
    
    if not detections:
        return f"No objects detected in {camera_id} (confidence > {confidence})"
    
    # Format results
    results = [f"Detected {len(detections)} object(s) in {camera_id}:"]
    for det in detections[:5]:  # Limit to top 5
        bbox = det["bbox"]
        results.append(
            f"- {det['class']} ({det['confidence']:.0%}) "
            f"at ({bbox['center_x']}, {bbox['center_y']})"
        )
    
    return "\n".join(results)


@mcp.tool()
async def get_camera_status() -> str:
    """获取所有相机的状态"""
    global _manager
    
    if _manager is None or not _manager.cameras:
        return "No cameras connected"
    
    results = [f"Camera Status ({len(_manager.cameras)} connected):"]
    
    for cam_id, state in _manager.cameras.items():
        intrinsics = state.intrinsics or {}
        results.append(f"\n📷 {cam_id}:")
        results.append(f"  Frames: {state.frame_count}")
        results.append(f"  Resolution: {intrinsics.get('width', '?')}x{intrinsics.get('height', '?')}")
        if intrinsics:
            results.append(f"  Intrinsics: fx={intrinsics.get('fx', 0):.1f}, fy={intrinsics.get('fy', 0):.1f}")
        results.append(f"  Last update: {time.time() - state.last_update:.1f}s ago")
    
    return "\n".join(results)


@mcp.tool()
async def capture_stereo_image(left_camera: str = "camera", 
                                right_camera: str = "camera_2",
                                quality: int = 85) -> str:
    """
    同时捕获两个相机的图像（立体视觉）。
    
    Args:
        left_camera: 左相机 ID
        right_camera: 右相机 ID
        quality: JPEG 质量
    """
    # Capture from both cameras
    left_result = await capture_from_camera(left_camera, quality)
    right_result = await capture_from_camera(right_camera, quality)
    
    if left_result.startswith("Error"):
        return f"Left camera error: {left_result}"
    if right_result.startswith("Error"):
        return f"Right camera error: {right_result}"
    
    return json.dumps({
        "left": left_result,
        "right": right_result,
        "timestamp": time.time()
    })


@mcp.tool()
async def disconnect_all() -> str:
    """断开所有相机连接"""
    global _manager
    
    if _manager:
        count = len(_manager.cameras)
        if _manager._node:
            _manager._node.destroy_node()
        _manager = None
        return f"Disconnected {count} camera(s)"
    
    return "No cameras to disconnect"


# ============ Main Entry ============

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ROSClaw Vision MCP Server - Enhanced Edition",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Enhanced Features:
  - Multi-camera support
  - Auto topic discovery
  - YOLO object detection (optional)
  - Stereo vision capture

Examples:
  # Standard mode
  python3 vision_mcp_enhanced.py
  
  # With YOLO detection
  pip install ultralytics
  python3 vision_mcp_enhanced.py --transport sse
        """.strip()
    )
    
    parser.add_argument(
        "--transport",
        choices=["stdio", "sse"],
        default=os.environ.get("MCP_TRANSPORT", "stdio"),
        help="Transport mode"
    )
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--host", default="127.0.0.1")
    
    args = parser.parse_args()
    
    print("🚀 ROSClaw Vision MCP Server - Enhanced")
    print(f"   Transport: {args.transport}")
    print(f"   Multi-camera: Enabled")
    print(f"   Auto-discovery: Enabled")
    print(f"   YOLO: {'Available' if _HAS_YOLO else 'Not installed'}")
    print()
    
    if args.transport == "sse":
        print(f"Starting SSE server on http://{args.host}:{args.port}")
        mcp.settings.host = args.host
        mcp.settings.port = args.port
        mcp.run(transport="sse")
    else:
        mcp.run(transport="stdio")
