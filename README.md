# rosclaw-vision-mcp

ROSClaw MCP Server for **Intel RealSense RGB-D Camera** via ROS2.

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Overview

This MCP server gives LLM agents **eyes** - the ability to see and understand their physical environment through an Intel RealSense depth camera. It implements the ROSClaw **Semantic-HAL** (语义硬件抽象层) pattern: the LLM never touches the raw 30Hz sensor stream, only on-demand semantic results.

```
RealSense Camera
      │ USB3
      ▼
realsense2_camera (ROS2, 30Hz)
      │ Fast lane → VLA/rosbag2 (data flywheel)  ← LLM never sees this
      │ Slow lane → rosclaw-vision-mcp
      ▼
LLM Agent (Claude, GPT-4o)
      │ capture_scene_snapshot() → Base64 JPEG
      │ get_object_3d_coordinates("cup") → {x:0.45, y:-0.12, z:0.05}
      ▼
rosclaw-ur-ros2-mcp
      │ pick_object(0.45, -0.12, 0.05)
```

## Why NOT stream raw data to the LLM?

A raw RealSense stream is ~27 MB/s. MCP over JSON-RPC would crash immediately.

Instead, the LLM calls tools **on-demand** and receives only compressed semantic results:
- A Base64 JPEG image (~50-150 KB) when it needs to "look"
- A JSON `{x, y, z}` coordinate when it needs to "locate an object"
- A `true/false` when it needs to "check if a workspace is clear"

## Features

- **Snapshot capture**: Base64-encoded JPEG for multimodal LLMs (Claude vision, GPT-4V)
- **3D object localization**: YOLO-World detection + depth back-projection → XYZ coordinates
- **Workspace collision checking**: Scan a 3D volume for obstacles before arm motion
- **Depth queries**: Get precise distance at any pixel
- **Data flywheel**: Launch/stop rosbag2 recording for LeRobot VLA training data
- **Thread-safe**: rclpy spin in daemon thread, FastMCP in main event loop

## Hardware

| Field | Value |
|-------|-------|
| Camera | Intel RealSense D415 / D435 / D455 |
| Interface | USB3 |
| Protocol | ROS2 via `realsense2_camera` driver |
| ROS2 Topics | `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` |
| Color | 640×480 RGB8, 30Hz |
| Depth | 640×480 Z16 (uint16 mm), 30Hz, aligned to color |

## Installation

```bash
# 1. Clone
git clone https://github.com/ros-claw/rosclaw-vision-mcp.git
cd rosclaw-vision-mcp

# 2. Source ROS2 (required)
source /opt/ros/humble/setup.bash

# 3. Install dependencies
uv venv --python /usr/bin/python3.10
source .venv/bin/activate
uv pip install -e .

# 4. Optional: YOLO-World for automatic object detection
uv pip install -e ".[detection]"
```

## Enhanced Features (New!)

- ✅ **Multi-Camera Support** — Connect and control multiple RealSense cameras simultaneously
- ✅ **Auto Topic Discovery** — Automatically find and connect to available cameras
- ✅ **YOLO Object Detection** — Optional AI-powered object detection with 3D localization
- ✅ **Stereo Vision** — Capture synchronized images from dual cameras
- ✅ **SSE Transport Mode** — Persistent server for stateful connections (fixes stdio state loss)
- ✅ **Systemd Service** — Run as a system service with auto-restart
- ✅ **Configurable Topics** — Support custom ROS2 topic namespaces

## Quick Start

### 1. Start RealSense camera(s)

**Single camera:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**Multiple cameras:**
```bash
./scripts/launch-multi-camera.sh
```

### 2. Start MCP Server

**Option A: Quick start script**
```bash
./scripts/start-server.sh sse 8000
```

**Option B: Systemd service**
```bash
sudo ./scripts/install-systemd.sh $USER
sudo systemctl start rosclaw-vision@$USER
```

**Option C: Manual**
```bash
source /opt/ros/humble/setup.bash
python3 src/vision_mcp_enhanced.py --transport sse --port 8000
```

### 3. Run Demos

```bash
# Run all demos
python3 demos/demo_all.py

# Or test individual features
mcporter call rosclaw-vision.discover_cameras
mcporter call rosclaw-vision.connect_multi_camera
mcporter call rosclaw-vision.detect_objects camera_id=camera confidence=0.5
```

## Installation

### Prerequisites

- Ubuntu 22.04+
- ROS2 Humble or Jazzy
- Python 3.10+
- Intel RealSense SDK 2.0

### Install Dependencies

```bash
# Clone repository
git clone https://github.com/ros-claw/rosclaw-vision-mcp.git
cd rosclaw-vision-mcp

# Install Python dependencies
pip install -e .

# Optional: Install YOLO for object detection
pip install ultralytics

# Install system service (optional)
sudo ./scripts/install-systemd.sh $USER
```

### Claude Desktop Configuration

**Stdio mode (stateless):**
```json
{
  "mcpServers": {
    "rosclaw-vision": {
      "command": "bash",
      "args": [
        "-c",
        "source /opt/ros/humble/setup.bash && python /path/to/rosclaw-vision-mcp/src/vision_mcp_server.py"
      ],
      "transportType": "stdio"
    }
  }
}
```

**SSE mode (stateful, recommended for production):**
```json
{
  "mcpServers": {
    "rosclaw-vision": {
      "url": "http://127.0.0.1:8000/sse"
    }
  }
}
```

## Configuration

### ROS2 Topic Names

If your RealSense camera publishes to different topic names (e.g., with namespace):

```bash
# Via environment variables
export ROSCLAW_VISION_COLOR_TOPIC=/camera/camera/color/image_raw
export ROSCLAW_VISION_DEPTH_TOPIC=/camera/camera/aligned_depth_to_color/image_raw
export ROSCLAW_VISION_INFO_TOPIC=/camera/camera/color/camera_info

python src/vision_mcp_server.py
```

### SSE Server Options

```bash
# Via command line
python src/vision_mcp_server.py --transport sse --host 0.0.0.0 --port 8080

# Via environment variables
export MCP_TRANSPORT=sse
export MCP_HOST=0.0.0.0
export MCP_PORT=8080
python src/vision_mcp_server.py
```

## Available Tools (Enhanced Edition)

### Core Tools (vision_mcp_server.py)

| Tool | Description |
|------|-------------|
| `connect_vision` | Connect to camera via ROS2 |
| `disconnect_vision` | Disconnect from camera |
| `capture_scene_snapshot` | Capture RGB snapshot as Base64 JPEG |
| `get_depth_at_pixel` | Get depth (meters) at a specific pixel |
| `get_object_3d_coordinates` | Detect object and get 3D XYZ position |
| `get_scene_description` | Get camera metadata and topic info |
| `check_workspace_clear` | Check if a 3D volume is obstacle-free |
| `start_data_recording` | Start rosbag2 recording for data flywheel |
| `stop_data_recording` | Stop recording and finalize bag file |

### Enhanced Tools (vision_mcp_enhanced.py) ⭐ New!

| Tool | Description |
|------|-------------|
| `discover_cameras` | 🔍 Auto-discover all available RealSense cameras |
| `connect_multi_camera` | 🔗 Connect multiple cameras simultaneously |
| `capture_from_camera` | 📷 Capture from specific camera by ID |
| `get_camera_status` | 📊 Get status of all connected cameras |
| `detect_objects` | 🎯 YOLO object detection with 3D localization |
| `capture_stereo_image` | 🎬 Capture synchronized stereo pair |
| `disconnect_all` | 🔌 Disconnect all cameras |

### Tool Usage Examples

**Auto-discover cameras:**
```bash
mcporter call rosclaw-vision.discover_cameras
```

**Connect all detected cameras:**
```bash
mcporter call rosclaw-vision.connect_multi_camera
```

**Detect objects with YOLO:**
```bash
mcporter call rosclaw-vision.detect_objects camera_id=camera confidence=0.5
```

**Capture stereo image:**
```bash
mcporter call rosclaw-vision.capture_stereo_image \
    left_camera=camera \
    right_camera=camera_2 \
    quality=85
```

## Available Resources

| Resource | Description |
|----------|-------------|
| `vision://status` | Camera status, resolution, intrinsics |
| `vision://topics` | ROS2 topic list and types |
| `vision://connection` | Connection status |

## End-to-End Pick & Place Example

```
User: "抓起桌上的红色杯子" ("Pick up the red cup on the table")

LLM workflow:
1. capture_scene_snapshot()
   → Base64 JPEG image

2. get_object_3d_coordinates("red cup")
   → {"x": 0.45, "y": -0.12, "z": 0.38, "confidence": 0.92}

3. check_workspace_clear(x_min=0.3, x_max=0.6, ...)
   → "✓ Workspace clear"

4. [switch to rosclaw-ur-ros2-mcp]
   pick_object(0.45, -0.12, 0.38)
   → "✓ Object picked"
```

## Object Detection

`get_object_3d_coordinates()` uses a two-stage strategy:

1. **With YOLO-World** (`pip install ultralytics`): Zero-shot detection - works for any object name without training. Finds bounding box, samples depth at center, back-projects to 3D.

2. **Without YOLO-World**: Returns the captured frame as Base64 for the LLM to visually locate the object, then the user can call `get_depth_at_pixel(u, v)` to get 3D coordinates.

## Data Flywheel

The `start_data_recording()` tool captures:
- `/camera/color/image_raw` - RGB video
- `/camera/aligned_depth_to_color/image_raw` - Depth video
- `/joint_states` - Robot arm state
- `/tf` - Transforms

This data feeds the LeRobot pipeline for VLA model training (π0, OpenVLA).

## Dependencies

- Python 3.10+
- ROS2 Humble or Jazzy
- `mcp[fastmcp]>=1.0.0` - MCP framework
- `Pillow>=10.0` - JPEG encoding (replaces heavy cv_bridge dependency)
- `numpy>=1.24` - Array operations
- `ultralytics>=8.0` (optional) - YOLO-World object detection

## Architecture

```
vision_mcp_server.py
├── VisionState      - Frame data (RGB bytes, depth bytes, intrinsics)
├── StateBuffer      - Thread-safe ring buffer (10 frames)
├── VisionROS2Bridge - rclpy.Node
│   ├── _color_callback()  - /camera/color/image_raw subscriber
│   ├── _depth_callback()  - /camera/aligned_depth_to_color subscriber
│   ├── _info_callback()   - /camera/color/camera_info subscriber
│   ├── get_jpeg_base64()  - RGB → JPEG → Base64
│   ├── get_depth_meters() - Z16 depth lookup
│   ├── pixel_to_3d()      - Pinhole back-projection
│   └── check_volume_clear() - Obstacle detection
└── MCP Tools        - FastMCP @mcp.tool() definitions
```

## License

MIT License - See [LICENSE](LICENSE)

## Part of ROSClaw

- [rosclaw-vision-mcp](https://github.com/ros-claw/rosclaw-vision-mcp) - RealSense camera (ROS2)
- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) - Unitree G1 (DDS)
- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) - UR5 arm (ROS2)
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) - GCU Gimbal (Serial)
