"""
Unit tests for VisionROS2Bridge — no hardware or ROS2 required.

Tests the image encoding, depth lookup, and 3D back-projection math
using synthetic injected VisionState objects.
"""

import asyncio
import base64
import io
import struct
import sys
import unittest
from unittest.mock import MagicMock, patch

# Mock rclpy before importing the server
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["sensor_msgs"] = MagicMock()
sys.modules["sensor_msgs.msg"] = MagicMock()

import numpy as np
from PIL import Image as PILImage

# Now safe to import
sys.path.insert(0, "../src")
from vision_mcp_server import VisionState, StateBuffer, VisionROS2Bridge  # noqa: E402


def make_synthetic_state(width=640, height=480, fx=600.0, fy=600.0, cx=320.0, cy=240.0):
    """Create a synthetic VisionState with known pixel values"""
    # Create a gradient RGB image
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    arr[:, :, 0] = 128   # Red channel
    arr[:, :, 1] = 64    # Green channel
    arr[:, :, 2] = 255   # Blue channel
    color_bytes = arr.tobytes()

    # Create depth image: all pixels at 1.0m = 1000mm
    depth_arr = np.full((height, width), 1000, dtype=np.uint16)
    depth_bytes = depth_arr.tobytes()

    return VisionState(
        timestamp=1000.0,
        color_image=color_bytes,
        depth_image=depth_bytes,
        image_width=width,
        image_height=height,
        fx=fx, fy=fy, cx=cx, cy=cy,
        color_encoding="rgb8",
    )


class TestStateBuffer(unittest.TestCase):

    def test_empty_buffer_returns_none(self):
        buf = StateBuffer()
        self.assertIsNone(buf.get_latest())

    def test_append_and_retrieve(self):
        buf = StateBuffer(max_size=5)
        state = make_synthetic_state()
        buf.append(state)
        result = buf.get_latest()
        self.assertIsNotNone(result)
        self.assertEqual(result.timestamp, 1000.0)

    def test_ring_buffer_max_size(self):
        buf = StateBuffer(max_size=3)
        for i in range(5):
            s = VisionState(timestamp=float(i))
            buf.append(s)
        # Should only have last 3
        latest = buf.get_latest()
        self.assertEqual(latest.timestamp, 4.0)

    def test_intrinsics_update(self):
        buf = StateBuffer()
        buf.update_intrinsics(615.0, 615.0, 320.0, 240.0, 640, 480, "camera_frame")
        info = buf.get_intrinsics()
        self.assertIsNotNone(info)
        self.assertAlmostEqual(info["fx"], 615.0)


class TestVisionBridge(unittest.TestCase):

    def setUp(self):
        # Create bridge without calling rclpy.Node.__init__
        with patch("vision_mcp_server._HAS_ROS2", False):
            self.bridge = VisionROS2Bridge.__new__(VisionROS2Bridge)
            self.bridge._connected = True
            self.bridge._state_buffer = StateBuffer(max_size=10)
            self.bridge._frame_lock = __import__("threading").Lock()
            self.bridge._pending_color = None
            self.bridge._pending_depth = None
            self.bridge._color_count = 0
            self.bridge._depth_count = 0

        # Inject synthetic state
        state = make_synthetic_state(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        self.bridge._state_buffer.append(state)

    def test_get_jpeg_base64_returns_string(self):
        b64 = self.bridge.get_jpeg_base64(quality=80)
        self.assertIsNotNone(b64)
        # Verify it's valid base64
        decoded = base64.b64decode(b64)
        # Verify it's a valid JPEG (starts with FF D8)
        self.assertEqual(decoded[:2], b"\xff\xd8")

    def test_get_depth_meters_center_pixel(self):
        # Center pixel should have depth of 1.0m (we set 1000mm)
        depth = self.bridge.get_depth_meters(320, 240)
        self.assertIsNotNone(depth)
        self.assertAlmostEqual(depth, 1.0, places=3)

    def test_get_depth_meters_out_of_bounds(self):
        depth = self.bridge.get_depth_meters(9999, 9999)
        self.assertIsNone(depth)

    def test_pixel_to_3d_at_principal_point(self):
        # At the principal point (cx, cy), X and Y should be 0
        result = self.bridge.pixel_to_3d(320, 240, 1.0)
        self.assertAlmostEqual(result["x"], 0.0, places=4)
        self.assertAlmostEqual(result["y"], 0.0, places=4)
        self.assertAlmostEqual(result["z"], 1.0, places=4)

    def test_pixel_to_3d_off_center(self):
        # Pixel (380, 240): u - cx = 60, so X = 60 * 1.0 / 600 = 0.1
        result = self.bridge.pixel_to_3d(380, 240, 1.0)
        self.assertAlmostEqual(result["x"], 0.1, places=4)
        self.assertAlmostEqual(result["y"], 0.0, places=4)
        self.assertAlmostEqual(result["z"], 1.0, places=4)

    def test_check_volume_clear_no_obstacles(self):
        # Volume way beyond camera range — should be clear
        is_clear, count = self.bridge.check_volume_clear(5.0, 6.0, 5.0, 6.0, 8.0, 10.0)
        self.assertTrue(is_clear)
        self.assertEqual(count, 0)

    def test_check_volume_contains_scene(self):
        # Volume containing most of the scene (depth=1.0m everywhere)
        is_clear, count = self.bridge.check_volume_clear(-0.5, 0.5, -0.5, 0.5, 0.5, 1.5)
        self.assertFalse(is_clear)
        self.assertGreater(count, 0)

    def test_get_status_info(self):
        info = self.bridge.get_status_info()
        self.assertTrue(info["has_frames"])
        self.assertEqual(info["resolution"], "640x480")


if __name__ == "__main__":
    unittest.main()
