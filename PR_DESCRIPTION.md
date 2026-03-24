# PR: Add SSE Transport Mode and Configurable ROS2 Topics

## Summary
This PR addresses a critical architecture limitation in the MCP server: the **state loss problem** when using stdio transport mode.

## The Problem

In stdio mode (default), each `mcporter call` spawns a new process:

```
Process 1: connect_vision() → ROS2 node initialized → returns success → EXIT ❌
Process 2: capture_snapshot() → _bridge is None → "Not connected" error
```

This makes the vision server unusable for any real-world scenario requiring stateful connections.

## The Solution

### 1. SSE Transport Mode
Added `--transport sse` flag to run as a persistent HTTP server:

```bash
python src/vision_mcp_server.py --transport sse --port 8000
```

Benefits:
- ✅ Single persistent process
- ✅ ROS2 connection maintained across requests
- ✅ Camera state (frames, intrinsics) preserved
- ✅ Suitable for production deployments

### 2. Configurable ROS2 Topics
Made topic names configurable via environment variables:

```bash
export ROSCLAW_VISION_COLOR_TOPIC=/camera/camera/color/image_raw
export ROSCLAW_VISION_DEPTH_TOPIC=/camera/camera/aligned_depth_to_color/image_raw
export ROSCLAW_VISION_INFO_TOPIC=/camera/camera/color/camera_info
```

This fixes issues where RealSense camera uses different namespace conventions.

## Usage Examples

### Stdio Mode (debugging, MCP Inspector)
```bash
python src/vision_mcp_server.py
# or
python src/vision_mcp_server.py --transport stdio
```

### SSE Mode (production, stateful)
```bash
python src/vision_mcp_server.py --transport sse --host 0.0.0.0 --port 8000

# Then configure client:
# URL: http://localhost:8000/sse
```

### With Custom Topics
```bash
export ROSCLAW_VISION_COLOR_TOPIC=/myrobot/camera/image_raw
python src/vision_mcp_server.py --transport sse
```

## Claude Desktop Configuration

**Before (stdio - broken for multi-call workflows):**
```json
{
  "mcpServers": {
    "rosclaw-vision": {
      "command": "bash",
      "args": ["-c", "source /opt/ros/humble/setup.bash && python .../vision_mcp_server.py"]
    }
  }
}
```

**After (SSE - stateful and reliable):**
```json
{
  "mcpServers": {
    "rosclaw-vision": {
      "url": "http://127.0.0.1:8000/sse"
    }
  }
}
```

## Backwards Compatibility

- ✅ Default behavior unchanged (stdio mode)
- ✅ All existing configurations continue to work
- ✅ No breaking changes to tool APIs

## Testing

Tested on:
- Ubuntu 22.04 + ROS2 Humble
- Intel RealSense D435I
- Python 3.10

Verified:
1. Stdio mode still works with `mcp dev`
2. SSE mode maintains state across multiple requests
3. Environment variable topic configuration works
4. All tools (connect, snapshot, depth, etc.) function correctly in both modes

## Related Issues

This PR addresses the fundamental architecture issue described in MCP discussions about stateful vs stateless transports. SSE mode is the recommended approach for any MCP server requiring:
- Long-lived connections (ROS2, DDS, camera streams)
- State accumulation (tracking, mapping)
- Resource management (GPU, file handles)

## Checklist

- [x] Code follows project style guidelines
- [x] README updated with new options
- [x] Backwards compatible
- [x] Tested on real hardware
- [x] Commit message follows conventional commits

---

**Note to maintainers:** This is a significant usability improvement that doesn't break any existing functionality. SSE mode should be the recommended approach for production deployments.
