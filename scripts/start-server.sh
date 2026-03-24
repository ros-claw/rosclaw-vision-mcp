#!/bin/bash
# Quick start script for ROSClaw Vision MCP

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
else
    echo "Error: ROS2 not found"
    exit 1
fi

# Default to SSE mode
TRANSPORT="${1:-sse}"
PORT="${2:-8000}"

echo "🚀 Starting ROSClaw Vision MCP Server"
echo "   Mode: $TRANSPORT"
echo "   Port: $PORT"
echo

if [ "$TRANSPORT" = "sse" ]; then
    python3 src/vision_mcp_enhanced.py --transport sse --host 0.0.0.0 --port "$PORT"
else
    python3 src/vision_mcp_enhanced.py --transport stdio
fi
