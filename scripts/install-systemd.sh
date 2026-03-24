#!/bin/bash
# Install ROSClaw Vision MCP Systemd Service

set -e

USERNAME=${1:-$USER}
SERVICE_NAME="rosclaw-vision@${USERNAME}"

echo "=== ROSClaw Vision MCP Service Installer ==="
echo "Installing for user: $USERNAME"
echo

# Check if running as root for system-wide install
if [ "$EUID" -eq 0 ]; then
    SYSTEMD_DIR="/etc/systemd/system"
    echo "Installing system-wide to $SYSTEMD_DIR"
else
    SYSTEMD_DIR="$HOME/.config/systemd/user"
    mkdir -p "$SYSTEMD_DIR"
    echo "Installing user service to $SYSTEMD_DIR"
fi

# Copy service file
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cp "${SCRIPT_DIR}/rosclaw-vision@.service" "$SYSTEMD_DIR/"

# Reload systemd
if [ "$EUID" -eq 0 ]; then
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    
    echo
    echo "=== Installation Complete ==="
    echo "Start service: sudo systemctl start $SERVICE_NAME"
    echo "Stop service:  sudo systemctl stop $SERVICE_NAME"
    echo "View logs:     sudo journalctl -u $SERVICE_NAME -f"
else
    systemctl --user daemon-reload
    systemctl --user enable "$SERVICE_NAME"
    
    echo
    echo "=== Installation Complete ==="
    echo "Start service: systemctl --user start $SERVICE_NAME"
    echo "Stop service:  systemctl --user stop $SERVICE_NAME"
    echo "View logs:     journalctl --user -u $SERVICE_NAME -f"
fi

echo
echo "Configuration file location:"
echo "  ${SCRIPT_DIR}/../config/vision-mcp.conf"
echo
echo "Edit the service file to customize:"
echo "  MCP_PORT (default: 8000)"
echo "  ROSCLAW_VISION_*_TOPIC (for custom topic names)"
