#!/bin/bash
# ==============================================================================
# Tankbot 2025 - Raspberry Pi 4B Automated Setup & Configuration Script
# Sets up Hardware PWM, I2C, High-Speed UART, GPIO permissions, and Web Service.
# ==============================================================================

set -e

echo "🤖 Setting up Tankbot 2025 on Raspberry Pi 4B..."

# 1. Check Root Privileges
if [ "$EUID" -ne 0 ]; then
  echo "❌ Please run this script with sudo: sudo bash setup_pi4b.sh"
  exit 1
fi

TARGET_USER="${SUDO_USER:-pi}"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "📂 Installation Directory: ${INSTALL_DIR}"
echo "👤 Target User: ${TARGET_USER}"

# 2. Update System Packages
echo "📦 Installing required system packages and Python libraries..."
apt-get update
apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-gpiozero \
    python3-lgpio \
    python3-serial \
    python3-smbus2 \
    python3-aiohttp \
    python3-websockets \
    i2c-tools \
    git

# 3. Configure Hardware Interfaces in config.txt (I2C, Hardware UART)
CONFIG_FILE=""
if [ -f "/boot/firmware/config.txt" ]; then
    CONFIG_FILE="/boot/firmware/config.txt" # Debian Bookworm
elif [ -f "/boot/config.txt" ]; then
    CONFIG_FILE="/boot/config.txt"          # Debian Bullseye
fi

if [ -n "$CONFIG_FILE" ]; then
    echo "⚙️ Configuring Hardware Interfaces in ${CONFIG_FILE}..."

    # Enable I2C (MPU6050 & Line Tracker)
    if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        echo "dtparam=i2c_arm=on" >> "$CONFIG_FILE"
        echo "  ✓ Enabled I2C"
    fi

    # Enable Primary Hardware UART for Bus Servos
    if ! grep -q "^enable_uart=1" "$CONFIG_FILE"; then
        echo "enable_uart=1" >> "$CONFIG_FILE"
        echo "  ✓ Enabled Hardware UART"
    fi

    # Switch Bluetooth to MiniUART to give PL011 UART0 to /dev/serial0 for rock-solid 115200 baud
    if ! grep -q "^dtoverlay=miniuart-bt" "$CONFIG_FILE"; then
        echo "dtoverlay=miniuart-bt" >> "$CONFIG_FILE"
        echo "  ✓ Configured high-stability UART0 overlay (dtoverlay=miniuart-bt)"
    fi
fi

# 4. Add User to Hardware Access Groups
echo "🔐 Configuring user permissions for GPIO, I2C, and Serial..."
usermod -a -G gpio,i2c,dialout "$TARGET_USER" || true

# 5. Configure systemd Service for Auto-start on Boot
echo "🚀 Configuring systemd service (tankbot.service)..."
cat << SERVICE_EOF > /etc/systemd/system/tankbot.service
[Unit]
Description=Tankbot 2025 Raspberry Pi 4B Controller & Mobile Web UI
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${TARGET_USER}
WorkingDirectory=${INSTALL_DIR}
ExecStart=/usr/bin/python3 ${INSTALL_DIR}/main.py --port 8080
Restart=on-failure
RestartSec=5s
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
SERVICE_EOF

systemctl daemon-reload
systemctl enable tankbot.service

echo ""
echo "=================================================================="
echo "  ✅ Tankbot 2025 Setup for Raspberry Pi 4B Completed!"
echo "=================================================================="
echo "  1. A reboot is recommended to apply UART & I2C hardware overlays:"
echo "     sudo reboot"
echo ""
echo "  2. The Tankbot service is enabled and will start on boot."
echo "     Check status: sudo systemctl status tankbot"
echo ""
echo "  3. To run manually in terminal:"
echo "     python3 ${INSTALL_DIR}/main.py --port 8080"
echo "=================================================================="
