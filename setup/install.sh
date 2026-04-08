#!/bin/bash
# install.sh  —  Raspberry Pi 4B full setup
# Run once on a fresh Ubuntu 22.04 (Jammy) install
# Usage:  chmod +x install.sh && ./install.sh

set -e
echo "=== terrain_bot RPi4 setup ==="

# ── 1. ROS2 Humble ──────────────────────────────────────────────────
echo "[1/6] Installing ROS2 Humble..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
     signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu \
     $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
                    python3-rosdep python3-pip

# ── 2. ROS2 dependencies ────────────────────────────────────────────
echo "[2/6] Installing ROS2 packages..."
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-rplidar-ros \
  ros-humble-tf2-ros \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization

# ── 3. micro-ROS agent ──────────────────────────────────────────────
echo "[3/6] Installing micro-ROS agent..."
pip3 install micro-ros-agent 2>/dev/null || true
# Alternative: build from source
sudo apt install -y ros-humble-micro-ros-agent 2>/dev/null || \
  echo "micro-ros-agent not in apt — install manually from snap:"
  echo "  sudo snap install micro-ros-agent"

# ── 4. Python ML deps (for AI node) ─────────────────────────────────
echo "[4/6] Installing Python AI/ML packages..."
pip3 install numpy scipy
# Uncomment when your model format is decided:
# pip3 install tflite-runtime          # TensorFlow Lite
# pip3 install onnxruntime             # ONNX Runtime
# pip3 install torch torchvision       # PyTorch

# ── 5. udev rules ────────────────────────────────────────────────────
echo "[5/6] Installing udev rules..."
sudo cp "$(dirname "$0")/99-stm32-usb.rules" /etc/udev/rules.d/
sudo cp "$(dirname "$0")/99-rplidar.rules"   /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  /dev/stm32   → USB-to-TTL (CP2102/CH340) connected to STM32"
echo "  /dev/rplidar → RPLidar A1"

# ── 6. Add user to dialout ───────────────────────────────────────────
echo "[6/6] Adding user to dialout group..."
sudo usermod -aG dialout $USER

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "=== Setup complete ==="
echo "Source ROS2:  source /opt/ros/humble/setup.bash"
echo "Build ws  :   cd ~/raspberry_pi4b/ros2_ws && colcon build"
echo "Run system:   ros2 launch terrain_bringup bringup.launch.py"
echo ""
echo "IMPORTANT: Log out and back in for dialout group to take effect."
