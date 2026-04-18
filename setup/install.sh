#!/bin/bash
# install.sh  —  Raspberry Pi 4B full setup
# Run once on a fresh Ubuntu 22.04 (Jammy) install
# Usage:  chmod +x install.sh && ./install.sh

set -e
echo "=== terrain_bot RPi4 setup ==="

# ── 1. ROS2 Humble ──────────────────────────────────────────────────
echo "[1/8] Installing ROS2 Humble..."
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
sudo apt install -y --fix-missing ros-humble-desktop python3-colcon-common-extensions \
                    python3-rosdep python3-pip

# ── 2. ROS2 packages ────────────────────────────────────────────────
echo "[2/8] Installing ROS2 packages..."
sudo apt install -y --fix-missing \
  ros-humble-slam-toolbox \
  ros-humble-rplidar-ros \
  ros-humble-tf2-ros \
  ros-humble-robot-localization \
  ros-humble-teleop-twist-keyboard \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-v4l2-camera \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  xterm

# ── 3. micro-ROS agent ──────────────────────────────────────────────
echo "[3/8] Installing micro-ROS agent..."
if sudo apt install -y ros-humble-micro-ros-agent 2>/dev/null; then
  echo "  micro-ros-agent installed via apt"
else
  echo "  apt failed — building micro-ros-agent from source..."
  pip3 install --upgrade colcon-common-extensions
  mkdir -p ~/microros_ws/src
  cd ~/microros_ws
  git clone --depth=1 -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent 2>/dev/null || true
  git clone --depth=1 -b humble https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs 2>/dev/null || true
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  echo "source ~/microros_ws/install/setup.bash" >> ~/.bashrc
  cd ~
fi

# ── 4. Camera Module 3 (IMX708) setup ───────────────────────────────
echo "[4/8] Installing Camera Module 3 (IMX708) support..."
# Add Raspberry Pi PPA for libcamera-apps and picamera2
sudo add-apt-repository -y ppa:raspberrypi/ppa 2>/dev/null || true
sudo apt update -qq
sudo apt install -y --fix-missing \
  libcamera-dev \
  libcamera-apps \
  libcamera-tools \
  python3-picamera2 \
  v4l-utils || \
sudo apt install -y --fix-missing libcamera-dev libcamera-tools v4l-utils

# Enable Camera Module 3 in /boot/firmware/config.txt
CONFIG=/boot/firmware/config.txt
if ! grep -q "dtoverlay=imx708" "$CONFIG"; then
  echo "" | sudo tee -a "$CONFIG" > /dev/null
  echo "# Raspberry Pi Camera Module 3 (IMX708)" | sudo tee -a "$CONFIG" > /dev/null
  echo "dtoverlay=imx708" | sudo tee -a "$CONFIG" > /dev/null
  echo "camera_auto_detect=0" | sudo tee -a "$CONFIG" > /dev/null
  echo "  Camera Module 3 dtoverlay added to $CONFIG — reboot required"
else
  echo "  Camera Module 3 dtoverlay already present in $CONFIG"
fi

# ── 5. Python AI/ML packages ────────────────────────────────────────
echo "[5/8] Installing Python AI/ML packages..."
pip3 install --upgrade pip

# PyTorch for Raspberry Pi (CPU only — no CUDA on Pi)
# Uses the official torch index for ARM64
pip3 install \
  numpy \
  pillow \
  torch torchvision --index-url https://download.pytorch.org/whl/cpu

# ── 6. Python utility packages ──────────────────────────────────────
echo "[6/8] Installing Python utility packages..."
pip3 install scipy

# ── 7. udev rules ────────────────────────────────────────────────────
echo "[7/8] Installing udev rules..."
sudo cp "$(dirname "$0")/99-stm32-usb.rules" /etc/udev/rules.d/
sudo cp "$(dirname "$0")/99-rplidar.rules"   /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  /dev/stm32   → USB-to-TTL (CP2102/CH340) connected to STM32"
echo "  /dev/rplidar → RPLidar A1"

# ── 8. Add user to dialout & video groups ───────────────────────────
echo "[8/8] Adding user to dialout and video groups..."
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "=== Setup complete ==="
echo ""
echo "NEXT STEPS:"
echo "  1. Log out and back in for dialout group to take effect"
echo ""
echo "  2. Copy trained model files to ~/terrain_dataset/model/:"
echo "       best_model.pth"
echo "       model_meta.json"
echo ""
echo "  3. Build the workspace:"
echo "       source /opt/ros/humble/setup.bash"
echo "       cd ~/raspberry_pi4b/ros2_ws"
echo "       colcon build --symlink-install"
echo "       source install/setup.bash"
echo ""
echo "  4. Launch the system:"
echo "       ros2 launch terrain_bringup bringup.launch.py"
echo ""
echo "  5. In the teleop terminal that opens, use:"
echo "       i/k = forward/back   j/l = left/right   u/o = diagonal"
echo ""
echo "TOPIC OVERVIEW:"
echo "  /imu/data              STM32 IMU at 100Hz"
echo "  /scan                  RPLidar A1 at 5.5Hz"
echo "  /odom                  Wheel odometry"
echo "  /odometry/filtered     EKF fused pose"
echo "  /map                   SLAM map"
echo "  /terrain/safety_level  Camera AI terrain (SAFE/CAUTION/UNSAFE/STOP)"
echo "  /terrain/speed_factor  Camera AI speed multiplier (0.0-1.0)"
echo "  /terrain_risk          Fused risk score (0.0-1.0)"
echo "  /cmd_vel               Final safe velocity to STM32"
