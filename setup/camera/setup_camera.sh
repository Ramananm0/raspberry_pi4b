#!/bin/bash
# setup_camera.sh — Camera Module 3 (IMX708) on Ubuntu 22.04 / Raspberry Pi 4B
# Run once after install.sh.  Builds libcamera from the RPi fork (Ubuntu's
# packaged version is from 2020 and does not support IMX708).
set -e

echo "=== Camera Module 3 (IMX708) setup ==="

# ── 1. Build dependencies ─────────────────────────────────────────────
echo "[1/5] Installing build dependencies..."
sudo apt-get install -y --fix-missing \
  libgnutls28-dev libudev-dev libyaml-dev \
  python3-yaml python3-ply python3-jinja2 \
  libevent-dev libglib2.0-dev \
  libdw-dev libunwind-dev \
  ninja-build

sudo pip3 install meson

# ── 2. Clone RPi libcamera fork ───────────────────────────────────────
echo "[2/5] Cloning Raspberry Pi libcamera fork..."
if [ ! -d ~/libcamera ]; then
  git clone --depth=1 https://github.com/raspberrypi/libcamera.git ~/libcamera
fi

# ── 3. Build libcamera (vc4 pipeline + Python bindings) ───────────────
echo "[3/5] Building libcamera (rpi/vc4 pipeline)..."
cd ~/libcamera
~/.local/bin/meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4 \
  -Dipas=rpi/vc4 \
  -Dpycamera=enabled
sudo ninja -C build install
sudo ldconfig

# ── 4. picamera2 Python library ───────────────────────────────────────
echo "[4/5] Installing picamera2..."
pip3 install "picamera2[nogui]"

# ── 5. Kernel / boot config ───────────────────────────────────────────
echo "[5/5] Configuring kernel module and boot overlay..."

# Enable media-controller mode for bcm2835_unicam (required by new libcamera)
echo "options bcm2835_unicam media_controller=1" \
  | sudo tee /etc/modprobe.d/bcm2835-unicam.conf

# Enable VideoCore I2C bus (CSI camera I2C) + explicit IMX708 overlay
CONFIG=/boot/firmware/config.txt
if ! grep -q "dtparam=i2c_vc=on" "$CONFIG"; then
  printf "\ndtparam=i2c_vc=on\n" | sudo tee -a "$CONFIG" > /dev/null
  echo "  Added dtparam=i2c_vc=on"
fi
if ! grep -q "dtoverlay=imx708" "$CONFIG"; then
  printf "dtoverlay=imx708\n" | sudo tee -a "$CONFIG" > /dev/null
  echo "  Added dtoverlay=imx708"
fi
# Remove camera_auto_detect if present (unreliable MCLK on Ubuntu)
sudo sed -i '/camera_auto_detect/d' "$CONFIG"

# Add libcamera env vars to .bashrc
if ! grep -q "LIBCAMERA_IPA_MODULE_PATH" ~/.bashrc; then
  cat >> ~/.bashrc << 'EOF'

# libcamera (RPi fork)
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa
export PYTHONPATH=/usr/local/lib/python3/dist-packages:/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
EOF
fi

# Copy stream script to home
cp "$(dirname "$0")/stream_camera.py" ~/stream_camera.py

echo ""
echo "=== Camera setup complete ==="
echo ""
echo "REBOOT required for dtoverlay to take effect:"
echo "  sudo reboot"
echo ""
echo "After reboot, verify the sensor is detected:"
echo "  sudo dmesg | grep imx708          # should show: camera module ID 0x0381"
echo "  v4l2-ctl --list-devices           # should show unicam -> /dev/video0"
echo ""
echo "Start live MJPEG stream (port 8080):"
echo "  python3 ~/stream_camera.py"
echo "  # then open http://<pi-ip>:8080/ in a browser"
echo ""
echo "Single snapshot:"
echo "  python3 ~/stream_camera.py &"
echo "  curl http://localhost:8080/snap -o snap.jpg"
