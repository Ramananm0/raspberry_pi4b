# Raspberry Pi 4B — Terrain Bot ROS2 Stack

Full ROS2 Humble navigation, SLAM, and AI stack for the terrain-aware
differential drive robot. Receives sensor data from the STM32F746G-DISCO
over USB-to-TTL RS232 (CP2102/CH340) at 2 Mbaud via micro-ROS.

---

## System architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 4B  (ROS2 Humble)                 │
│                                                                    │
│  /dev/stm32  (USB-TTL)                                            │
│  ──────────► micro_ros_agent ──► /imu/data       (100 Hz)        │
│                                ► /wheel_ticks    ( 50 Hz)        │
│                                ► /wheel_velocity ( 50 Hz)        │
│                                                                    │
│  /dev/rplidar (USB)                                               │
│  ──────────► rplidar_node ──────► /scan          (5.5 Hz)        │
│                                                                    │
│  terrain_odom ─────────────────► /odom  + TF odom→base_footprint │
│                                                                    │
│  robot_localization EKF ───────► /odometry/filtered (50 Hz)      │
│    (/odom + /imu/data fused)       TF odom→base_footprint        │
│                                                                    │
│  slam_toolbox ─────────────────► /map  + TF map→odom             │
│                                                                    │
│  terrain_traversability ───────► /terrain/risk                   │
│    (/imu/data + /scan)          ► /terrain/costmap               │
│                                 ► /terrain/slope                 │
│                                 ► /terrain/roughness             │
│                                 ► /terrain/proximity             │
│                                                                    │
│  terrain_ai ───────────────────► /cmd_vel  (scaled, safe)        │
│    (/cmd_vel_raw + risk)          /terrain/features  (φ vector)   │
│                                                                    │
│  teleop_twist_keyboard ────────► /cmd_vel_raw                    │
└────────────────────────────────────────────────────────────────────┘
               │
               │ USB-to-TTL RS232 (2 Mbaud)
               ▼
┌──────────────────────────────────┐
│  STM32F746G-DISCO                │
│  ICM-20948 + 2× RMCS-3070 enc  │
└──────────────────────────────────┘
```

---

## Package overview

| Package                  | Purpose                                                    |
|--------------------------|------------------------------------------------------------|
| `terrain_bringup`        | Launch files + YAML configs for full system                |
| `terrain_odom`           | Encoder ticks → /odom + TF (differential drive kinematics)|
| `terrain_ai`             | Velocity safety supervisor (heuristic + TFLite/ONNX)      |
| `terrain_traversability` | LiDAR + IMU → traversability risk + OccupancyGrid costmap |
| `terrain_robot_description` | URDF/xacro model for robot_state_publisher              |

---

## Sensor fusion pipeline

### 1. Wheel odometry (terrain_odom)

Differential drive kinematics (Siegwart & Nourbakhsh, 2011):

```
left_m  = Δticks_L × (π × d_wheel / ticks_per_rev)
right_m = Δticks_R × (π × d_wheel / ticks_per_rev)

Δs     = (right_m + left_m)  / 2        # arc length
Δθ     = (right_m - left_m)  / b        # b = wheelbase

x     += Δs × cos(θ + Δθ/2)
y     += Δs × sin(θ + Δθ/2)
θ     += Δθ
```

Parameters:
- `wheel_diameter_mm` = 8.5 mm (RMCS-3070)
- `ticks_per_rev` = 2264 (566 CPR × 4 quadrature)
- `wheel_base_mm` = 200 mm (set to measured track width)

### 2. Extended Kalman Filter (robot_localization)

Fuses `/odom` (wheel kinematics) with `/imu/data` (Madgwick quaternion):

**State vector** x ∈ ℝ¹⁵:
```
x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, φ̇, θ̇, ψ̇, ẍ, ÿ, z̈]
```

**Measurement assignments:**
```
Odom  → [x, y, ẋ]          (wheel dead-reckoning)
IMU   → [φ, θ, ψ, φ̇, θ̇, ψ̇, ẍ]  (Madgwick AHRS + gyro + accel)
```

The Madgwick filter on the STM32 provides pre-fused orientation
(quaternion) which the EKF receives as a direct measurement,
avoiding re-fusion of raw accel/gyro on the RPi.

Reference: Moore & Stouch, IAS-13, 2014.

---

## Traversability assessment

### Algorithm (Papadakis, 2013; Thrun et al., 2006)

Three orthogonal risk channels are estimated and linearly combined:

**Channel 1 — Slope risk R_s** (from Madgwick quaternion):
```
inclination = √(pitch² + roll²)
R_s = clip(inclination / θ_max, 0, 1)    θ_max = 30° = 0.524 rad
```

**Channel 2 — Roughness risk R_r** (from IMU angular velocity variance):
```
R_r = clip(σ²_ω / σ²_max, 0, 1)          σ²_max = 0.01 (rad/s)²
```
The ω² variance over a 200ms window is an effective vibration proxy
for terrain roughness (Brooks & Iagnemma, 2005; Weiss et al., 2006).

**Channel 3 — Proximity risk R_p** (from RPLidar forward cone):
```
R_p = clip(1 - min_fwd_range / d_safe, 0, 1)    d_safe = 0.5 m
```

**Composite traversability risk:**
```
T = w_s × R_s + w_r × R_r + w_p × R_p
  = 0.4 × R_s + 0.2 × R_r + 0.4 × R_p     (default weights)
```

**EMA smoothing** (removes single-scan spikes):
```
T_smooth(k) = α × T(k) + (1-α) × T_smooth(k-1)     α = 0.3
```

Published topics:
- `/terrain/risk` — composite risk score ∈ [0, 1]
- `/terrain/costmap` — 5×5m OccupancyGrid centred on robot (5cm cells)
- `/terrain/slope`, `/terrain/roughness`, `/terrain/proximity` — raw channels

---

## AI velocity supervisor

### Feature vector φ ∈ ℝ⁸

| Index | Feature         | Source       | Unit        |
|-------|-----------------|--------------|-------------|
| φ₀    | pitch           | Madgwick     | rad         |
| φ₁    | roll            | Madgwick     | rad         |
| φ₂    | \|ω\|²          | IMU gyro     | (rad/s)²    |
| φ₃    | σ²_ω            | IMU window   | (rad/s)²    |
| φ₄    | \|a_lateral\|   | IMU accel    | m/s²        |
| φ₅    | min_fwd_range   | RPLidar      | m           |
| φ₆    | scan_entropy    | RPLidar      | nats        |
| φ₇    | terrain_risk    | traversability| [0,1]      |

### Velocity scaling (Alami et al., 2006)

```
v_out = v_in × max(0, 1 - α × risk)    α = 0.85
ω_out = ω_in × max(0, 1 - β × risk)    β = 0.60

if risk ≥ r_stop (0.85): v_out = ω_out = 0   # emergency stop
```

### Model drop-in interface

When an ML model is ready, point `model_path` parameter at:
- `.tflite` file → runs via `tflite-runtime` (quantized, fast on RPi 4)
- `.onnx` file  → runs via `onnxruntime`

Model must accept **float32[1, 8]** input and return **float32[1, 1]** risk.
No code changes required — just set the parameter.

Training data: the `/terrain/features` topic logs the φ vector + current
terrain label for supervised learning.

---

## Hardware connection (USB-to-TTL RS232 cable)

```
Raspberry Pi 4B          USB-to-TTL cable       STM32F746G-DISCO
────────────────         ─────────────────       ────────────────
USB port         ──USB── CP2102/CH340 chip ───► D1 (PC6, USART6 TX)
                                           ◄─── D0 (PC7, USART6 RX)
GND                                        ──── GND
```

**udev rules** (auto-creates `/dev/stm32` stable symlink):
```bash
# /etc/udev/rules.d/99-terrain-bot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="stm32", MODE="0666"   # CP2102
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="stm32", MODE="0666"   # CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="rplidar", MODE="0666"
```

---

## Installation

### Prerequisites

```bash
# Ubuntu 22.04 (64-bit) on Raspberry Pi 4B
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-slam-toolbox \
    ros-humble-robot-localization ros-humble-rplidar-ros \
    ros-humble-teleop-twist-keyboard ros-humble-robot-state-publisher \
    ros-humble-xacro

# micro-ROS agent
sudo snap install micro-ros-agent
# OR build from source: https://micro.ros.org/docs/tutorials/core/first_application_linux/
```

### Build

```bash
cd ~/raspberry_pi4b/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch

```bash
# Terminal 1: full system
ros2 launch terrain_bringup bringup.launch.py

# Terminal 2: visualise in RViz (on desktop machine)
rviz2 -d ~/raspberry_pi4b/rviz/terrain_bot.rviz
```

### Verify topics

```bash
ros2 topic hz /imu/data          # should be ~100 Hz
ros2 topic hz /wheel_ticks       # should be ~50 Hz
ros2 topic hz /odometry/filtered # should be ~50 Hz (EKF)
ros2 topic hz /terrain/risk      # should be ~10 Hz
ros2 topic echo /terrain/risk    # watch risk score change while moving
```

---

## Topic reference

| Topic                   | Type                         | Hz    | Publisher              |
|-------------------------|------------------------------|-------|------------------------|
| `/imu/data`             | `sensor_msgs/Imu`            | 100   | STM32 / micro-ROS      |
| `/wheel_ticks`          | `std_msgs/Int32MultiArray`   | 50    | STM32 / micro-ROS      |
| `/wheel_velocity`       | `std_msgs/Float32MultiArray` | 50    | STM32 / micro-ROS      |
| `/scan`                 | `sensor_msgs/LaserScan`      | 5.5   | rplidar_node           |
| `/odom`                 | `nav_msgs/Odometry`          | 50    | terrain_odom           |
| `/odometry/filtered`    | `nav_msgs/Odometry`          | 50    | robot_localization EKF |
| `/map`                  | `nav_msgs/OccupancyGrid`     | ~0.5  | slam_toolbox           |
| `/terrain/risk`         | `std_msgs/Float32`           | 10    | terrain_traversability |
| `/terrain/costmap`      | `nav_msgs/OccupancyGrid`     | 10    | terrain_traversability |
| `/terrain/features`     | `std_msgs/Float32MultiArray` | 100   | terrain_ai             |
| `/cmd_vel_raw`          | `geometry_msgs/Twist`        | user  | teleop                 |
| `/cmd_vel`              | `geometry_msgs/Twist`        | user  | terrain_ai             |

---

## Configuration files

| File                              | Purpose                               |
|-----------------------------------|---------------------------------------|
| `config/ekf.yaml`                 | robot_localization EKF parameters     |
| `config/slam_params.yaml`         | SLAM Toolbox async mode settings      |
| `config/rplidar_a1.yaml`          | RPLidar A1 serial port + scan rate    |

---

## References

1. **Moore, T., Stouch, D.** (2014). "A Generalized Extended Kalman Filter
   Implementation for the Robot Operating System." *Proc. 13th Int. Conf.
   Intelligent Autonomous Systems (IAS-13)*. Springer.

2. **Papadakis, P.** (2013). "Terrain traversability analysis methods for
   unmanned ground vehicles: A survey." *Engineering Applications of Artificial
   Intelligence*, 26(8), pp. 1373-1397.
   doi:[10.1016/j.engappai.2013.04.016](https://doi.org/10.1016/j.engappai.2013.04.016)

3. **Thrun, S., et al.** (2006). "Stanley: The robot that won the DARPA Grand
   Challenge." *Journal of Field Robotics*, 23(9), pp. 661-692.
   doi:[10.1002/rob.20147](https://doi.org/10.1002/rob.20147)

4. **Brooks, C.A., Iagnemma, K.** (2005). "Vibration-based terrain
   classification for planetary exploration rovers." *IEEE Transactions on
   Robotics*, 21(6), pp. 1185-1191.
   doi:[10.1109/TRO.2005.853567](https://doi.org/10.1109/TRO.2005.853567)

5. **Weiss, C., Fröhlich, H., Zell, A.** (2006). "Terrain classification with
   support vector machines on IMU data." *Proc. European Conf. on Mobile Robots
   (ECMR)*, pp. 209-214.

6. **Siegwart, R., Nourbakhsh, I.R., Scaramuzza, D.** (2011). *Introduction to
   Autonomous Mobile Robots*, 2nd ed. MIT Press. (Chapter 5: Odometry)

7. **Madgwick, S.O.H., Harrison, A.J.L., Vaidyanathan, R.** (2011).
   "Estimation of IMU and MARG orientation using a gradient descent algorithm."
   *IEEE ICORR*. doi:[10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)
