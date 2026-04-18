#!/usr/bin/env python3
# Copyright 2025 Ramana
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class TerrainRiskNode(Node):
    def __init__(self):
        super().__init__('terrain_risk_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('imu_topic',      '/imu/data')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('cmd_in_topic',   '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic',  '/cmd_vel')

        # IMU params
        self.declare_parameter('alpha_cf',       0.98)   # complementary filter
        self.declare_parameter('max_roll_deg',   22.0)   # emergency stop trigger
        self.declare_parameter('max_pitch_deg',  25.0)   # emergency stop trigger
        self.declare_parameter('resume_roll_deg', 18.0)  # hysteresis: must drop below this to resume
        self.declare_parameter('resume_pitch_deg',20.0)  # hysteresis: must drop below this to resume
        self.declare_parameter('estop_cooldown',   2.0)  # seconds before resume allowed
        self.declare_parameter('safe_slope_deg',  6.0)
        self.declare_parameter('danger_slope_deg',20.0)
        self.declare_parameter('w_pitch',         0.6)   # weighted inclination
        self.declare_parameter('w_roll',          0.4)
        self.declare_parameter('w_imu_risk',      0.75)  # IMU risk weights
        self.declare_parameter('w_rough_risk',    0.25)
        self.declare_parameter('alpha_filter',    0.1)   # low pass filter
        self.declare_parameter('calib_count',     100)   # gyro bias samples

        # LiDAR params
        self.declare_parameter('d_safe',          0.5)   # metres
        self.declare_parameter('sigma_max',       2.0)   # variance max
        self.declare_parameter('w_proximity',     0.5)
        self.declare_parameter('w_roughness',     0.3)
        self.declare_parameter('w_freespace',     0.2)

        # Fusion
        self.declare_parameter('alpha_fusion',    0.6)   # IMU weight in fusion
        self.declare_parameter('vmax',            0.68)  # hardware speed limit m/s

        # Camera AI terrain risk (optional — 0.0 disables it)
        self.declare_parameter('w_camera_risk',   0.0)   # weight for camera AI risk in fusion

        # Get params
        self.imu_topic      = self.get_parameter('imu_topic').value
        self.scan_topic     = self.get_parameter('scan_topic').value
        self.cmd_in_topic   = self.get_parameter('cmd_in_topic').value
        self.cmd_out_topic  = self.get_parameter('cmd_out_topic').value

        self.alpha_cf       = self.get_parameter('alpha_cf').value
        self.max_roll       = math.radians(self.get_parameter('max_roll_deg').value)
        self.max_pitch      = math.radians(self.get_parameter('max_pitch_deg').value)
        self.resume_roll    = math.radians(self.get_parameter('resume_roll_deg').value)
        self.resume_pitch   = math.radians(self.get_parameter('resume_pitch_deg').value)
        self.estop_cooldown = self.get_parameter('estop_cooldown').value
        self.safe_slope     = math.radians(self.get_parameter('safe_slope_deg').value)
        self.danger_slope   = math.radians(self.get_parameter('danger_slope_deg').value)
        self.w_pitch        = self.get_parameter('w_pitch').value
        self.w_roll         = self.get_parameter('w_roll').value
        self.w_imu_risk     = self.get_parameter('w_imu_risk').value
        self.w_rough_risk   = self.get_parameter('w_rough_risk').value
        self.alpha_filter   = self.get_parameter('alpha_filter').value
        self.calib_count    = self.get_parameter('calib_count').value

        self.d_safe         = self.get_parameter('d_safe').value
        self.sigma_max      = self.get_parameter('sigma_max').value
        self.w_proximity    = self.get_parameter('w_proximity').value
        self.w_roughness    = self.get_parameter('w_roughness').value
        self.w_freespace    = self.get_parameter('w_freespace').value

        self.alpha_fusion    = self.get_parameter('alpha_fusion').value
        self.vmax            = self.get_parameter('vmax').value
        self.w_camera_risk   = self.get_parameter('w_camera_risk').value

        # ── State ────────────────────────────────────────────────
        # IMU complementary filter state
        self.roll           = 0.0
        self.pitch          = 0.0
        self.last_imu_time  = None

        # Gyro bias calibration
        self.gyro_bias_x    = 0.0
        self.gyro_bias_y    = 0.0
        self.gyro_bias_z    = 0.0
        self.calib_samples  = []
        self.calibrated     = False
        self.g              = 9.81  # calibrated at startup from accel magnitude

        # Risk state
        self.imu_risk_filt  = 0.0
        self.lidar_risk     = 0.0
        self.camera_risk    = 0.0   # from terrain AI (1 - speed_factor)
        self.last_cmd       = None
        self.emergency_stop = False
        self.estop_time     = None   # wall time when e-stop was triggered

        # ── Publishers ───────────────────────────────────────────
        self.pub_cmd        = self.create_publisher(Twist,   self.cmd_out_topic, 10)
        self.pub_risk       = self.create_publisher(Float32, '/terrain_risk',    10)
        self.pub_imu_risk   = self.create_publisher(Float32, '/imu_risk',        10)
        self.pub_lidar_risk = self.create_publisher(Float32, '/lidar_risk',      10)
        self.pub_slope      = self.create_publisher(Float32, '/slope_risk',      10)
        self.pub_rough      = self.create_publisher(Float32, '/roughness_risk',  10)
        self.pub_scale      = self.create_publisher(Float32, '/speed_scale',     10)
        self.pub_roll        = self.create_publisher(Float32, '/roll_deg',        10)
        self.pub_pitch       = self.create_publisher(Float32, '/pitch_deg',       10)
        self.pub_cam_risk    = self.create_publisher(Float32, '/camera_ai_risk',  10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(Imu,       self.imu_topic,    self.on_imu,  50)
        self.create_subscription(LaserScan, self.scan_topic,   self.on_scan, 10)
        self.create_subscription(Twist,     self.cmd_in_topic, self.on_cmd,  50)

        # Camera AI speed factor (optional — only used if w_camera_risk > 0)
        from std_msgs.msg import Float32 as Float32Msg
        self.create_subscription(Float32Msg, '/terrain/speed_factor',
                                 self.on_camera_speed, 10)

        self.get_logger().info('terrain_risk_node STARTED')
        self.get_logger().info(f'Calibrating gyro bias — keep robot still for {self.calib_count} samples...')

    # ── IMU CALLBACK ─────────────────────────────────────────────
    def on_imu(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y

        # Step 1 — Gyro bias + gravity calibration
        if not self.calibrated:
            self.calib_samples.append((gx, gy, msg.angular_velocity.z, ax, ay, az))
            if len(self.calib_samples) >= self.calib_count:
                self.gyro_bias_x = sum(s[0] for s in self.calib_samples) / self.calib_count
                self.gyro_bias_y = sum(s[1] for s in self.calib_samples) / self.calib_count
                self.gyro_bias_z = sum(s[2] for s in self.calib_samples) / self.calib_count
                self.g = sum(
                    math.sqrt(s[3]**2 + s[4]**2 + s[5]**2) for s in self.calib_samples
                ) / self.calib_count
                self.calibrated = True
                self.get_logger().info(
                    f'Calibrated: gyro_bias=({self.gyro_bias_x:.4f}, {self.gyro_bias_y:.4f}, '
                    f'{self.gyro_bias_z:.4f}), g={self.g:.4f} m/s²')
            return

        # Step 2 — Remove gyro bias
        gx -= self.gyro_bias_x
        gy -= self.gyro_bias_y

        # Step 3 — Roll and pitch from accelerometer
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # Step 4 — Complementary filter (gyro integration + accel correction)
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0 < dt < 0.5:
                roll_gyro  = self.roll  + gx * dt
                pitch_gyro = self.pitch + gy * dt
                self.roll  = self.alpha_cf * roll_gyro  + (1 - self.alpha_cf) * roll_acc
                self.pitch = self.alpha_cf * pitch_gyro + (1 - self.alpha_cf) * pitch_acc
        else:
            self.roll  = roll_acc
            self.pitch = pitch_acc

        self.last_imu_time = now

        # Step 5 — Emergency stop with hysteresis
        if abs(self.roll) > self.max_roll or abs(self.pitch) > self.max_pitch:
            if not self.emergency_stop:
                self.emergency_stop = True
                self.estop_time = now
                self.get_logger().warn(
                    f'EMERGENCY STOP! roll={math.degrees(self.roll):.1f}° pitch={math.degrees(self.pitch):.1f}°')
        elif self.emergency_stop:
            # Only clear if cooldown elapsed AND angles dropped below resume thresholds
            cooldown_ok = (now - self.estop_time) >= self.estop_cooldown
            angles_ok   = (abs(self.roll)  < self.resume_roll and
                           abs(self.pitch) < self.resume_pitch)
            if cooldown_ok and angles_ok:
                self.emergency_stop = False
                self.get_logger().info('Emergency stop cleared — resuming')

        # Step 6 — Weighted inclination
        Iw = self.w_pitch * abs(self.pitch) + self.w_roll * abs(self.roll)

        # Step 7 — Roughness from accelerometer shake
        g = self.g
        Amag = math.sqrt(ax*ax + ay*ay + az*az)
        S = abs(Amag - g)
        rough_risk = clamp(S / 5.0, 0.0, 1.0)

        # Step 8 — Terrain risk from IMU (both terms now dimensionless [0,1])
        # Iw is normalised via safe/danger_slope in step 9; rough_risk already [0,1]
        R = self.w_imu_risk * Iw + self.w_rough_risk * rough_risk

        # Step 9 — Risk normalization
        R_norm = clamp(
            (R - self.safe_slope) / max(self.danger_slope - self.safe_slope, 1e-6),
            0.0, 1.0
        )

        # Step 10 — Low pass filter
        self.imu_risk_filt = (1.0 - self.alpha_filter) * self.imu_risk_filt + self.alpha_filter * R_norm

        # Publish IMU diagnostics
        self.pub_roll.publish(Float32(data=float(math.degrees(self.roll))))
        self.pub_pitch.publish(Float32(data=float(math.degrees(self.pitch))))
        self.pub_slope.publish(Float32(data=float(R_norm)))
        self.pub_rough.publish(Float32(data=float(rough_risk)))
        self.pub_imu_risk.publish(Float32(data=float(self.imu_risk_filt)))

        if self.last_cmd is not None:
            self.publish_safe(self.last_cmd)

    # ── CAMERA AI CALLBACK ───────────────────────────────────────
    def on_camera_speed(self, msg: Float32):
        """Convert terrain AI speed_factor → risk score (1 - speed_factor)."""
        self.camera_risk = clamp(1.0 - msg.data, 0.0, 1.0)
        self.pub_cam_risk.publish(Float32(data=float(self.camera_risk)))

    # ── LIDAR CALLBACK ───────────────────────────────────────────
    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        r_min  = msg.range_min
        r_max  = msg.range_max
        n      = len(ranges)

        # Step 1 — Filter invalid readings
        valid = np.where(
            (ranges >= r_min) & (ranges <= r_max),
            ranges,
            r_max  # treat invalid as max range (open space)
        )

        # Step 2 — Median filter (3-sample window)
        filtered = np.array([
            np.median(valid[max(0, i-1):min(n, i+2)])
            for i in range(n)
        ])

        # Step 3 — Sector indices
        # Front: middle ±45° = ±n/8 samples
        front_width = n // 8
        mid = n // 2
        front = np.concatenate([
            filtered[max(0, mid - front_width): mid + front_width]
        ])

        # Step 4 — Obstacle proximity risk
        d_front = float(np.min(front)) if len(front) > 0 else r_max
        d_all   = float(np.min(filtered))
        P = clamp(1.0 - (d_front / max(self.d_safe, 1e-6)), 0.0, 1.0)

        # Step 5 — Scan variance roughness proxy
        sigma2 = float(np.var(filtered))
        L_r = clamp(sigma2 / max(self.sigma_max, 1e-6), 0.0, 1.0)

        # Step 6 — Free space index
        F      = float(np.mean(front)) if len(front) > 0 else r_max
        F_norm = clamp(F / r_max, 0.0, 1.0)

        # Step 7 — LiDAR risk fusion
        self.lidar_risk = clamp(
            self.w_proximity  * P +
            self.w_roughness  * L_r +
            self.w_freespace  * (1.0 - F_norm),
            0.0, 1.0
        )

        self.pub_lidar_risk.publish(Float32(data=float(self.lidar_risk)))

    # ── CMD CALLBACK ─────────────────────────────────────────────
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.publish_safe(msg)
    # ── PUBLISH SAFE VELOCITY ────────────────────────────────────
    def publish_safe(self, raw: Twist):
        # ── EMERGENCY STOP — FULL STOP ──
        if self.emergency_stop:
            self.pub_cmd.publish(Twist())
            self.pub_scale.publish(Float32(data=0.0))
            self.pub_risk.publish(Float32(data=1.0))
            self.get_logger().warn(
                f'E-STOP! roll={math.degrees(self.roll):.1f}° '
                f'pitch={math.degrees(self.pitch):.1f}°')
            return

        # ── NORMAL OPERATION ──
        imu_lidar = (self.alpha_fusion * self.imu_risk_filt +
                     (1.0 - self.alpha_fusion) * self.lidar_risk)

        # Optionally blend in camera AI terrain risk
        w_cam   = clamp(self.w_camera_risk, 0.0, 0.5)   # cap at 50% influence
        w_rest  = 1.0 - w_cam
        R_total = clamp(w_rest * imu_lidar + w_cam * self.camera_risk, 0.0, 1.0)
        scale = clamp((1.0 - R_total) ** 2, 0.0, 1.0)

        safe = Twist()
        safe.linear.x  = clamp(raw.linear.x  * scale, -self.vmax, self.vmax)
        safe.linear.y  = clamp(raw.linear.y  * scale, -self.vmax, self.vmax)
        safe.angular.z = raw.angular.z

        self.pub_cmd.publish(safe)
        self.pub_risk.publish(Float32(data=float(R_total)))
        self.pub_scale.publish(Float32(data=float(scale)))


def main():
    rclpy.init()
    node = TerrainRiskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
