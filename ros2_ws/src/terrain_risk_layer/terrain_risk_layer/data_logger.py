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

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import csv
import os
import math
from datetime import datetime

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def quat_to_rpy(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(t2)
    return roll, pitch

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Create log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/terrain_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(log_dir, f'terrain_data_{timestamp}.csv')

        self.csv_file = open(self.log_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write header
        self.writer.writerow([
            'timestamp',
            'sim_time',
            # IMU raw
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            # IMU computed
            'roll_deg', 'pitch_deg',
            'accel_magnitude',
            'shake_factor',
            # Odometry
            'pos_x', 'pos_y', 'vel_linear', 'vel_angular',
            # LiDAR
            'scan_min_range', 'scan_mean_range', 'scan_variance',
            'front_min_range', 'free_space_index',
            # Risk
            'terrain_risk', 'imu_risk', 'lidar_risk',
            'slope_risk', 'roughness_risk', 'speed_scale',
            'filtered_roll_deg', 'filtered_pitch_deg',
            # Camera AI terrain
            'camera_ai_safety', 'camera_ai_speed_factor',
            'camera_ai_path_action',
        ])

        # State
        self.imu_data = None
        self.odom_data = None
        self.scan_data = None
        self.terrain_risk = 0.0
        self.imu_risk = 0.0
        self.lidar_risk = 0.0
        self.slope_risk = 0.0
        self.roughness_risk = 0.0
        self.speed_scale = 1.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        # Camera AI terrain state
        self.camera_ai_safety = 'UNKNOWN'
        self.camera_ai_speed  = 1.0
        self.camera_ai_action = 'UNKNOWN'

        # Subscribers
        self.create_subscription(Imu,       '/imu/data',       self.on_imu,   50)
        self.create_subscription(Odometry,  '/odom',           self.on_odom,  50)
        self.create_subscription(LaserScan, '/scan',           self.on_scan,  10)
        self.create_subscription(Float32,   '/terrain_risk',   self.on_risk,  10)
        self.create_subscription(Float32,   '/imu_risk',       self.on_imu_risk, 10)
        self.create_subscription(Float32,   '/lidar_risk',     self.on_lidar_risk, 10)
        self.create_subscription(Float32,   '/slope_risk',     self.on_slope, 10)
        self.create_subscription(Float32,   '/roughness_risk', self.on_rough, 10)
        self.create_subscription(Float32,   '/speed_scale',    self.on_scale, 10)
        self.create_subscription(Float32,   '/roll_deg',       self.on_roll,  10)
        self.create_subscription(Float32,   '/pitch_deg',      self.on_pitch, 10)
        # Camera AI terrain topics
        from std_msgs.msg import String
        self.create_subscription(String,  '/terrain/safety_level',   self.on_cam_safety, 10)
        self.create_subscription(Float32, '/terrain/speed_factor',   self.on_cam_speed,  10)
        self.create_subscription(String,  '/terrain/path_decision',  self.on_cam_path,   10)

        # Log at 10Hz
        self.create_timer(0.1, self.log_data)
        self.get_logger().info(f'Data logger started — saving to {self.log_file}')

    def on_cam_safety(self, msg): self.camera_ai_safety = msg.data
    def on_cam_speed(self, msg):  self.camera_ai_speed  = msg.data
    def on_cam_path(self, msg):
        try:
            import json
            d = json.loads(msg.data)
            self.camera_ai_action = d.get('action', 'UNKNOWN')
        except Exception:
            pass

    def on_imu(self, msg):
        self.imu_data = msg

    def on_odom(self, msg):
        self.odom_data = msg

    def on_scan(self, msg):
        self.scan_data = msg

    def on_risk(self, msg):   self.terrain_risk   = msg.data
    def on_imu_risk(self, msg): self.imu_risk     = msg.data
    def on_lidar_risk(self, msg): self.lidar_risk = msg.data
    def on_slope(self, msg):  self.slope_risk     = msg.data
    def on_rough(self, msg):  self.roughness_risk = msg.data
    def on_scale(self, msg):  self.speed_scale    = msg.data
    def on_roll(self, msg):   self.roll_deg       = msg.data
    def on_pitch(self, msg):  self.pitch_deg      = msg.data

    def log_data(self):
        now = self.get_clock().now()
        timestamp = datetime.now().strftime('%H:%M:%S.%f')
        sim_time = now.nanoseconds * 1e-9

        # IMU values
        ax = ay = az = gx = gy = gz = 0.0
        roll_deg = pitch_deg = accel_mag = shake = 0.0
        if self.imu_data:
            ax = self.imu_data.linear_acceleration.x
            ay = self.imu_data.linear_acceleration.y
            az = self.imu_data.linear_acceleration.z
            gx = self.imu_data.angular_velocity.x
            gy = self.imu_data.angular_velocity.y
            gz = self.imu_data.angular_velocity.z
            q = self.imu_data.orientation
            roll, pitch = quat_to_rpy(q.x, q.y, q.z, q.w)
            roll_deg  = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
            shake = abs(accel_mag - 9.807)

        # Odometry values
        pos_x = pos_y = vel_lin = vel_ang = 0.0
        if self.odom_data:
            pos_x   = self.odom_data.pose.pose.position.x
            pos_y   = self.odom_data.pose.pose.position.y
            vel_lin = self.odom_data.twist.twist.linear.x
            vel_ang = self.odom_data.twist.twist.angular.z

        # LiDAR values
        scan_min = scan_mean = scan_var = front_min = free_space = 0.0
        if self.scan_data:
            ranges = np.array(self.scan_data.ranges)
            valid = ranges[
                (ranges >= self.scan_data.range_min) &
                (ranges <= self.scan_data.range_max)
            ]
            if len(valid) > 0:
                scan_min  = float(np.min(valid))
                scan_mean = float(np.mean(valid))
                scan_var  = float(np.var(valid))
                n = len(ranges)
                front_width = n // 8
                mid = n // 2
                front = valid[max(0, mid-front_width):mid+front_width]
                if len(front) > 0:
                    front_min  = float(np.min(front))
                    free_space = float(np.mean(front)) / self.scan_data.range_max

        self.writer.writerow([
            timestamp, f'{sim_time:.3f}',
            f'{ax:.4f}', f'{ay:.4f}', f'{az:.4f}',
            f'{gx:.4f}', f'{gy:.4f}', f'{gz:.4f}',
            f'{roll_deg:.3f}', f'{pitch_deg:.3f}',
            f'{accel_mag:.4f}', f'{shake:.4f}',
            f'{pos_x:.3f}', f'{pos_y:.3f}',
            f'{vel_lin:.3f}', f'{vel_ang:.3f}',
            f'{scan_min:.3f}', f'{scan_mean:.3f}',
            f'{scan_var:.4f}', f'{front_min:.3f}',
            f'{free_space:.3f}',
            f'{self.terrain_risk:.4f}',
            f'{self.imu_risk:.4f}',
            f'{self.lidar_risk:.4f}',
            f'{self.slope_risk:.4f}',
            f'{self.roughness_risk:.4f}',
            f'{self.speed_scale:.4f}',
            f'{self.roll_deg:.3f}',
            f'{self.pitch_deg:.3f}',
            self.camera_ai_safety,
            f'{self.camera_ai_speed:.3f}',
            self.camera_ai_action,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Data saved to {self.log_file}')
        super().destroy_node()

def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
