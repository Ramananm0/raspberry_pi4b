#!/usr/bin/env python3
"""
ai_node.py  —  Terrain AI inference node  (Raspberry Pi 4B)

Current state: PLACEHOLDER — ML model not yet ready.
               Passes cmd_vel through unchanged, logs IMU + scan data.

When your ML model is ready, replace the infer() method with your
TFLite / ONNX / PyTorch inference call.

Architecture:
  STM32F7  ──USB-TTL──►  micro-ROS  ──►  /imu/data       ┐
  RPLidar A1           ──────────────►  /scan             ├─► AI ──► /cmd_vel
  Nav stack            ──────────────►  /cmd_vel_raw      ┘

Inputs (subscriptions):
  /imu/data          sensor_msgs/Imu        from STM32
  /scan              sensor_msgs/LaserScan  from RPLidar A1
  /cmd_vel_raw       geometry_msgs/Twist    from nav stack

Outputs (publications):
  /cmd_vel           geometry_msgs/Twist    to motor controller
  /terrain/risk      std_msgs/Float32       0.0=safe  1.0=danger
  /terrain/label     std_msgs/String        terrain class label
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class TerrainAI(Node):
    def __init__(self):
        super().__init__('terrain_ai')

        self.declare_parameter('imu_topic',      '/imu/data')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('cmd_in_topic',   '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic',  '/cmd_vel')
        self.declare_parameter('model_path',     '')      # set when model ready
        self.declare_parameter('vmax',           0.044)   # 44.5 mm/s = 0.0445 m/s

        imu_topic  = self.get_parameter('imu_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_in     = self.get_parameter('cmd_in_topic').value
        cmd_out    = self.get_parameter('cmd_out_topic').value
        model_path = self.get_parameter('model_path').value
        self.vmax  = self.get_parameter('vmax').value

        # ── Load model if path provided ──────────────────────────
        self.model = None
        if model_path:
            self._load_model(model_path)
        else:
            self.get_logger().warn(
                'No model_path set — running in PASSTHROUGH mode. '
                'Set model_path parameter when ML model is ready.')

        # ── State ────────────────────────────────────────────────
        self.roll_rad  = 0.0
        self.pitch_rad = 0.0
        self.scan_ranges = []
        self.last_cmd  = None

        # ── Publishers ───────────────────────────────────────────
        self.pub_cmd   = self.create_publisher(Twist,   cmd_out,         10)
        self.pub_risk  = self.create_publisher(Float32, '/terrain/risk',  10)
        self.pub_label = self.create_publisher(String,  '/terrain/label', 10)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(Imu,       imu_topic,  self.on_imu,  20)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.create_subscription(Twist,     cmd_in,     self.on_cmd,  20)

        self.get_logger().info('terrain_ai node started (Raspberry Pi 4B)')

    # ── Model loader — fill this in when ML is ready ─────────────
    def _load_model(self, path: str):
        """
        TODO: Load your trained model here.
        Options:
          TFLite  : import tflite_runtime.interpreter as tflite
          ONNX    : import onnxruntime as ort
          PyTorch : import torch
        """
        self.get_logger().info(f'Loading model from {path}')
        # self.model = ...
        self.get_logger().warn('_load_model() not yet implemented — passthrough mode')

    # ── Inference — replace with real model call ─────────────────
    def infer(self) -> tuple[float, str]:
        """
        Returns (risk_score 0.0-1.0, label string).

        Feature vector fed to model (extend as needed):
          [roll_rad, pitch_rad,
           scan_min_m, scan_mean_m, scan_std_m,
           ... your IMU-derived features ...]
        """
        if self.model is None:
            # Passthrough: compute simple heuristic risk
            risk = self._heuristic_risk()
            label = 'flat' if risk < 0.3 else ('slope' if risk < 0.7 else 'danger')
            return risk, label

        # ── Real inference (uncomment when model is ready) ───────
        # features = np.array([[
        #     self.roll_rad, self.pitch_rad,
        #     np.min(self.scan_ranges) if self.scan_ranges else 10.0,
        #     np.mean(self.scan_ranges) if self.scan_ranges else 10.0,
        #     np.std(self.scan_ranges) if self.scan_ranges else 0.0,
        # ]], dtype=np.float32)
        # risk  = float(self.model.predict(features)[0])
        # label = self._risk_to_label(risk)
        # return risk, label

        return 0.0, 'unknown'

    def _heuristic_risk(self) -> float:
        """Simple fallback risk using IMU roll/pitch until model is loaded."""
        slope = max(abs(self.roll_rad), abs(self.pitch_rad))
        slope_risk = min(slope / math.radians(30.0), 1.0)

        if self.scan_ranges:
            r = np.array(self.scan_ranges)
            r = r[np.isfinite(r) & (r > 0.1)]
            prox_risk = max(0.0, 1.0 - float(np.min(r)) / 0.5) if len(r) else 0.0
        else:
            prox_risk = 0.0

        return min(0.7 * slope_risk + 0.3 * prox_risk, 1.0)

    # ── IMU callback (data comes from STM32 via micro-ROS) ────────
    def on_imu(self, msg: Imu):
        # STM32 already runs the complementary filter; we can read
        # roll/pitch from angular velocity + linear acceleration if needed.
        # For now: derive pitch/roll from accel for AI feature vector.
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.roll_rad  = math.atan2(ay, az)
        self.pitch_rad = math.atan2(-ax, math.sqrt(ay*ay + az*az))

    # ── Scan callback ─────────────────────────────────────────────
    def on_scan(self, msg: LaserScan):
        self.scan_ranges = list(msg.ranges)

    # ── Command callback ──────────────────────────────────────────
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        risk, label = self.infer()

        # Scale speed by (1 - risk)²  — same formula as terrain_risk_node
        scale = (1.0 - risk) ** 2

        safe = Twist()
        safe.linear.x  = max(-self.vmax, min(self.vmax, msg.linear.x * scale))
        safe.linear.y  = max(-self.vmax, min(self.vmax, msg.linear.y * scale))
        safe.angular.z = msg.angular.z   # turn rate unchanged

        self.pub_cmd.publish(safe)
        self.pub_risk.publish(Float32(data=float(risk)))
        self.pub_label.publish(String(data=label))


def main():
    rclpy.init()
    node = TerrainAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
