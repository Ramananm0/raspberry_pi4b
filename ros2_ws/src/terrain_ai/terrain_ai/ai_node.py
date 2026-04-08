#!/usr/bin/env python3
"""
ai_node.py  —  Terrain-Aware Velocity Safety Supervisor
                 Raspberry Pi 4B  /  ROS2 Humble

Purpose:
  Intercepts operator velocity commands (/cmd_vel_raw) and scales them
  down proportionally to terrain risk before forwarding to /cmd_vel.
  When an ML model is loaded (TFLite or ONNX), it replaces the
  heuristic risk estimator with a learned classifier.

Architecture (IEEE-grade):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Input feature vector  φ ∈ ℝ⁸  (updated @ IMU rate, 100 Hz)
  ──────────────────────────────
   φ₀  pitch            [rad]    ← Madgwick AHRS
   φ₁  roll             [rad]    ← Madgwick AHRS
   φ₂  |ω|²             [(rad/s)²] ← angular velocity magnitude²
   φ₃  σ²_ω             [(rad/s)²] ← ω variance over 20 samples
   φ₄  |a_lateral|      [m/s²]   ← centripetal acceleration proxy
   φ₅  min_fwd_range    [m]      ← RPLidar forward cone ±30°
   φ₆  scan_entropy     [nats]   ← Shannon entropy of range histogram
   φ₇  terrain_risk_ext [0..1]   ← from /terrain/risk if available

  Risk score  r = f(φ)  where f is:
    • Without model: weighted heuristic (transparent, explainable)
    • With TFLite:   quantized neural network (drop .tflite file)
    • With ONNX:     any ONNX-compatible model (pytorch/sklearn export)

  Velocity scaling (Alami et al., 2006; Murphy, 2000):
    v_out = v_in × max(0, 1 - α × r)   α=0.85 (tunable)
    ω_out = ω_in × max(0, 1 - β × r)   β=0.60 (yaw less sensitive)
    Emergency stop:  if r > r_stop (default 0.85) → v_out = ω_out = 0

References:
  [1] Weiss, C., Fröhlich, H., Zell, A. (2006). "Terrain classification
      with support vector machines on IMU data." Proc. ECMR, pp.209-214.
  [2] Brooks, C.A., Iagnemma, K. (2005). "Vibration-based terrain
      classification for planetary exploration rovers." IEEE Trans.
      Robotics 21(6). doi:10.1109/TRO.2005.853567
  [3] Murphy, R.R. (2000). "Marsupial and shape-shifting robots for
      urban search and rescue." IEEE Intelligent Systems 15(2), pp.14-19.
  [4] Madgwick, S.O.H. et al. (2011). IEEE ICORR. doi:10.1109/ICORR.2011.5975346
  [5] Papadakis, P. (2013). Eng. Appl. AI 26(8):1373-1397.

Topics:
  Sub: /imu/data           sensor_msgs/Imu        (100 Hz)
  Sub: /scan               sensor_msgs/LaserScan   (5.5 Hz, RPLidar A1)
  Sub: /cmd_vel_raw        geometry_msgs/Twist     (operator commands)
  Sub: /terrain/risk       std_msgs/Float32        (optional, from traversability_node)
  Pub: /cmd_vel            geometry_msgs/Twist     (scaled, safe commands)
  Pub: /terrain/features   std_msgs/Float32MultiArray (feature vector φ for logging)
"""

import math
import collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg      import Imu, LaserScan
from geometry_msgs.msg    import Twist, Quaternion
from std_msgs.msg         import Float32, Float32MultiArray

# ── Optional ML backends ───────────────────────────────────────────────────
_TFLITE_OK = False
_ONNX_OK   = False
try:
    import numpy as np
    import tflite_runtime.interpreter as tflite
    _TFLITE_OK = True
except ImportError:
    pass
try:
    import numpy as np
    import onnxruntime as ort
    _ONNX_OK = True
except ImportError:
    pass
if not _TFLITE_OK and not _ONNX_OK:
    try:
        import numpy as np
        _NP_OK = True
    except ImportError:
        _NP_OK = False


def _quat_to_euler(q: Quaternion):
    """ZYX Euler from geometry_msgs Quaternion. Returns (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll  = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sp    = max(-1.0, min(1.0, 2.0*(w*y - z*x)))
    pitch = math.asin(sp)
    yaw   = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return roll, pitch, yaw


def _scan_entropy(ranges, bins=16, r_max=12.0):
    """
    Shannon entropy of the range histogram [nats].
    Uniform distribution → high entropy (open space).
    Peaked distribution → low entropy (structured environment).
    """
    step = r_max / bins
    hist = [0] * bins
    n = 0
    for r in ranges:
        if 0 < r < r_max:
            idx = min(int(r / step), bins - 1)
            hist[idx] += 1
            n += 1
    if n == 0:
        return 0.0
    H = 0.0
    for c in hist:
        if c > 0:
            p = c / n
            H -= p * math.log(p)
    return H


class TerrainAINode(Node):
    """
    Terrain-aware velocity safety supervisor.
    Runs in heuristic mode until a model file is provided via 'model_path'.
    """

    # Feature vector index constants
    F_PITCH      = 0
    F_ROLL       = 1
    F_OMEGA2     = 2
    F_OMEGA_VAR  = 3
    F_ACCEL_LAT  = 4
    F_MIN_FWD    = 5
    F_ENTROPY    = 6
    F_EXT_RISK   = 7
    FEAT_DIM     = 8

    def __init__(self):
        super().__init__('terrain_ai')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('imu_topic',      '/imu/data')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('cmd_in_topic',   '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic',  '/cmd_vel')
        self.declare_parameter('risk_topic',     '/terrain/risk')

        self.declare_parameter('vmax',           0.0445)   # 44.5 mm/s
        self.declare_parameter('alpha_vel',      0.85)     # linear scale
        self.declare_parameter('beta_yaw',       0.60)     # angular scale
        self.declare_parameter('r_stop',         0.85)     # emergency stop

        self.declare_parameter('max_slope_rad',  0.524)    # 30°
        self.declare_parameter('safe_dist_m',    0.50)
        self.declare_parameter('fwd_cone_deg',   30.0)
        self.declare_parameter('omega_var_max',  0.01)
        self.declare_parameter('roughness_window', 20)

        self.declare_parameter('model_path',     '')       # '' = heuristic mode

        imu_topic   = self.get_parameter('imu_topic').value
        scan_topic  = self.get_parameter('scan_topic').value
        cmd_in      = self.get_parameter('cmd_in_topic').value
        cmd_out     = self.get_parameter('cmd_out_topic').value
        risk_topic  = self.get_parameter('risk_topic').value
        self.vmax        = self.get_parameter('vmax').value
        self.alpha_v     = self.get_parameter('alpha_vel').value
        self.beta_w      = self.get_parameter('beta_yaw').value
        self.r_stop      = self.get_parameter('r_stop').value
        self.max_slope   = self.get_parameter('max_slope_rad').value
        self.safe_dist   = self.get_parameter('safe_dist_m').value
        self.fwd_cone    = math.radians(self.get_parameter('fwd_cone_deg').value)
        self.ov_max      = self.get_parameter('omega_var_max').value
        win              = self.get_parameter('roughness_window').value
        model_path       = self.get_parameter('model_path').value

        # ── State ─────────────────────────────────────────────────────
        self.phi       = [0.0] * self.FEAT_DIM   # feature vector
        self.roll      = 0.0
        self.pitch     = 0.0
        self.omega_buf = collections.deque(maxlen=win)
        self.ax_buf    = collections.deque(maxlen=win)
        self.scan      = None
        self.ext_risk  = 0.0
        self.risk_ema  = 0.0     # smoothed risk

        # ── Load ML model ─────────────────────────────────────────────
        self.model = None
        self.mode  = 'heuristic'
        if model_path:
            self._load_model(model_path)

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(Imu,       imu_topic,  self._on_imu,     sensor_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan,    sensor_qos)
        self.create_subscription(Float32,   risk_topic, self._on_ext_risk, 10)
        self.create_subscription(Twist,     cmd_in,     self._on_cmd,     10)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_cmd  = self.create_publisher(Twist,             cmd_out,           10)
        self.pub_feat = self.create_publisher(Float32MultiArray, '/terrain/features', 10)

        self.get_logger().info(
            f'terrain_ai ready  mode={self.mode}  vmax={self.vmax*1000:.1f}mm/s')

    # ── Model loader ──────────────────────────────────────────────────
    def _load_model(self, path: str):
        if path.endswith('.tflite') and _TFLITE_OK:
            try:
                self.model = tflite.Interpreter(model_path=path)
                self.model.allocate_tensors()
                self.mode = 'tflite'
                self.get_logger().info(f'TFLite model loaded: {path}')
            except Exception as e:
                self.get_logger().error(f'TFLite load failed: {e}  → heuristic mode')
        elif path.endswith('.onnx') and _ONNX_OK:
            try:
                self.model = ort.InferenceSession(path)
                self.mode  = 'onnx'
                self.get_logger().info(f'ONNX model loaded: {path}')
            except Exception as e:
                self.get_logger().error(f'ONNX load failed: {e}  → heuristic mode')
        else:
            self.get_logger().warn(
                f'model_path={path!r} set but backend unavailable → heuristic mode')

    # ── IMU callback ──────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        self.roll, self.pitch, _ = _quat_to_euler(msg.orientation)
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        om2 = gx*gx + gy*gy + gz*gz
        self.omega_buf.append(om2)
        self.ax_buf.append(abs(msg.linear_acceleration.y))   # lateral accel

        # Update feature vector channels computed from IMU
        self.phi[self.F_PITCH]  = self.pitch
        self.phi[self.F_ROLL]   = self.roll
        self.phi[self.F_OMEGA2] = om2
        if len(self.omega_buf) >= 2:
            om = list(self.omega_buf)
            mu = sum(om) / len(om)
            self.phi[self.F_OMEGA_VAR] = sum((v-mu)**2 for v in om) / len(om)
        self.phi[self.F_ACCEL_LAT] = (sum(self.ax_buf) / len(self.ax_buf)
                                      if self.ax_buf else 0.0)

    # ── Scan callback ─────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        fwd, valid = [], []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                valid.append(r)
                if abs(angle) <= self.fwd_cone:
                    fwd.append(r)
            angle += msg.angle_increment
        self.phi[self.F_MIN_FWD] = min(fwd) if fwd else self.safe_dist
        self.phi[self.F_ENTROPY] = _scan_entropy(valid)

    # ── External risk callback ─────────────────────────────────────────
    def _on_ext_risk(self, msg: Float32):
        self.ext_risk = float(msg.data)
        self.phi[self.F_EXT_RISK] = self.ext_risk

    # ── Command velocity callback ─────────────────────────────────────
    def _on_cmd(self, msg: Twist):
        risk = self._compute_risk()

        # ── Velocity scaling ──────────────────────────────────────────
        if risk >= self.r_stop:
            out = Twist()   # full stop
        else:
            scale_v = max(0.0, 1.0 - self.alpha_v * risk)
            scale_w = max(0.0, 1.0 - self.beta_w  * risk)
            out = Twist()
            # Clamp linear velocity to vmax (RMCS-3070 limit)
            raw_v = max(-self.vmax, min(self.vmax, msg.linear.x))
            out.linear.x  = raw_v * scale_v
            out.angular.z = msg.angular.z * scale_w

        self.pub_cmd.publish(out)

        # ── Publish feature vector for logging/analysis ───────────────
        fm = Float32MultiArray()
        fm.data = [float(v) for v in self.phi]
        self.pub_feat.publish(fm)

    # ── Risk estimation ───────────────────────────────────────────────
    def _compute_risk(self) -> float:
        """
        Returns composite terrain risk in [0, 1].
        Delegates to ML model if loaded, otherwise uses heuristic.
        """
        if self.mode == 'tflite' and self.model is not None:
            return self._infer_tflite()
        elif self.mode == 'onnx' and self.model is not None:
            return self._infer_onnx()
        else:
            return self._heuristic_risk()

    def _heuristic_risk(self) -> float:
        """
        Transparent heuristic risk score — same weights as traversability_node
        so both nodes stay consistent.  [Papadakis, 2013]
        """
        pitch, roll = self.phi[self.F_PITCH], self.phi[self.F_ROLL]
        inclination = math.sqrt(pitch**2 + roll**2)
        R_s = min(1.0, inclination / self.max_slope)

        R_r = min(1.0, self.phi[self.F_OMEGA_VAR] / self.ov_max)

        min_fwd = self.phi[self.F_MIN_FWD]
        R_p = max(0.0, min(1.0, 1.0 - min_fwd / self.safe_dist))

        # If traversability_node is running, blend its risk in
        R_e = self.phi[self.F_EXT_RISK]

        # Combined (weights match traversability_node defaults)
        T = 0.35*R_s + 0.15*R_r + 0.35*R_p + 0.15*R_e

        # EMA smoothing (α=0.4)
        self.risk_ema = 0.4*T + 0.6*self.risk_ema
        return self.risk_ema

    def _infer_tflite(self) -> float:
        """
        Run feature vector φ through TFLite model.
        Model must accept float32[1, FEAT_DIM] and output float32[1,1].
        """
        try:
            inp_idx = self.model.get_input_details()[0]['index']
            out_idx = self.model.get_output_details()[0]['index']
            inp = np.array([self.phi], dtype=np.float32)
            self.model.set_tensor(inp_idx, inp)
            self.model.invoke()
            return float(np.clip(self.model.get_tensor(out_idx)[0][0], 0.0, 1.0))
        except Exception as e:
            self.get_logger().warn(f'TFLite inference error: {e}')
            return self._heuristic_risk()

    def _infer_onnx(self) -> float:
        """
        Run feature vector φ through ONNX model.
        Model must accept float32[1, FEAT_DIM] and output float32[1,1].
        """
        try:
            inp_name = self.model.get_inputs()[0].name
            inp = np.array([self.phi], dtype=np.float32)
            out = self.model.run(None, {inp_name: inp})
            return float(np.clip(out[0][0][0], 0.0, 1.0))
        except Exception as e:
            self.get_logger().warn(f'ONNX inference error: {e}')
            return self._heuristic_risk()


def main():
    rclpy.init()
    node = TerrainAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
