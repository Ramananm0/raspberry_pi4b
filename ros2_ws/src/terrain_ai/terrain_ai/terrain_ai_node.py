#!/usr/bin/env python3
"""
Terrain AI Node (ROS 2)

Subscribes to the camera, runs patch-grid EfficientNet-B4 terrain
classification at a fixed rate, and publishes safety level, speed factor,
and full terrain detail for downstream nodes.

Parameters:
  model_dir     : path to trained model dir  (default ~/terrain_dataset/model)
  camera_topic  : camera topic to subscribe  (default /camera/image_raw)
  rate_hz       : inference rate in Hz       (default 3.0)

Published topics:
  /terrain/safety_level   std_msgs/String   SAFE | CAUTION | UNSAFE | STOP
  /terrain/speed_factor   std_msgs/Float32  0.0 – 1.0
  /terrain/flag           std_msgs/Bool     True if UNSAFE or STOP
  /terrain/detected       std_msgs/String   JSON: full grid + all_terrains + message
"""

import os
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

import numpy as np
from PIL import Image as PILImage

from terrain_ai.terrain_inference import TerrainGridEngine

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = True
except ImportError:
    _CV_BRIDGE = False

_MODEL_DIR_DEFAULT = os.path.expanduser('~/terrain_dataset/model')


class TerrainAINode(Node):
    def __init__(self):
        super().__init__('terrain_ai_node')

        self.declare_parameter('model_dir',    _MODEL_DIR_DEFAULT)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('rate_hz',      3.0)

        model_dir    = self.get_parameter('model_dir').value
        camera_topic = self.get_parameter('camera_topic').value
        rate_hz      = self.get_parameter('rate_hz').value

        if _CV_BRIDGE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn(
                'cv_bridge not found — using numpy fallback for image decode')

        try:
            self.engine = TerrainGridEngine(model_dir=model_dir)
            self.get_logger().info(
                f'Model loaded from {model_dir}  '
                f'arch=efficientnet_b4  device={self.engine.device}')
        except Exception as e:
            self.get_logger().error(
                f'Model load failed: {e}\n'
                f'Copy best_model.pth + model_meta.json to {model_dir}\n'
                f'Running in PASSTHROUGH mode.')
            self.engine = None

        self.latest_msg = None

        self.pub_safety  = self.create_publisher(String,  '/terrain/safety_level', 1)
        self.pub_speed   = self.create_publisher(Float32, '/terrain/speed_factor',  1)
        self.pub_flag    = self.create_publisher(Bool,    '/terrain/flag',           1)
        self.pub_details = self.create_publisher(String,  '/terrain/detected',       1)

        self.create_subscription(Image, camera_topic, self._image_cb, 1)
        self.create_timer(1.0 / rate_hz, self._inference_cb)

        self.get_logger().info(
            f'terrain_ai_node STARTED — camera={camera_topic}  rate={rate_hz}Hz')

    def _image_cb(self, msg: Image):
        self.latest_msg = msg

    def _inference_cb(self):
        if self.engine is None:
            self._pub_safe_defaults()
            return
        if self.latest_msg is None:
            return

        msg             = self.latest_msg
        self.latest_msg = None

        pil_img = self._ros_to_pil(msg)
        if pil_img is None:
            return

        result = self.engine.analyse(pil_img)
        self._publish(result)

    def _pub_safe_defaults(self):
        self.pub_safety.publish(String(data='SAFE'))
        self.pub_speed.publish(Float32(data=1.0))
        self.pub_flag.publish(Bool(data=False))
        self.pub_details.publish(String(
            data='{"safety_level":"SAFE","speed_factor":1.0,"flag":false,"message":"passthrough"}'))

    def _ros_to_pil(self, msg: Image):
        try:
            if _CV_BRIDGE and self.bridge:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                return PILImage.fromarray(cv_img)
            data   = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ('bgr8', 'bgra8'):
                img_np = data.reshape((msg.height, msg.width, -1))[:, :, :3]
                img_np = img_np[:, :, ::-1].copy()
            else:
                img_np = data.reshape((msg.height, msg.width, 3))
            return PILImage.fromarray(img_np)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return None

    def _publish(self, result: dict):
        self.pub_safety.publish(String(data=result['safety_level']))
        self.pub_speed.publish(Float32(data=float(result['speed_factor'])))
        self.pub_flag.publish(Bool(data=result['flag']))

        grid_json = [
            {
                'row':      c['row'],
                'col':      c['col'],
                'terrains': c['terrains'],
                'safety':   c['safety'],
                'speed':    c['speed'],
                'bbox':     list(c['bbox']),
            }
            for c in result['grid']
        ]
        payload = json.dumps({
            'grid':         grid_json,
            'all_terrains': result['all_terrains'],
            'safety_level': result['safety_level'],
            'speed_factor': result['speed_factor'],
            'flag':         result['flag'],
            'message':      result['message'],
        })
        self.pub_details.publish(String(data=payload))
        self.get_logger().info(result['message'])


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
