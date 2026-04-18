#!/usr/bin/env python3
"""
camera_node.py — publishes Camera Module 3 (IMX708) frames via picamera2.

Replaces v4l2_camera_node which cannot configure the unicam media-controller
pipeline.  Uses picamera2 (libcamera RPi fork) directly.

Parameters:
  width        : capture width  (default 640)
  height       : capture height (default 480)
  fps          : capture framerate (default 15)
  camera_topic : publish topic  (default /camera/image_raw)
"""

import sys
import unittest.mock

# Headless Pi has no DRM display — mock pykms before picamera2 imports it
for _m in ("kms", "pykms"):
    sys.modules.setdefault(_m, unittest.mock.MagicMock())

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('terrain_camera_node')

        self.declare_parameter('width',        640)
        self.declare_parameter('height',       480)
        self.declare_parameter('fps',          15)
        self.declare_parameter('camera_topic', '/camera/image_raw')

        w     = self.get_parameter('width').value
        h     = self.get_parameter('height').value
        fps   = self.get_parameter('fps').value
        topic = self.get_parameter('camera_topic').value

        from picamera2 import Picamera2
        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={"size": (w, h), "format": "RGB888"},
            controls={"FrameRate": float(fps)},
        )
        self._cam.configure(cfg)
        self._cam.start()
        self.get_logger().info(f'Camera started  {w}×{h} @ {fps}fps → {topic}')

        self._pub = self.create_publisher(Image, topic, 1)
        self._w   = w
        self._h   = h
        self.create_timer(1.0 / fps, self._capture)

    def _capture(self):
        frame = self._cam.capture_array()          # numpy (H,W,3) RGB888
        msg              = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height       = self._h
        msg.width        = self._w
        msg.encoding     = 'rgb8'
        msg.is_bigendian = False
        msg.step         = self._w * 3
        msg.data         = frame.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
        self._cam.stop()
        self._cam.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
