#!/usr/bin/env python3
"""
odom_node.py  —  Wheel odometry from STM32 encoder ticks

Subscribes : /wheel_ticks  (std_msgs/Int32MultiArray)
             Accepts EITHER:
               2-value format  [LEFT, RIGHT]        ← STM32 sends this
               4-value format  [FL, FR, RL, RR]     ← future 4-encoder upgrade

Publishes  : /odom         (nav_msgs/Odometry)
             /tf            odom → base_footprint

Wheel geometry (matches STM32 encoder.h):
  Diameter  : 8.5 mm   → circumference = 26.70 mm
  Ticks/rev : 2264
  Wheelbase : set via parameter  wheel_base_mm  (default 200 mm)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomNode(Node):
    def __init__(self):
        super().__init__('terrain_odom')

        self.declare_parameter('ticks_topic',       '/wheel_ticks')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('base_frame',        'base_footprint')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('wheel_diameter_mm',  8.5)
        self.declare_parameter('ticks_per_rev',      2264)
        self.declare_parameter('wheel_base_mm',      200.0)

        ticks_topic = self.get_parameter('ticks_topic').value
        odom_topic  = self.get_parameter('odom_topic').value
        self.base   = self.get_parameter('base_frame').value
        self.odom_f = self.get_parameter('odom_frame').value

        d_mm              = self.get_parameter('wheel_diameter_mm').value
        tpr               = self.get_parameter('ticks_per_rev').value
        wb_mm             = self.get_parameter('wheel_base_mm').value
        self.m_per_tick   = (math.pi * d_mm / 1000.0) / tpr   # metres/tick
        self.wb_m         = wb_mm / 1000.0                     # metres

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.prev_ticks = None   # initialised on first message
        self.first      = True

        self.pub_odom = self.create_publisher(Odometry, odom_topic, 20)
        self.tf_br    = TransformBroadcaster(self)

        self.create_subscription(Int32MultiArray, ticks_topic,
                                 self.on_ticks, 20)
        self.get_logger().info(
            f'odom_node ready  m/tick={self.m_per_tick:.6f}  '
            f'wheelbase={self.wb_m:.3f} m')

    def on_ticks(self, msg: Int32MultiArray):
        # ── Require at least 2 values (LEFT + RIGHT) ──────────────
        if len(msg.data) < 2:
            return

        ticks = list(msg.data)

        # ── Seed on first message ──────────────────────────────────
        if self.first:
            self.prev_ticks = ticks[:]
            self.first = False
            return

        # ── Compute deltas ─────────────────────────────────────────
        n = len(ticks)
        d = [ticks[i] - self.prev_ticks[i] for i in range(n)]
        self.prev_ticks = ticks[:]

        # ── Distance per side (handles both 2-value and 4-value) ──
        if n == 2:
            # STM32 current format: [LEFT, RIGHT]
            left_m  = d[0] * self.m_per_tick
            right_m = d[1] * self.m_per_tick
        else:
            # 4-value format: [FL, FR, RL, RR]
            left_m  = (d[0] + d[2]) / 2.0 * self.m_per_tick
            right_m = (d[1] + d[3]) / 2.0 * self.m_per_tick

        # ── Differential drive kinematics ─────────────────────────
        ds     = (right_m + left_m)  / 2.0
        dtheta = (right_m - left_m)  / self.wb_m

        self.x     += ds * math.cos(self.theta + dtheta / 2.0)
        self.y     += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        now = self.get_clock().now().to_msg()

        # ── Publish /odom ──────────────────────────────────────────
        odom                         = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = self.odom_f
        odom.child_frame_id          = self.base
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.covariance[0]      = 0.001
        odom.pose.covariance[7]      = 0.001
        odom.pose.covariance[35]     = 0.002
        self.pub_odom.publish(odom)

        # ── Broadcast TF odom → base_footprint ────────────────────
        tf                           = TransformStamped()
        tf.header.stamp              = now
        tf.header.frame_id           = self.odom_f
        tf.child_frame_id            = self.base
        tf.transform.translation.x   = self.x
        tf.transform.translation.y   = self.y
        tf.transform.rotation.z      = math.sin(self.theta / 2.0)
        tf.transform.rotation.w      = math.cos(self.theta / 2.0)
        self.tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
