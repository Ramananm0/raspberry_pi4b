#!/usr/bin/env python3
"""
Terrain Path Node (ROS 2)

Reads the per-patch terrain grid from /terrain/detected and computes
the safest column to drive through (Left / Centre / Right).
Publishes steering + speed commands to /cmd_vel_raw, which
terrain_risk_node then filters through its IMU+LiDAR safety layer
before forwarding to /cmd_vel.

Column cost model
-----------------
Each grid cell carries a safety level. Cells closer to the rover
(bottom rows) are weighted more heavily. The column with the lowest
accumulated cost is selected. Steering is proportional to cost gap.

         near row (row 2) weight × 3
         mid  row (row 1) weight × 2
         far  row (row 0) weight × 1

Safety costs: SAFE=0  CAUTION=1  UNSAFE=5  STOP=20

Steering: left=+angular.z  right=-angular.z  (ROS convention)

Parameters:
  cmd_topic       : topic to publish commands  (default /cmd_vel_raw)
  max_speed       : max linear speed m/s        (default 0.5)
  max_turn_rate   : max angular rate rad/s      (default 0.6)
  steer_deadband  : min cost gap to trigger turn (default 1.5)

Published:
  /cmd_vel_raw              geometry_msgs/Twist
  /terrain/path_decision    std_msgs/String  JSON diagnostic

Subscribed:
  /terrain/detected         std_msgs/String  JSON from terrain_ai_node
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

SAFETY_COST = {'SAFE': 0, 'CAUTION': 1, 'UNSAFE': 5, 'STOP': 20}
ROW_WEIGHT  = {0: 1, 1: 2, 2: 3}
COL_LEFT    = 0
COL_CENTRE  = 1
COL_RIGHT   = 2
ESTOP_THRESHOLD      = 18
FORCE_TURN_THRESHOLD = 8


def _column_costs(grid_cells, n_cols=3):
    costs = {c: 0.0 for c in range(n_cols)}
    for cell in grid_cells:
        lvl  = cell.get('safety', 'CAUTION')
        cost = SAFETY_COST.get(lvl, 5) * ROW_WEIGHT.get(cell['row'], 1)
        costs[cell['col']] += cost
    return costs


def _steer_gain(centre_cost, best_cost):
    return min(1.0, (centre_cost - best_cost) / 10.0)


def _decide(costs, speed_factor, max_speed, max_turn_rate, deadband):
    left   = costs[COL_LEFT]
    centre = costs[COL_CENTRE]
    right  = costs[COL_RIGHT]
    best   = min(left, centre, right)

    debug = {'col_costs': {'left': left, 'centre': centre, 'right': right},
             'best_cost': best}

    if best >= ESTOP_THRESHOLD:
        debug['action'] = 'ESTOP'
        return 0.0, 0.0, 'ESTOP', debug

    linear_x = max_speed * speed_factor

    if centre == best or (centre - best) < deadband:
        if centre >= FORCE_TURN_THRESHOLD:
            if left <= right:
                g = _steer_gain(centre, left)
                debug['action'] = f'TURN_LEFT (force, gap={centre-left:.1f})'
                return linear_x * 0.5, max_turn_rate * g, 'TURN_LEFT', debug
            else:
                g = _steer_gain(centre, right)
                debug['action'] = f'TURN_RIGHT (force, gap={centre-right:.1f})'
                return linear_x * 0.5, -max_turn_rate * g, 'TURN_RIGHT', debug
        debug['action'] = 'STRAIGHT'
        return linear_x, 0.0, 'STRAIGHT', debug

    if left < right:
        g   = _steer_gain(centre, left)
        spd = linear_x * max(0.3, 1.0 - g * 0.5)
        debug['action'] = f'TURN_LEFT (gap={centre-left:.1f})'
        return spd, max_turn_rate * g, 'TURN_LEFT', debug

    g   = _steer_gain(centre, right)
    spd = linear_x * max(0.3, 1.0 - g * 0.5)
    debug['action'] = f'TURN_RIGHT (gap={centre-right:.1f})'
    return spd, -max_turn_rate * g, 'TURN_RIGHT', debug


class TerrainPathNode(Node):
    def __init__(self):
        super().__init__('terrain_path_node')

        self.declare_parameter('cmd_topic',     '/cmd_vel_raw')
        self.declare_parameter('max_speed',      0.5)
        self.declare_parameter('max_turn_rate',  0.6)
        self.declare_parameter('steer_deadband', 1.5)

        cmd_topic  = self.get_parameter('cmd_topic').value
        self.vmax  = self.get_parameter('max_speed').value
        self.wmax  = self.get_parameter('max_turn_rate').value
        self.dband = self.get_parameter('steer_deadband').value

        self.pub_cmd  = self.create_publisher(Twist,  cmd_topic,               1)
        self.pub_path = self.create_publisher(String, '/terrain/path_decision', 1)
        self.create_subscription(String, '/terrain/detected', self._terrain_cb, 1)

        self.get_logger().info(
            f'terrain_path_node STARTED — publishing to {cmd_topic}')

    def _terrain_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Bad JSON on /terrain/detected')
            return

        grid_cells   = data.get('grid', [])
        speed_factor = data.get('speed_factor', 1.0)
        safety_level = data.get('safety_level', 'CAUTION')

        if not grid_cells:
            return

        costs = _column_costs(grid_cells)
        lin, ang, action, debug = _decide(
            costs, speed_factor, self.vmax, self.wmax, self.dband)

        twist           = Twist()
        twist.linear.x  = lin
        twist.angular.z = ang
        self.pub_cmd.publish(twist)

        self.pub_path.publish(String(data=json.dumps({
            'action':       action,
            'linear_x':     round(lin, 3),
            'angular_z':    round(ang, 3),
            'col_costs':    debug['col_costs'],
            'safety_level': safety_level,
            'speed_factor': speed_factor,
            'detail':       debug['action'],
        })))
        self.get_logger().info(
            f'{debug["action"]}  lin={lin:.2f}  ang={ang:+.2f}  [{safety_level}]')


def main():
    rclpy.init()
    node = TerrainPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
