"""
bringup.launch.py  —  Full Raspberry Pi 4B system launch

Starts in order:
  1. micro-ROS agent  (STM32 bridge via USB-to-TTL)
  2. RPLidar A1       (scan publisher)
  3. Wheel odometry   (ticks → /odom + /tf)
  4. SLAM Toolbox     (map + /tf map→odom)
  5. AI node          (terrain inference — placeholder until ML is ready)
"""

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('terrain_bringup')

    # ── 1. micro-ROS agent ──────────────────────────────────────────
    microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'microros_agent.launch.py')))

    # ── 2. RPLidar A1 ───────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'rplidar_a1.yaml')],
    )

    # ── 3. Wheel odometry (from STM32 encoder ticks) ────────────────
    odom = Node(
        package='terrain_odom',
        executable='odom_node',
        name='terrain_odom',
        output='screen',
        parameters=[{
            'use_sim_time':       False,
            'ticks_topic':        '/wheel_ticks',
            'odom_topic':         '/odom',
            'base_frame':         'base_footprint',
            'odom_frame':         'odom',
            # Wheel geometry — matches STM32 encoder.h
            'wheel_diameter_mm':  8.5,
            'ticks_per_rev':      2264,
            'wheel_base_mm':      200.0,   # distance between left and right wheels
        }],
    )

    # ── 4. SLAM Toolbox ─────────────────────────────────────────────
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_params.yaml'),
            {'use_sim_time': False},
        ],
    )

    # ── 5. AI terrain node ──────────────────────────────────────────
    ai = Node(
        package='terrain_ai',
        executable='ai_node',
        name='terrain_ai',
        output='screen',
        parameters=[{
            'use_sim_time':   False,
            'imu_topic':      '/imu/data',
            'scan_topic':     '/scan',
            'cmd_out_topic':  '/cmd_vel',
        }],
    )

    return LaunchDescription([
        microros,
        rplidar,
        TimerAction(period=2.0, actions=[odom]),   # wait for micro-ROS
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=3.0, actions=[ai]),
    ])
