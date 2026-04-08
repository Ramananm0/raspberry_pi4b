"""
bringup.launch.py  —  Full Raspberry Pi 4B system launch

Start order:
  1. micro-ROS agent      (bridges STM32 → ROS2 via USB-TTL cable)
  2. RPLidar A1           (/scan publisher)
  3. Robot State Publisher (URDF joint/link transforms for SLAM + RViz)
  4. Wheel odometry       (STM32 /wheel_ticks → /odom + /tf)
  5. SLAM Toolbox         (/map + tf map→odom)
  6. AI node              (terrain safety — passthrough until ML model ready)
  7. Teleop               (keyboard drive via /cmd_vel_raw → /cmd_vel)

Topics flow:
  STM32 ──USB-TTL──► /imu/data          → AI node
                   ► /wheel_ticks       → odom_node → /odom
  RPLidar          ► /scan              → SLAM + AI node
  Teleop           ► /cmd_vel_raw       → AI node
  AI node          ► /cmd_vel           → (motor driver / future STM32 control)
"""

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_pkg = get_package_share_directory('terrain_bringup')
    desc_pkg    = get_package_share_directory('terrain_robot_description')

    xacro_file  = os.path.join(desc_pkg, 'urdf', 'terrain_bot.urdf.xacro')

    # ── 1. micro-ROS agent ─────────────────────────────────────────
    microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'microros_agent.launch.py')))

    # ── 2. RPLidar A1 ──────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[os.path.join(bringup_pkg, 'config', 'rplidar_a1.yaml')],
    )

    # ── 3. Robot State Publisher ────────────────────────────────────
    # Publishes URDF link/joint transforms so SLAM can find lidar_link
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str),
        }],
    )

    # ── 4. Wheel odometry ──────────────────────────────────────────
    # Accepts 2 values [LEFT, RIGHT] from STM32 encoder ticks
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
            'wheel_diameter_mm':  8.5,
            'ticks_per_rev':      2264,
            'wheel_base_mm':      200.0,
        }],
    )

    # ── 5. SLAM Toolbox ────────────────────────────────────────────
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'slam_params.yaml'),
            {'use_sim_time': False},
        ],
    )

    # ── 6. AI terrain node ─────────────────────────────────────────
    # Reads /imu/data + /scan + /cmd_vel_raw, publishes /cmd_vel
    ai = Node(
        package='terrain_ai',
        executable='ai_node',
        name='terrain_ai',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'imu_topic':     '/imu/data',
            'scan_topic':    '/scan',
            'cmd_in_topic':  '/cmd_vel_raw',
            'cmd_out_topic': '/cmd_vel',
            'vmax':          0.0445,     # 44.5 mm/s max for RMCS-3070
            'model_path':    '',         # set this when ML model is ready
        }],
    )

    # ── 7. Keyboard teleop ─────────────────────────────────────────
    # Publishes to /cmd_vel_raw → AI node scales it → /cmd_vel
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        prefix='xterm -e',   # opens in separate terminal window
    )

    return LaunchDescription([
        microros,
        rplidar,
        rsp,
        TimerAction(period=2.0, actions=[odom]),    # wait for micro-ROS ready
        TimerAction(period=3.0, actions=[slam]),     # wait for odom + RSP
        TimerAction(period=3.0, actions=[ai]),
        TimerAction(period=4.0, actions=[teleop]),   # last — user drives robot
    ])
