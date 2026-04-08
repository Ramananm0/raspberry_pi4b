"""
bringup.launch.py  —  Full Raspberry Pi 4B system launch

Start order:
  1. micro-ROS agent      (bridges STM32 → ROS2 via USB-TTL cable)
  2. RPLidar A1           (/scan publisher)
  3. Robot State Publisher (URDF joint/link transforms for SLAM + RViz)
  4. Wheel odometry       (STM32 /wheel_ticks → /odom + /tf)
  5. EKF (robot_localization)  (/odom + /imu/data → /odometry/filtered)
  6. SLAM Toolbox         (/map + tf map→odom)
  7. Traversability node  (/terrain/risk + /terrain/costmap)
  8. AI safety node       (terrain risk → velocity scaling /cmd_vel)
  9. Teleop               (keyboard drive via /cmd_vel_raw → /cmd_vel)

Full topic flow:
  STM32 ──USB-TTL──► /imu/data        → EKF, traversability, AI
                   ► /wheel_ticks     → odom_node → /odom
                   ► /wheel_velocity  → (logging)
  odom_node        ► /odom            → EKF
  EKF              ► /odometry/filtered → SLAM (pose) + AI
  RPLidar          ► /scan            → SLAM, traversability, AI
  traversability   ► /terrain/risk    → AI node
  Teleop           ► /cmd_vel_raw     → AI node
  AI node          ► /cmd_vel         → (future: STM32 motor control)
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

    # ── 5. EKF — fuses /odom + /imu/data → /odometry/filtered ─────
    # Reference: Moore & Stouch, IAS-13, 2014
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # ── 6. SLAM Toolbox ────────────────────────────────────────────
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

    # ── 7. Traversability node ─────────────────────────────────────
    # Fuses /imu/data + /scan → /terrain/risk + /terrain/costmap
    # Reference: Papadakis (2013), Thrun et al. (2006)
    traversability = Node(
        package='terrain_traversability',
        executable='traversability_node',
        name='terrain_traversability',
        output='screen',
        parameters=[{
            'use_sim_time':      False,
            'max_slope_rad':     0.524,     # 30°
            'safe_dist_m':       0.50,
            'fwd_cone_deg':      30.0,
            'w_slope':           0.40,
            'w_roughness':       0.20,
            'w_proximity':       0.40,
        }],
    )

    # ── 8. AI terrain safety node ──────────────────────────────────
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
            'vmax':          0.0445,         # 44.5 mm/s (RMCS-3070 limit)
            'alpha_vel':     0.85,
            'beta_yaw':      0.60,
            'r_stop':        0.85,           # emergency stop threshold
            'model_path':    '',             # set this when ML model ready
        }],
    )

    # ── 9. Keyboard teleop ─────────────────────────────────────────
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        prefix='xterm -e',
    )

    return LaunchDescription([
        microros,
        rplidar,
        rsp,
        TimerAction(period=2.0, actions=[odom]),            # wait for micro-ROS
        TimerAction(period=2.5, actions=[ekf]),             # wait for odom
        TimerAction(period=3.0, actions=[slam]),            # wait for odom+EKF
        TimerAction(period=2.5, actions=[traversability]), # needs IMU+scan
        TimerAction(period=3.0, actions=[ai]),              # needs traversability
        TimerAction(period=4.0, actions=[teleop]),
    ])
