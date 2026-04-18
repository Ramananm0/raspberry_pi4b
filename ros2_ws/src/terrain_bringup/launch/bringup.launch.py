"""
bringup.launch.py  —  Full Raspberry Pi 4B hardware launch
                       terrain_bot (STM32F7 + RPLidar A1 + Camera)

Start order and data flow:
──────────────────────────────────────────────────────────────────────
  STM32F746G  ──USB-TTL──►  micro_ros_agent
                                ├─► /imu/data        (100 Hz)
                                ├─► /wheel_ticks     ( 50 Hz)
                                └─► /wheel_velocity  ( 50 Hz)

  RPLidar A1  ──USB──────────►  rplidar_node  ──► /scan  (5.5 Hz)

  /wheel_ticks  ──► terrain_odom  ──► /odom + TF odom→base_footprint
  /odom + /imu/data  ──► EKF  ──► /odometry/filtered

  /odometry/filtered + /scan  ──► slam_toolbox  ──► /map

  /imu/data + /scan  ──► terrain_traversability  ──► /terrain/risk
                                                  ──► /terrain/costmap

  /camera/image_raw  ──► terrain_ai_node (EfficientNet-B4 @ 0.2 Hz)
                              ──► /terrain/detected
                              ──► /terrain/speed_factor
                              ──► /terrain/safety_level
                              ──► /terrain/flag

  /terrain/detected  ──► terrain_path_node  ──► /cmd_vel_raw (autonomous)
  teleop keyboard                           ──► /cmd_vel_raw (manual)

  /cmd_vel_raw + /imu/data + /scan + /terrain/speed_factor
      ──► terrain_risk_node  ──► /cmd_vel  ──► STM32 motors

  all topics  ──► data_logger  ──► ~/terrain_ws/logs/terrain_data_*.csv
──────────────────────────────────────────────────────────────────────
Teleop vs autonomous:
  Both teleop and terrain_path_node publish to /cmd_vel_raw.
  Use teleop for manual driving — it overrides autonomous commands
  when keys are held. Release keys to let terrain_path_node steer.
  terrain_risk_node ALWAYS applies safety scaling regardless of source.
──────────────────────────────────────────────────────────────────────
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

    # ── 1. micro-ROS agent  (STM32 ↔ ROS2 bridge) ─────────────────────────────
    microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'microros_agent.launch.py')))

    # ── 2. RPLidar A1 ─────────────────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[os.path.join(bringup_pkg, 'config', 'rplidar_a1.yaml')],
    )

    # ── 3. Robot State Publisher (URDF → TF) ───────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str),
        }],
    )

    # ── 4. Wheel odometry  (/wheel_ticks → /odom + TF) ────────────────────────
    odom = Node(
        package='terrain_odom',
        executable='odom_node',
        name='terrain_odom',
        output='screen',
        parameters=[{
            'use_sim_time':      False,
            'ticks_topic':       '/wheel_ticks',
            'odom_topic':        '/odom',
            'base_frame':        'base_footprint',
            'odom_frame':        'odom',
            'wheel_diameter_mm': 8.5,
            'ticks_per_rev':     2264,
            'wheel_base_mm':     200.0,
        }],
    )

    # ── 5. EKF  (/odom + /imu/data → /odometry/filtered) ─────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # ── 6. SLAM Toolbox  (/odometry/filtered + /scan → /map) ──────────────────
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

    # ── 7. Traversability  (/imu/data + /scan → /terrain/risk + costmap) ──────
    traversability = Node(
        package='terrain_traversability',
        executable='traversability_node',
        name='terrain_traversability',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'max_slope_rad': 0.524,    # 30°
            'safe_dist_m':   0.50,
            'fwd_cone_deg':  30.0,
            'w_slope':       0.40,
            'w_roughness':   0.20,
            'w_proximity':   0.40,
        }],
    )

    # ── 8. Camera Module 3  (picamera2/libcamera → /camera/image_raw) ───────────
    # IMX708 unicam needs media-controller pipeline; use picamera2 directly.
    # PYTHONPATH must include /usr/local/lib/python3/dist-packages so that
    # the RPi-fork libcamera Python bindings (not Ubuntu's empty stub) are found.
    _cam_env = dict(os.environ)
    _cam_env['PYTHONPATH'] = (
        '/usr/local/lib/python3/dist-packages:'
        '/usr/local/lib/python3.10/dist-packages:'
        + _cam_env.get('PYTHONPATH', '')
    )
    camera = Node(
        package='terrain_ai',
        executable='terrain_camera_node',
        name='terrain_camera_node',
        output='screen',
        env=_cam_env,
        parameters=[{
            'width':         640,
            'height':        480,
            'fps':           15,
            'camera_topic':  '/camera/image_raw',
        }],
    )

    # ── 9. Terrain AI  (/camera/image_raw → EfficientNet-B4 → /terrain/*) ─────
    # rate_hz=0.2: B4 3×3 grid takes ~5s on Pi 4B ARM (ONNX Runtime, no GPU)
    terrain_ai = Node(
        package='terrain_ai',
        executable='terrain_ai_node',
        name='terrain_ai_node',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'camera_topic':  '/camera/image_raw',
            'rate_hz':       0.2,
            'model_dir':     os.path.expanduser('~/terrain_dataset/model'),
        }],
    )

    # ── 10. Terrain path planner  (/terrain/detected → /cmd_vel_raw) ──────────
    terrain_path = Node(
        package='terrain_ai',
        executable='terrain_path_node',
        name='terrain_path_node',
        output='screen',
        parameters=[{
            'use_sim_time':   False,
            'cmd_topic':      '/cmd_vel_raw',
            'max_speed':      0.04,     # conservative: ~40 mm/s
            'max_turn_rate':  0.5,
            'steer_deadband': 1.5,
        }],
    )

    # ── 11. Risk supervisor  (/cmd_vel_raw → safety filter → /cmd_vel) ─────────
    # w_camera_risk=0.25: camera AI contributes 25% to speed scaling
    # Set w_camera_risk=0.0 to disable camera AI influence on speed
    terrain_risk = Node(
        package='terrain_risk_layer',
        executable='terrain_risk_node',
        name='terrain_risk_node',
        output='screen',
        parameters=[{
            'use_sim_time':     False,
            'imu_topic':        '/imu/data',
            'scan_topic':       '/scan',
            'cmd_in_topic':     '/cmd_vel_raw',
            'cmd_out_topic':    '/cmd_vel',
            'vmax':             0.0445,    # 44.5 mm/s hardware limit
            'max_roll_deg':     30.0,
            'max_pitch_deg':    35.0,
            'resume_roll_deg':  25.0,
            'resume_pitch_deg': 28.0,
            'safe_slope_deg':   10.0,
            'danger_slope_deg': 30.0,
            'alpha_fusion':     0.7,       # 70% IMU, 30% LiDAR
            'w_proximity':      0.5,
            'w_roughness':      0.3,
            'w_freespace':      0.2,
            'd_safe':           0.4,
            'w_camera_risk':    0.25,      # camera AI blended in at 25%
        }],
    )

    # ── 12. Data logger ────────────────────────────────────────────────────────
    data_logger = Node(
        package='terrain_risk_layer',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        # Hardware bridges first
        microros,
        rplidar,
        rsp,
        camera,

        # Wait for micro-ROS to connect STM32 before starting dependent nodes
        TimerAction(period=2.0, actions=[odom]),
        TimerAction(period=2.5, actions=[ekf]),

        # SLAM + traversability after odom/EKF are running
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=3.0, actions=[traversability]),

        # AI stack after sensors are ready
        TimerAction(period=3.5, actions=[terrain_ai]),
        TimerAction(period=4.0, actions=[terrain_path]),

        # Risk supervisor last (needs IMU + scan + camera speed_factor)
        TimerAction(period=4.0, actions=[terrain_risk]),
        TimerAction(period=4.5, actions=[data_logger]),
        # Manual teleop: SSH in and run:
        #   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        #        --ros-args -r /cmd_vel:=/cmd_vel_raw
    ])
