"""
microros_agent.launch.py

Starts the micro-ROS agent that bridges the STM32F746G-DISCO
(connected via USB-to-TTL converter) into ROS2.

USB-to-TTL → /dev/stm32  (fixed by udev rule setup/99-stm32-usb.rules)
Baud rate  : 2 000 000

Topics received FROM STM32:
  /imu/data            sensor_msgs/Imu
  /wheel_ticks         std_msgs/Int32MultiArray
  /wheel_velocity      std_msgs/Float32MultiArray
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # micro_ros_agent is built from source in ~/microros_ws (not in system ROS)
    microros_agent = ExecuteProcess(
        cmd=[
            '/home/ubuntu/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent',
            'serial',
            '--dev', '/dev/stm32',   # udev symlink for USB-to-TTL
            '-b',   '2000000',       # 2 Mbaud — must match STM32 UART1
            '-v4',                   # verbose level (reduce to 0 in production)
        ],
        output='screen',
        name='micro_ros_agent',
    )

    return LaunchDescription([microros_agent])
