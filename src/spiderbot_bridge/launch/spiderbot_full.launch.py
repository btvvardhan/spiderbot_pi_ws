#!/usr/bin/env python3
"""
Complete Spiderbot Launch File for Raspberry Pi
- Starts YDLidar driver
- Starts Pi bridge node (Arduino communication + IMU)
- Republishes all sensor data to network
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='YDLidar serial port'
    )

    # YDLidar launch file
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ])
    )

    # Pi Bridge Node
    pi_bridge_node = Node(
        package='spiderbot_bridge',
        executable='pi_bridge_node',
        name='pi_bridge_node',
        output='screen',
        parameters=[{
            'arduino_port': LaunchConfiguration('arduino_port'),
            'baudrate': 115200,
            'deg_offset': 90.0,
            'servo_min_deg': 0.0,
            'servo_max_deg': 180.0,
        }],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        arduino_port_arg,
        lidar_port_arg,
        ydlidar_launch,
        pi_bridge_node,
    ])