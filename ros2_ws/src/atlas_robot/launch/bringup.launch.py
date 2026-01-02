#!/usr/bin/env python3
"""
Bringup launch file
Starts Arduino driver node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.21',
        description='Wheel base distance in meters'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='Maximum speed in m/s'
    )
    
    # Arduino driver node
    arduino_driver = Node(
        package='atlas_robot',
        executable='arduino_driver',
        name='arduino_driver',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'max_speed': LaunchConfiguration('max_speed'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        wheel_base_arg,
        max_speed_arg,
        arduino_driver,
    ])