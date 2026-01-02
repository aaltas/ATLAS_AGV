#!/usr/bin/env python3
"""
Teleop launch file
Starts Arduino driver + keyboard teleop
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('atlas_robot')
    
    # Include bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'bringup.launch.py')
        )
    )
    
    # Teleop keyboard node
    teleop_keyboard = Node(
        package='atlas_robot',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
    )
    
    return LaunchDescription([
        bringup_launch,
        teleop_keyboard,
    ])
