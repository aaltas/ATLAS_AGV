from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlas_robot',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'motor_pin': 33,
                'motor_pwm': 95,
                'angle_compensate': True,
                'scan_frequency': 10.0
            }],
            prefix='sudo -E env PATH=${PATH}'  # Sudo ile çalıştır
        )
    ])