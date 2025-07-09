from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_odom_imu',
            executable='serial_odom_imu_node',
            name='serial_odom_imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'frame_odom': 'odom',
                'frame_base': 'base_link',
                'frame_imu': 'imu_link',
            }],
        ),
    ])
