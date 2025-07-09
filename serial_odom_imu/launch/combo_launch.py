from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
        # 1) 串口读数节点
    serial_node=Node(
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
                    'frame_footprint': 'base_footprint'
            }],
        )
        
        # 2) robot_localization 的 EKF 融合节点
    ekf_node=Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/app/wel_ws/src/serial_odom_imu/config/ekf.yaml'],
        )
    return LaunchDescription([
        serial_node,
        ekf_node
    ])