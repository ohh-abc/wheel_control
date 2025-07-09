from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 定义可配置参数：串口设备路径
    return LaunchDescription([
        # 声明一个启动参数（可在命令行覆盖）
        DeclareLaunchArgument(
            name='serial_port',
            default_value='/dev/ttyUSB0',  # 默认设备
            description='/dev/ttyUSB0'
        ),

        # 定义节点配置
        Node(
            package='bipedal_control',
            executable='base_node',
            name='base_controller',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port')  # 传递参数
            }]
        )
    ])