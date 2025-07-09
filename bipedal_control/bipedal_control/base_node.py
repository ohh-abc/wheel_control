import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# 通道取值范围
RCCHANNEL_MIN = 600
RCCHANNEL_MID = 1000
RCCHANNEL_MAX = 1400

class ChassisNode(Node):
    def __init__(self):
        super().__init__('base_node')
        
        # 初始化串口
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # 替换为ESP32的串口号
            baudrate=115200,
            timeout=1
        )
        if not self.ser.is_open:
            self.get_logger().error("无法打开串口!")
            rclpy.shutdown()

        # 订阅/cmd_vel话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # 防止未使用警告

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f"收到速度指令: v={linear_x:.2f}, ω={angular_z:.2f}")
        
        # 初始化通道值
        CH1, CH2 = RCCHANNEL_MID, RCCHANNEL_MID
        CH3, CH4 = 200, 1000  # 固定值

        # 处理转向（独立于线速度）
        if angular_z > 0:    # 左转
            CH1 = RCCHANNEL_MIN
        elif angular_z < 0:  # 右转
            CH1 = RCCHANNEL_MAX

        # 处理前进/后退
        if linear_x > 0:   # 前进
            CH2 = 1400
        elif linear_x < 0:  # 后退
            CH2 = 970
        
        self.get_logger().info(f"通道数据: CH1={CH1}, CH2={CH2}, CH3={CH3}, CH4={CH4}")
        
        # 构造数据字符串
        data = f"{CH1},{CH2},{CH3},{CH4}\n"
        
        # 通过串口发送
        if self.ser.is_open:
            self.ser.write(data.encode('utf-8'))
            self.get_logger().debug(f"发送数据: {data.strip()}")
        else:
            self.get_logger().error("串口未打开!")
            
def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点已关闭")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()