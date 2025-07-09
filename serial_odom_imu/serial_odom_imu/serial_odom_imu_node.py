#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import tf2_ros
import tf_transformations 
import serial
import math

class SerialOdomImuNode(Node):
    def __init__(self):
        super().__init__('serial_odom_imu')
        # 声明并读取参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('frame_imu', 'imu_link')
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value
        self.frame_imu  = self.get_parameter('frame_imu').get_parameter_value().string_value

        # 打开串口
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # 发布者
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.imu_pub  = self.create_publisher(Imu, 'imu/data_raw', 50)

        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定时器：50ms 回调一次
        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            return
        parts = line.split(',')
        if len(parts) != 12:
            self.get_logger().warn(f'Bad line: {line}')
            return

        t_ms, x, y, yaw, v_lin, v_ang, roll, pitch, imu_yaw, gx, gy, gz = map(float, parts)
        stamp = self.get_clock().now().to_msg()

        # 1) 发布 wheel_odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        odom.twist.twist.linear.x  = v_lin
        odom.twist.twist.angular.z = v_ang
        self.odom_pub.publish(odom)

        # 2) 广播 TF odom->base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id  = self.frame_base
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z    = odom.pose.pose.orientation.z
        t.transform.rotation.w    = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

        # 3) 发布 imu/data_raw
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_imu
        # 欧拉角 -> 四元数
        q = tf_transformations.quaternion_from_euler(roll, pitch, imu_yaw)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
