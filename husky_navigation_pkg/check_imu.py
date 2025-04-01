#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

def euler_from_quaternion(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class YawPrinter(Node):
    def __init__(self):
        super().__init__('yaw_printer')

        imu_topic = 'a200_1093/sensors/imu_0/data'
        odom_topic = '/a200_1093/platform/odom/filtered'  

        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(f"Subscribed to IMU: {imu_topic} and Odom: {odom_topic}")

    def imu_callback(self, msg):
        q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_deg = math.degrees(yaw)
        print(f"IMU Yaw: {yaw_deg:.2f}°")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_deg = math.degrees(yaw)
        print(f"Odom Yaw: {yaw_deg:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = YawPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
