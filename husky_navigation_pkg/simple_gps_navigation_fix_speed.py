#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import utm
import math
import yaml
import sys
import os
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist

def euler_from_quaternion(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def load_namespace():
    yaml_path = os.path.expanduser("~/clearpath/robot.yaml")
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace: {e}")
        return ""

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('simple_gps_navigation')

        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}" if namespace else "No namespace loaded.")

        # Parameters
        self.k_angular = 1.5
        self.angular_max = 1.0
        self.yaw_threshold_deg = 5.0
        self.constant_velocity = 0.5
        self.arrival_threshold = 0.8

        # State
        self.current_utm = None
        self.yaw = None
        self.target_x = None
        self.target_y = None
        self.target_received = False
        self.yaw_alignment_attempted = False

        # Topics
        gps_topic = f"/{namespace}/sensors/gps_0/fix" if namespace else "/gps/fix"
        imu_topic =  f"/{namespace}/platform/odom/filtered" if namespace else "/imu/data" #  when use odom    #f"/{namespace}/sensors/imu_0/data" if namespace else "/imu/data"
        cmd_topic = f"/{namespace}/cmd_vel" if namespace else "/cmd_vel"
        target_topic = f"/{namespace}/target_gps_coord" if namespace else "/target_gps_coord"

        self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(NavSatFix, target_topic, self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.timer = self.create_timer(0.1, self.run)

    def gps_callback(self, msg):
        utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
        self.current_utm = (utm_coord[0], utm_coord[1])

    def imu_callback(self, msg):
        #orientation_q = msg.orientation # WHEN USE IMU
        orientation_q = msg.pose.pose.orientation # when use odom
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.yaw = yaw

    def target_callback(self, msg):
        utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
        self.target_x, self.target_y = utm_coord[0], utm_coord[1]
        self.target_received = True
        self.yaw_alignment_attempted = False  # Reset for every new target
        self.get_logger().info(f"Received target: ({self.target_x}, {self.target_y})")

    def run(self):
        if not self.target_received:
            return
        if self.current_utm is None or self.yaw is None:
            return

        dx = self.target_x - self.current_utm[0]
        dy = self.target_y - self.current_utm[1]
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)
        yaw_error_deg = math.degrees(abs(yaw_error))

        cmd = Twist()

        # Check for arrival
        if distance < self.arrival_threshold:
            self.get_logger().info("Target Reached. Stopping.")
            self.cmd_pub.publish(Twist())
            self.target_received = False
            self.yaw_alignment_attempted = False
            return

        # Do Yaw Alignment ONLY ONCE
        if not self.yaw_alignment_attempted:
            if yaw_error_deg > self.yaw_threshold_deg:
                cmd.angular.z = max(min(self.k_angular * yaw_error, self.angular_max), -self.angular_max)
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f"Aligning Yaw | Error: {yaw_error_deg:.2f}°")
                return  # keep aligning until threshold
            else:
                self.yaw_alignment_attempted = True
                self.get_logger().info("Yaw aligned. Starting navigation.")

        # Navigation
        ux = dx / distance
        uy = dy / distance
        cmd.linear.x = self.constant_velocity * ux
        cmd.linear.y = self.constant_velocity * uy
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = GPSNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
