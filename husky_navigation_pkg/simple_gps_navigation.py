#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import utm
import math
import yaml
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math
import sys

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


def load_namespace():
    yaml_path = "/home/safwan/clearpath/robot.yaml"
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return ""  # Fallback


class GPSNavigator(Node):
    def __init__(self):
        super().__init__('simple_gps_navigation')

        namespace = load_namespace()
        if namespace:
            self.get_logger().info(f"Using namespace: {namespace}")
        else:
            self.get_logger().info("No namespace loaded. Using default topics.")

        # Parameters
        self.declare_parameter("target_lat", 41.86987886952341)
        self.declare_parameter("target_lon", -87.64843775598163)
        self.k_linear = 0.8
        self.k_angular = 1.5
        self.linear_max = 0.8
        self.angular_max = 1.0
        self.arrival_threshold = 0.8

        self.target_lat = self.get_parameter("target_lat").value
        self.target_lon = self.get_parameter("target_lon").value

        # State
        self.current_utm = None
        self.yaw = None

        # Target UTM
        target_utm = utm.from_latlon(self.target_lat, self.target_lon)
        self.target_x, self.target_y = target_utm[0], target_utm[1]

        # Topics
        gps_topic = f"/{namespace}/sensors/gps_0/fix" if namespace else "/gps/fix"
        imu_topic = f"/{namespace}/sensors/imu_0/data" if namespace else "/imu/data"
        cmd_topic = f"/{namespace}/cmd_vel" if namespace else "/cmd_vel"

        # Publishers & Subscribers
        self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.run)

        self.get_logger().info("GPS Navigator Node Started")

    def gps_callback(self, msg):
        utm_coord = utm.from_latlon(msg.latitude, msg.longitude)
        self.current_utm = (utm_coord[0], utm_coord[1])

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.yaw = yaw

    def run(self):
        print("Running navigation loop")
        if self.current_utm and self.yaw is not None:
            dx = self.target_x - self.current_utm[0]
            dy = self.target_y - self.current_utm[1]
            distance = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            cmd = Twist()
            if distance > self.arrival_threshold:
                cmd.linear.x = min(self.k_linear * distance, self.linear_max)
                cmd.angular.z = max(min(self.k_angular * yaw_error, self.angular_max), -self.angular_max)
            else:
                self.get_logger().info("Target Reached. Shutting down the node...")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.timer.cancel()
                rclpy.shutdown()
                return

            print(f"Distance: {distance:.2f}, Yaw Error: {yaw_error:.2f}")
            print(f"Current UTM: {self.current_utm}, Target UTM: ({self.target_x}, {self.target_y})")
            print(f"Command: Linear={cmd.linear.x}, Angular={cmd.angular.z}")

            self.cmd_pub.publish(cmd)
        else:
            print("Waiting for GPS and IMU data...")

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
