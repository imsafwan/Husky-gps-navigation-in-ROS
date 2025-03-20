#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Subscriber to get pose from /a200_1093/pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/a200_1093/pose',
            self.pose_callback,
            10)
        
        # Publisher to send initial pose to AMCL
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'a200_1093/initialpose',
            10)

    def pose_callback(self, msg):
        self.get_logger().info("Received pose, setting as AMCL initial pose.")
        
        # Modify header to match AMCL expectations
        msg.header.frame_id = "map"
        
        # Publish the received pose as initial pose
        self.publisher.publish(msg)

        # Shutdown after publishing to prevent continuous updates
        self.get_logger().info("Initial pose set. Shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
