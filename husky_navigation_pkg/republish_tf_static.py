import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class TFStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')

        # Set QoS to match /tf_static
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to /tf_static with correct QoS
        self.subscriber = self.create_subscription(
            TFMessage, '/a200_1093/tf_static', self.tf_callback, qos_profile)

        # Publisher for modified /tf_static with correct QoS
        self.publisher = self.create_publisher(
            TFMessage, '/a200_1093/tf_static', qos_profile)

    def tf_callback(self, msg):
        # Get current ROS2 time
        current_time = self.get_clock().now().to_msg()

        # Update timestamps for all transforms
        for transform in msg.transforms:
            transform.header.stamp = current_time

        # Publish updated tf_static
        self.publisher.publish(msg)
        self.get_logger().info(f"Published tf_static with updated timestamp: {current_time.sec}")

def main():
    rclpy.init()
    node = TFStaticRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()