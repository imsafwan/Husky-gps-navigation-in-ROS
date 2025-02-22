#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import utm
from std_msgs.msg import Float64MultiArray 
import yaml
import os

# Load namespace dynamically from YAML
def load_namespace():
    yaml_path = "/home/safwan/clearpath/robot.yaml"  # Path to your YAML file
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return "default_ns"  # Fallback namespace



class GPSNavigation(Node):
    def __init__(self):
        super().__init__('gps_navigation_node')

        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}")
        

        # Create Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, f'{namespace}/navigate_to_pose')

        # Subscribers
        self.map_origin_sub = self.create_subscription(NavSatFix, '/map_origin_gps', self.map_origin_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/target_gps_coord', self.gps_callback, 10)

        # UTM origin reference (Set from a separate topic)
        self.utm_origin_easting = None
        self.utm_origin_northing = None
        self.utm_zone_number = None  # Zone will be set dynamically

        # Goal status tracking
        self.goal_active = False  # Ensures we only process one goal at a time

    def map_origin_callback(self, msg):
        """ Store the fixed map origin GPS coordinates from a separate topic. """
        lat, lon = msg.latitude, msg.longitude
        utm_x, utm_y, zone_number, _ = utm.from_latlon(lat, lon)

        self.utm_origin_easting = utm_x
        self.utm_origin_northing = utm_y
        self.utm_zone_number = zone_number

        self.get_logger().info(f"Set Map Origin: Lat={lat}, Lon={lon}, UTM X={utm_x}, UTM Y={utm_y}")

    def gps_callback(self, msg):
        """ Convert target GPS to UTM and then to Map Frame """
        if self.utm_origin_easting is None:
            self.get_logger().warn("Map origin GPS not received yet! Ignoring target GPS.")
            return

        if self.goal_active:
            self.get_logger().info("Goal is still active. Ignoring new GPS input.")
            return  # Ignore new GPS data until the current goal completes

        lat, lon = msg.latitude, msg.longitude
        self.get_logger().info(f"Received Target GPS: Lat={lat}, Lon={lon}")

        # Convert Target GPS to UTM
        utm_x, utm_y, zone_number, _ = utm.from_latlon(lat, lon)

        # Convert UTM to `map` frame by subtracting the fixed UTM origin
        map_x = utm_x - self.utm_origin_easting
        map_y = utm_y - self.utm_origin_northing

        self.get_logger().info(f"Converted to Map Frame: X={map_x}, Y={map_y}")

        # Send goal to Nav2 in `map` frame
        self.send_navigation_goal(map_x, map_y)

    def send_navigation_goal(self, x, y):
        """ Send goal to Nav2 in `map` frame """
        if self.goal_active:
            return  # Avoid sending multiple goals at once

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        self.get_logger().info(f"Sending Goal to Nav2: X={x}, Y={y}")

        # Mark goal as active
        self.goal_active = True

        # Wait for the action server to be ready
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available! Navigation goal aborted.")
            self.goal_active = False
            self.terminate_node()  # Terminate the node on failure
            return

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Handle goal response from Nav2 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected!")
            self.goal_active = False  # Allow new goals to be sent
            self.terminate_node()  # Terminate the node on failure
            return

        self.get_logger().info("Goal Accepted! Tracking progress...")

        # Get feedback while goal is active
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """ Handle result of navigation goal """
        result = future.result()
        if result:
            self.get_logger().info("Navigation Goal Reached! Shutting down node.")
        else:
            self.get_logger().error("Navigation Failed! Shutting down node.")

        # Reset goal status to allow new commands
        self.goal_active = False

        # Terminate after goal completion
        self.terminate_node()

    def terminate_node(self):
        """ Shutdown the node gracefully """
        self.get_logger().info("Terminating GPS Navigation Node...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    gps_navigation = GPSNavigation()
    
    try:
        rclpy.spin(gps_navigation)  # Run the node until goal is reached or failed
    except KeyboardInterrupt:
        gps_navigation.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        if rclpy.ok():  # Ensure shutdown is only called if still active
            gps_navigation.destroy_node()
            rclpy.shutdown()  # Shutdown only if not already called


if __name__ == "__main__":
    main()

