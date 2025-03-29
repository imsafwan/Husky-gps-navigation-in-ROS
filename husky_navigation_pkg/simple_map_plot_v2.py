#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import numpy as np
import math
import yaml
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger

def load_namespace():
    yaml_path = os.path.expanduser("~/clearpath/robot.yaml")
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return ""  # Fallback


class MapClickPublisher(Node):
    def __init__(self):
        super().__init__("map_click_publisher")

        # Load namespace
        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}")

        # Publisher for target GPS coordinates
        self.gps_pub = self.create_publisher(NavSatFix, f'{namespace}/target_gps_coord', 10)

        # Subscriber for Husky's GPS
        self.create_subscription(NavSatFix, f'{namespace}/sensors/gps_0/fix', self.husky_gps_callback, 10)

        # Service client
        self.navigation_client = self.create_client(Trigger, f"{namespace}/start_navigation")

        # Map area setup
        map_area = "spot3"
        self.map_bounds = {}

        if map_area == "spot1":
            self.map_bounds["top_left"] = (41.870008, -87.648743)
            self.map_bounds["bottom_right"] = (41.869698, -87.648438)
        elif map_area == "spot2":       
            self.map_bounds["top_left"] = (41.871492, -87.648932)
            self.map_bounds["bottom_right"] = (41.871018, -87.648314)
        elif map_area == "spot3":                                       
            self.map_bounds["top_left"] = (41.873783, -87.649809)
            self.map_bounds["bottom_right"] = (41.873130, -87.649130)

        if map_area == "spot1":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/uic_map_1.png"
        elif map_area == "spot2":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/spot2.png"
        elif map_area == "spot3":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/spot3.png"
        self.map_image = plt.imread(image_path)

        # Variables
        self.husky_position = None  
        self.current_target = None

        # GUI setup
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.map_image)
        plt.title("Click on map to set target GPS")

        # Scaling factors
        img_height, img_width = self.map_image.shape[0], self.map_image.shape[1]
        lat_range = self.map_bounds["top_left"][0] - self.map_bounds["bottom_right"][0]
        lon_range = self.map_bounds["bottom_right"][1] - self.map_bounds["top_left"][1]
        self.lat_per_pixel = lat_range / img_height
        self.lon_per_pixel = lon_range / img_width

        # Markers
        self.husky_marker, = self.ax.plot([], [], marker="s", markersize=8,
                                          markerfacecolor="yellow", markeredgecolor="black",
                                          markeredgewidth=2, label="Husky Position")
        self.target_marker, = self.ax.plot([], [], marker="o", markersize=10, color="red",
                                           markerfacecolor="none", label="Target Position")

        # Events
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Timers
        self.create_timer(0.1, self.update_husky_marker)
        self.create_timer(1.0, self.publish_target_gps)  # Optional continuous publishing

    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            # Convert to GPS
            lat = self.map_bounds["top_left"][0] - event.ydata * self.lat_per_pixel
            lon = self.map_bounds["top_left"][1] + event.xdata * self.lon_per_pixel
            self.get_logger().info(f"Clicked Target: Lat={lat}, Lon={lon}")

            # Update current target
            self.current_target = [lat, lon]
            self.target_marker.set_data([event.xdata], [event.ydata])
            self.fig.canvas.draw_idle()

            # Call navigation service after target is set
            self.call_start_navigation_service()

    def publish_target_gps(self):
        if self.current_target is not None:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "map"
            gps_msg.latitude = self.current_target[0]
            gps_msg.longitude = self.current_target[1]
            gps_msg.altitude = 0.0
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Publishing target: Lat={gps_msg.latitude}, Lon={gps_msg.longitude}")

    def call_start_navigation_service(self):
        if not self.navigation_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Start navigation service not available.")
            return
        request = Trigger.Request()
        future = self.navigation_client.call_async(request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Navigation Started: {response.message}")
            else:
                self.get_logger().error(f"Navigation Service Failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def update_husky_marker(self):
        if self.husky_position is not None:
            lat, lon = self.husky_position
            x_pixel = (lon - self.map_bounds["top_left"][1]) / self.lon_per_pixel
            y_pixel = (self.map_bounds["top_left"][0] - lat) / self.lat_per_pixel
            self.husky_marker.set_data(np.array([x_pixel]), np.array([y_pixel]))
            self.fig.canvas.draw_idle()

    def husky_gps_callback(self, msg):
        self.husky_position = (msg.latitude, msg.longitude)


def main(args=None):
    rclpy.init(args=args)
    node = MapClickPublisher()
    plt.ion()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
