#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os
import numpy as np
import time
from std_msgs.msg import Float64MultiArray  # (Unused; remove if not needed)
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import yaml
from io import BytesIO
import requests
import math
from PIL import Image
import logging

# Load namespace dynamically from YAML
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

        # Load namespace and log it
        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}")

        # Track gps_navigation.py process
        self.gps_navigation_process = None

        # Publisher for target GPS coordinates
        self.gps_pub = self.create_publisher(NavSatFix, f'{namespace}/target_gps_coord', 10)

        # Subscriber for Husky's GPS
        self.create_subscription(NavSatFix, f'{namespace}/sensors/gps_0/fix', self.husky_gps_callback, 10)
        
        map_area = "spot1"  # Default map area
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
        
        
         # T_L =  (41.873783, -87.649809) (spot3), (41.871492, -87.648932)(spot2), (41.870008, -87.648743) (spot1) (latitude, longitude) of top-left corner
         # B_R =(41.873130, -87.649130) (spot3), (41.871018, -87.648314) (spot2), (41.869698, -87.648438) (spot1) (latitude, longitude) of bottom-right corner

        if map_area == "spot1":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/uic_map_1.png"  # Path to my map image
        elif map_area == "spot2":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/spot2.png"
        elif map_area == "spot3":
            image_path = "/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/spot3.png"
        self.map_image = plt.imread(image_path)
        
        # Variables for Husky position and current target (from mouse click)
        self.husky_position = None  
        self.current_target = None

        # Set up Matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.map_image)
        plt.title("Click on the map to set a target GPS coordinate")

        # Calculate pixel-to-GPS scale factors
        img_height, img_width = self.map_image.shape[0], self.map_image.shape[1]
        lat_range = self.map_bounds["top_left"][0] - self.map_bounds["bottom_right"][0]
        lon_range = self.map_bounds["bottom_right"][1] - self.map_bounds["top_left"][1]
        self.lat_per_pixel = lat_range / img_height
        self.lon_per_pixel = lon_range / img_width

        # Set up markers for Husky and target positions
        self.husky_marker, = self.ax.plot([], [], marker="s", markersize=8,
                                          markerfacecolor="yellow", markeredgecolor="black",
                                          markeredgewidth=2, label="Husky Position")
        self.target_marker, = self.ax.plot([], [], marker="o", markersize=10, color="red",
                                           markerfacecolor="none", label="Target Position")

        # Connect the click event
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Timers for updating Husky marker and publishing target GPS
        self.create_timer(0.1, self.update_husky_marker)
        self.create_timer(1.0, self.publish_target_gps)  # Publish target every 1 second

    

    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            # Convert pixel coordinates to GPS coordinates
            lat = self.map_bounds["top_left"][0] - event.ydata * self.lat_per_pixel
            lon = self.map_bounds["top_left"][1] + event.xdata * self.lon_per_pixel
            self.get_logger().info(f"Clicked GPS Coordinates: Latitude = {lat}, Longitude = {lon}")

            # Restart gps_navigation.py with the new target
            self.restart_gps_navigation()

            # Update the current target and marker
            self.current_target = [lat, lon]
            self.target_marker.set_data([event.xdata], [event.ydata])
            self.fig.canvas.draw_idle()

        


    

    def publish_target_gps(self):
        if self.current_target is not None:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "map"
            gps_msg.latitude = self.current_target[0]
            gps_msg.longitude = self.current_target[1]
            gps_msg.altitude = 0.0
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Continuously Publishing: Lat={gps_msg.latitude}, Lon={gps_msg.longitude}, Alt={gps_msg.altitude}")

    def restart_gps_navigation(self):
        # Terminate existing process if running
        if self.gps_navigation_process is not None:
            self.get_logger().info("Stopping any existing gps_navigation process...")
            subprocess.run(["pkill", "-f", "ros2 run husky_navigation_pkg simple_gps_navigation_fix_speed"], stderr=subprocess.DEVNULL)
            self.get_logger().info("Waiting for process to terminate...")
            subprocess.run(["sleep", "2"])
        # Start a new process in a separate terminal
        self.get_logger().info("Launching new gps_navigation process in a separate terminal...")
        if os.name == "nt":  # Windows
            self.gps_navigation_process = subprocess.Popen(
                ["start", "cmd", "/c", "ros2 run husky_navigation_pkg simple_gps_navigation_fix_speed.py"], shell=True
            )
        else:  # Linux/macOS
            self.gps_navigation_process = subprocess.Popen(
                [
                    "gnome-terminal", "--", "bash", "-c",
                    "source /opt/ros/humble/setup.bash && source ~/clearpath_ws/install/setup.bash && ros2 run husky_navigation_pkg simple_gps_navigation_fix_speed; exec bash"
                ]
            )

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
    plt.ion()  # Enable interactive mode
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.1)  # Refresh the GUI
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
