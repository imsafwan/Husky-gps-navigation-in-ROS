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
    yaml_path = "/home/safwan/clearpath/robot.yaml"  # Path to your YAML file
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return ""  # Return empty namespace if loading fails

class MapClickPublisher(Node):
    def __init__(self):
        super().__init__("map_click_publisher")

        # Load namespace and log it
        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}")

        # Track gps_navigation.py process
        self.gps_navigation_process = None

        # Publisher for target GPS coordinates
        self.gps_pub = self.create_publisher(NavSatFix, "/target_gps_coord", 10)

        # Subscriber for Husky's GPS
        self.create_subscription(NavSatFix, f'{namespace}/sensors/gps_0/fix', self.husky_gps_callback, 10)

        # Define GPS origin (center of the map)
        self.origin_gps = (38.041764, -75.373154)  # Replace with your actual origin

        # Define map bounds dynamically (e.g., 300m x 300m area centered at origin)
        self.half_dis = 150  # Half of the total map width/height (150m for 300m x 300m area)
        self.map_bounds = self.calculate_map_bounds(self.origin_gps, self.half_dis)
        
        # Fetch the satellite map dynamically from Yandex
        self.map_image = self.get_satellite_map()
        
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

    def calculate_map_bounds(self, origin, distance_m):
        """
        Calculate GPS bounds (top-left & bottom-right) for a given origin.
        :param origin: (lat, lon) tuple of the origin point.
        :param distance_m: Half of the total map width/height (in meters).
        :return: Dictionary with "top_left" and "bottom_right" GPS coordinates.
        """
        lat, lon = origin
        earth_radius = 6378137.0  # Earth's radius in meters

        # Calculate offsets in degrees
        lat_offset = (distance_m / earth_radius) * (180 / np.pi)
        lon_offset = (distance_m / (earth_radius * np.cos(np.pi * lat / 180))) * (180 / np.pi)

        return {
            "top_left": (float(lat + lat_offset), float(lon - lon_offset)),
            "bottom_right": (float(lat - lat_offset), float(lon + lon_offset))
        }

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

    def get_satellite_map(self):
        """
        Fetch a satellite image using Yandex Static Maps based on the computed GPS bounds.
        :return: Satellite image as a NumPy array.
        """
        # Unpack the bounds
        top_left_lat, top_left_lon = self.map_bounds["top_left"]
        bottom_right_lat, bottom_right_lon = self.map_bounds["bottom_right"]

        # Compute map center
        center_lat = (top_left_lat + bottom_right_lat) / 2
        center_lon = (top_left_lon + bottom_right_lon) / 2

        # Full map width in meters (twice the half distance)
        full_map_width = 2 * self.half_dis
        zoom = self.estimate_zoom_level(center_lat, full_map_width, 600)

        # Construct the URL for Yandex Static Maps
        url = f"https://static-maps.yandex.ru/1.x/?ll={center_lon},{center_lat}&z={zoom}&size=600,450&l=sat"
        self.get_logger().info(f"Fetching map from URL: {url}")

        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()  # Check for HTTP errors
            img = Image.open(BytesIO(response.content))
            if img.mode != "RGB":
                img = img.convert("RGB")
            return np.array(img)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to fetch Yandex map: {e}")
            return np.zeros((450, 600, 3), dtype=np.uint8)
        


    def estimate_zoom_level(self, center_lat, map_width_meters, map_width_px=600):
        """
        Estimate the zoom level using:
            zoom = log2((156543.03392 * cos(latitude)) / meters_per_pixel)
        :param center_lat: Center latitude of the map.
        :param map_width_meters: Total width of the map area in meters.
        :param map_width_px: Width of the map in pixels.
        :return: Integer zoom level.
        """
        meters_per_pixel = map_width_meters / map_width_px
        zoom = math.log2((156543.03392 * math.cos(center_lat * math.pi / 180)) / meters_per_pixel)
        return int(zoom)

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
            subprocess.run(["pkill", "-f", "ros2 run husky_navigation_pkg gps_navigation"], stderr=subprocess.DEVNULL)
            self.get_logger().info("Waiting for process to terminate...")
            subprocess.run(["sleep", "2"])
        # Start a new process in a separate terminal
        self.get_logger().info("Launching new gps_navigation process in a separate terminal...")
        if os.name == "nt":  # Windows
            self.gps_navigation_process = subprocess.Popen(
                ["start", "cmd", "/c", "ros2 run husky_navigation_pkg gps_navigation.py"], shell=True
            )
        else:  # Linux/macOS
            self.gps_navigation_process = subprocess.Popen(
                [
                    "gnome-terminal", "--", "bash", "-c",
                    "source /opt/ros/humble/setup.bash && source ~/clearpath_ws/install/setup.bash && ros2 run husky_navigation_pkg gps_navigation; exec bash"
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
