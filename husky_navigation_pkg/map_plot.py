import rclpy
from rclpy.node import Node
import subprocess
import os
import numpy as np
import time
from std_msgs.msg import Float64MultiArray  # Message type for sending latitude and longitude
from sensor_msgs.msg import NavSatFix  # Message type for receiving GPS position of Husky
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import yaml 
from io import BytesIO
import requests
import requests
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO
from PIL import Image



# Load namespace dynamically from YAML
def load_namespace():
    yaml_path = "/home/safwan/clearpath/robot.yaml"  # Path to your YAML file
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")


class MapClickPublisher(Node):
    def __init__(self):
        super().__init__("map_click_publisher")

        namespace = load_namespace()
        self.get_logger().info(f"Using namespace: {namespace}")

        # Track gps_navigation.py process
        self.gps_navigation_process = None

        # Publisher for target GPS
        self.gps_pub = self.create_publisher(NavSatFix, "/target_gps_coord", 10)

        # Subscriber for Husky's GPS
        self.create_subscription(NavSatFix, f'{namespace}/sensors/gps_0/fix', self.husky_gps_callback, 10)

        # Load Map
        package_path = get_package_share_directory("husky_navigation_pkg")
        image_path = '/home/safwan/clearpath_ws/src/husky_navigation_pkg/Assets/map.png'

        # Define GPS origin (center of the map)
        self.origin_gps = (38.041764, -75.373154)  # Example: Replace with actual origin

        # Define map bounds dynamically (600m x 600m area centered at origin)
        self.map_bounds = self.calculate_map_bounds(self.origin_gps, 25)
        
        

        # Fetch the satellite map dynamically
        self.map_image = self.get_satellite_map()
        

        self.husky_position = None  
        self.current_target = None  # Store the last clicked GPS coordinates

        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.map_image)
        plt.title("Click on the map to set a target GPS coordinate")

        # Calculate pixel-to-GPS scale factors
        img_height, img_width = self.map_image.shape[0], self.map_image.shape[1]
        lat_range = self.map_bounds["top_left"][0] - self.map_bounds["bottom_right"][0]
        lon_range = self.map_bounds["bottom_right"][1] - self.map_bounds["top_left"][1]
        self.lat_per_pixel = lat_range / img_height
        self.lon_per_pixel = lon_range / img_width

        # Markers
        self.husky_marker, = self.ax.plot([], [], marker="s", markersize=8, markerfacecolor="yellow",
                                          markeredgecolor="black", markeredgewidth=2, label="Husky Position")
        self.target_marker, = self.ax.plot([], [], marker="o", markersize=10, color="red",
                                           markerfacecolor="none", label="Target Position")

        # Connect Click Event
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        # Timer for Husky Position Updates
        self.create_timer(0.1, self.update_husky_marker)

        # Timer to continuously publish the last target GPS
        self.create_timer(1.0, self.publish_target_gps)  # Publishes every 1 second




    def calculate_map_bounds(self, origin, distance_m):
        """
        Calculate GPS bounds (top-left & bottom-right) for a given origin.
        :param origin: (lat, lon) tuple of the origin point
        :param distance_m: Half of the total map width/height (e.g., 300m for a 600m x 600m area)
        """
        lat, lon = origin
        earth_radius = 6378137.0  # Earth's radius in meters
        
        lat_offset = (distance_m / earth_radius) * (180 / np.pi)
        lon_offset = (distance_m / (earth_radius * np.cos(np.pi * lat / 180))) * (180 / np.pi)

        return {
            "top_left": (float(lat + lat_offset), float(lon - lon_offset)),
            "bottom_right": (float(lat - lat_offset), float(lon + lon_offset))
        }

    # Handle Click to Set Target GPS and Restart gps_navigation.py
    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            # Convert pixel to GPS
            
            lat = self.map_bounds["top_left"][0] - event.ydata * self.lat_per_pixel
            lon = self.map_bounds["top_left"][1] + event.xdata * self.lon_per_pixel
            self.get_logger().info(f"Clicked GPS Coordinates: Latitude = {lat}, Longitude = {lon}")

            # Restart gps_navigation.py with the new target
            self.restart_gps_navigation()

            

            # Update the last clicked target
            self.current_target = [lat, lon]

            # Update Target Marker
            self.target_marker.set_data([event.xdata], [event.ydata])
            self.fig.canvas.draw_idle()



    

    

    

    

    def get_satellite_map(self):
        """
        Fetches a satellite image dynamically using Yandex Static Maps based on map bounds.
        :return: Loaded satellite image (numpy array)
        """
        # Extract GPS bounds (convert to floats to avoid np.float64 issues)
        top_left_lat, top_left_lon = float(self.map_bounds["top_left"][0]), float(self.map_bounds["top_left"][1])
        bottom_right_lat, bottom_right_lon = float(self.map_bounds["bottom_right"][0]), float(self.map_bounds["bottom_right"][1])

        # Compute center latitude and longitude
        center_lat = (top_left_lat + bottom_right_lat) / 2
        center_lon = (top_left_lon + bottom_right_lon) / 2

        # Estimate zoom level based on lat/lon difference
        lat_diff = abs(top_left_lat - bottom_right_lat)
        lon_diff = abs(top_left_lon - bottom_right_lon)

        zoom = self.estimate_zoom_level(lat_diff, lon_diff)  # Use a function to calculate the zoom level

        # Yandex Static Maps API (returns satellite images)
        url = f"https://static-maps.yandex.ru/1.x/?ll={center_lon},{center_lat}&z={zoom}&size=600,450&l=sat"

        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()  # Raise error for bad responses

            # 🔹 Detect image format dynamically
            img = Image.open(BytesIO(response.content))  # Open image with PIL
            img_format = img.format.lower()  # Get actual format (e.g., 'jpeg', 'png')

            # Convert to RGB if necessary (avoid errors with grayscale formats)
            if img.mode != "RGB":
                img = img.convert("RGB")

            # Convert to NumPy array for Matplotlib display
            img_np = np.array(img)

            return img_np  # Return image array

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to fetch Yandex map: {e}")
            return np.zeros((450, 600, 3))  # Return a blank image on failure


    def estimate_zoom_level(self, lat_diff, lon_diff):
        """
        Estimate a zoom level based on the GPS bounds difference.
        Yandex uses zoom levels from 1 (global view) to 19 (highest zoom).
        """
        max_diff = max(lat_diff, lon_diff)  # Get the larger difference
        if max_diff > 1:
            return 6  # Very large area (continent level)
        elif max_diff > 0.5:
            return 8  # Country level
        elif max_diff > 0.1:
            return 12  # City level
        elif max_diff > 0.05:
            return 14  # District level
        elif max_diff > 0.01:
            return 16  # Local area
        else:
            return 17  # Small neighborhood (default)






    

    def publish_target_gps(self):
        if self.current_target is not None:
            gps_msg = NavSatFix()
            
            # Set header with timestamp
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "map"  # Adjust if needed
            
            # Assign latitude, longitude, and altitude
            gps_msg.latitude = self.current_target[0]
            gps_msg.longitude = self.current_target[1]
            gps_msg.altitude = 0.0  # Set altitude to 0 unless available
            
            # Publish the message
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Continuously Publishing: Lat={gps_msg.latitude}, Lon={gps_msg.longitude}, Alt={gps_msg.altitude}")


    # Terminate the Existing gps_navigation Process and Start a New One
    def restart_gps_navigation(self):
        # Terminate if running
        if self.gps_navigation_process is not None:
            # Kill any running instance of gps_navigation.py
            self.get_logger().info("Stopping any existing gps_navigation process...")

            # Use `pkill` to forcefully terminate any running instance of `gps_navigation.py`
            subprocess.run(["pkill", "-f", "ros2 run husky_navigation_pkg gps_navigation"], stderr=subprocess.DEVNULL)

            # Ensure previous process is fully stopped before starting a new one
            self.get_logger().info("Waiting for process to terminate...")
            subprocess.run(["sleep", "2"])  # Wait 2 seconds before launching new process

        # Start a new gps_navigation process in a separate terminal
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

    # Handle Husky Position Update
    def update_husky_marker(self):
        if self.husky_position is not None:
            lat, lon = self.husky_position
            x_pixel = (lon - self.map_bounds["top_left"][1]) / self.lon_per_pixel
            y_pixel = (self.map_bounds["top_left"][0] - lat) / self.lat_per_pixel
            self.husky_marker.set_data(np.array([x_pixel]), np.array([y_pixel]))
            self.fig.canvas.draw_idle()

    # Husky GPS Callback
    def husky_gps_callback(self, msg):
        self.husky_position = (msg.latitude, msg.longitude)

def main(args=None):
    rclpy.init(args=args)
    node = MapClickPublisher()

    plt.ion()  # Enable interactive mode

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Process ROS 2 messages
            plt.pause(0.1)  # Forces GUI to refresh
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
