import requests
import numpy as np
import math
from PIL import Image
from io import BytesIO
import logging
import matplotlib.pyplot as plt

class SatelliteMapFetcher:
    def __init__(self, origin_gps=(38.041764, -75.373154), half_map_distance=50):
        """
        Initialize the fetcher with a GPS origin and half-map distance in meters.
        The half_map_distance is half of the total map width/height (e.g., 50m for a 100m x 100m area).
        """
        self.logger = logging.getLogger(__name__)
        self.origin_gps = origin_gps
        self.half_map_distance = half_map_distance  # Set this attribute first
        self.map_bounds = self.calculate_map_bounds(self.origin_gps, half_map_distance)
        self.map_image = self.get_satellite_map()

    def calculate_map_bounds(self, origin, distance_m):
        """
        Calculate GPS bounds (top-left & bottom-right) for a given origin.

        :param origin: (lat, lon) tuple of the origin point.
        :param distance_m: Half of the total map width/height (e.g., 50m for a 100m x 100m area).
        :return: A dictionary with "top_left" and "bottom_right" GPS coordinates.
        """
        lat, lon = origin
        earth_radius = 6378137.0  # Earth's radius in meters

        # Calculate the offset in degrees
        lat_offset = (distance_m / earth_radius) * (180 / np.pi)
        lon_offset = (distance_m / (earth_radius * np.cos(np.pi * lat / 180))) * (180 / np.pi)

        return {
            "top_left": (float(lat + lat_offset), float(lon - lon_offset)),
            "bottom_right": (float(lat - lat_offset), float(lon + lon_offset))
        }

    def estimate_zoom_level(self, center_lat, map_width_meters, map_width_px=600):
        """
        Estimate the zoom level using the formula:
            zoom = log2((156543.03392 * cos(latitude)) / meters_per_pixel)
        where meters_per_pixel is map_width_meters divided by map_width_px.

        :param center_lat: Center latitude of the map.
        :param map_width_meters: The total width of the map area in meters.
        :param map_width_px: The width of the map in pixels.
        :return: An integer zoom level.
        """
        meters_per_pixel = map_width_meters / map_width_px
        zoom = math.log2((156543.03392 * math.cos(center_lat * math.pi / 180)) / meters_per_pixel)
        return int(zoom)

    def get_satellite_map(self):
        """
        Fetch a satellite image using Yandex Static Maps based on the computed GPS bounds.

        :return: The satellite image as a NumPy array.
        """
        # Unpack the GPS bounds
        top_left_lat, top_left_lon = self.map_bounds["top_left"]
        bottom_right_lat, bottom_right_lon = self.map_bounds["bottom_right"]

        # Compute the center of the map
        center_lat = (top_left_lat + bottom_right_lat) / 2
        center_lon = (top_left_lon + bottom_right_lon) / 2

        # Use the full map width in meters (100m for a half_map_distance of 50m)
        full_map_width = 2 * self.half_map_distance
        zoom = self.estimate_zoom_level(center_lat, full_map_width, 600)

        # Construct the Yandex Static Maps API URL
        url = f"https://static-maps.yandex.ru/1.x/?ll={center_lon},{center_lat}&z={zoom}&size=600,450&l=sat"
        self.logger.info(f"Fetching map from URL: {url}")

        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()  # Check for HTTP errors

            # Open the image with PIL and convert to RGB if necessary
            img = Image.open(BytesIO(response.content))
            if img.mode != "RGB":
                img = img.convert("RGB")
            img_np = np.array(img)
            return img_np

        except requests.exceptions.RequestException as e:
            self.logger.error(f"Failed to fetch Yandex map: {e}")
            # Return a blank image (450 x 600 with 3 channels) on failure
            return np.zeros((450, 600, 3), dtype=np.uint8)


if __name__ == "__main__":
    # Set up basic logging
    logging.basicConfig(level=logging.INFO)

    # Create an instance of the SatelliteMapFetcher
    fetcher = SatelliteMapFetcher(origin_gps=(38.041764, -75.373154), half_map_distance=30)

    # Print the corner GPS coordinates
    print("Top Left GPS:", fetcher.map_bounds["top_left"])
    print("Bottom Right GPS:", fetcher.map_bounds["bottom_right"])

    # Display the fetched satellite map using Matplotlib
    plt.imshow(fetcher.map_image)
    plt.title("Satellite Map")
    plt.axis("off")
    plt.show()
