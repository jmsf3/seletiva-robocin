#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sqrt, floor
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Constants
R = 0
G = 1
B = 2

COLORS = {
    "RED": (218, 2, 11),
    "GREEN": (0, 218, 26),
    "BLUE": (2, 53, 218),
    "PINK": (218, 2, 202),
    "WHITE": (218, 218, 218),
}


def detect_color(r, g, b):
    result = None
    min_distance = None
    
    for C in COLORS:
        color = COLORS[C]
        distance = sqrt((r - color[R])**2 + (g - color[G])**2 + (b - color[B])**2)
        
        if min_distance is None or distance < min_distance:
            min_distance = distance
            result = C
    
    return result
            
        
class CameraFeedNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__("camera_feed")
        
        # Create ROS publisher
        self.camera_publisher = self.create_publisher(String, "/line_color", 10)
        
        # Create ROS subscriber
        self.camera_subscriber = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
        
    def camera_callback(self, msg: Image):
        # Define image specifications
        image = msg.data
        width = msg.width
        height = msg.height
        
        # Calculate the (r, g, b) values for the center pixel
        center_pixel = int((3 * (height ** 2) / 2) + (3 * width / 2))
        r = image[center_pixel + R]
        g = image[center_pixel + G]
        b = image[center_pixel + B]
        
        # Detect the line's color
        line_color = String()
        line_color.data = detect_color(r, g, b)
        
        # Publish
        self.camera_publisher.publish(line_color)
        self.get_logger().info(f"({r}, {g}, {b}) -> {line_color.data}")


# Initialize rclpy
rclpy.init()

# Initialize the camera feed node
node = CameraFeedNode()

# Spin the camera feed node
rclpy.spin(node)

# Shutdown rclpy
rclpy.shutdown()
