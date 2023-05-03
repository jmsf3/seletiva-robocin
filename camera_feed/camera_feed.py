#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sqrt, floor
from std_msgs.msg import String
from sensor_msgs.msg import Image

R = 0
G = 1
B = 2
RANGE = 75
COLORS = {"RED": (218, 2, 11), "GREEN": (0, 218, 26), "BLUE": (2, 53, 218), "PINK": (218, 2, 202), "WHITE": (218, 218, 218)}


def detect_color(r, g, b):
    """
    Identify a color as RED, GREEN, BLUE, PINK or WHITE based on it's RGB values.
    """
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
        self.camera_publisher = self.create_publisher(String, "/movement", 10)
        
        # Create ROS subscriber
        self.camera_subscriber = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
        
    def camera_callback(self, msg: Image):
        # Define image specifications
        image = msg.data
        width = msg.width
        height = msg.height
        
        # Calculate theleft and right side colors relative to the image's center
        center = 3 * width * (height // 2) + 3 * (width // 2)
        
        left = center - 3 * RANGE
        left_color = detect_color(image[left + R], image[left + G], image[left + B])
        
        right = center + 3 * RANGE
        right_color = detect_color(image[right + R], image[right + G], image[right + B])
        
        # Define movement
        movement = String()
        
        if left_color == right_color == "WHITE":
            movement.data = "MOVE_FORWARD"
        elif left_color == "WHITE" and right_color != "WHITE":
            movement.data = "TURN_RIGHT"
        elif left_color != "WHITE" and right_color == "WHITE":
            movement.data = "TURN_LEFT"
      
        # Publish
        self.camera_publisher.publish(movement)
        self.get_logger().info(f"MOVEMENT: {movement.data}")


# Initialize rclpy
rclpy.init()

# Initialize the camera feed node
node = CameraFeedNode()

# Spin the camera feed node
rclpy.spin(node)

# Shutdown rclpy
rclpy.shutdown()
