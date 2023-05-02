#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy import Future
from rclpy.node import Node
from std_msgs.msg import String
from dronekit import connect, mavutil, VehicleMode

# Constants
CMD_DURATION = 0.01 # s
GROUND_ALTITUDE = 0.1 # m
TARGET_ALTITUDE = 2.0 # m
VEHICLE_VELOCITY = 1.0 # m / s


def send_velocity_cmd(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Send command to vehicle
    vehicle.send_mavlink(msg)
    time.sleep(duration)
        

class PilotNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__("robocin_pilot")
        
        # Node attributes
        self.line_color = "GREEN"
        self.previous_line_color = "GREEN"
        
        # Create ROS subscriber
        self.line_color_subscriber = self.create_subscription(String, "/line_color", self.line_color_callback, 10)
        
        # Don't try to arm until autopilot is ready
        self.get_logger().info("Basic pre-arm checks.")
        while not vehicle.is_armable:
            self.get_logger().info("Waiting for vehicle to initialize...")
            time.sleep(1)

        # Copter should arm in GUIDED mode
        self.get_logger().info("Arming motors.")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            self.get_logger().info("Waiting for arming...")
            time.sleep(1)

        # Take off to target altitude
        self.get_logger().info("Taking off!")
        vehicle.simple_takeoff(TARGET_ALTITUDE)
        time.sleep(5)
    
    def line_color_callback(self, msg: String):
        # Update line color
        if (msg.data != "WHITE"):
            self.previous_line_color = self.line_color
            self.line_color = msg.data
        
        # If vehicle is on endpoint start descent
        if self.line_color == "GREEN" and self.previous_line_color == "RED":
            self.get_logger().info("Starting descent...")
            
            while (vehicle.location.global_relative_frame.alt > GROUND_ALTITUDE):
                send_velocity_cmd(0.0, 0.0, VEHICLE_VELOCITY, CMD_DURATION)

            future.set_result(True)
            self.get_logger().info("Vehicle landed successfully!")
            
        # If the line is green move forward
        elif self.line_color == "GREEN":
            self.get_logger().info("Moving forward...")
            send_velocity_cmd(-VEHICLE_VELOCITY, 0.0, 0.0, CMD_DURATION)

        # If the line is blue move rightward
        elif self.line_color == "BLUE":
            self.get_logger().info("Moving rightward...")
            send_velocity_cmd(0.0, -VEHICLE_VELOCITY, 0.0, CMD_DURATION)

        # If the line is pink move backward
        elif self.line_color == "PINK":
            self.get_logger().info("Moving backward...")
            send_velocity_cmd(VEHICLE_VELOCITY, 0.0, 0.0, CMD_DURATION)

        # If the line is red move leftward
        elif self.line_color == "RED":
            self.get_logger().info("Moving leftward...")
            send_velocity_cmd(0.0, VEHICLE_VELOCITY, 0.0, CMD_DURATION)
            
        
# Connect to the vehicle
connection_string = "127.0.0.1:14550"
vehicle = connect(connection_string, wait_ready=True)

# Initialize rclpy
rclpy.init()

# Start pilot node
node = PilotNode()

# Spin pilot node
future = Future()
rclpy.spin_until_future_complete(node, future)

# Desconnect to the vehicle
vehicle.close()

# Shutdown rclpy
rclpy.shutdown()