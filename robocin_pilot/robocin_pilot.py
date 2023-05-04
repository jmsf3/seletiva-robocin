#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy import Future
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String
from dronekit import connect, VehicleMode

# Constants
LONG_DURATION = 3.0 # s
STOP_DURATION = 1.0 # s
SHORT_DURATION = 0.01 # s
TARGET_ALTITUDE = 2.00 # m
VEHICLE_VELOCITY = 0.75 # m/s


def send_velocity_cmd(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        90, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)


def condition_yaw(heading, clockwise=True, relative=True):
    """
    Change vehicle's yaw based on a heading value.
    """
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
        
    if clockwise:
        direction=1 #clockwise direction
    else:
        direction=-1 #counterclockwise direction
        
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

        
class PilotNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__("robocin_pilot")
        self.amount_of_turns = 0
        
        # Create ROS subscriber
        self.movement_subscriber = self.create_subscription(String, "/movement", self.movement_callback, 10)
        
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
        
        # Move forward
        self.get_logger().info("Moving forward...")
        self.move_forward(LONG_DURATION)
        
    def stop(self):
        # Stop
        send_velocity_cmd(0.0, 0.0, 0.0)
        condition_yaw(0)
        time.sleep(STOP_DURATION)
    
    def land(self):
        # Start descent
        self.get_logger().info("Starting descent...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5 * LONG_DURATION)

        # Finish descent
        future.set_result(True)
        self.get_logger().info("Vehicle landed successfully!")
        
    def move_forward(self, duration):
        # Move forward
        send_velocity_cmd(VEHICLE_VELOCITY, 0.0, 0.0)
        condition_yaw(0)
        time.sleep(duration)

    def turn_right(self):
        # Turn right
        self.get_logger().info("Turning right...")
        condition_yaw(90)
        self.amount_of_turns += 1
        time.sleep(LONG_DURATION)

    def turn_left(self):
        # Turn left
        self.get_logger().info("Turning left...")
        condition_yaw(90, clockwise=False)
        self.amount_of_turns += 1
        time.sleep(LONG_DURATION)
    
    def movement_callback(self, msg: String):
        movement = msg.data
        
        if self.amount_of_turns == 4:
            # Stop
            self.stop()
            
            # Land
            self.land()
        
        elif movement == "MOVE_FORWARD":
            # Move forward
            self.move_forward(SHORT_DURATION)

        elif movement == "TURN_RIGHT":
            # Stop
            self.stop()
                      
            # Turn right
            self.turn_right()
            
            if self.amount_of_turns < 4:
                # Move forward
                self.get_logger().info("Moving forward...")
                self.move_forward(LONG_DURATION)
        
        elif movement == "TURN_LEFT":
            # Stop
            self.stop
                      
            # Turn left
            self.turn_left()
            
            if self.amount_of_turns < 4:
                # Move forward
                self.get_logger().info("Moving forward...")
                self.move_forward(LONG_DURATION)

           
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