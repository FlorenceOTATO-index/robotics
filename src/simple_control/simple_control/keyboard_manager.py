#!/usr/bin/env python
import numpy as np
import copy
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import Key
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
import time

# Create a class which we will use to take keyboard commands and convert them to a position
class KeyboardManager(Node):
    # On node initialization
    def __init__(self):
        super().__init__('KeyboardManagerNode')
        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/keyboardmanager/position', 1)
        self.keyboard_sub = self.create_subscription(Key, '/keydown', self.get_key, 1)
        # Create the position message we are going to be sending
        self.pos = Vector3()
        self.prev_pos = Vector3()
        # Create a variable we will use to hold the key code
        self.key_code = -1
        # Give the simulation enough time to start
        time.sleep(10)
        # Call the mainloop of our class
        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    # Callback for the keyboard sub
    def get_key(self, msg):
        self.key_code = msg.code

    # Converts a position to string for printing
    def goalToString(self, msg):
        pos_str = "(" + str(msg.x)
        pos_str += ", " + str(msg.y)
        pos_str += ", " + str(msg.z) + ")"
        return pos_str


    def mainloop(self):
        # Check if any key has been pressed
        # Left
        if self.key_code == Key.KEY_LEFT:
            self.pos.x = self.pos.x + np.cos(0)
            # Up
        if self.key_code == Key.KEY_UP:
            self.pos.y = self.pos.y + np.sin(np.pi/2)
            # Right
        if self.key_code == Key.KEY_RIGHT:
            self.pos.x = self.pos.x + np.sin(3*np.pi/2)
            # Down
        if self.key_code == Key.KEY_DOWN:
            self.pos.y = self.pos.y + np.cos(np.pi)


        # Publish the position
        if self.key_code == Key.KEY_RETURN:
            # TO BE COMPLETED FOR CHECKPOINT 1
            # Publish the position (self.pos)
            self.position_pub.publish(self.pos)
            self.get_logger().info("Sending Position")

        # TO BE COMPLETED FOR CHECKPOINT 1
        # Check if the position has changed by comparing it to the current position
        # Note you will need to change the if statement below from if False -> if #... 
        if self.pos != self.prev_pos:
            self.prev_pos = copy.deepcopy(self.pos)
            self.get_logger().info(f"Keyboard: {self.goalToString(self.pos)}")

        # Reset the code
        if self.key_code != -1:
            self.key_code = -1

def main():
    rclpy.init()
    try:
        rclpy.spin(KeyboardManager())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()