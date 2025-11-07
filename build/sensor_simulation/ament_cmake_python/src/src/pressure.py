#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import random

# Create a class which saves the altitude of the drone and esimates the pressure
class PressureSensor(Node):

  # Node initialization
  def __init__(self):
    super().__init__('PressureSensorNode')
    # Sleep to allow simulation to start
    time.sleep(5)

    # Create the publisher and subscriber
    self.pressure_pub = self.create_publisher(Float64, '/uav/sensors/pressure', 1)
    self.position_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.getPosition, 1)
    
    # Save the altitude of the drone
    self.altitude = 0

    # Used to estimate the pressure
    self.pressure = 0

    # Determines the baseline value of the perssure sensor at height 0m
    self.baseline_value = 0

    self.pressure_msg = Float64()

    self.rate = 10
    self.dt = 1.0 / self.rate
    # Call the mainloop of our class
    self._timer = self.create_timer(self.dt, self.mainloop)

  # Callback for the keyboard manager
  def getPosition(self, msg):
    # Save the drones alitude
    self.altitude = msg.pose.position.z

  # The main loop of the function
  def mainloop(self):
    # Publish the altitude
    self.pressure_pub.publish(self.pressure_msg)

    # Compute the pressure in milibars (according to https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf)
    h = self.altitude
    common = pow((44307.7 - h),(12145.0/47571.0))
    self.pressure = (-3.86423)*(pow(10,-22))*(pow(h,5))*(common) + (8.56075)*(pow(10,-17))*(pow(h,4))*(common) - (7.58614)*(pow(10,-12))*(pow(h,3))*(common) + (3.36124)*(pow(10,-7))*(pow(h,2))*(common) - (0.00744645)*(h)*(common) + (65.987)*(common)

    # Add sensor noise
    noise = pow(10,-5) * random.uniform(-1, 1)

    # Set the pressure
    self.pressure_msg.data = self.pressure + noise - self.baseline_value


if __name__ == '__main__':
  rclpy.init()
  try:
    rclpy.spin(PressureSensor())
  except (ExternalShutdownException, KeyboardInterrupt):
    pass
  finally:
    rclpy.try_shutdown()