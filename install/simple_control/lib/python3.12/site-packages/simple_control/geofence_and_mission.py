#!/usr/bin/env python
import copy
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Bool

# A class to keep track of the quadrotors state
class DroneState(Enum):
    HOVERING = 1
    VERIFYING = 2
    MOVING = 3


# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class GeofenceAndMission(Node):

    # Node initialization
    def __init__(self):
        super().__init__('GeofenceAndMission')
        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position', 1)
        self.at_goal_pub = self.create_publisher(Bool, '/uav/sensors/at_waypoint', 1)
        self.keyboard_sub = self.create_subscription(Vector3, '/uav/input/position_request',self.getPositionRequest, 1)

        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_gps, 1)

        # get the acceptance range
        acceptance_range_param = '/geofence_and_mission_node/acceptance_range'
        self.declare_parameter(acceptance_range_param, 0.5)
        self.acceptance_range = self.get_parameter(acceptance_range_param).get_parameter_value().double_value
        # get the geofence
        geofence_params = '/geofence_and_mission_node/geofence'
        self.declare_parameter(f'{geofence_params}/x', 10.0)
        self.declare_parameter(f'{geofence_params}/y', 10.0)
        self.declare_parameter(f'{geofence_params}/z', 10.0)
        geofence_x = self.get_parameter(f'{geofence_params}/x').get_parameter_value().double_value
        geofence_y = self.get_parameter(f'{geofence_params}/y').get_parameter_value().double_value
        geofence_z = self.get_parameter(f'{geofence_params}/z').get_parameter_value().double_value
        self.geofence_x = [-1*geofence_x, geofence_x]
        self.geofence_y = [-1*geofence_y, geofence_y]
        self.geofence_z = [0, geofence_z]
        # Display the parameters
        self.get_logger().info(f'Geofence X: {self.geofence_x}')
        self.get_logger().info(f'Geofence Y: {self.geofence_y}')
        self.get_logger().info(f'Geofence Z: {self.geofence_z}')
        self.get_logger().info(f'Acceptance Range: {self.acceptance_range}')
        # Create the drones state as hovering
        self.state = DroneState.HOVERING
        self.get_logger().info("Current State: HOVERING")
        # Create the goal messages we are going to be sending
        self.goal_cmd = Vector3()

        self.unverified_goal_cmd = Vector3()

        self.geofence_on = False

        # Create a point message that saves the drones current position
        self.drone_position = Point()

        # Start the drone a little bit off the ground
        self.goal_cmd.z = 3.0

        # Keeps track of whether the goal  position was changed or not
        self.goal_changed = False

        # Set the timer to call the mainloop of our class
        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)


    # Callback for the keyboard manager
    def getPositionRequest(self, msg):
        # Save the keyboard command
        if self.state == DroneState.HOVERING:
            self.goal_changed = True
            self.unverified_goal_cmd = copy.deepcopy(msg)


    def toggle_geofence(self, request, response):
        self.geofence_on = request.geofence_on
        response.success = True
        return response


    def get_gps(self, msg):
        self.drone_position = msg.pose.position

    # Converts a position to string for printing
    def goalToString(self, msg):
        pos_str = "(" + str(msg.x)
        pos_str += ", " + str(msg.y)
        pos_str += ", " + str(msg.z) + ")"
        return pos_str


    def processVerifying(self):
        # Check if the new goal is inside the cage
        x_check = self.geofence_x[0] <= self.unverified_goal_cmd.x <= self.geofence_x[1]
        y_check = self.geofence_y[0] <= self.unverified_goal_cmd.y <= self.geofence_y[1]
        z_check = self.geofence_z[0] <= self.unverified_goal_cmd.z <= self.geofence_z[1]

        if not self.geofence_on:
            self.state = DroneState.MOVING
            self.goal_cmd = self.unverified_goal_cmd
            return

        # If it is change state to moving
        if x_check and y_check and z_check:
            self.state = DroneState.MOVING
            self.goal_cmd = self.unverified_goal_cmd
        # If it is not change to hovering
        else:
            self.state = DroneState.HOVERING


    # This function is called when we are in the hovering state
    def processHovering(self):
        # Print the requested goal if the position changed
        if self.goal_changed:
            self.state = DroneState.VERIFYING
            self.goal_changed = False


    # This function is called when we are in the moving state
    def processMoving(self):
        # Compute the distance between requested position and current position
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        dz = self.goal_cmd.z - self.drone_position.z

        # Euclidean distance
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2) + np.power(dz, 2))
        # If goal is reached transition to hovering
        if distance_to_goal < self.acceptance_range:
            self.state = DroneState.HOVERING
            bool = Bool()
            bool.data = True
            self.at_goal_pub.publish(bool)

    # The main loop of the function
    def mainloop(self):
        # Publish the position
        self.position_pub.publish(self.goal_cmd)

        # Check if the drone is in a moving state
        if self.state == DroneState.MOVING:
            self.processMoving()
        # If we are hovering then accept keyboard commands
        elif self.state == DroneState.HOVERING:
            self.processHovering()
        elif self.state == DroneState.VERIFYING:
            self.processVerifying()

        # Euclidean distance
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        bool = Bool()
        bool.data = True if (distance_to_goal < self.acceptance_range) else False
        self.at_goal_pub.publish(bool)
def main():
    rclpy.init()
    try:
        rclpy.spin(GeofenceAndMission())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()