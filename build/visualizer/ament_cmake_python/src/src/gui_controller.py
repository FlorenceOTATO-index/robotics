#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np

from gui import GUI
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
import time
from transforms3d._gohlketransforms import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class GUI_Controller(Node):

    def __init__(self):
        # When this node shutsdown
        super().__init__('GUIControllerNode')

        # Set the rate
        self.rate = 2.0
        self.dt = 1.0 / self.rate

        # Create the position
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)


        # map data
        # Environment details
        self.obstacle_list = []
        self.map_size = (10, 10)
        self.path = np.array([[0, 0]])
        env_data = {'path': self.path, 'environment': self.obstacle_list}
        gui_data = {'quad1': {'position': [0, 0, 0], 'orientation': [0, 0, 0], 'L': 1}}

        # Create the gui object
        self.gui_object = GUI(quads=gui_data, env=env_data, map_size=self.map_size)

        # Create the subscribers and publishers
        self.gps_sub = self.create_subscription(PoseStamped, "uav/sensors/gps", self.get_gps, 10)
        self.path_sub = self.create_subscription(Int32MultiArray, '/uav/path', self.get_path, 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.get_map, qos)

        # Run the communication node
        self._timer = self.create_timer(self.dt, self.UpdateLoop)

    def get_path(self, msg):
        self.path = np.array(np.reshape(msg.data, (-1, 2)))
        self.path[:, 0] = self.path[:, 0]
        self.path[:, 1] = self.path[:, 1]

    def get_map(self, msg):
        self.updateaxis = True

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution

        self.start_x = msg.info.origin.position.x
        self.start_y = msg.info.origin.position.y

        self.end_x = self.start_x + (width * res)
        self.end_y = self.start_y + (height * res)

        map_data = np.reshape(msg.data, (width, height))
        self.map_size = (width / 2.0, height / 2.0)

        for xi in range(0, width):
            for yi in range(0, height):

                if map_data[xi, yi] > 50:
                    self.obstacle_list.append((xi + self.start_x, yi + self.start_y))

    # This is the main loop of this class
    def UpdateLoop(self):
        # Display the position
        self.gui_object.world["path"] = self.path
        self.gui_object.world["environment"] = self.obstacle_list
        self.gui_object.world["map_size"] = self.map_size
        self.gui_object.quads['quad1']['position'] = list(self.position)
        self.gui_object.quads['quad1']['orientation'] = list(euler_from_quaternion(self.quaternion))
        self.gui_object.update()

    # Call back to get the gps data
    def get_gps(self, msg):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.quaternion = (msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)


# Main function
if __name__ == '__main__':
    rclpy.init()
    try:
        rclpy.spin(GUI_Controller())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()
