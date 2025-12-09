#!/usr/bin/env python
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, Point, Vector3
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid

class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('GlobalPlannerNode')

        self.plan_pub = self.create_publisher(Int32MultiArray, "/global_plan", 1)
        self.path_pub = self.create_publisher(Int32MultiArray, "/uav/path", 1)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.get_map, 1)
        self.goal_sub = self.create_subscription(Point, "/uav/input/goal", self.get_goal, 1)
        self.gps_sub = self.create_subscription(PoseStamped, "/uav/sensors/gps", self.get_gps, 1)

        self.goal = None
        #self.added_goal = False
        #self.added_pos = False
        self.waypoints = None
        self.map = None
        self.position = None
        self.pos_cell_x = self.pos_cell_y = 0

        self.new_map = False
        self.moved = True

        width_param = 'map_width'
        self.declare_parameter(width_param, 23)
        self.width = self.get_parameter(width_param).get_parameter_value().integer_value

        height_param = 'map_height'
        self.declare_parameter(height_param, 23)
        self.height = self.get_parameter(height_param).get_parameter_value().integer_value

        # Set the timer to call the mainloop of our class
        self.rate = 5
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def get_goal(self, msg):
        self.goal = msg

    def get_gps(self, msg):
        if not self.position is None and (abs(self.position.x - msg.pose.position.x) >= 1 or abs(self.position.y - msg.pose.position.y) >= 1):
            self.moved = True

        cell = self.gps_to_cell(msg.pose.position)
        self.pos_cell_x = cell[0]
        self.pos_cell_y = cell[1]

        self.position = msg.pose.position

    def get_map(self, msg):
        self.map = np.array(msg.data)
        self.map = np.reshape(self.map, (self.height, -1))
        self.new_map = True
        # self.get_logger().info(str(self.map.shape))

    def gps_to_cell(self, point):
        new_x = point.x
        new_y = point.y
        return [int(math.floor(new_x + self.width/2)), int(math.floor(new_y + self.height/2))]

    def cell_to_gps(self, point):
        x = int(0.5  + (point[0] - self.width/2))
        y = int(0.5  + (point[1] - self.height/2))
        return x, y

    def mainloop(self):
        if self.goal and self.new_map:
            self.waypoints = np.array([self.gps_to_cell(self.goal)])
            door = [0, 0]
            for i in range(self.height):
                for j in range(self.width):
                    if self.map[i, j] == -1:
                        door = [i, j]

            if door[0] and door[1]:
                hor_diff = door[0] - self.pos_cell_x
                ver_diff = door[1] - self.pos_cell_y

                #self.get_logger().info(f"{door[0]}, {door[1]}, {self.pos_cell_x}, {self.pos_cell_y}, {self.position}")

                x_move = y_move = 0

                if abs(hor_diff) > abs(ver_diff):
                    x_move = int(hor_diff/abs(hor_diff))
                    # y_move = int((self.goal.y - door[1])/abs(self.goal.y - door[1]))
                elif ver_diff:
                    y_move = int(ver_diff/abs(ver_diff))
                    # x_move = int((self.goal.x - door[0])/abs(self.goal.x - door[0]))
                else:
                    pass

                goal_pos = self.cell_to_gps([door[0] + x_move, door[1] + y_move])

                #self.get_logger().info(np.array2string(self.waypoints))

                if self.waypoints is None:
                    self.waypoints = np.array(goal_pos)
                elif self.waypoints[0,0] != goal_pos[0] or self.waypoints[0, 1] != goal_pos[1]:
                    #self.get_logger().info(f"{np.array(goal_pos).shape}, {self.waypoints.shape}")
                    self.waypoints = np.concatenate((np.reshape(np.array(goal_pos), (1, 2)), self.waypoints))
                time.sleep(1)

        if not self.waypoints is None and len(self.waypoints):
            #self.get_logger().info(np.array2string(self.waypoints))
            arr = Int32MultiArray()
            arr.data = self.waypoints.flatten()
            self.plan_pub.publish(arr)
            self.path_pub.publish(arr)

def main():
    rclpy.init()
    try:
        rclpy.spin(GlobalPlannerNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()