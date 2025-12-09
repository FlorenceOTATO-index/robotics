#!/usr/bin/env python
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, Point, Vector3
from std_msgs.msg import Bool, Int32MultiArray
from nav_msgs.msg import OccupancyGrid

from threading import Thread

#import sys
#sys.path.append("../../environment_controller/srv")

from environment_controller.srv import UseKey

class LocalPlannerNode(Node):

    def __init__(self):
        super().__init__('LocalPlannerNode')
        self.cli = self.create_client(UseKey, 'use_key')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = UseKey.Request()
        width_param = 'map_width'
        self.declare_parameter(width_param, 23)
        self.width = self.get_parameter(width_param).get_parameter_value().integer_value

        height_param = 'map_height'
        self.declare_parameter(height_param, 23)
        self.height = self.get_parameter(height_param).get_parameter_value().integer_value

        self.position = Point()
        self.pos_cell_x = -1
        self.pos_cell_y = -1
        self.moved = True
        self.door = [0, 0]

        self.arrival_threshold = 0.4    
        self.step_size = 0.8            
        self.avoidance_turn_step = 0

        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_gps, 10)
        self.global_plan_sub = self.create_subscription(Int32MultiArray, '/global_plan', self.get_global_plan, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.get_map, 10)
        self.pos_pub = self.create_publisher(Vector3, '/uav/input/position', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/uav/sensors/lidar', self.lidar_callback, 10)


        self.waypoints = None
        self.map = None
        self.cur = []
        self.cur_n = 0

        self.opening_door = False

        # Set the timer to call the mainloop of our class
        self.rate = 10
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def get_gps(self, msg):
        if abs(self.position.x - msg.pose.position.x) >= 1 or abs(self.position.y - msg.pose.position.y) >= 1:
            self.moved = True

        #self.get_logger().info("received gps")

        # update the position
        self.position = msg.pose.position

    def get_global_plan(self, msg):
        self.waypoints = np.array(msg.data)
        self.waypoints = np.reshape(np.array(msg.data), (-1, 2))
        if not len(self.cur):
            self.cur = self.waypoints[self.cur_n]

    def get_map(self, msg):
        self.map = np.array(msg.data)
        self.map = np.reshape(self.map, (self.height, -1))
        self.new_map = True

    def open_door(self, door):
        self.get_logger().info("opening door")
        d_x, d_y = self.cell_to_gps(door)
        self.req.door_loc = Point()
        self.req.door_loc.x = d_x
        self.req.door_loc.y = d_y
        self.req.door_loc.z = self.position.z
        try:
            self.get_logger().info(str(self.cli.call(self.req).success))
            self.opening_door = False
        except:
            self.opening_door = False
        self.opening_door = False

    def gps_to_cell(self, point):
        x = point.x
        y = point.y
        return math.floor(x + self.width/2), math.floor(y + self.height/2)

    def gps_to_cell_arr(self, point):
        x = point[0]
        y = point[1]
        return math.floor(x + self.width/2), math.floor(y + self.height/2)

    def cell_to_gps(self, point):
        x = 0.5  + (point[0] - self.width/2)
        y = 0.5  + (point[1] - self.height/2)
        return x, y
    
    def lidar_callback(self, msg):
        self.obstacle_ahead = False
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < math.radians(60):
                if not math.isnan(r) and 0.2 < r < 1.0: 
                    self.obstacle_ahead = True
                    break

    def mainloop(self):
        # self.get_logger().info(str(self.opening_door))
        if not (self.opening_door or self.waypoints is None or self.map is None):
            self.pos_cell_x, self.pos_cell_y = self.gps_to_cell(self.position)

            goal = Vector3()
            goal.x = self.position.x
            goal.y = self.position.y
            goal.z = self.position.z

            hor_diff = self.cur[0] - self.pos_cell_x
            ver_diff = self.cur[1] - self.pos_cell_y

            if hor_diff or ver_diff:

                # self.get_logger().info(f"{hor_diff}, {ver_diff}")

                x_move = y_move = 0

                # self.get_logger().info(f"{self.cur[0]}, {self.cur[1]}, {self.pos_cell_x}, {self.pos_cell_y}")

                if abs(hor_diff) > abs(ver_diff):
                    y_move = -int(hor_diff/abs(hor_diff))
                elif ver_diff:
                    x_move = -int(ver_diff/abs(ver_diff))
                else:
                    pass

                goal_cell_x, goal_cell_y = self.gps_to_cell_arr([goal.x + x_move, goal.y + y_move])
                # self.get_logger().info(f"{goal_cell_x}, {goal_cell_y}")

                if self.map[goal_cell_x, goal_cell_y] == -1:
                     self.opening_door = True
                     door_open = Thread(target = self.open_door, args = ([goal_cell_x, goal_cell_y],))
                     door_open.start()

                # while the next cell in the path is occupied, turn "left" (rotate the intended move 90 degrees ccw)
                while not self.opening_door and self.map[goal_cell_x, goal_cell_y] > 75:
                    if self.obstacle_ahead:
                        goal.x = self.position.x  
                        goal.y = self.position.y
                        self.get_logger().info("Obstacle ahead")
                        
                    self.get_logger().info(f"turning left, original:{goal_cell_x}, {goal_cell_y}, {x_move}, {y_move}")
                    temp = x_move
                    x_move = -1 * y_move
                    y_move = temp
                    goal_cell_x, goal_cell_y = self.gps_to_cell_arr([goal.x + x_move, goal.y + y_move])
                    self.get_logger().info(f"new:{goal_cell_x}, {goal_cell_y}, {x_move}, {y_move}")

                if not self.opening_door:
                    goal.x = float(int(x_move + goal.x))
                    goal.y = float(int(y_move + goal.y))

                    self.pos_pub.publish(goal)

            else:
                #self.get_logger().info(np.array2string(self.waypoints))
                #self.get_logger().info(f"{self.waypoints.shape}")
                self.cur_n = (self.cur_n + 1) % self.waypoints.shape[0]
                self.cur = self.waypoints[self.cur_n]


def main():
    rclpy.init()
    try:
        rclpy.spin(LocalPlannerNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
