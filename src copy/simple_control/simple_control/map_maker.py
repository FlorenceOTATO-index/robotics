#!/usr/bin/env python
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, Point
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class MapMaker(Node):
    def __init__(self):
        super().__init__('MapMakerNode')
        self.lidar_sub = self.create_subscription(LaserScan, '/uav/sensors/lidar', self.get_lidar, 1)
        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_gps, 1)
        self.goal_sub = self.create_subscription(Point, "/uav/input/goal", self.get_goal, 1)
        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)

        width_param = 'map_width'
        self.declare_parameter(width_param, 23)
        self.width = self.get_parameter(width_param).get_parameter_value().integer_value

        height_param = 'map_height'
        self.declare_parameter(height_param, 23)
        self.height = self.get_parameter(height_param).get_parameter_value().integer_value

        self.get_logger().info(f"map is of size {self.height}, {self.width}")

        self.lidar = None
        self.prev_lidar = None
        self.new_lidar = False
        self.moved = False
        self.goal = None

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.info.resolution = 1.0
        self.occupancy_grid.info.width = self.width
        self.occupancy_grid.info.height = self.height

        origin = Pose()
        origin.position = Point()

        origin.position.x = float(int(-1 * self.width/2))
        origin.position.y = float(int(-1 * self.height/2))
        origin.position.z = 0.0
        self.occupancy_grid.info.origin = origin

        self.occupancy_grid_np = np.full([self.height, self.width], 50)

        self.doors = np.full([self.height, self.width, 4], 0)

        self.position = None

        self.cumulative_noise = []
        self.ave_noise = 0.1
        self.n_explored = 0

        # Set the timer to call the mainloop of our class
        self.rate = 40
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def init_grid(self):
        # initialize grid with 50 for all spaces
        self.occupancy_grid.data = [50] * self.height * self.width

    def get_gps(self, msg):
        # update the position
        #self.get_logger().info("" + str(msg.pose.position.x) + ", " + str(msg.pose.position.y))
        if self.position:
            old = self.gps_to_cell(self.position)
            new = self.gps_to_cell(msg.pose.position)
            if new[0] != old[0] or new[1] != old[1]:
                self.cumulative_noise = []
                self.ave_noise = 0.1
        self.position = msg.pose.position
        self.moved = True
        #self.occupancy_grid_np[math.floor(self.height * 0.5), math.floor(self.width * 0.5)] = 0

    def get_lidar(self, msg):
        # update the map with the new LIDAR information
        # self.get_logger().info("lidar received")
        self.prev_lidar = self.lidar
        self.lidar = msg
        self.new_lidar = True

    def get_goal(self, msg):
        self.goal = msg

    def gps_to_cell(self, point):
        x = point.x
        y = point.y
        return math.floor(x + self.width/2), math.floor(y + self.height/2)

    def mainloop(self):
        if self.new_lidar or self.moved and self.lidar and self.position:
            angle = self.lidar.angle_min
            ave_noise = 0
            cumulative_noise = 0
            for i in range(0, len(self.lidar.ranges)):
                if self.lidar.ranges[i] < self.lidar.range_min or self.lidar.ranges[i] > self.lidar.range_max:
                    continue
                x_dist = round(math.cos(angle) * self.lidar.ranges[i])
                y_dist = round(math.sin(angle) * self.lidar.ranges[i])

                x_pos, y_pos = self.gps_to_cell(self.position)

                for j in range(round(self.lidar.ranges[i])):
                    point = Point()
                    point.x = self.position.x + round(math.cos(angle) * j)
                    point.y = self.position.y + round(math.sin(angle) * j)
                    cell_x, cell_y = self.gps_to_cell(point)
                    self.occupancy_grid_np[cell_y, cell_x] *= 0.875

                point = Point()
                point.x = self.position.x + x_dist
                point.y = self.position.y + y_dist
                cell_x, cell_y = self.gps_to_cell(point)

                self.occupancy_grid_np[cell_y, cell_x] = min(100, 1.25 * self.occupancy_grid_np[cell_y, cell_x])

                if not self.prev_lidar is None:

                    noise = abs(self.lidar.ranges[i] - round(self.prev_lidar.ranges[i]))
                    try:
                        self.cumulative_noise[i] = (self.cumulative_noise[i] + noise)/2
                    except:
                        self.cumulative_noise.append(noise)

                    ave_noise = sum(self.cumulative_noise)/len(self.lidar.ranges)

                    # self.get_logger().info(f"{cell_x}, {cell_y}")

                    cand = False

                    try:
                        sum_neighbors = sum([self.occupancy_grid_np[cell_y + 1, cell_x], self.occupancy_grid_np[cell_y - 1, cell_x], self.occupancy_grid_np[cell_y, cell_x + 1], self.occupancy_grid_np[cell_y, cell_x - 1]])
                        cand = self.goal and not (self.goal.y - cell_y + self.goal.x - cell_x) % 2 and 250 >= sum_neighbors and 200 < sum_neighbors
                    except:
                        pass
                    if cand and noise > ave_noise and (self.occupancy_grid_np[cell_y, cell_x] == -1 or self.lidar.ranges[i] < 1):
                        self.occupancy_grid_np[cell_y, cell_x] = -1
                        self.cumulative_noise[i] = 0

                angle += self.lidar.angle_increment

            if self.goal:
                goal_cell_x, goal_cell_y = self.gps_to_cell(self.goal)
                # self.get_logger().info(f"{goal_cell_x}, {goal_cell_y}")
                self.occupancy_grid_np[goal_cell_y, goal_cell_x] = -3
            self.occupancy_grid.data = np.swapaxes(self.occupancy_grid_np, 0, 1).flatten()
            self.new_lidar = False
            self.moved = False

            if self.goal:
                self.pub.publish(self.occupancy_grid)
        #self.get_logger().info("published map")

def main():
    rclpy.init()
    try:
        rclpy.spin(MapMaker())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()