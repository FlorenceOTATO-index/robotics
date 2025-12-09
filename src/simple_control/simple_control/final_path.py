#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32MultiArray

import heapq
import math
import numpy as np


class FinalPathNode(Node):
    def __init__(self):
        super().__init__("FinalPathNode")
        self.map = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 1.0

        self.goal_world = None

        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.create_subscription(PointStamped, "/cell_tower/position", self.goal_cb, 10)
        self.final_pub = self.create_publisher(Int32MultiArray, "/uav/final_path", 10)
        self.create_timer(1.0, self.try_compute)

        self.get_logger().info("FinalPathNode ACTIVE")

    def map_cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height

        raw = np.array(msg.data).reshape((w, h))
        self.map = raw.T.copy()  # convert to (rows, cols)

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution

    def goal_cb(self, msg: PointStamped):
        self.goal_world = (msg.point.x, msg.point.y)
        self.get_logger().info(f"Goal received: {self.goal_world}")

    def try_compute(self):
        if self.map is None or self.goal_world is None:
            return

        start = self.world_to_grid(0.0, 0.0) 
        goal = self.world_to_grid(self.goal_world[0], self.goal_world[1])

        if not self.valid(start) or not self.valid(goal):
            return

        path = self.astar(start, goal)

        if path is None:
            self.get_logger().warn("A* failed to find a path.")
            return

        self.publish_final_path(path)
        self.get_logger().info(f"Final path published with {len(path)} points.")

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return (gy, gx)  # row, col

    def grid_to_world(self, r, c):
        x = c * self.resolution + self.origin_x
        y = r * self.resolution + self.origin_y
        return (x, y)

    def valid(self, cell):
        r, c = cell
        return 0 <= r < self.map.shape[0] and 0 <= c < self.map.shape[1]

    def astar(self, start, goal):
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        dirs = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]

        rows, cols = self.map.shape

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            r, c = current

            for dr, dc in dirs:
                nr, nc = r + dr, c + dc

                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue

                cell = self.map[nr, nc]

                # obstacle unless opened door
                if cell >= 50 and cell != -2:  
                    continue

                step_cost = math.hypot(dr, dc)
                tentative = g_score[current] + step_cost
                neighbor = (nr, nc)

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def publish_final_path(self, path):
        msg = Int32MultiArray()
        data = []

        for r, c in path:
            x, y = self.grid_to_world(r, c)
            data.extend([int(round(x)), int(round(y))])

        msg.data = data
        self.final_pub.publish(msg)


def main():
    rclpy.init()
    node = FinalPathNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
