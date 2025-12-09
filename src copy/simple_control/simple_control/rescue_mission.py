#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid


STATE_START         = 0
STATE_EXPLORE       = 1
STATE_APPROACH_DOOR = 2
STATE_VERIFY_DOOR   = 3
STATE_GO_TO_DOG     = 4
STATE_DONE          = 5


class RescueMission(Node):
    def __init__(self):
        super().__init__("RescueMission")

        width_param = "map_width"
        self.declare_parameter(width_param, 23)
        self.width = self.get_parameter(width_param).value

        height_param = "map_height"
        self.declare_parameter(height_param, 23)
        self.height = self.get_parameter(height_param).value

        self.state = STATE_START
        self.position = None
        self.map = None
        self.goal_world = None       
        self.current_door = None
        self.verify_steps = 0

        self.gps_sub = self.create_subscription(
            PoseStamped, "/uav/sensors/gps", self.cb_gps, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.cb_map, 10
        )
        self.goal_sub = self.create_subscription(
            Point, "/dog_position_world", self.cb_goal, 10
        )

        self.goal_pub = self.create_publisher(Point, "/uav/input/goal", 10)

        self.timer = self.create_timer(0.3, self.mainloop)

        self.get_logger().info("[RescueMission] Initialized")


    def world_to_cell(self, x, y):
        cx = int(math.floor(x + self.width / 2))
        cy = int(math.floor(y + self.height / 2))
        cx = max(0, min(self.width - 1, cx))
        cy = max(0, min(self.height - 1, cy))
        return cx, cy

    def cell_to_world(self, cx, cy):
        x = (cx - self.width/2) + 0.5
        y = (cy - self.height/2) + 0.5
        return x, y

    def publish_goal_world(self, x, y):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        self.goal_pub.publish(p)

    def cb_gps(self, msg):
        self.position = msg.pose.position

    def cb_map(self, msg):
        data = np.array(msg.data)
        if data.size == self.width * self.height:
            self.map = data.reshape((self.height, self.width))

    def cb_goal(self, msg):
        self.goal_world = msg

    def find_detected_door(self):
        if self.map is None:
            return None

        doors = np.argwhere(self.map == -1)
        if doors.size == 0:
            return None

        if self.position is None:
            return None

        px, py = self.world_to_cell(self.position.x, self.position.y)

        best = None
        best_d = 9999
        for (cy, cx) in doors:
            d = abs(cx - px) + abs(cy - py)
            if d < best_d:
                best_d = d
                best = (cx, cy)

        return best

    def mainloop(self):
        if self.position is None or self.map is None or self.goal_world is None:
            return

        gx, gy = self.world_to_cell(self.goal_world.x, self.goal_world.y)
        px, py = self.world_to_cell(self.position.x, self.position.y)

        if self.state == STATE_START:
            self.state = STATE_EXPLORE
            self.get_logger().info("[RescueMission] -> EXPLORE")
            return

     
        if self.state == STATE_EXPLORE:
            door = self.find_detected_door()

            if door is not None:
                self.current_door = door
                wx, wy = self.cell_to_world(*door)
                self.publish_goal_world(wx, wy)
                self.get_logger().info(f"[RescueMission] Door spotted at {door}, approaching it")
                self.state = STATE_APPROACH_DOOR
                return

            self.publish_goal_world(self.goal_world.x, self.goal_world.y)
            return

        if self.state == STATE_APPROACH_DOOR:
            if self.current_door is None:
                self.state = STATE_EXPLORE
                return

            cx, cy = self.current_door
            wx, wy = self.cell_to_world(cx, cy)

            self.publish_goal_world(wx, wy)

            if abs(px - cx) + abs(py - cy) <= 1:
                self.verify_steps = 6
                self.get_logger().info("[RescueMission] Verifying door with LiDAR wiggles")
                self.state = STATE_VERIFY_DOOR
            return

        if self.state == STATE_VERIFY_DOOR:

            offsets = [(0,0), (1,0), (0,1), (-1,0), (0,-1)]

            idx = self.verify_steps % len(offsets)
            dx, dy = offsets[idx]

            cx, cy = self.current_door
            wx, wy = self.cell_to_world(cx + dx, cy + dy)
            self.publish_goal_world(wx, wy)

            self.verify_steps -= 1

            if self.verify_steps <= 0:
                cx, cy = self.current_door
                if self.map[cy, cx] == -1:
                    self.get_logger().info("[RescueMission] Door validated, letting LocalPlanner handle opening")
                else:
                    self.get_logger().info("[RescueMission] Door no longer detected → rejecting")

                self.state = STATE_EXPLORE
            return

        if self.state == STATE_GO_TO_DOG:
            self.publish_goal_world(self.goal_world.x, self.goal_world.y)

            if abs(px - gx) + abs(py - gy) <= 1:
                self.get_logger().info("[RescueMission] Dog reached. Mission complete.")
                self.state = STATE_DONE
            return

        if self.state == STATE_DONE:
            return  


def main():
    rclpy.init()
    try:
        rclpy.spin(RescueMission())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
