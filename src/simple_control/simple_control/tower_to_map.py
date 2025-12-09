#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
from tf2_ros import TransformException
import time
import copy
from geometry_msgs.msg import Vector3, PointStamped, Point
from tf2_geometry_msgs import do_transform_point
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TowerToMap(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('TowerToMapNode')
        # Used by the callback for the topic /tower/goal
        self.goal = Point()
        self.goal.x = -1
        # TODO: Instantiate the Buffer and TransformListener
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)

        # TODO: Goal publisher on topic /uav/input/goal

        self.goal_pub = self.create_publisher(Point, '/uav/input/goal', 1)

        # TODO: Tower goal subscriber to topic /tower/goal

        self.goal_sub = self.create_subscription(Point, '/cell_tower/position', self.get_goal, 1)

        # start main loop
        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    #TODO: Callback for the tower goal subscriber
    def get_goal(self, msg):
        self.goal.x = msg.x
        self.goal.y = msg.y
        self.goal.z = msg.z

    def mainloop(self):
        if self.goal.x != -1:
            try:
                #TODO: Lookup the tower to world transform
                t = self.tfBuffer.lookup_transform('cell_tower', 'world', rclpy.time.Time())

                #TODO: Convert the goal to a PointStamped
                goal_ps = PointStamped()
                goal_ps.point = self.goal

                #TODO: Use the do_transform_point function to convert the point using the transform
                goalpoint_trans = do_transform_point(goal_ps, t)

                #TODO: Convert the point back into a vector message containing integers
                msg = Point()
                msg.x = goalpoint_trans.point.x
                msg.y = goalpoint_trans.point.y
                msg.z = goalpoint_trans.point.z

                self.goal_pub.publish(msg)

                #TODO: Publish the vector
                #self.get_logger().info(f'Publishing Transformed Goal: {msg.x}, {msg.y}')

                # The tower will automatically send you a new goal once the drone reaches the requested position.
                #TODO: Reset the goal
                self.goal = Point()
                self.goal.x = -1

            except TransformException as ex:
                self.get_logger().info(f'Error getting the tower transformation')


def main():
    rclpy.init()
    try:
        rclpy.spin(TowerToMap())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()