#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Declare parameter to decide whether to catch the closest turtle
        self.declare_parameter("catch_closest_turtle", False)

        self.turtle_to_catch: Turtle = None
        self.catch_closest_turtle = self.get_parameter("catch_closest_turtle").value
        self.pose: Pose = None

        # Create publisher to send velocity commands to turtle1
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscribe to the pose of turtle1
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

        # Subscribe to the list of currently alive turtles
        self.alive_turtle_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)

        # Create a client to request catching turtles
        self.catch_turtle_client = self.create_client(CatchTurtle, "catch_turtle")

        # Set up a timer for running the control loop periodically
        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

    def callback_pose(self, pose: Pose):
        # Update the current pose of turtle1
        self.pose = pose

    def callback_alive_turtles(self, turtle_list: TurtleArray):
        # Select the turtle to catch based on closest or first in list
        if len(turtle_list.turtles) > 0:
            if self.catch_closest_turtle:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in turtle_list.turtles:
                    dist_x = turtle.x - self.pose.x
                    dist_y = turtle.y - self.pose.y
                    dist = math.sqrt(dist_x ** 2 + dist_y ** 2)

                    if closest_turtle == None or dist < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = dist

                self.turtle_to_catch = closest_turtle
            else:
                self.turtle_to_catch = turtle_list.turtles[0]

    def control_loop(self):
        # Move towards the target turtle and catch it if close enough
        if self.pose is None or self.turtle_to_catch is None:
            return

        dist_x = self.turtle_to_catch.x - self.pose.x
        dist_y = self.turtle_to_catch.y - self.pose.y
        dist = math.sqrt(dist_x ** 2 + dist_y ** 2)

        cmd = Twist()

        if dist > 0.5:
            cmd.linear.x = 2.0 * dist

            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            cmd.angular.z = 6.0 * diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None

        self.cmd_vel_publisher.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        # Request the service to catch the specified turtle
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Catch Turtle Server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name))

    def callback_call_catch_turtle_service(self, future, *, turtle_name):
        # Handle the response from the catch turtle service
        response = future.result()
        if not response.success:
            self.get_logger().error("Turtle: " + turtle_name + " could not be removed")
        else:
            self.get_logger().info("Turtle: " + turtle_name + " successfully removed")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()