#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

class ThiefTurtleNode(Node):
    def __init__(self):
        super().__init__("thief_turtle")

        self.police_pose: Pose = None
        self.thief_pose: Pose = None

        # Parameters
        self.threshold = 2.0  # Distance threshold to start escaping
        self.min_x = 2.5
        self.max_x = 8.5
        self.min_y = 2.5
        self.max_y = 8.5

        # Client to spawn the thief turtle
        self.turtle_spawner_client = self.create_client(Spawn, "/spawn")

        # Publisher to control thief turtle's velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, "/thief_turtle/cmd_vel", 10)

        # Timer to run control loop at fixed intervals
        self.control_publisher_timer = self.create_timer(0.1, self.control_loop)

        # Subscribing to police and thief turtles' pose
        self.police_turtle_pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.callback_police_pose, 10)
        self.thief_turtle_pose_subscriber = self.create_subscription(Pose, "/thief_turtle/pose", self.callback_thief_pose, 10)

    # Callback to update police turtle's pose
    def callback_police_pose(self, pose = Pose):
        self.police_pose = pose

    # Callback to update thief turtle's pose
    def callback_thief_pose(self, pose = Pose):
        self.thief_pose = pose

    # Control logic for the thief turtle
    def control_loop(self):
        if self.police_pose == None or self.thief_pose == None:
            return

        # Compute distance between thief and police
        dx = self.police_pose.x - self.thief_pose.x
        dy = self.police_pose.y - self.thief_pose.y
        distance = math.sqrt(pow(dx, 2) + pow(dy, 2))

        cmd = Twist()

        if distance < self.threshold:  # If police is close
            # Compute escape angle
            goal_theta = math.atan2(-dy, -dx)

            # Predict next position and handle edge cases
            projected_x = self.thief_pose.x - math.cos(goal_theta)
            projected_y = self.thief_pose.y - math.sin(goal_theta)

            if projected_x < self.min_x:
                goal_theta = 0
            elif projected_x > self.max_x:
                goal_theta = math.pi
            elif projected_y < self.min_y:
                goal_theta = math.pi / 2
            elif projected_y > self.max_y:
                goal_theta = -math.pi / 2 

            # Normalize angle difference
            diff = goal_theta - self.thief_pose.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            # Set angular and linear velocity
            cmd.angular.z = 6 * diff
            cmd.linear.x = 2 * distance

        else:  # Police is far enough
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd)

    # Function to request turtle spawn
    def call_turtle_spawner_client(self):
        while not self.turtle_spawner_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn Server...")
        
        request = Spawn.Request()
        request.x = 7.0
        request.y = 7.0
        request.theta = 0.0
        request.name = "thief_turtle"

        future = self.turtle_spawner_client.call_async(request)
        future.add_done_callback(self.callback_call_turtle_spawner_client)

    # Callback for spawn service result
    def callback_call_turtle_spawner_client(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

# Entry point
def main(args = None):
    rclpy.init(args = args)
    node = ThiefTurtleNode()
    node.call_turtle_spawner_client()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()