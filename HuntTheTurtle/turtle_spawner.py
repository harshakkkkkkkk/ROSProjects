#!/usr/bin/env python3

import rclpy
import random
import math
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        # Declare and initialize parameters for spawn frequency and name prefix
        self.declare_parameter("spawn_frequency", 0.5)
        self.declare_parameter("turtle_name_prefix", "turtle")

        self.frequency = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value
        self.turtle_counter = 0
        self.alive_turtles = []

        # Create a service server to handle turtle catch requests
        self.catch_turtle_service = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        # Create clients for spawning and killing turtles
        self.turtle_spawner_client = self.create_client(Spawn, "/spawn")
        self.kill_client = self.create_client(Kill, "/kill")

        # Create a publisher to broadcast the list of currently alive turtles
        self.alive_turtle_publisher = self.create_publisher(TurtleArray, "alive_turtles", 10)

        # Set up a timer to periodically spawn new turtles
        self.turtle_spawner_timer = self.create_timer(self.frequency, self.spawn_new_turtle)

    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        # Kill the turtle with the given name and send response
        self.call_kill_service(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        # Publish the current list of alive turtles
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtle_publisher.publish(msg)

    def spawn_new_turtle(self):
        # Generate and spawn a turtle at a random location
        self.turtle_counter += 1
        name = self.turtle_name_prefix + str(self.turtle_counter)
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_turtle_spawner_client(name, x, y, theta)

    def call_turtle_spawner_client(self, turtle_name, x, y, theta):
        # Call the spawn service to create a new turtle
        while not self.turtle_spawner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Spawn Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.turtle_spawner_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_spawner_client, request=request))

    def callback_call_turtle_spawner_client(self, future, request: Spawn.Request):
        # Handle the response from the spawn service and update the alive turtle list
        response = future.result()
        if response and response.name:
            self.get_logger().info("New alive turtle: " + response.name)
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles.append(new_turtle)
            self.publish_alive_turtles()

    def call_kill_service(self, turtle_name):
        # Call the kill service to remove a turtle by name
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Kill Server...")

        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name=turtle_name))

    def callback_call_kill_service(self, future, turtle_name):
        # Remove the turtle from the list and publish the update
        for i, turtle in enumerate(self.alive_turtles):
            if turtle.name == turtle_name:
                del self.alive_turtles[i]
                self.publish_alive_turtles()
                self.get_logger().info(f"Turtle '{turtle_name}' was removed.")
                return
        self.get_logger().warn(f"Turtle '{turtle_name}' not found in alive list.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()