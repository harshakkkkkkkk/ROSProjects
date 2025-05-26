# ⚠️ Note

This repository includes the core source code for my ROS2 turtlesim project (main control and spawning logic). It does not include full configuration files like setup.py, package.xml, or custom message/service definitions.

To run this project successfully, you’ll need to:

    Set up a ROS2 workspace (colcon-based)

    Create a custom interface package with the Turtle and TurtleArray message types, and the CatchTurtle service

    Configure your package.xml and CMakeLists.txt accordingly

    Source your workspace after building (source install/setup.bash)

This repo is meant to showcase the core logic and structure of the project. If you're already familiar with ROS2, setting it up should be straightforward. Otherwise, feel free to reach out or refer to official ROS2 documentation for help setting up the rest of the workspace.
