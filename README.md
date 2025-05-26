## ⚠️ Note

This repository includes the **core source code** for my ROS2 turtlesim project (main control and spawning logic). It **does not include** full configuration files like `setup.py`, `package.xml`, or custom message/service definitions.

To run this project successfully, you’ll need to:

- Set up a ROS2 workspace using `colcon`
- Create a custom interface package containing:
  - `Turtle` (message)
  - `TurtleArray` (message)
  - `CatchTurtle` (service)
- Add and configure `package.xml` and `CMakeLists.txt` for all packages
- Build your workspace using `colcon build`
- Source your environment:  
  ```bash
  source install/setup.bash
