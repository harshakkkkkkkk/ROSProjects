## ⚠️ Note to Users

This repository includes the **core source code** for the project (main logic and functionality). It **does not include** full configuration files such as `setup.py`, `package.xml`, `CMakeLists.txt`, or custom message/service definitions.

To run this project successfully, you’ll need to:

- Set up a ROS2 workspace using `colcon`
- Create a custom interface package if needed (e.g., messages or services)
- Add and configure the required `package.xml` and `CMakeLists.txt` files
- Build your workspace using:
  ```bash
  colcon build
- Source your environment:
  ```basg
  source install/setup.bash
