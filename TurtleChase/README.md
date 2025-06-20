# 🐢 Turtle Chase – ROS 2 TurtleSim

This repository contains a **single Python script** that defines the behavior of a "thief turtle" in the ROS 2 `turtlesim` simulation. The turtle dynamically reacts to a "police turtle" by fleeing when it comes too close, while also avoiding the simulation boundaries.

---

## 🧠 Project Logic Explained

- The **police turtle** is the default turtle in TurtleSim (`/turtle1`).
- A new turtle named **`thief_turtle`** is spawned at a fixed location using the `/spawn` service.
- The script subscribes to the `/turtle1/pose` and `/thief_turtle/pose` topics.
- It continuously calculates the distance between the thief and police turtles.
- If the **police gets within a certain threshold distance (2.0 units)**:
  - The thief computes an escape direction (opposite to the police).
  - It also ensures that the next move **won’t take it outside the visible boundary** of the simulator.
  - Based on angle difference and distance, the script publishes velocity commands to `/thief_turtle/cmd_vel`.
- If the police is farther than the threshold, the thief stops moving.

---

## 📁 What's in This Repository?

✅ `thief_turtle_node.py`  
❌ No full ROS 2 package setup (no `setup.py`, `package.xml`, or `CMakeLists.txt`)

---

## ⚠️ Note to Users

This repository **only contains the Python script** implementing the core logic.  
To run this in a ROS 2 environment, you will need to:

- Create your own ROS 2 package
- Place the script inside a `scripts/` or `src/` folder
- Add the necessary `setup.py`, `package.xml`, and other files
- Build the package using `colcon build`
- Run the node using `ros2 run`

---

## 👨‍💻 Author

**Name:** Harshak V P
**LinkedIn:** [linkedin.com/in/harshakvp](https://www.linkedin.com/in/harshakvp/)

This project was developed as a learning exercise in ROS 2 and turtlesim.  
Feel free to use, modify, and extend it as needed!

---
