# 🔹 Hunt The Turtle – ROS 2 TurtleSim

This project simulates a dynamic game of **turtle chasing** using ROS 2 and the `turtlesim` simulator. A central controller (`turtle1`) is used to **chase and remove spawned turtles** from the simulation.

<p align="center">
  <img src="https://github.com/user-attachments/assets/40829d57-f72f-4df8-a7dd-6d96f7fd7e38" width="100%" />
</p>

---

## 🔹 Project Logic

The system involves two main components:

### 🔹 Turtle Spawner Node (`TurtleSpawnerNode`)
- Periodically **spawns turtles** at random positions within the simulator.
- Each turtle is given a **unique name** using a configurable prefix (`turtle1`, `turtle2`, ...).
- Maintains and **publishes a list** of all currently alive turtles using a custom `TurtleArray` message.
- Offers a service `catch_turtle` to **remove a specific turtle** from the simulation (via the `/kill` service).

### 🔹 Turtle Controller Node (`TurtleControllerNode`)
- Subscribes to the `alive_turtles` topic.
- Tracks the pose of the main turtle (`/turtle1`) and all alive turtles.
- Depending on a parameter (`catch_closest_turtle`), it either:
  - Chases the **first turtle** in the list, or
  - Calculates and chases the **closest turtle**.
- Once `turtle1` is **close enough to a target turtle**, it calls the `catch_turtle` service to remove it from the simulation.

---

## 🔹 What's Included?

-  `turtle_controller_node.py`  
-  `turtle_spawner_node.py`  
-  No full ROS 2 package setup (`setup.py`, `package.xml`, etc.)

---

## ⚠️ Note to Users

This repository **only contains the core Python scripts**.

To run the system, you will need to:

1. Create a ROS 2 Python package.
2. Add the scripts to your package's `scripts/` or `src/` directory.
3. Define custom messages and services (`Turtle`, `TurtleArray`, `CatchTurtle`) in your interface files (`msg/` and `srv/`).
4. Update `setup.py`, `package.xml`, and `CMakeLists.txt` accordingly.
5. Build the package with `colcon build`.

---

## 🔹 Example Parameters (Optional)

You can adjust parameters like spawn frequency or behavior preference:

```yaml
# Turtle Spawner Node
spawn_frequency: 0.5           # Spawn a turtle every 0.5 seconds
turtle_name_prefix: "turtle"

# Turtle Controller Node
catch_closest_turtle: true     # If true, controller will catch the closest turtle
```

---

## 🔹 Author

**Name:** Harshak V P  
**LinkedIn:** [linkedin.com/in/harshakvp](https://www.linkedin.com/in/harshakvp/)

This project was developed as a learning exercise in ROS 2 and turtlesim.  
Feel free to use, modify, and extend it as needed!

---
