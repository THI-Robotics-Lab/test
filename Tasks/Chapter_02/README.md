# Chapter 2 — Robot Locomotion (TurtleBot3 Simulation & Real Robot)

This chapter introduces basic robot locomotion concepts using the TurtleBot3 in both **Gazebo simulation** and **real hardware**.  
No solutions are provided — only tasks and hints.

---

# ----------------------------------------------------
# 📝 Exercise 1 — Run TurtleBot3 in Gazebo & Move It  
# ----------------------------------------------------

## 🎯 Goal  
Launch the TurtleBot3 simulator and control the robot manually.

---

## 🧩 Task Description  

1. Launch the TurtleBot3 Gazebo simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. In another terminal, start teleop control:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

3. Drive the robot using the keyboard:
   - Forward / backward  
   - Rotate left / right  
   - Combined movements  

4. Observe velocity commands by echoing `/cmd_vel`:

```bash
ros2 topic echo /cmd_vel
```

---

## 💡 Hints

<details>
  <summary>Teleop not responding?</summary>
  Ensure the teleop terminal is selected (active window).
</details>

<details>
  <summary>Robot not moving?</summary>
  Gazebo may be paused — press the ▶ button.
</details>

<details>
  <summary>How to stop the robot?</summary>
  Release all keys or press <code>Ctrl+C</code> in teleop.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 2 — Visualize Simulation Odometry & TF  
# ----------------------------------------------------

## 🎯 Goal  
Inspect the robot’s pose estimation and coordinate frames in simulation.

---

## 🧩 Task Description  

1. Ensure the simulation is running  
2. Echo odometry data:

```bash
ros2 topic echo /odom
```

3. Generate a TF frames PDF:

```bash
ros2 run tf2_tools view_frames.py
```
4. Open PDF using Evince:
```bash
evince __.pdf
```

5. Open RViz and add:
   - TF  
   - Odometry  
   - RobotModel  

6. Move the robot using teleop and observe changes in:
   - `/odom` values  
   - TF tree (`map → odom → base_link`)  

---

## 💡 Hints


<details>
  <summary>Why does odom change?</summary>
  It updates continuously as the robot moves.
</details>

<details>
  <summary>No TFs visible in RViz?</summary>
  Make sure the Fixed Frame is set to <code>odom</code> .
</details>

---

# ----------------------------------------------------
# 📝 Exercise 3 — Move the Real TurtleBot3 with Teleop  
# ----------------------------------------------------

## 🎯 Goal  
Control the real TurtleBot3 on the floor using teleop and inspect velocity commands.

---

## 🧩 Task Description  

1. Turn on the TurtleBot3  
2. SSH into the robot:

```bash
ssh ubuntu@<robot-ip>
```

3. Start the robot bringup:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

4. On your laptop, start teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

5. Observe the real robot movement  

---

## 💡 Hints

<details>
  <summary>Default SSH password?</summary>
  <code>turtlebot</code>
</details>

<details>
  <summary>Robot not responding?</summary>
  Ensure your laptop and robot are on the same WiFi.
</details>

<details>
  <summary>How to stop quickly?</summary>
  Press <code>Ctrl+C</code> in the teleop terminal.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 4 — View Real Robot Odometry  
# ----------------------------------------------------

## 🎯 Goal  
Read the odometry of the real TurtleBot3 and observe how its pose updates.

---

## 🧩 Task Description  

1. Ensure the robot bringup is running  
2. Echo real odometry:

```bash
ros2 topic echo /odom
```

3. Open RViz and add:
   - TF  
   - Odometry  
   - RobotModel  

4. Move the robot using teleop and observe:
   - Position  
   - Orientation (quaternion)  
   - TF frame changes  

---

## 💡 Hints

<details>
  <summary>Odom not updating?</summary>
  The robot must physically move for changes to appear.
</details>

<details>
  <summary>Robot drifting?</summary>
  Wheel odometry naturally drifts — this is expected.
</details>

<details>
  <summary>No TF frames?</summary>
  Ensure <code>robot.launch.py</code> is running on the robot.
</details>

---

# ✔ End of Chapter 2
