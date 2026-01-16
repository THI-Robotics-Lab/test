# Chapter 3 — Sensors & Perception (Laser Scanner & Visualization)

This chapter introduces how a mobile robot perceives its environment using sensors,
focusing on the 2D LiDAR (Laser Scanner) available on TurtleBot3.

---

# 🎯 Learning Objectives

After completing this chapter, you will be able to:

✔ Understand LaserScan messages  
✔ Visualize sensor data in RViz  
✔ Interpret distance measurements  
✔ Relate robot motion to sensor readings  
✔ Write basic sensor-processing ROS2 nodes  

---

# ----------------------------------------------------
# 📝 Exercise 1 — Visualize LaserScan in RViz (Simulation)
# ----------------------------------------------------

## 🎯 Goal  
Understand how a 2D LiDAR perceives the environment.

---

## 🧩 Task Description

1. Launch the TurtleBot3 Gazebo simulation  
2. Open RViz  
3. Add a **LaserScan** display  
4. Set the topic to `/scan`  
5. Move the robot using teleop  
6. Observe how scan data changes near obstacles  

---

## 💡 Hints

<details>
  <summary>Which topic?</summary>
  <code>/scan</code>
</details>

<details>
  <summary>Nothing visible?</summary>
  Set RViz Fixed Frame to <code>odom</code> or <code>base_scan</code>
</details>

<details>
  <summary>Why do points move?</summary>
  The LiDAR measures distances relative to the robot frame.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 2 — Inspect LaserScan Data via CLI
# ----------------------------------------------------

## 🎯 Goal  
Understand the structure of a LaserScan message.

---

## 🧩 Task Description

1. Echo the scan topic:

```bash
ros2 topic echo /scan
```

2. Identify the Following Fields

- `angle_min`  
- `angle_max`  
- `angle_increment`  
- `ranges[]`  

3. Move the robot and observe how the values change.

---

## 💡 Hints

<details>
  <summary>What does <code>ranges[]</code> mean?</summary>
  Each value represents the distance to an obstacle at a specific angle.
</details>

<details>
  <summary>Why are there many values?</summary>
  LiDAR scans the environment over a wide angular range.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 3 — Obstacle Distance Detector (Custom Node)
# ----------------------------------------------------

## 🎯 Goal

Create a ROS2 node that detects nearby obstacles.

---

## 🧩 Task Description

1. Create a ROS2 Python node  
2. Subscribe to the `/scan` topic  
3. Compute the **minimum valid distance** in `ranges[]`  
4. Print a warning if the distance is below a defined threshold  
5. Test the node by driving the robot near walls  

---

## 💡 Hints

<details>
  <summary>Message type?</summary>
  <code>sensor_msgs/msg/LaserScan</code>
</details>

<details>
  <summary>Invalid values?</summary>
  Ignore <code>inf</code> and <code>nan</code> values in <code>ranges[]</code>.
</details>
