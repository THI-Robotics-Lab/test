# Chapter 5 — System Integration & Launching

This chapter focuses on integrating multiple ROS2 nodes into a complete system.
Students learn how to properly structure, configure, and launch robotic applications.

---

# 🎯 Learning Objectives

After completing this chapter, you will be able to:

✔ Use ROS2 launch files  
✔ Start multiple nodes with a single command  
✔ Manage parameters using YAML files  
✔ Structure complete ROS2 systems  
✔ Run reproducible robotic applications  

---

# ----------------------------------------------------
# 📝 Exercise 1 — Create a ROS2 Launch File
# ----------------------------------------------------

## 🎯 Goal  
Launch multiple ROS2 nodes using a single command.

---

## 🧩 Task Description

1. Create a ROS2 launch file (Python)  
2. Launch:
   - TurtleBot3 simulation **or** real robot bringup  
   - Your custom autonomous node  
3. Start the entire system with one command  

---

## 💡 Hints

<details>
  <summary>Launch file language?</summary>
  Python
</details>

<details>
  <summary>How to run a launch file?</summary>
  <code>ros2 launch &lt;package_name&gt; &lt;file_name&gt;.launch.py</code>
</details>

<details>
  <summary>Why use launch files?</summary>
  They ensure repeatable and organized system startup.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 2 — Parameter Tuning via YAML
# ----------------------------------------------------

## 🎯 Goal  
Modify robot behavior without changing source code.

---

## 🧩 Task Description

1. Define parameters in a YAML file:
   - Linear velocity  
   - Angular velocity  
   - Obstacle distance threshold  
2. Load parameters into your node using a launch file  
3. Change robot behavior by editing the YAML file only  

---

## 💡 Hints

<details>
  <summary>Why YAML?</summary>
  Separates configuration from implementation logic.
</details>

<details>
  <summary>Where are parameters loaded?</summary>
  In the launch file using the <code>parameters</code> argument.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 3 — Mini Project (System Integration)
# ----------------------------------------------------

## 🎯 Goal  
Integrate all learned concepts into a single autonomous system.

---

## 🧩 Task Description

Design a ROS2 system that:

✔ Launches using one command  
✔ Moves autonomously  
✔ Detects and avoids obstacles  
✔ Uses configurable parameters  
✔ Works in simulation  
✔ (Optional) Works on the real TurtleBot3  

---

## 💡 Hints

<details>
  <summary>Think like a system engineer</summary>
  Nodes, topics, parameters, launch files must work together.
</details>

<details>
  <summary>Keep it simple</summary>
  Focus on stability and clarity rather than complex behavior.
</details>

---

# ✔ End of Chapter 5
