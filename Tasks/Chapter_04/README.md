# Chapter 4 — Autonomous Behavior (Reactive Control)

This chapter introduces autonomous robot behavior based on sensor feedback.
Students move from manual control to **reactive decision-making**, where the robot
responds automatically to its environment.

---

# 🎯 Learning Objectives

After completing this chapter, you will be able to:

✔ Understand closed-loop control  
✔ Combine sensing and actuation  
✔ Implement reactive robot behaviors  
✔ Write autonomous ROS2 nodes  
✔ Deploy behavior in simulation and on real hardware  

---

# ----------------------------------------------------
# 📝 Exercise 1 — Stop-on-Obstacle Behavior
# ----------------------------------------------------

## 🎯 Goal  
Make the robot automatically stop when an obstacle is detected in front of it.

---

## 🧩 Task Description

1. Create a ROS2 Python node  
2. Subscribe to the `/scan` topic  
3. Publish velocity commands to `/cmd_vel`  
4. If an obstacle is closer than a defined threshold:
   - Publish zero velocity  
5. If no obstacle is detected:
   - Move the robot forward  

---

## 💡 Hints

<details>
  <summary>Velocity message type?</summary>
  <code>geometry_msgs/msg/Twist</code>
</details>

<details>
  <summary>Why publish continuously?</summary>
  Velocity commands expire if they are not updated regularly.
</details>

<details>
  <summary>Robot not stopping?</summary>
  Ensure the threshold distance is large enough.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 2 — Simple Obstacle Avoidance
# ----------------------------------------------------

## 🎯 Goal  
Make the robot avoid obstacles instead of stopping completely.

---

## 🧩 Task Description

1. Subscribe to the `/scan` topic  
2. Divide the LaserScan data into regions:
   - Front  
   - Left  
   - Right  
3. Implement the following behavior:
   - Obstacle in front → rotate  
   - Path clear → move forward  
4. Test the behavior in Gazebo  

---

## 💡 Hints

<details>
  <summary>How to divide the scan?</summary>
  Use index ranges in <code>ranges[]</code>.
</details>

<details>
  <summary>Robot oscillates?</summary>
  Reduce angular speed or increase the obstacle threshold.
</details>

<details>
  <summary>Behavior feels unstable?</summary>
  This is expected for simple reactive logic.
</details>

---

# ----------------------------------------------------
# 📝 Exercise 3 — Run Autonomous Behavior on the Real Robot
# ----------------------------------------------------

## 🎯 Goal  
Deploy your reactive control algorithm on the real TurtleBot3.

---

## 🧩 Task Description

1. Turn on the TurtleBot3  
2. Start the robot bringup:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
3. Run your autonomous node from your laptop

4. Observe robot behavior in the real environment

5. Compare simulation vs real-world performance

## 💡 Hints
<details> 
    <summary>Robot behaves differently than simulation?</summary> 
    Real sensors introduce noise and delays. 
</details> 

<details> 
    <summary>Robot too slow?</summary> 
    Increase linear velocity slightly. 
</details>

<details> 
    <summary>Emergency stop?</summary>
    Press <code>Ctrl+C</code> to stop your node immediately. 
</details>