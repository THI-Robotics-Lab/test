# Chapter 1 — ROS2 Basics (Practical Exercises)

This chapter introduces the essential concepts of ROS2 through four progressive exercises.  
Each concept is learned first using an **existing ROS2 tool**, then implemented by **writing your own node**.

---

# 🎯 Learning Objectives

After completing this chapter, you will be able to:

✔ Use ROS2 command-line tools  
✔ Understand nodes, topics, and messages  
✔ Run existing ROS2 demo nodes  
✔ Create your own ROS2 packages  
✔ Implement publishers & subscribers  
✔ Control Turtlesim with teleop  
✔ Program robot motion using Twist  
✔ Use parameters and timers  
✔ Understand basic launch file usage  

---

# ----------------------------------------------------
# 📝 Exercise 1 — Use Existing Talker & Listener  
# ----------------------------------------------------

## 🎯 Goal  
Understand ROS2 topics and message flow **without writing code**.

---

## 🧩 Task Description  
1. Run ROS2’s built-in talker node  
2. Run ROS2’s built-in listener node  
3. Observe the communication between them  
4. Use ROS2 CLI tools to inspect topics, nodes, and messages  

---

## 📈 What You Learn

- How ROS2 message passing works  
- How subscribers receive data  
- How to inspect topics with CLI tools  
- How ROS2 demo nodes are structured  

---

## 💡 Hints

<details>
  <summary>How to run the talker?</summary>
  <code>ros2 run demo_nodes_cpp talker</code>
</details>

<details>
  <summary>How to run the listener?</summary>
  <code>ros2 run demo_nodes_cpp listener</code>
</details>

<details>
  <summary>How to inspect messages?</summary>
  <code>ros2 topic echo /chatter</code>
</details>

<details>
  <summary>How to list topics?</summary>
  <code>ros2 topic list</code>
</details>

---

# ----------------------------------------------------
# 📝 Exercise 2 — Custom Publisher & Subscriber  
# ----------------------------------------------------

## 🎯 Goal  
Create your own ROS2 package with a custom publisher and subscriber.

---

## 🧩 Task Description  
1. Create a new ROS2 package  
2. Write your own **publisher** node  
3. Write your own **subscriber** node  
4. Register executables in `setup.py`  
5. Build and run both nodes in separate terminals  

---

## 📈 What You Learn

- How to write ROS2 Python nodes  
- How publishing & subscribing work  
- How to register executables in `setup.py`  
- How to use timers in ROS2  
- How messages flow between custom nodes  

---

## 💡 Hints

<details>
  <summary>Which message type to use?</summary>
  <code>std_msgs/msg/String</code>
</details>

<details>
  <summary>Subscriber prints nothing?</summary>
  Make sure both nodes use the **same topic**.
</details>

<details>
  <summary>Where to add executables?</summary>
  <code>setup.py</code> → <code>console_scripts</code>
</details>

<details>
  <summary>How to build only your package?</summary>
  <code>colcon build --packages-select &lt;your_package&gt;</code>
</details>

---

# ---------------------------------------------------------
# 📝 Exercise 3 — Turtlesim Teleop  
# ---------------------------------------------------------

## 🎯 Goal  
Control a simulated robot using ROS2’s existing teleop node.

---

## 🧩 Task Description  
1. Start the Turtlesim simulator  
2. Start the teleop node  
3. Control turtle movement  
4. Observe velocity messages being published  

---

## 📈 What You Learn

- Manual robot control through teleop  
- How Twist messages encode robot motion  
- How topics update in real-time  
- How teleop nodes publish velocity commands  

---

## 💡 Hints

<details>
  <summary>How to start Turtlesim?</summary>
  <code>ros2 run turtlesim turtlesim_node</code>
</details>

<details>
  <summary>How to start teleop?</summary>
  <code>ros2 run turtlesim turtle_teleop_key</code>
</details>

<details>
  <summary>Which topic does teleop publish to?</summary>
  <code>/turtle1/cmd_vel</code>
</details>

<details>
  <summary>How to inspect teleop commands?</summary>
  <code>ros2 topic echo /turtle1/cmd_vel</code>
</details>

---

# ---------------------------------------------------------
# 📝 Exercise 4 — Draw a Circle using Turtlesim  
# ---------------------------------------------------------

## 🎯 Goal  
Write a ROS2 node that moves the turtle in a **smooth circular path** using Twist messages.

---

## 🧩 Task Description  
1. Create a new Python node  
2. Publish `Twist` messages at a fixed rate  
3. Control the turtle to move in a **circle**  
4. Add parameters for:  
   - Linear speed  
   - Angular speed  
   - Radius (optional)  
5. (Optional) Create a launch file that starts both turtlesim and your node  

---

## 📈 What You Learn

- How to publish velocity commands  
- How linear & angular motion combine  
- How to use timers in ROS2 nodes  
- How parameters make nodes configurable  
- Basics of writing launch files  


---

## 💡 Hints

<details>
  <summary>Which topic to publish to?</summary>
  <code>/turtle1/cmd_vel</code>
</details>

<details>
  <summary>Message type?</summary>
  <code>geometry_msgs/msg/Twist</code>
</details>

<details>
  <summary>How to create a circle?</summary>
  Set:
  <br>• <code>linear.x</code> to a positive value
  <br>• <code>angular.z</code> to a non-zero value
</details>

<details>
  <summary>Circle too small?</summary>
  Lower <code>angular.z</code> or increase <code>linear.x</code>
</details>

<details>
  <summary>Turtle not moving?</summary>
  Ensure you publish continuously (e.g., timer at 10 Hz).
</details>

---

# ✔ End of Chapter 1
