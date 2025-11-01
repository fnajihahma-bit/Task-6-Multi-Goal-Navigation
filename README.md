# Task-6-Multi-Goal-Navigation
(Waypoint Mission) with ROS2 Nav2


## 🗂 Project Structure

```markdown
waypoint_sender_pkg/
├── package.xml
├── setup.py
├── waypoint_sender_pkg/
│   ├── __init__.py
│   └── send_waypoints.py
├── README.md
└── .gitignore
```

## 📦 Package Description


waypoint_sender_pkg is a ROS 2 Python package that sends waypoint missions to a TurtleBot3 using Navigation2 and a YAML file.

It allows you to define a list of waypoints in a .yaml file and send them to the /follow_waypoints action server in Navigation2. This automates waypoint-following for your TurtleBot3 (Burger or Waffle) in simulation or real-world environments.

---

## ✅ Features Completed

### ✅ 1. Created a ROS 2 Workspace

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```

### ✅ 2. Created the Package

```bash
ros2 pkg create --build-type ament_python waypoint_sender_pkg
```

### ✅ 3. Added Dependencies in package.xml

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>tf-transformations</exec_depend>
```

And added in the <export> section:

```xml
<build_type>ament_python</build_type>
```

### ✅ 4. Wrote the Python Script

File: waypoint_sender_pkg/send_waypoints.py

Main logic:
- Load YAML file with waypoints
- Convert each to PoseStamped
- Send goal to /follow_waypoints action server

### ✅ 5. Added Entry Point in setup.py

```python
entry_points={
    'console_scripts': [
        'send_waypoints = waypoint_sender_pkg.send_waypoints:main',
    ],
}
```

### ✅ 6. Created Waypoints YAML File

File path: ~/waypoints.yaml

Example content:

```yaml
waypoints:
  - {x: 2.0, y: 1.0, theta: 0.0}
  - {x: -1.5, y: 2.5, theta: 1.57}
  - {x: 0.0, y: -2.0, theta: 3.14}
```


### ✅ 7. Installed tf-transformations and Fixed NumPy (NumPy had to be downgraded to avoid np.float error)

```bash
pip3 install tf-transformations numpy==1.23.5 --user
```

### ✅ 8. Built the Package

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

### ✅ 9. Ran the Script (Confirmed robot received and followed waypoints)

```bash
ros2 run waypoint_sender_pkg send_waypoints
```

## 🛠 How to Use GAZEBO Simulation (🧭 STEP-BY-STEP: Open everything manually)

### Step 1: 🧩 1️⃣ Source your workspace and set the model

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

### Step 2: 🧩 2️⃣ Launch Gazebo world

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
✅ You should now see Gazebo open with your robot in the world.

### Step 3: 🧩 3️⃣ Launch Navigation2 (Nav2 + AMCL + Map Server)

Use your map:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/sim_map.yaml
```
✅ Wait until all Nav2 lifecycle nodes are active.
You can check with:
```bash
ros2 lifecycle list
```
They should all say active.

### Step 4: 🧩 4️⃣ Open RViz2 manually

If it didn’t open automatically with step 3, run:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz
```
✅ RViz2 should now open with your map loaded.

### Step 5: 🧩 5️⃣ Set the Initial Pose in RViz
🧩 5️⃣ Set the Initial Pose in RViz

- Click “2D Pose Estimate” (green arrow icon)
- Click near where your robot is in Gazebo

You’ll see the AMCL warnings disappear.

### Step 6: 🧩 6️⃣ (Optional) Launch the Waypoint Follower

If your version doesn’t have a launch file, just start Nav2’s waypoint follower node manually:
```bash
ros2 run nav2_waypoint_follower waypoint_follower
```
(If this fails, we’ll confirm the correct executable name.)

### Step 7: 🧩 7️⃣ Finally — send your waypoints

Now that everything is running:
```bash
ros2 run waypoint_sender_pkg send_waypoints
```
✅ You should see:
```css
[INFO] Connected to /follow_waypoints!
[INFO] Sending waypoints to Nav2 Waypoint Follower...
```
And the robot will start moving along the waypoints in Gazebo.

### 📁 Optional Improvements

### 1. Move waypoints.yaml into the package directory

```bash
mkdir -p ~/turtlebot3_ws/src/waypoint_sender_pkg/waypoints
mv ~/waypoints.yaml ~/turtlebot3_ws/src/waypoint_sender_pkg/waypoints/
```

### 2. Update Script to Load from New Path

```python
from pathlib import Path

yaml_path = Path(__file__).parent.parent / 'waypoints' / 'waypoints.yaml'

```

### 📌 Notes

- Ensure all dependencies are sourced correctly before running the script.
- You may need to use --ros-args or --log-level if debugging.
- Test in Gazebo first before running on a real robot.



