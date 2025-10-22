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

## waypoint_sender_pkg

A ROS 2 Python package that sends waypoint missions to TurtleBot3 using Navigation2 and a YAML file.

---

## 📌 Overview

This package lets you define a set of waypoints in a `.yaml` file and send them to the `/follow_waypoints` action server in Navigation2. It automates waypoint-following for your TurtleBot3 (Burger or Waffle) using ROS 2.

---

## ✅ What Has Been Done

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

### ✅ 5. Created Entry Point in setup.py

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

## 🚀 How to Use

### 1. Make sure Navigation2 is running:

```bash
export TURTLEBOT3_MODEL=burger  # or waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:=$HOME/real_map.yaml
```

### 2. Start Waypoint Follower

```bash
ros2 launch nav2_waypoint_follower waypoint_follower.launch.py use_sim_time:=True
```

### 3. Run the Script

```bash
ros2 run waypoint_sender_pkg send_waypoints
```

### ✅ Final Recommendations (Optional)

### 1. Move waypoints.yaml into a waypoints/ folder:

```bash
mkdir -p ~/turtlebot3_ws/src/waypoint_sender_pkg/waypoints
mv ~/waypoints.yaml ~/turtlebot3_ws/src/waypoint_sender_pkg/waypoints/
```

### 2. Update your script to read from the new path:

```python
yaml_path = str(Path(__file__).parent.parent / 'waypoints' / 'waypoints.yaml')
```





