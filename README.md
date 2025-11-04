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

----
## Step-by-step bringup & waypoint run guide for a real TurtleBot3 Waffle using ROS 2 Humble.

### Important assumptions: both machines are on the same LAN, you have ROS 2 Humble + TurtleBot3 packages installed on both, and your robot IP is 10.9.10.153 and your laptop IP is 10.9.10.29 (adjust if different).

### 🧩 0 — Quick checklist before starting

- Robot powered on, OpenCR board flashed and connected, motor power switch ON.
- You know robot IP (example 10.9.10.153).
- Map file exists on Remote PC (example ~/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/real_map.yaml).
- Both machines have same ROS 2 packages installed (turtlebot3, turtlebot3_bringup, turtlebot3_gazebo if needed, navigation2, nav2_waypoint_follower, etc.)

### 🧩 1 — Add persistent env (run on both machines once)

Add these lines to the bottom of ~/.bashrc on both robot and laptop (use nano ~/.bashrc):

```bash
# TurtleBot3 / ROS2 networking
source /opt/ros/humble/setup.bash
# If you have a local workspace:
[ -f ~/turtlebot3_ws/install/setup.bash ] && source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp    # or rmw_cyclonedds_cpp if you prefer
```

Then apply:
```bash
source ~/.bashrc
```

### 🧩 2 — Network & OS checks (both machines)

Run on both to confirm network and env:

```bash
# Confirm IPs
hostname -I

# Confirm env
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
echo $TURTLEBOT3_MODEL
echo $RMW_IMPLEMENTATION
```

If any differ, fix them before proceeding. If a firewall is active on laptop, temporarily disable while testing:

```bash
sudo ufw status
sudo ufw disable
```

### 🧩 3 — On the ROBOT SBC (do this first)

Open an SSH session to the robot or use its terminal:

```bash
ssh ubuntu@10.9.10.153   # or login directly
source ~/.bashrc
```

### 3.1 Launch bringup (drivers + sensors):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave this terminal open. Expected outputs: Connected to OpenCR / diff_drive_controller / LD08 driver or similar.


### 3.2 Quick robot checks (new terminal on robot)

```bash
source ~/.bashrc
ros2 node list
ros2 topic list | egrep 'scan|odom|tf|map|joint_states'
# Check LIDAR:
ros2 topic echo /scan --once
# Check odom:
ros2 topic echo /odom --once
```

If scan or odom missing: re-check OpenCR connections, LIDAR cable, firmware.


### 🧩 4 — On the REMOTE PC (your laptop)

Open a terminal and source:

```bash
source ~/.bashrc
```

### 4.1 Verify the laptop can reach the robot and discover ROS nodes

```bash
ping -c3 10.9.10.153
ros2 node list          # should list robot nodes (if discovery ok)
ros2 topic list | egrep 'scan|odom|tf|map'
```
If you only see /rosout and /parameter_events, DDS discovery not working — ensure ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=0 match the robot, and consider switching RMW_IMPLEMENTATION to rmw_cyclonedds_cpp on both sides (install package then set env).

### 🧩 5 — Launch Navigation2 (Remote PC)

Use your real map. Example command (adjust path to your real_map.yaml):

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=False \
  map:=$HOME/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/real_map.yaml
```
Watch the console — lifecycle managers should configure/activate managed nodes. If map_server or /amcl is unconfigured, you’ll see that in node list.

### 🧩 6 — Verify Nav2 & map server

In a new terminal (Remote PC):

```bash
source ~/.bashrc
ros2 node list | egrep 'map_server|amcl|planner_server|controller_server|waypoint_follower|lifecycle_manager'
# Check map data
ros2 topic list | grep map
ros2 topic echo /map --once
# Check AMCL lifecycle
ros2 lifecycle get /amcl
```

If map_server missing: re-run the launch ensuring map:=<path> is correct (file must exist). If /amcl is unconfigured or inactive, configure and activate:

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

If you have lifecycle managers:

```bash
ros2 lifecycle set /lifecycle_manager_localization configure
ros2 lifecycle set /lifecycle_manager_localization activate
ros2 lifecycle set /lifecycle_manager_navigation configure
ros2 lifecycle set /lifecycle_manager_navigation activate
```

### 🧩 7 — Open RViz and set fixed frame

If RViz didn't open automatically (Skip if already opened):

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz
```


In RViz
    - Global Options → Fixed Frame = map
    - Add / ensure the Map display is subscribed to /map (nav_msgs/OccupancyGrid)
    - Add LaserScan /scan, TF, RobotModel
    - Use 2D Pose Estimate (green arrow) to click robot's position on the map.


### 🧩 8 — Confirm localization (/amcl_pose)

```bash
ros2 topic info /amcl_pose
ros2 topic echo /amcl_pose
```

After clicking 2D Pose Estimate you should see continuous pose updates. If not: check TF chain.

Check TF chain:

```bash
ros2 run tf2_tools view_frames
# opens frames.pdf; confirm map -> odom -> base_link exists
ros2 run tf2_ros tf2_echo map base_link
```

If the TF chain missing, make sure the robot publishes odom and AMCL is active.


### 🧩 9 — If Needed: Test manual navigation (single goal)

Before waypoints, validate Nav2 can reach a simple goal (choose a safe nearby point inside map):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
If robot moves → Nav2 is healthy.


### 🧩 10 — Start waypoint follower (make sure only one)

Kill duplicates if any:

```bash
ros2 node list | grep waypoint_follower
ps aux | grep waypoint_follower
pkill -f waypoint_follower || true
```

Then start follower on Remote PC:

```bash
ros2 run nav2_waypoint_follower waypoint_follower
```

### 🧩 11 — If Not Available Yet: Prepare and validate your waypoint file

Waypoints must be PoseStamped with header.frame_id='map'. Example Python snippet:

```python
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

wp = PoseStamped()
wp.header.frame_id = 'map'
wp.pose.position = Point(x=1.0, y=0.5, z=0)
wp.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
# repeat for list of waypoints
```

Check valid coordinate ranges from /map metadata:

```bash
ros2 topic echo /map_metadata -n1
# compute map bounds from origin, width, height, resolution
```

You can visualize candidate waypoints by publishing markers before sending mission.


### 🧩 12 — Send the waypoints

On Remote PC:

```bash
ros2 run waypoint_sender_pkg send_waypoints
```

Expected log flow:

```css
[INFO] Waiting for /follow_waypoints action server...
[INFO] Connected to /follow_waypoints!
[INFO] Sending waypoints to Nav2 Waypoint Follower...
[INFO] Waypoint mission accepted
```

If rejected, proceed to Troubleshooting below.


### Troubleshooting — quick commands & causes

- If /map not visible in RViz

```bash
ros2 topic list | grep map
ros2 topic echo /map --once
ros2 node list | grep map_server
ros2 lifecycle get /map_server
```
Fix: relaunch Nav2 with correct map:=<path> and activate map_server.

- If /amcl_pose not publishing

```bash
ros2 lifecycle get /amcl
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 topic echo /amcl_pose --once
ros2 run tf2_tools view_frames
```

Fix: ensure /scan and TF exist, then activate AMCL.

- If waypoints rejected

Possible reasons: AMCL not localized, Nav2 nodes not active, waypoints out of map, multiple waypoint follower instances.
Check:

```bash
# Node states:
for n in /amcl /controller_server /planner_server /bt_navigator /waypoint_follower; do ros2 lifecycle get $n || true; done

# Debug logs:
ros2 run nav2_waypoint_follower waypoint_follower --ros-args --log-level debug
```

If DDS discovery issues between machines
  - Ensure same ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=0.
  - Try switching RMW: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (install package if needed).
  - Temporarily disable firewall.


### Only if Needed: Helpful one-liners

Activate common Nav2 nodes (runs configure+activate ignoring errors):

```bash
for n in /amcl /controller_server /planner_server /bt_navigator /waypoint_follower /smoother_server; do ros2 lifecycle set $n configure 2>/dev/null || true; ros2 lifecycle set $n activate 2>/dev/null || true; done
```

Kill duplicate waypoint followers:

```bash
pkill -f waypoint_follower || true
```

Check map metadata:

```bash
ros2 topic echo /map_metadata -n1
```

Echo cmd_vel to confirm teleop publishing:

```bash
ros2 topic echo /cmd_vel
```

