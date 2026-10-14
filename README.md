# XGO Mini 2S - Simulation Branch (`sim`)

This branch contains the **Gazebo Harmonic** digital twin, sensor bridges, SLAM mapping, and **Nav2 autonomous navigation** with MPPI local controller.

## Features

* **Gazebo Harmonic (`gz-sim8`):** Physics simulation with enclosed room world (`worlds/room.sdf`).
* **URDF & Plugins:** Exact XGO Mini 2S kinematic model with `gz-sim-velocity-control-system` and `gz-sim-odometry-publisher-system`.
* **Simulated Sensors:**
  * 360° LiDAR (`/scan`) matching LD19/D300 specs.
  * 6-DOF IMU (`/imu/data`).
  * Odometry (`/odom` and TF tree).
* **SLAM Toolbox (`slam_toolbox`):** Online asynchronous 2D mapping with automated lifecycle management.
* **AMCL & Map Server (`nav2_amcl`, `nav2_map_server`):** Global localization on pre-built room maps.
* **Autonomous Navigation (`nav2_bringup`):** Full Nav2 stack with **omnidirectional MPPI local controller** (`motion_model: "Omni"`).

---

## Build Instructions

```bash
cd /home/alvaro/Desktop/TFM/xgo_tfm
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Modes

### 1. Autonomous Navigation with Nav2 (Recommended)
Loads the room map, AMCL localization, Gazebo simulation, and Nav2 MPPI controller:
```bash
ros2 launch xgo_sim nav.launch.py
```
*In RViz2, click the **"Nav2 Goal"** tool and click anywhere on the map to navigate autonomously.*

### 2. SLAM Mapping Mode
Explore and generate a new 2D room map:
```bash
ros2 launch xgo_sim mapping.launch.py
```
*In a second terminal, drive the dog:*
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Save the generated map:*
```bash
./scripts/save_map.sh my_map
```

### 3. Static Localization Mode (AMCL only)
```bash
ros2 launch xgo_sim localization.launch.py
```

### 4. Raw Simulation & Teleop
```bash
ros2 launch xgo_sim sim.launch.py
```
