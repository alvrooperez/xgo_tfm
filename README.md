# XGO Mini 2S - Functional Hardware Branch (`funcional`)

This branch contains the **production embedded deployment** for the physical **XGO Mini 2S quadruped robot** running on the **Raspberry Pi Compute Module 5 Lite (CM5)**.

## Key Hardware Specifications

* **Robot Platform:** Luwu Dynamics XGO Mini 2S (12-DOF Quadruped)
* **Compute Board:** Raspberry Pi Compute Module 5 Lite (Quad-core ARM Cortex-A76 @ 2.4 GHz)
* **Embedded Serial Bus:** `/dev/ttyAMA0` (115200 baud, 8N1)
* **Driver Performance:** High-performance C++20 core library (**20.5x faster** than vendor Python implementation, <0.5% CPU load)

---

## Production Docker Deployment

The physical robot runs a containerized ROS 2 Jazzy stack configured for ARM64 (`aarch64`) architecture with direct hardware device mapping and host networking.

### Requirements on the Raspberry Pi CM5

* OS: Debian 12 (Bookworm) / Ubuntu 24.04 aarch64
* Docker & Docker Compose plugin installed:
  ```bash
  sudo apt update && sudo apt install -y docker.io docker-compose-plugin
  sudo usermod -aG docker $USER
  ```
* Disable any background Python scripts holding `/dev/ttyAMA0`:
  ```bash
  sudo killall python python3 2>/dev/null || true
  ```

---

## Quick Start (Physical Robot)

### 1. Build and Run Container
From the repository root on the Raspberry Pi:
```bash
docker compose up --build -d
```

### 2. View Real-time Telemetry & Logs
```bash
docker compose logs -f
```

### 3. Check Active ROS 2 Topics
In a container shell or on the host network:
```bash
ros2 topic list
```
Published topics:
* `/odom` (`nav_msgs/msg/Odometry`) - Real-time 2D dead-reckoning odometry and TF transform
* `/battery_state` (`sensor_msgs/msg/BatteryState`) - Battery percentage (0-100%) and voltage
* `/imu/data` (`sensor_msgs/msg/Imu`) - Real-time Roll, Pitch, Yaw from internal MCU IMU

Subscribed topics:
* `/cmd_vel` (`geometry_msgs/msg/Twist`) - Safe velocity commands ($v_x, v_y, \omega$) with 0.5s watchdog timeout

### 4. Stop the Robot
```bash
docker compose down
```

---

## Native Build (Without Docker)

```bash
colcon build --packages-select xgo_driver_cpp
source install/setup.bash
ros2 launch xgo_driver_cpp xgo_driver.launch.py
```
