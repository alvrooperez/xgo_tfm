# XGO Mini 2S - Functional Hardware Branch (`funcional`)

This branch contains the **production embedded deployment** for the physical **XGO Mini 2S quadruped robot** running on the **Raspberry Pi Compute Module 5 Lite (CM5)**, with fully decoupled Docker microservices and a virtual hardware simulator (Mock ESP32).

---

## 1. Decoupled Microservices Architecture (`compose.yaml`)

Following professional robotics standards, the system is separated into decoupled containers running over the host network:

```
┌────────────────────────────────────────────────────────┐
│             Host Network (ROS 2 Jazzy)                 │
│    /cmd_vel  │  /odom  │  /battery_state  │  /imu/data  │
└──────┬───────────────┬───────────────────┬─────────────┘
       │               │                   │
┌──────▼───────┐┌──────▼──────┐     ┌──────▼─────────────┐
│  xgo_driver  ││   xgo_nav   │     │    xgo_mock_mcu    │
│  (C++20 RT)  ││ (Nav2 MPPI) │     │ (Virtual Hardware) │
└──────┬───────┘└─────────────┘     └──────┬─────────────┘
       │                                   │
  /dev/ttyAMA0                       /tmp/tty_robot
(Hardware MCU)                     (Simulated ESP32)
```

1. **`xgo_driver` (Hardware Driver Container):**
   * High-performance C++20 driver (<0.5% CPU, 20.5x faster than vendor Python).
   * Manages hardware serial connection, safety watchdog (0.5s timeout), and publishes `/odom`, `/battery_state`, and `/imu/data`.
   * **Crash Isolation:** If navigation crashes or restarts, the driver stays alive and safely stops the dog.
2. **`xgo_nav` (Autonomous Navigation Container):**
   * Decoupled Nav2 stack with **omnidirectional MPPI local controller**, AMCL localization, and costmaps.
   * Can be restarted or reconfigured independently without touching the robot hardware bus.
3. **`xgo_mock_mcu` (Virtual ESP32 Hardware Simulator):**
   * Simulates the low-level MCU serial protocol over a virtual pseudo-terminal (`/tmp/tty_robot`).
   * Includes a live **visual ASCII terminal dashboard** showing robot motion, battery drain, and packet telemetry.

---

## 2. Production Deployment (On the Physical Raspberry Pi CM5)

From the repository root on the Raspberry Pi CM5:

```bash
docker compose up -d --build
```

To monitor logs:
```bash
docker compose logs -f
```

To stop:
```bash
docker compose down
```

---

## 3. Mock Testing & Deployment Flow (On PC without Robot)

You can test the entire deploy pipeline on your PC without having the physical robot connected.

### Option A: Automated Integration Test (1-liner)
Runs the Mock ESP32, starts the C++ driver, queries telemetry, sends `/cmd_vel` velocity, and verifies odometry integration:
```bash
./scripts/test_mock_deploy.sh
```

### Option B: Interactive Visual Dashboard with Docker Compose
1. In Terminal 1, run the Virtual ESP32 MCU with the live visual dashboard:
   ```bash
   docker compose --profile mock up xgo_mock_mcu
   ```
2. In Terminal 2, launch the C++ driver connected to the virtual port:
   ```bash
   SERIAL_PORT=/tmp/tty_robot docker compose up xgo_driver
   ```
3. In Terminal 3, send velocity commands to see the virtual dog move on the dashboard:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0}, angular: {z: 0.2}}"
   ```
