# XGO Mini 2S Autonomous Quadruped Robot (TFM)

End-to-End Autonomous Navigation and Fleet Coordination System in Modern C++20 and ROS 2 Jazzy for the XGO Mini 2S quadruped robot dog.

## System Overview

* **Robot Platform:** Luwu Dynamics XGO Mini 2S (12-DOF Quadruped, 1.2 kg)
* **Compute Unit:** Raspberry Pi Compute Module 5 Lite (Quad-core ARM Cortex-A76 @ 2.4 GHz)
* **Embedded Bus:** Hardware UART `/dev/ttyAMA0` (115200 baud, 8N1)
* **LiDAR Sensor:** LDROBOT LD19 / D300 (360° 2D Laser Rangefinder, 12m)
* **Software Core:** Modern C++20, ROS 2 Jazzy Jalisco, Gazebo Harmonic, Nav2 (MPPI Controller), Docker

---

## Repository Branch Structure

This repository follows a strict **Sim-to-Real** branch architecture:

| Branch | Target Environment | Description |
| :--- | :--- | :--- |
| **`sim`** | Host PC / Simulation | Gazebo Harmonic digital twin, world models, sensor bridges, Nav2 MPPI simulation, SLAM mapping, and multi-robot testing. |
| **`funcional`** | Physical Robot (Raspberry Pi CM5) | Production embedded deployment with Docker (`compose.yaml`), C++20 hardware driver over `/dev/ttyAMA0`, real LiDAR node, and edge navigation. |

---

## Quick Navigation

* To run the **Gazebo simulation & Nav2 on PC**:
  ```bash
  git checkout sim
  ```
* To deploy on the **physical robot using Docker**:
  ```bash
  git checkout funcional
  ```
