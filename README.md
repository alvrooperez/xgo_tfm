# TFM — Brigada Multi-Robot RASTREO (XGO Mini 2S + ROS 2 Jazzy)

Repositorio unificado del Trabajo de Fin de Máster para la arquitectura de control, navegación autónoma, simulación multi-robot y despliegue en hardware de una brigada de cuadrúpedos **XGO Mini 2S** sobre **ROS 2 Jazzy**.

> El driver base en C++20 se mantiene también de forma independiente en [`alvrooperez/xgo_driver_cpp`](https://github.com/alvrooperez/xgo_driver_cpp).  
> Las ramas históricas `sim` y `funcional` se conservan en el repositorio como referencia previa a la unificación en `main`.

---

## Plataforma hardware y pila software

- **Plataforma robótica:** Luwu Dynamics XGO Mini 2S (cuadrúpedo 12-DOF, 1.2 kg)
- **Unidad de cómputo:** Raspberry Pi Compute Module 5 Lite (Quad-core ARM Cortex-A76 @ 2.4 GHz)
- **Bus embebido:** UART hardware `/dev/ttyAMA0` (115200 baud, 8N1)
- **Sensor LiDAR:** LDROBOT LD19 / D300 (360° 2D, 12 m)
- **Núcleo software:** C++20, ROS 2 Jazzy Jalisco, Gazebo Harmonic, Nav2 (MPPI), BehaviorTree.CPP v4 + Groot2, Docker Compose

---

## Estructura del repositorio

```text
TFM/
├── xgo_behavior_tree/   # Ejecutor BehaviorTree.CPP v4 (C++20), nodos custom, XML y Groot2
├── xgo_sim/             # Simulación Gazebo Harmonic 3D (single y flota de 3 robots), URDF, SLAM y Nav2
├── xgo_tfm/             # Despliegue hardware embebido (CM5), driver C++20, Docker Compose y Mock ESP32
├── scripts/             # Lanzadores integrados de flota (Gazebo / Mock) y despachador de misiones
└── docs/
    ├── diseno_bt/       # Catálogo de misiones M1-M6, análisis de escenarios y diseño del BT
    └── memoria_latex/   # Memoria completa del TFM en LaTeX
```

---

## Compilación (ROS 2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash

# Compilar los tres paquetes ROS 2
(cd xgo_behavior_tree && colcon build --symlink-install)
(cd xgo_sim && colcon build --symlink-install)
(cd xgo_tfm && colcon build --symlink-install)
```

---

## Ejecución rápida

### 1. Flota de 3 robots en Gazebo Harmonic 3D + Behavior Trees
```bash
./scripts/launch_fleet_gazebo_and_bt.sh --with-missions
# O ciclo completo (tránsito -> SWEEP_AREA -> RTB):
./scripts/launch_fleet_gazebo_and_bt.sh --full-cycle
# Modo sin interfaz gráfica:
./scripts/launch_fleet_gazebo_and_bt.sh --headless --with-missions
```

- **Groot2** escucha en los puertos `1667` (`robot1`), `1668` (`robot2`) y `1669` (`robot3`).
- Los ficheros `.btlog` se graban por defecto en `/tmp/xgo_bt_logs/` (configurable mediante `log_dir`).

### 2. Simulación ligera con Mock ESP32 (GUI Tkinter) + Drivers C++ + Behavior Trees
```bash
./scripts/launch_fleet_mock_and_bt.sh --with-missions
```

### 3. Despacho manual de misiones
```bash
python3 scripts/dispatch_missions.py --fleet
python3 scripts/dispatch_missions.py --robot robot1 --mission GOTO_POINT --args 1.2 0.8
python3 scripts/dispatch_missions.py --cancel
```

### 4. Despliegue en hardware real (Raspberry Pi CM5 con Docker Compose)
```bash
cd xgo_tfm
docker compose up -d --build
```
Consulta [xgo_tfm/README.md](xgo_tfm/README.md) para el detalle del despliegue en microservicios y pruebas con el perfil `mock`.
