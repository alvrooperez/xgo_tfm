# Fase 2: Catálogo de Nodos Hoja y Contrato del Blackboard
**Proyecto:** RASTREO - Orquestador de Misiones y Behavior Trees  
**Plataforma:** Flota de Cuadrúpedos XGO Mini 2S (Raspberry Pi CM5)  
**Fecha:** 2026-10-21  
**Estado:** Cerrado y Validado contra Catálogo de Misiones (M1 - M6)  

---

## 1. Diccionario de Datos del Blackboard

Variables fuertemente tipadas que se almacenan en la memoria compartida del Behavior Tree:

| Clave en Blackboard | Tipo C++ / ROS 2 | Descripción |
|:---|:---|:---|
| `has_task` | `bool` | Indicador booleano de si el orquestador ha asignado una misión activa. |
| `task_type` | `std::string` | Tipo de misión asignada (`"MAP_UNKNOWN"`, `"SWEEP_AREA"`, `"GOTO_POINT"`, `"DEPLOY_CABLE"`, `"RTB"`). |
| `goal_pose` | `geometry_msgs::msg::PoseStamped` | Coordenadas $(x,y,\theta)$ de destino enviadas a Nav2. |
| `waypoints_list` | `std::vector<PoseStamped>` | Secuencia de waypoints para barrido o retrazado del cable guía. |
| `current_pose` | `geometry_msgs::msg::PoseStamped` | Posición y orientación actual del robot en el marco `map`. |
| `target_pose` | `geometry_msgs::msg::PoseStamped` | Posición estimada de la víctima/objetivo detectado. |
| `battery_level` | `double` | Porcentaje de carga restante de la batería ($0.0 - 100.0$). |
| `battery_threshold` | `double` | Umbral crítico de retorno a base (por defecto $15.0\%$). |
| `arm_action_type` | `std::string` | Comando para el brazo robótico (`"DEPLOY_CABLE"`, `"PARK"`, `"GRIP"`). |
| `wifi_timeout_s` | `double` | Tiempo máximo sin señal de la estación base (por defecto $5.0$ s). |
| `wait_duration_s` | `double` | Segundos de espera para ceder el paso o realizar pausas (por defecto $3.0$ s). |
| `map_name` | `std::string` | Nombre del archivo para guardar el mapa SLAM generado. |
| `alert_message` | `std::string` | Mensaje textual de telemetría o evento transmitido a la base. |

---

## 2. Catálogo de Nodos de Condición (`BT::ConditionNode`)

Nodos de inspección instantánea (solo lectura, sin efectos secundarios, tiempo $\approx 0$ ms):

| Nombre del Nodo | Puertos Entrada | Puertos Salida | Lógica de Retorno |
|:---|:---|:---|:---|
| `IsEmergencyStopActive` | Ninguno | Ninguno | `SUCCESS` si el operador humano activó E-STOP; `FAILURE` en caso contrario. |
| `HasFallen` | `tilt_threshold_deg` | Ninguno | `SUCCESS` si roll o pitch de la IMU superan el umbral (robot volcado). |
| `IsBatteryLow` | `threshold` | Ninguno | `SUCCESS` si `battery_level < threshold`. |
| `HasWifiConnection` | `timeout_s` | Ninguno | `SUCCESS` si el heartbeat de la estación base llega dentro del timeout. |
| `HasTaskAssigned` | Ninguno | Ninguno | `SUCCESS` si `has_task == true` (hay trabajo que hacer). |
| `IsTaskType` | `expected_type` | Ninguno | `SUCCESS` si `task_type == expected_type` (coincide con la misión evaluada). |
| `IsTargetDetected` | Ninguno | `target_pose` | `SUCCESS` si los sensores identifican a la víctima; escribe pose en Blackboard. |
| `IsTargetLost` | `timeout_s` | Ninguno | `SUCCESS` si transcurren $> T$ s sin confirmación visual de la víctima. |
| `IsPathBlocked` | Ninguno | Ninguno | `SUCCESS` si el planificador de Nav2 no encuentra ruta válida. |
| `IsLocated` | `cov_threshold` | Ninguno | `SUCCESS` si la traza de la covarianza de AMCL es inferior al umbral. |

---

## 3. Catálogo de Nodos de Acción (`BT::ActionNodeBase`)

Habilidades físicas y de comunicación (asíncronas, devuelven `RUNNING` mientras operan y cancelan en `onHalted()`):

| Nombre del Nodo | Puertos Entrada | Puertos Salida | Interfaz ROS 2 / Driver |
|:---|:---|:---|:---|
| `NavigateToPose` | `goal` | Ninguno | Acción ROS 2: `nav2_msgs/action/NavigateToPose`. Nav2 genera la ruta y esquiva obstáculos. |
| `NavigateThroughPoses` | `waypoints` | Ninguno | Acción ROS 2: `nav2_msgs/action/NavigateThroughPoses` para múltiples puntos seguidos. |
| `StopRobot` | Ninguno | Ninguno | Cancela goal activo de Nav2 y publica `cmd_vel = 0` para frenar en seco. |
| `Wait` | `duration_s` | Ninguno | Pausa asíncrona temporizada (para ceder el paso en cruces o esperas). |
| `RecoverNavigation` | `recovery_type` | Ninguno | Ejecuta comportamiento de recuperación (`"SPIN"`, `"BACKUP"`, `"CLEAR_COSTMAP"`). |
| `ScanArea` | `duration_s` | Ninguno | Inspección estática o barrido sensorial detallado en un punto clave. |
| `OperateArm` | `action_type` | Ninguno | Servicio/Topic serie con MCU para accionar el brazo (ej. soltar cable guía). |
| `SaveMap` | `map_name` | Ninguno | Llama al servicio de guardado de mapa de `slam_toolbox`. |
| `SendAlert` | `message`, `pose` | Ninguno | Publicador ROS 2 en topic de telemetría para la estación base. |
