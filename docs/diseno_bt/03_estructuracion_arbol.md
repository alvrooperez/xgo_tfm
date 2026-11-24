# Fase 3: Estructuración Top-Down y Priorización del Árbol de Comportamiento
**Proyecto:** RASTREO - Orquestador de Misiones y Behavior Trees  
**Plataforma:** Flota de Cuadrúpedos XGO Mini 2S (Raspberry Pi CM5)  
**Fecha:** 2026-10-27  
**Estado:** Cerrado y Validado  

---

## 1. Filosofía Arquitectónica: Orquestador Central vs. Behavior Tree Local

El sistema se estructura en dos capas claramente desacopladas:

1. **Orquestador Central (Puesto de Mando):**
   - Mantiene la visión global de la brigada (los 3 perros XGO).
   - Calcula particiones espaciales, coberturas dinámicas y asignaciones óptimas (MRTA).
   - Inyecta y actualiza misiones en caliente a lo largo del tiempo mediante interfaces de Acción de ROS 2 (`ExecuteTask.action`).
2. **Behavior Tree Local (A bordo de cada XGO):**
   - Actúa como el ejecutor táctico reactivo (músculo y reflejos del robot).
   - Ejecuta de forma segura los tramos de misión asignados por el orquestador.
   - Responde inmediatamente a contingencias locales de salud física, energía y conectividad.
   - Emite telemetría continua y eventos de percepción (ej. detección de víctimas) al orquestador.

---

## 2. Árbol Principal (`MainTree`): Jerarquía en 5 Ramas Priorizadas

El nodo raíz es un `ReactiveFallback` que evalúa continuamente (a 10–20 Hz) cinco ramas de izquierda a derecha. Si una rama prioritaria a la izquierda se activa, cancela inmediatamente (`onHalted`) la ejecución de cualquier rama situada a su derecha.

```
                     [ Selector Raíz: ReactiveFallback ]
       ______________________________|_____________________________
      |              |               |               |             |
  [ Rama 1 ]     [ Rama 2 ]      [ Rama 3 ]      [ Rama 4 ]    [ Rama 5 ]
  SEGURIDAD       BATERÍA          WI-FI          MISIONES       REPOSO
 (E-STOP /       (Batería         (Pérdida        (Selector     (Espera /
   Caída)          baja)          de señal)       M1 a M6)       Standby)
```

### Tabla de Prioridades del Árbol Raíz

| Prioridad | Rama | Disparador (Condición) | Comportamiento Reactivo Asociado |
|:---:|:---|:---|:---|
| **P1 (Máx)** | **Seguridad Crítica** | `IsEmergencyStopActive` o `HasFallen` | `StopRobot` (corte `cmd_vel` y freno de motores) + `SendAlert("CRITICAL_SAFETY")`. |
| **P2** | **Contingencia Batería** | `IsBatteryLow` ($< 15\%$) | Aborta misión en curso, `SendAlert("LOW_BATTERY")` y deriva a `SubTree_M6_RTB`. |
| **P3** | **Contingencia Enlace** | Pérdida de `HasWifiConnection` ($> 5$\,s) | `Wait(5s)` de gracia local; si persiste, `NavigateToPose` de repliegue al último punto con cobertura. |
| **P4** | **Despachador de Tareas** | `HasTaskAssigned == true` | Evalúa `task_type` y conmuta al subárbol de misión asignado por el orquestador. |
| **P5** | **Reposo / Disponibilidad** | Por defecto (sin tarea activa) | `StopRobot` + `Wait(1s)` en bucle a la espera de órdenes del puesto de mando. |

---

## 3. Desglose Estructural de las Ramas de Contingencia

### 3.1. Rama 1: Seguridad Crítica (E-STOP y Caída)
```text
[Fallback: SafetyCheck]
 ├── [Sequence: Handle_EStop]
 │    ├── <Condition: IsEmergencyStopActive />
 │    ├── <Action: StopRobot />
 │    └── <Action: SendAlert message="E-STOP_ACTIVE" />
 └── [Sequence: Handle_Fallen]
      ├── <Condition: HasFallen tilt_threshold_deg="60" />
      ├── <Action: StopRobot />
      └── <Action: SendAlert message="ROBOT_FALLEN" />
```

### 3.2. Rama 2: Batería Crítica
```text
[Sequence: Handle_LowBattery]
 ├── <Condition: IsBatteryLow threshold="{battery_threshold}" />
 ├── <Action: SendAlert message="LOW_BATTERY_ABORT" />
 └── <SubTree ID="SubTree_M6_RTB" dock_pose="{dock_pose}" />
```

### 3.3. Rama 3: Pérdida de Conectividad Wi-Fi
```text
[Sequence: Handle_WifiLoss]
 ├── <Inverter>
 │    └── <Condition: HasWifiConnection timeout_s="5.0" />
 ├── <Action: Wait duration_s="5.0" />
 └── <Action: NavigateToPose goal="{last_known_wifi_pose}" />
```

---

## 4. Despachador de Misiones (`TaskDispatcher`)

```text
[Sequence: TaskDispatcher]
 ├── <Condition: HasTaskAssigned />
 └── [Fallback: MissionSelector]
      ├── [Sequence: Select_M1]
      │    ├── <Condition: IsTaskType expected_type="MAP_UNKNOWN" />
      │    └── <SubTree ID="SubTree_M1_ActiveSLAM" frontier_pose="{goal_pose}" map_name="{map_name}" />
      ├── [Sequence: Select_M2]
      │    ├── <Condition: IsTaskType expected_type="SWEEP_AREA" />
      │    └── <SubTree ID="SubTree_M2_SweepArea" waypoints="{waypoints_list}" />
      ├── [Sequence: Select_M3]
      │    ├── <Condition: IsTaskType expected_type="GOTO_POINT" />
      │    └── <SubTree ID="SubTree_M3_GoToPoint" goal="{goal_pose}" />
      ├── [Sequence: Select_M4]
      │    ├── <Condition: IsTaskType expected_type="TARGET_BEACON" />
      │    └── <SubTree ID="SubTree_M4_BeaconTarget" target_pose="{target_pose}" />
      ├── [Sequence: Select_M5]
      │    ├── <Condition: IsTaskType expected_type="DEPLOY_CABLE" />
      │    └── <SubTree ID="SubTree_M5_DeployCable" waypoints="{waypoints_list}" />
      └── [Sequence: Select_M6]
           ├── <Condition: IsTaskType expected_type="RTB" />
           └── <SubTree ID="SubTree_M6_RTB" dock_pose="{goal_pose}" />
```

---

## 5. Catálogo de Subárboles de Misión (M1 a M6)

### SubTree M1: Active SLAM Continuo (`SubTree_M1_ActiveSLAM`)
Exploración dinámica guiada por fronteras con `slam_toolbox` en segundo plano:
```text
[Fallback: M1_FrontierNavigation]
 ├── <Action: NavigateToPose goal="{frontier_pose}" />
 └── [Sequence: SLAM_Recovery]
      ├── <Action: ScanArea duration_s="4.0" />
      └── <Action: NavigateToPose goal="{frontier_pose}" />
```
*(El guardado de mapa `SaveMap` se dispara de forma desacoplada al completar el área o periódicamente).*

### SubTree M2: Barrido Sistemático por Tramos (`SubTree_M2_SweepArea`)
Recorrido de lotes de waypoints asignados dinámicamente por el orquestador:
```text
[Fallback: M2_ChunkTraversal]
 ├── <Action: NavigateThroughPoses waypoints="{waypoints}" />
 └── [Sequence: RecoveryPipeline]
      ├── <Action: RecoverNavigation recovery_type="CLEAR_COSTMAP" />
      ├── <Action: RecoverNavigation recovery_type="BACKUP" />
      └── <Action: NavigateThroughPoses waypoints="{waypoints}" />
```
*(Si durante el avance `IsTargetDetected` detecta víctima, emite `SendAlert("VICTIM_FOUND", target_pose)` para que el orquestador gestione el balizamiento).*

### SubTree M3: Despacho a un Punto con Recuperación (`SubTree_M3_GoToPoint`)
```text
[Fallback: M3_NavWithRecovery]
 ├── <Action: NavigateToPose goal="{goal}" />
 └── [Sequence: RecoveryPipeline]
      ├── <Action: RecoverNavigation recovery_type="CLEAR_COSTMAP" />
      ├── <Action: RecoverNavigation recovery_type="BACKUP" />
      └── <Action: NavigateToPose goal="{goal}" />
```

### SubTree M4: Balizamiento de Víctima (`SubTree_M4_BeaconTarget`)
Aproximación segura, confirmación sensorial y modo baliza:
```text
[Sequence: M4_BeaconSequence]
 ├── <Action: NavigateToPose goal="{target_pose}" />
 ├── <Action: ScanArea duration_s="3.0" />
 └── <Action: SendAlert message="BEACONING_CONFIRMED" pose="{target_pose}" />
```

### SubTree M5: Despliegue de Cable Guía Físico (`SubTree_M5_DeployCable`)
Coordinación de manipulación y locomoción sobre la trayectoria segura:
```text
[Sequence: M5_DeployTether]
 ├── <Action: OperateArm action_type="GRIP_CABLE" />
 ├── <Action: NavigateThroughPoses waypoints="{waypoints}" />
 ├── <Action: OperateArm action_type="RELEASE_CABLE" />
 └── <Action: SendAlert message="CABLE_DEPLOYED_SUCCESSFULLY" />
```

### SubTree M6: Retorno a Base y Relevo (`SubTree_M6_RTB`)
Reutilizado tanto para relevos ordenados como para contingencia de batería baja (Rama 2):
```text
[Sequence: M6_ReturnToBase]
 ├── <Action: SendAlert message="START_RTB" />
 ├── <Action: NavigateToPose goal="{dock_pose}" />
 ├── <Action: StopRobot />
 └── <Action: SendAlert message="DOCKED_SAFELY" />
```
