# Fase 4: Modelado Visual XML y Validación en Groot2
**Proyecto:** RASTREO - Orquestador de Misiones y Behavior Trees  
**Plataforma:** Flota de Cuadrúpedos XGO Mini 2S (Raspberry Pi CM5)  
**Fecha:** 2026-10-27  
**Estado:** Cerrado y Validado  
**Archivos Generados:** 
- [`docs/diseno_bt/rastreo_bt.xml`](rastreo_bt.xml)
- [`xgo_tfm/config/bt_trees/rastreo_bt.xml`](../../xgo_tfm/config/bt_trees/rastreo_bt.xml)

---

## 1. Estándar y Compatibilidad: BehaviorTree.CPP v4

El archivo XML se ha generado bajo la especificación formal **BTCPP v4** (`<root BTCPP_format="4" main_tree_to_execute="MainTree">`), garantizando:
1. **Compatibilidad nativa con Groot2:** Reconocimiento de la paleta completa de nodos, puertos y jerarquías sin advertencias ni tipos desconocidos.
2. **Encapsulamiento de Subárboles (`<BehaviorTree ID="...">`):** El árbol principal (`MainTree`) y las 6 misiones (`SubTree_M1_ActiveSLAM` a `SubTree_M6_RTB`) están completamente independizados.
3. **Mapeo explícito de Blackboard:** Todas las entradas y salidas se enlazan mediante la sintaxis canónica `{variable}`.

---

## 2. Paleta de Nodos Personalizados (`TreeNodesModel`)

Para que Groot2 cargue automáticamente los nodos sin necesidad de conectarse a un proceso ROS 2 en vivo, el XML incluye la sección `<TreeNodesModel>` con la declaración de tipos de puertos:

### Condiciones
* `<Condition ID="IsEmergencyStopActive" />`
* `<Condition ID="HasFallen"><input_port name="tilt_threshold_deg" default="60.0"/></Condition>`
* `<Condition ID="IsBatteryLow"><input_port name="threshold" default="15.0"/></Condition>`
* `<Condition ID="HasWifiConnection"><input_port name="timeout_s" default="5.0"/></Condition>`
* `<Condition ID="HasTaskAssigned" />`
* `<Condition ID="IsTaskType"><input_port name="expected_type" /></Condition>`
* `<Condition ID="IsTargetDetected"><output_port name="output_pose" /></Condition>`
* `<Condition ID="IsTargetLost"><input_port name="timeout_s" default="5.0"/></Condition>`
* `<Condition ID="IsPathBlocked" />`
* `<Condition ID="IsLocated"><input_port name="cov_threshold" default="0.5"/></Condition>`

### Acciones
* `<Action ID="NavigateToPose"><input_port name="goal" /></Action>`
* `<Action ID="NavigateThroughPoses"><input_port name="waypoints" /></Action>`
* `<Action ID="StopRobot" />`
* `<Action ID="Wait"><input_port name="duration_s" default="1.0"/></Action>`
* `<Action ID="RecoverNavigation"><input_port name="recovery_type" /></Action>`
* `<Action ID="ScanArea"><input_port name="duration_s" default="3.0"/></Action>`
* `<Action ID="OperateArm"><input_port name="action_type" /></Action>`
* `<Action ID="SaveMap"><input_port name="map_name" /></Action>`
* `<Action ID="SendAlert"><input_port name="message" /><input_port name="pose" /></Action>`

---

## 3. Estructura de Subárboles Implementados en el XML

| ID del Árbol | Tipo | Puertos de Interfaz | Función Principal |
|:---|:---|:---|:---|
| **`MainTree`** | Raíz | Blackboard Global | Árbol principal con selector reactivo en 5 ramas. |
| **`SubTree_M1_ActiveSLAM`** | Misión 1 | `frontier_pose`, `map_name` | Navegación reactiva hacia fronteras con estabilización de grafo SLAM. |
| **`SubTree_M2_SweepArea`** | Misión 2 | `waypoints` | Recorrido de tramos de barrido con recuperación local de atascos. |
| **`SubTree_M3_GoToPoint`** | Misión 3 | `goal` | Navegación punto a punto con pipeline `CLEAR_COSTMAP` + `BACKUP`. |
| **`SubTree_M4_BeaconTarget`**| Misión 4 | `target_pose` | Aproximación, confirmación sensorial 3s y emisión continua de baliza. |
| **`SubTree_M5_DeployCable`** | Misión 5 | `waypoints` | Agarre con servobrazo (`GRIP`), tendido de línea física y suelta (`RELEASE`). |
| **`SubTree_M6_RTB`** | Misión 6 | `dock_pose` | Navegación a base de carga, corte de motores y confirmación de atraque. |

---

## 4. Cómo Visualizar y Validar en Groot2

1. Iniciar la aplicación **Groot2** en tu entorno.
2. Seleccionar la opción **"Editor Mode"** o **"Open File"**.
3. Abrir cualquiera de las dos rutas:
   - `docs/diseno_bt/rastreo_bt.xml`
   - `xgo_tfm/config/bt_trees/rastreo_bt.xml`
4. Comprobar que en el selector superior de árboles puedes conmutar entre `MainTree` y cualquiera de los subárboles `SubTree_M1` a `SubTree_M6`.
