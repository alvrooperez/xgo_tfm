# Fase 1: Análisis de Escenarios y Modos de Fallo
**Proyecto:** RASTREO - Orquestador de Misiones y Behavior Trees  
**Plataforma:** Flota de Cuadrúpedos XGO Mini 2S (Raspberry Pi CM5)  
**Fecha:** 2026-10-21  
**Estado:** Cerrado y Validado  

---

## 1. Escenario Nominal (Happy Path)
El flujo ideal de operación cuando ningún subsistema falla:

1. **Recepción de Misión:** El robot recibe una secuencia de waypoints o sector asignado desde el orquestador global.
2. **Navegación:** El robot sigue la trayectoria calculada por Nav2 de forma continua y fluida.
3. **Ejecución de Acciones:**
   - *Durante el recorrido:* Escaneo sensorial continuo (LiDAR/cámara en búsqueda de víctimas/indicios).
   - *En los waypoints:* Ejecución de acciones puntuales (inspección 360°, comprobación de estabilidad, despliegue de cable guía).
4. **Detección de Objetivo (Núcleo RASTREO):** Identificación positiva de la víctima/objetivo.
5. **Retorno de Información:** Transmisión continua de telemetría (pose, batería, estado) y confirmación de éxito de tarea a la estación base.

---

## 2. Escenarios de Contingencia y Modos de Fallo (Unhappy Paths)

Clasificación exhaustiva de fallos y comportamiento esperado de contingencia:

| ID | Categoría | Evento / Modo de Fallo | Causa en el Mundo Real | Respuesta Requerida del Behavior Tree |
|:---|:---|:---|:---|:---|
| **U1** | **Navegación** | Robot queda parado durante la trayectoria | Costmap bloqueado, escombros no superables, fallo del planificador local. | Disparar comportamientos de recuperación (*Spin/Back-up/Clear costmap*). Si persiste, abortar y pedir re-planificación. |
| **U2** | **Energía** | Batería baja del umbral crítico ($< X\%$) | Consumo por caminata prolongada o resistencia del terreno. | Cancelar la acción en curso de inmediato (`onHalted`), cambiar de rol y navegar a base de carga. |
| **U3** | **Multi-Robot** | Trayectorias de los robots se cruzan | Dos perros intentan cruzar la misma puerta o pasillo estrecho al mismo tiempo. | Prioridad por rol/batería: uno cede el paso (*Wait/Yield*) mientras el otro cruza. |
| **U4** | **Dinámico** | Encontrado obstáculo dinámico | Persona en movimiento, escombro que cae o animal. | Frenado de emergencia local, re-cálculo de trayectoria local en Nav2 MPPI sin abortar la misión global. |
| **U5** | **Comunicaciones**| Pérdida de cobertura Wi-Fi | Robot entra en sótano o detrás de muros densos (heartbeat timeout). | Mantener autonomía local. Si la tarea expira sin reconexión, ejecutar retrazado seguro al último punto con señal. |
| **U6** | **Localización** | Pérdida de localización / Salto de mapa | El filtro AMCL diverge por simetría de pasillos o patinaje de patas. | Detener avance, ejecutar giro lento de re-localización sensorial. Si no converge, declarar fallo de misión. |
| **U7** | **Hardware / Caída** | Vuelco o caída del robot | Pérdida de equilibrio en desniveles o tropiezo con obstáculo (inclinación IMU $> 60^\circ$). | Parar servomotores al instante para evitar daños mecánicos y publicar alerta crítica de asistencia. |
| **U8** | **Seguridad** | Parada de emergencia del operador (E-STOP) | Orden remota del puesto de mando por riesgo inminente. | Congelar todo movimiento inmediatamente en el ciclo actual del tick (`onHalted`) y esperar rearme manual. |
| **U9** | **Percepción** | Pérdida de objetivo tras detección inicial | La víctima se mueve, oclusiones por polvo o falso positivo momentáneo. | Activar búsqueda local (giro $360^\circ$ o patrón espiral). Si no se re-adquiere en $T$ s, reportar y reanudar patrulla. |
