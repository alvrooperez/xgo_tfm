# Catálogo Oficial de Misiones de la Brigada
**Proyecto:** RASTREO - Orquestador de Misiones Multi-Robot  
**Plataforma:** 3x Cuadrúpedos XGO Mini 2S (Raspberry Pi CM5)  
**Fecha:** 2026-10-21  

---

### M1: Exploración Cooperativa de Mapa Desconocido (Active SLAM)
* **Entorno:** Sin mapa previo (espacio desconocido tras catástrofe).
* **Objetivo:** Construir la rejilla de ocupación métrica coordinada entre los 3 perros mediante `slam_toolbox`.
* **Rol de la Flota:** Se reparten las fronteras del mapa (*Frontier Exploration*) para no solaparse y mapear el edificio en el menor tiempo.

---

### M2: Barrido Sistemático en Mapa Conocido (Cooperative Coverage)
* **Entorno:** Mapa previo conocido (`room_map.yaml`).
* **Objetivo:** Búsqueda exhaustiva de víctimas cubriendo el 100% del área libre sin dejar zonas ciegas.
* **Rol de la Flota:** Particionado espacial (Voronoi/Sectores). Cada perro recibe una ruta de cobertura para inspeccionar su zona asignada.

---

### M3: Despacho Multi-Punto Óptimo (Point-to-Point MRTA)
* **Entorno:** Mapa conocido.
* **Objetivo:** Enviar a los 3 perros a 3 puntos críticos de interés (ej. salidas de emergencia, grietas estructurales, salas clave).
* **Rol de la Flota:** Asignación 1-a-1 de coste mínimo (algoritmo de asignación que evita cruces y bloqueos mutuos en pasillos).

---

### M4: Marcaje y Balizamiento de Víctima (Target Stalking & Beaconing)
* **Entorno:** Dinámico (al detectar a la persona).
* **Objetivo:** Confirmar la detección desde varios ángulos, eliminar sombras y transmitir la posición exacta al puesto de mando.
* **Rol de la Flota:** El perro detector se frena y marca a distancia segura; los otros perros acuden a posiciones de apoyo para triangular.

---

### M5: Despliegue de Cable Guía Físico (Tether Deployment)
* **Entorno:** Conocido / Explorado.
* **Objetivo:** Tender físicamente una línea de vida desde la víctima hasta la salida segura para guiar a los rescatistas humanos.
* **Rol de la Flota:** El perro equipado con el carrete recorre la trayectoria segura libre de obstáculos/columnas (*snag-free retracing*) desenrollando el cable.

---

### M6: Relevo de Batería y Retorno Seguro a Base (Dynamic Handover & RTB)
* **Entorno:** Cualquier fase de la misión.
* **Objetivo:** Garantizar que ningún robot se quede sin batería en zona peligrosa y evacuar ordenadamente.
* **Rol de la Flota:** Si la batería de un perro cae del umbral crítico ($< 15\%$), un perro de reserva ocupa su posición y el perro agotado regresa a recargar.
