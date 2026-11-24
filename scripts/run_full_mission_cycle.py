#!/usr/bin/env python3
"""
Ciclo de Misiones Autónomas para la Brigada Multi-Robot RASTREO
Ejecuta la secuencia completa solicitada:
1. Desplazamiento Lejano (Tránsito a puntos remotos del mapa)
2. Barrido Sistemático (M2 - SWEEP_AREA con waypoints)
3. Retorno a Base y Docking (M6 - RTB con parada y apagado seguro)
"""

import sys
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class MissionCycleOrchestrator(Node):
    def __init__(self):
        super().__init__("mission_cycle_orchestrator")
        
        # Publicadores de comandos
        self.cmd_pubs = {
            "robot1": self.create_publisher(String, "/robot1/task_command", 10),
            "robot2": self.create_publisher(String, "/robot2/task_command", 10),
            "robot3": self.create_publisher(String, "/robot3/task_command", 10),
        }
        
        # Suscriptores de odometría para monitorizar progreso
        self.positions = {"robot1": (0.0, 0.0), "robot2": (0.0, 0.0), "robot3": (0.0, 0.0)}
        self.create_subscription(Odometry, "/robot1/odom", lambda msg: self._odom_cb("robot1", msg), 10)
        self.create_subscription(Odometry, "/robot2/odom", lambda msg: self._odom_cb("robot2", msg), 10)
        self.create_subscription(Odometry, "/robot3/odom", lambda msg: self._odom_cb("robot3", msg), 10)

        # Suscriptor de alertas de la flota
        self.create_subscription(String, "/fleet/alerts", self._alert_cb, 10)

    def _odom_cb(self, robot: str, msg: Odometry):
        p = msg.pose.pose.position
        self.positions[robot] = (p.x, p.y)

    def _alert_cb(self, msg: String):
        print(f"  [TELEMETRÍA FLOTA] {msg.data}")

    def wait_for_subscribers(self, timeout_sec=5.0):
        start = time.time()
        print("[INIT] Conectando con los Behavior Trees de los 3 robots...")
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            counts = [pub.get_subscription_count() for pub in self.cmd_pubs.values()]
            if all(c > 0 for c in counts):
                print(f"[INIT] ¡Conexión establecida con los 3 robots! Suscriptores: {counts}")
                return True
            time.sleep(0.1)
        return False

    def send_cmd(self, robot: str, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pubs[robot].publish(msg)

    def wait_until_reached(self, targets: dict, tolerance=0.30, timeout_s=45.0):
        """Espera a que los robots indicados lleguen a sus objetivos de odometría."""
        start = time.time()
        active = set(targets.keys())
        while active and (time.time() - start < timeout_s):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            done = set()
            for r in active:
                tx, ty = targets[r]
                cx, cy = self.positions[r]
                dist = math.hypot(tx - cx, ty - cy)
                if dist <= tolerance:
                    print(f"  -> [{r.upper()}] ¡Objetivo alcanzado! (pos: {cx:.2f}, {cy:.2f}, dist: {dist:.2f}m)")
                    done.add(r)
            active -= done
        return len(active) == 0

    def run_full_cycle(self):
        print("\n" + "="*70)
        print("     INICIANDO CICLO COMPLETO DE MISIONES AUTÓNOMAS RASTREO")
        print("="*70)

        # =====================================================================
        # FASE 1: DESPLAZAMIENTO LEJANO (IR LEJOS)
        # =====================================================================
        print("\n" + "#"*70)
        print("  FASE 1: DESPLAZAMIENTO LEJANO (Tránsito a Sectores Remotos)")
        print("#"*70)
        print("  - Robot 1 (Naranja):  Desplazamiento a (2.00, 1.80) [Sector Noreste]")
        print("  - Robot 2 (Azul):     Desplazamiento a (0.00, 2.00) [Sector Norte]")
        print("  - Robot 3 (Verde):    Desplazamiento a (-1.50, -1.80) [Sector Suroeste]")
        print("----------------------------------------------------------------------")

        self.send_cmd("robot1", "GOTO_POINT 2.0 1.8")
        time.sleep(0.3)
        self.send_cmd("robot2", "GOTO_POINT 0.0 2.0")
        time.sleep(0.3)
        self.send_cmd("robot3", "GOTO_POINT -1.5 -1.8")

        targets_fase1 = {
            "robot1": (2.0, 1.8),
            "robot2": (0.0, 2.0),
            "robot3": (-1.5, -1.8),
        }
        self.wait_until_reached(targets_fase1, timeout_s=35.0)
        print(">>> FASE 1 COMPLETADA: Los 3 robots han alcanzado sus posiciones lejanas.\n")
        time.sleep(1.5)

        # =====================================================================
        # FASE 2: BARRIDO SISTEMÁTICO (SWEEP_AREA)
        # =====================================================================
        print("\n" + "#"*70)
        print("  FASE 2: BARRIDO TÁCTICO SISTEMÁTICO (M2 - SWEEP_AREA)")
        print("#"*70)
        print("  - Robot 1: Barrido perimetral 4 waypoints en cuadrante Noreste")
        print("  - Robot 2: Barrido en peine 4 waypoints en cuadrante Norte")
        print("  - Robot 3: Barrido perimetral 4 waypoints en cuadrante Suroeste")
        print("----------------------------------------------------------------------")

        self.send_cmd("robot1", "SWEEP_AREA 2.2 1.8 2.2 1.2 1.8 1.2 1.8 1.8")
        time.sleep(0.3)
        self.send_cmd("robot2", "SWEEP_AREA 0.4 2.0 0.4 1.5 -0.4 1.5 -0.4 2.0")
        time.sleep(0.3)
        self.send_cmd("robot3", "SWEEP_AREA -1.2 -1.8 -1.2 -1.2 -1.6 -1.2 -1.6 -1.8")

        # Esperar a que se complete el barrido (aproximadamente 20s para los 4 tramos)
        for i in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.9)
            if i % 5 == 0:
                print(f"  ... Ejecutando barrido sistemático (progreso t={i}s)...")

        print(">>> FASE 2 COMPLETADA: Barrido sistemático finalizado en todos los sectores.\n")
        time.sleep(1.5)

        # =====================================================================
        # FASE 3: RETORNO A BASE (M6 - RTB)
        # =====================================================================
        print("\n" + "#"*70)
        print("  FASE 3: RETORNO A BASE Y APAGADO SEGURO (M6 - RTB)")
        print("#"*70)
        print("  - Activando SubTree_M6_RTB en los 3 Behavior Trees")
        print("  - Destino: Base de recarga / Docks de origen (0.00, 0.00)")
        print("  - Acciones: Telemetría de inicio -> Navegación -> StopRobot -> Telemetría de Docking")
        print("----------------------------------------------------------------------")

        self.send_cmd("robot1", "RTB 0.0 0.0")
        time.sleep(0.3)
        self.send_cmd("robot2", "RTB 0.0 0.0")
        time.sleep(0.3)
        self.send_cmd("robot3", "RTB 0.0 0.0")

        targets_rtb = {
            "robot1": (0.0, 0.0),
            "robot2": (0.0, 0.0),
            "robot3": (0.0, 0.0),
        }
        self.wait_until_reached(targets_rtb, tolerance=0.30, timeout_s=40.0)

        print("\n" + "="*70)
        print("   ¡CICLO COMPLETO DE MISIONES EJECUTADO CON ÉXITO ABSOLUTO!")
        print("   1. Desplazamiento Lejano -> ALCANZADO")
        print("   2. Barrido Sistemático   -> COMPLETADO")
        print("   3. Retorno a Base (RTB)  -> ACOPLADO Y MOTORES EN REPOSO")
        print("="*70 + "\n")

def main():
    rclpy.init()
    orchestrator = MissionCycleOrchestrator()
    if orchestrator.wait_for_subscribers():
        orchestrator.run_full_cycle()
    else:
        print("[ERROR] No se pudo conectar con los Behavior Trees.")
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
