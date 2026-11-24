#!/usr/bin/env python3
"""
Despachador de Misiones para la Brigada Multi-Robot RASTREO
Permite enviar misiones autónomas M1-M6 a robot1, robot2 y robot3.
"""

import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class MissionDispatcher(Node):
    def __init__(self):
        super().__init__("rastreo_mission_dispatcher")
        
        # Publicadores para cada robot
        self.cmd_pubs = {
            "robot1": self.create_publisher(String, "/robot1/task_command", 10),
            "robot2": self.create_publisher(String, "/robot2/task_command", 10),
            "robot3": self.create_publisher(String, "/robot3/task_command", 10),
        }
        self.goal_pubs = {
            "robot1": self.create_publisher(PoseStamped, "/robot1/goal_pose", 10),
            "robot2": self.create_publisher(PoseStamped, "/robot2/goal_pose", 10),
            "robot3": self.create_publisher(PoseStamped, "/robot3/goal_pose", 10),
        }
        self.get_logger().info("Despachador de misiones RASTREO inicializado.")

    def wait_for_subscribers(self, timeout_sec=5.0):
        """Espera a que DDS descubra a los 3 suscriptores de los robots."""
        start = time.time()
        self.get_logger().info("Esperando descubrimiento DDS de los 3 Behavior Trees...")
        while time.time() - start < timeout_sec:
            counts = [pub.get_subscription_count() for pub in self.cmd_pubs.values()]
            if all(c > 0 for c in counts):
                self.get_logger().info(f"¡Todos los robots conectados! Subcriptores: {counts}")
                return True
            time.sleep(0.1)
        counts = [pub.get_subscription_count() for pub in self.cmd_pubs.values()]
        self.get_logger().warn(f"Descubrimiento parcial: {counts}")
        return False

    def send_command(self, robot: str, cmd: str):
        if robot not in self.cmd_pubs:
            self.get_logger().error(f"Robot desconocido: {robot}")
            return
        msg = String()
        msg.data = cmd
        self.cmd_pubs[robot].publish(msg)
        self.get_logger().info(f"[{robot.upper()}] <-- Publicado comando: '{cmd}'")

    def send_fleet_demo(self):
        """Despacha misiones coordinadas concurrentes a los 3 robots."""
        print("\n" + "="*65)
        print("   DESPACHANDO MISIONES COORDINADAS A LA BRIGADA RASTREO")
        print("="*65)
        print(" [R1 / Naranja] Misión M1/M3: Exploración y avance hacia (1.20, 0.80)")
        print(" [R2 / Azul]    Misión M2: Barrido sistemático (SWEEP_AREA)")
        print(" [R3 / Verde]   Misión M3: Despacho táctico hacia (-0.80, -0.60)")
        print("="*65 + "\n")

        # R1: GOTO_POINT a (1.2, 0.8)
        self.send_command("robot1", "GOTO_POINT 1.2 0.8")
        time.sleep(0.3)

        # R2: SWEEP_AREA con 4 waypoints cuadrantes
        self.send_command("robot2", "SWEEP_AREA 0.8 0.0 0.8 0.8 0.0 0.8 0.0 0.0")
        time.sleep(0.3)

        # R3: GOTO_POINT a (-0.8, -0.6)
        self.send_command("robot3", "GOTO_POINT -0.8 -0.6")
        time.sleep(0.3)

        print("\n>>> ¡Todas las misiones han sido transmitidas a los Behavior Trees!\n")

    def cancel_all(self):
        print("\n[CANCELACIÓN GENERAL] Enviando STOP a toda la flota...")
        for r in ["robot1", "robot2", "robot3"]:
            self.send_command(r, "CANCEL")
        print(">>> Flota en reposo (IDLE).\n")

def main():
    parser = argparse.ArgumentParser(description="Despachador de Misiones RASTREO")
    parser.add_argument("--fleet", action="store_true", help="Enviar misiones coordinadas a los 3 robots")
    parser.add_argument("--cancel", action="store_true", help="Cancelar misiones y poner la flota en reposo")
    parser.add_argument("--robot", choices=["robot1", "robot2", "robot3"], help="Robot objetivo")
    parser.add_argument("--mission", choices=["GOTO_POINT", "SWEEP_AREA", "MAP_UNKNOWN", "RTB", "CANCEL"], help="Tipo de misión")
    parser.add_argument("--args", nargs="*", default=[], help="Parámetros (coordenadas x y ...)")

    args = parser.parse_args()

    rclpy.init()
    dispatcher = MissionDispatcher()

    # Esperar a que DDS descubra a los 3 robots
    dispatcher.wait_for_subscribers(timeout_sec=4.0)

    if args.fleet:
        dispatcher.send_fleet_demo()
    elif args.cancel:
        dispatcher.cancel_all()
    elif args.robot and args.mission:
        full_cmd = args.mission + (" " + " ".join(args.args) if args.args else "")
        dispatcher.send_command(args.robot, full_cmd)
    else:
        # Por defecto si no se pasan argumentos, enviar demo de flota
        dispatcher.send_fleet_demo()

    # Dar margen para enviar el paquete antes de salir
    time.sleep(0.5)
    dispatcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
