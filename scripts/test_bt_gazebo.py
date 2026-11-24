#!/usr/bin/env python3
"""
Suite de pruebas automatizadas de los Behavior Trees sobre Gazebo Harmonic (3 robots).
Cubre todas las ramas prioritarias de rastreo_bt.xml:
  1. Conectividad ROS 2 <-> Gazebo <-> 3 Behavior Trees
  2. Rama 4 (M3 GOTO_POINT) concurrente en los 3 robots
  3. Rama 1 Seguridad Crítica: Preempción por E-STOP durante movimiento
  4. Rama 1 Seguridad Crítica: Preempción por vuelco IMU (HasFallen > 60 deg)
  5. Rama 2 Contingencia Energética: Batería baja (<15%) -> Emergency RTB automático
  6. Rama 3 Contingencia Wi-Fi: Pérdida de /heartbeat -> Retrace a last_known_wifi_pose
  7. Rama 4 (M2 SWEEP_AREA) multi-waypoint secuencial en robot3
  8. Rama 4 (M6 RTB) explícito en robot1 + comprobación de ficheros .btlog en /tmp/xgo_bt_logs
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Header, String


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str


class BTGazeboTestSuite(Node):
    def __init__(self):
        super().__init__("bt_gazebo_test_suite")
        self.robots = ["robot1", "robot2", "robot3"]
        self.hb_enabled: Set[str] = set(self.robots)

        self.cmd_pubs = {
            r: self.create_publisher(String, f"/{r}/task_command", 10)
            for r in self.robots
        }
        self.estop_pubs = {
            r: self.create_publisher(Bool, f"/{r}/emergency_stop", 10)
            for r in self.robots
        }
        self.battery_pubs = {
            r: self.create_publisher(BatteryState, f"/{r}/battery_state", 10)
            for r in self.robots
        }
        self.imu_pubs = {
            r: self.create_publisher(Imu, f"/{r}/imu/data", 10)
            for r in self.robots
        }
        self.hb_pubs = {
            r: self.create_publisher(Header, f"/{r}/heartbeat", 10)
            for r in self.robots
        }

        self.odom: Dict[str, Optional[Tuple[float, float]]] = {r: None for r in self.robots}
        self.trajectories: Dict[str, List[Tuple[float, float]]] = {r: [] for r in self.robots}
        self.events: List[Tuple[str, float, float, str]] = []
        self.last_cmd_vel: Dict[str, Optional[Tuple[float, float]]] = {r: None for r in self.robots}
        self.alerts: List[str] = []

        for r in self.robots:
            self.create_subscription(
                Odometry, f"/{r}/odom", lambda msg, rob=r: self._odom_cb(rob, msg), 10
            )
            self.create_subscription(
                Twist, f"/{r}/cmd_vel", lambda msg, rob=r: self._cmd_vel_cb(rob, msg), 10
            )

        self.create_subscription(String, "/fleet/alerts", self._alert_cb, 10)
        self.hb_timer = self.create_timer(0.5, self._publish_heartbeats)

    def _publish_heartbeats(self):
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        for r in list(self.hb_enabled):
            self.hb_pubs[r].publish(hdr)

    def _odom_cb(self, robot: str, msg: Odometry):
        p = msg.pose.pose.position
        self.odom[robot] = (p.x, p.y)
        traj = self.trajectories[robot]
        if not traj or math.hypot(p.x - traj[-1][0], p.y - traj[-1][1]) > 0.015:
            traj.append((p.x, p.y))

    def _cmd_vel_cb(self, robot: str, msg: Twist):
        self.last_cmd_vel[robot] = (msg.linear.x, msg.angular.z)

    def _alert_cb(self, msg: String):
        self.alerts.append(msg.data)

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout_s: float) -> bool:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def send_task(self, robot: str, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pubs[robot].publish(msg)

    def set_estop(self, robot: str, active: bool):
        msg = Bool()
        msg.data = active
        self.estop_pubs[robot].publish(msg)

    def set_battery(self, robot: str, pct_0_to_1: float):
        msg = BatteryState()
        msg.percentage = float(pct_0_to_1)
        msg.present = True
        self.battery_pubs[robot].publish(msg)

    def publish_rolled_imu(self, robot: str, roll_deg: float):
        rad = math.radians(roll_deg)
        msg = Imu()
        msg.orientation.x = math.sin(rad / 2.0)
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = math.cos(rad / 2.0)
        self.imu_pubs[robot].publish(msg)

    def dist_to(self, robot: str, tx: float, ty: float) -> float:
        pos = self.odom[robot]
        if pos is None:
            return float("inf")
        return math.hypot(tx - pos[0], ty - pos[1])

    def run_all(self) -> List[TestResult]:
        results: List[TestResult] = []

        # ------------------------------------------------------------------
        # TEST 1: Conectividad Gazebo + 3 BT
        # ------------------------------------------------------------------
        print("\n[TEST 1/8] Verificando conectividad Gazebo <-> 3 Behavior Trees...")
        ok_subs = self.wait_until(
            lambda: all(p.get_subscription_count() > 0 for p in self.cmd_pubs.values())
            and all(self.odom[r] is not None for r in self.robots),
            timeout_s=15.0,
        )
        if ok_subs:
            # Publicar heartbeats iniciales y dar 1s para asegurar enlace Wi-Fi sano antes de Test 2
            self._publish_heartbeats()
            self.spin_for(1.0)
            pos_str = ", ".join(
                f"{r}=({self.odom[r][0]:.2f},{self.odom[r][1]:.2f})" for r in self.robots
            )
            results.append(TestResult("1. Conectividad Gazebo + 3 BTs", True, f"Odom activa: {pos_str}"))
        else:
            subs = {r: self.cmd_pubs[r].get_subscription_count() for r in self.robots}
            results.append(
                TestResult("1. Conectividad Gazebo + 3 BTs", False, f"Timeout. subs={subs}, odom={self.odom}")
            )
            return results

        # ------------------------------------------------------------------
        # TEST 2: M3 (GOTO_POINT) concurrente en los 3 robots
        # ------------------------------------------------------------------
        print("[TEST 2/8] M3 GOTO_POINT concurrente en los 3 robots...")
        targets = {}
        offsets = {"robot1": (0.6, 0.5), "robot2": (0.0, 0.8), "robot3": (-0.6, 0.5)}
        for r in self.robots:
            x0, y0 = self.odom[r]
            tx, ty = x0 + offsets[r][0], y0 + offsets[r][1]
            targets[r] = (tx, ty)
            self.send_task(r, f"GOTO_POINT {tx:.2f} {ty:.2f}")

        t0 = time.time()
        ok_goto = self.wait_until(
            lambda: all(self.dist_to(r, *targets[r]) < 0.30 for r in self.robots),
            timeout_s=25.0,
        )
        dt = time.time() - t0
        dists = ", ".join(f"{r}:{self.dist_to(r, *targets[r]):.2f}m" for r in self.robots)
        results.append(
            TestResult(
                "2. M3 GOTO_POINT concurrente (3 robots)",
                ok_goto,
                f"t={dt:.1f}s | distancias finales: {dists}",
            )
        )

        # ------------------------------------------------------------------
        # TEST 3: Rama 1 Seguridad Crítica — Preempción por E-STOP
        # ------------------------------------------------------------------
        print("[TEST 3/8] Rama 1 Seguridad Crítica (preempción E-STOP en robot1)...")
        self.alerts.clear()
        x1, y1 = self.odom["robot1"]
        self.send_task("robot1", f"GOTO_POINT {x1 + 1.5:.2f} {y1:.2f}")
        moving = self.wait_until(
            lambda: self.last_cmd_vel["robot1"] is not None
            and (abs(self.last_cmd_vel["robot1"][0]) > 0.02 or abs(self.last_cmd_vel["robot1"][1]) > 0.05),
            timeout_s=5.0,
        )
        self.set_estop("robot1", True)
        if self.odom["robot1"]:
            self.events.append(("E-STOP (R1)", *self.odom["robot1"], "#d95f02"))
        estop_ok = self.wait_until(
            lambda: any("E-STOP" in a for a in self.alerts)
            and self.last_cmd_vel["robot1"] is not None
            and abs(self.last_cmd_vel["robot1"][0]) < 1e-3
            and abs(self.last_cmd_vel["robot1"][1]) < 1e-3,
            timeout_s=5.0,
        )
        self.send_task("robot1", "CANCEL")
        self.set_estop("robot1", False)
        self.spin_for(0.5)
        results.append(
            TestResult(
                "3. Rama 1 E-STOP preempción",
                moving and estop_ok,
                f"en_movimiento={moving}, alerta_y_parada={estop_ok}, cmd_vel={self.last_cmd_vel['robot1']}",
            )
        )

        # ------------------------------------------------------------------
        # TEST 4: Rama 1 Seguridad Crítica — Vuelco IMU (HasFallen > 60 deg)
        # ------------------------------------------------------------------
        print("[TEST 4/8] Rama 1 Seguridad Crítica (HasFallen IMU > 60 deg en robot3)...")
        self.alerts.clear()
        x3, y3 = self.odom["robot3"]
        self.send_task("robot3", f"GOTO_POINT {x3 - 1.2:.2f} {y3:.2f}")
        moving3 = self.wait_until(
            lambda: self.last_cmd_vel["robot3"] is not None
            and (abs(self.last_cmd_vel["robot3"][0]) > 0.02 or abs(self.last_cmd_vel["robot3"][1]) > 0.05),
            timeout_s=5.0,
        )
        # Publicar varias veces la IMU volcada (75°) para sobrescribir la IMU nominal de Gazebo
        if self.odom["robot3"]:
            self.events.append(("HasFallen (R3)", *self.odom["robot3"], "#1b9e77"))
        fall_ok = False
        end_t = time.time() + 4.0
        while time.time() < end_t:
            self.publish_rolled_imu("robot3", 75.0)
            rclpy.spin_once(self, timeout_sec=0.03)
            if any("Rollover/Fallen" in a for a in self.alerts):
                fall_ok = True
                break
        self.send_task("robot3", "CANCEL")
        self.publish_rolled_imu("robot3", 0.0)
        self.spin_for(0.5)
        results.append(
            TestResult(
                "4. Rama 1 HasFallen (vuelco IMU > 60 deg)",
                moving3 and fall_ok,
                f"en_movimiento={moving3}, alerta_vuelco={fall_ok}, cmd_vel={self.last_cmd_vel['robot3']}",
            )
        )

        # ------------------------------------------------------------------
        # TEST 5: Rama 2 Contingencia Energética — Batería baja -> Emergency RTB
        # ------------------------------------------------------------------
        print("[TEST 5/8] Rama 2 Batería baja (<15%) -> Emergency RTB en robot2...")
        self.alerts.clear()
        dist_before = self.dist_to("robot2", 0.0, 0.0)
        self.set_battery("robot2", 0.10)
        alert_low = self.wait_until(
            lambda: any("BATTERY_CRITICAL" in a for a in self.alerts),
            timeout_s=5.0,
        )
        reached_dock = self.wait_until(
            lambda: self.dist_to("robot2", 0.0, 0.0) < 0.30,
            timeout_s=20.0,
        )
        docked_alert = self.wait_until(
            lambda: any("DOCKED_SAFELY" in a for a in self.alerts),
            timeout_s=3.0,
        )
        dist_after = self.dist_to("robot2", 0.0, 0.0)
        self.set_battery("robot2", 1.0)
        self.spin_for(0.5)
        results.append(
            TestResult(
                "5. Rama 2 Batería baja -> Emergency RTB",
                alert_low and reached_dock and docked_alert,
                f"dist_dock: {dist_before:.2f}m -> {dist_after:.2f}m | alerta_bat={alert_low}, docked={docked_alert}",
            )
        )

        # ------------------------------------------------------------------
        # TEST 6: Rama 3 Contingencia Wi-Fi — Retrace a last_known_wifi_pose
        # ------------------------------------------------------------------
        print("[TEST 6/8] Rama 3 Contingencia Wi-Fi (pérdida de heartbeat en robot2)...")
        # Guardamos la posición con Wi-Fi sano (miga de pan) y refrescamos el heartbeat justo antes de cortar
        self._publish_heartbeats()
        self.spin_for(0.5)
        self._publish_heartbeats()
        self.spin_for(0.2)
        breadcrumb = self.odom["robot2"]
        self.events.append(("Wi-Fi Breadcrumb (R2)", *breadcrumb, "#2b6cb0"))
        # Quitamos el heartbeat a robot2 y le ordenamos avanzar hacia -Y (dirección en la que ya mira tras el RTB)
        self.hb_enabled.discard("robot2")
        self.send_task("robot2", f"GOTO_POINT {breadcrumb[0]:.2f} {breadcrumb[1] - 1.2:.2f}")
        moved_away = self.wait_until(
            lambda: self.dist_to("robot2", *breadcrumb) > 0.45,
            timeout_s=6.0,
        )
        max_dist = self.dist_to("robot2", *breadcrumb)
        # Esperamos timeout_s (5s) + Wait_WifiGracePeriod (5s) + retorno a breadcrumb
        retraced = self.wait_until(
            lambda: self.dist_to("robot2", *breadcrumb) < 0.32,
            timeout_s=25.0,
        )
        final_wifi_dist = self.dist_to("robot2", *breadcrumb)
        # Restauramos heartbeat y cancelamos tarea previa
        self.send_task("robot2", "CANCEL")
        self.hb_enabled.add("robot2")
        self.spin_for(1.0)
        results.append(
            TestResult(
                "6. Rama 3 Contingencia Wi-Fi (Retrace breadcrumb)",
                moved_away and retraced,
                f"alejado={moved_away} ({max_dist:.2f}m) -> retorno={retraced} (dist final={final_wifi_dist:.2f}m)",
            )
        )

        # ------------------------------------------------------------------
        # TEST 7: M2 (SWEEP_AREA) multi-waypoint en robot3
        # ------------------------------------------------------------------
        print("[TEST 7/8] M2 SWEEP_AREA multi-waypoint (3 waypoints) en robot3...")
        x3, y3 = self.odom["robot3"]
        wp1 = (x3 + 0.4, y3)
        wp2 = (x3 + 0.4, y3 - 0.5)
        wp3 = (x3, y3 - 0.5)
        self.send_task(
            "robot3",
            f"SWEEP_AREA {wp1[0]:.2f} {wp1[1]:.2f} {wp2[0]:.2f} {wp2[1]:.2f} {wp3[0]:.2f} {wp3[1]:.2f}",
        )
        t0 = time.time()
        hit_wp1 = self.wait_until(lambda: self.dist_to("robot3", *wp1) < 0.30, timeout_s=15.0)
        hit_wp2 = hit_wp1 and self.wait_until(lambda: self.dist_to("robot3", *wp2) < 0.30, timeout_s=15.0)
        hit_wp3 = hit_wp2 and self.wait_until(lambda: self.dist_to("robot3", *wp3) < 0.30, timeout_s=15.0)
        dt = time.time() - t0
        results.append(
            TestResult(
                "7. M2 SWEEP_AREA (3 waypoints)",
                hit_wp1 and hit_wp2 and hit_wp3,
                f"t={dt:.1f}s | wp1={hit_wp1}, wp2={hit_wp2}, wp3={hit_wp3} (dist final={self.dist_to('robot3', *wp3):.2f}m)",
            )
        )

        # ------------------------------------------------------------------
        # TEST 8: M6 (RTB) explícito en robot1 + comprobación de .btlog
        # ------------------------------------------------------------------
        print("[TEST 8/8] M6 RTB explícito en robot1 + logs Groot2 (.btlog)...")
        self.alerts.clear()
        self.send_task("robot1", "RTB 0.60 -0.20")
        ok_rtb_start = self.wait_until(lambda: any("START_RTB" in a for a in self.alerts), timeout_s=5.0)
        ok_rtb_pos = self.wait_until(lambda: self.dist_to("robot1", 0.60, -0.20) < 0.30, timeout_s=20.0)
        ok_rtb_dock = self.wait_until(lambda: any("DOCKED_SAFELY" in a for a in self.alerts), timeout_s=3.0)

        log_dir = "/tmp/xgo_bt_logs"
        btlogs_ok = all(
            os.path.isfile(os.path.join(log_dir, f"ejecucion_{r}.btlog"))
            and os.path.getsize(os.path.join(log_dir, f"ejecucion_{r}.btlog")) > 0
            for r in self.robots
        )
        results.append(
            TestResult(
                "8. M6 RTB explícito + .btlog Groot2",
                ok_rtb_start and ok_rtb_pos and ok_rtb_dock and btlogs_ok,
                f"start={ok_rtb_start}, reached={ok_rtb_pos}, docked={ok_rtb_dock}, btlogs={btlogs_ok}",
            )
        )

        return results


def save_trajectory_plot(node: BTGazeboTestSuite, output_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    fig, ax = plt.subplots(figsize=(7.2, 6.2), dpi=300)

    # Paredes de room.sdf (8m x 8m) y obstáculos reales
    ax.add_patch(Rectangle((-4.0, -4.0), 8.0, 8.0, fill=False, edgecolor="#444444", linewidth=2.0, label="Perímetro room.sdf (8×8 m)"))
    ax.add_patch(Circle((1.5, 1.5), 0.3, facecolor="#a8d5ba", edgecolor="#2d6a4f", linewidth=1.2, label="Obstáculo: pillar_1"))
    ax.add_patch(Rectangle((-1.9, 0.9), 0.8, 1.2, facecolor="#f4b6b6", edgecolor="#9b2226", linewidth=1.2, label="Obstáculo: box_obstacle_1"))

    styles = {
        "robot1": ("#e66101", "Robot 1 (Naranja): M3, E-STOP, M6 RTB"),
        "robot2": ("#1f78b4", "Robot 2 (Azul): M3, Bat. RTB, Wi-Fi Retrace"),
        "robot3": ("#2ca25f", "Robot 3 (Verde): M3, HasFallen, M2 SWEEP"),
    }

    for r, (color, label) in styles.items():
        pts = node.trajectories[r]
        if not pts:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=color, linewidth=2.0, alpha=0.9, label=label)
        ax.scatter(xs[0], ys[0], color=color, marker="o", s=55, zorder=5, edgecolor="black", linewidth=0.7)
        ax.scatter(xs[-1], ys[-1], color=color, marker="s", s=55, zorder=5, edgecolor="black", linewidth=0.7)

    # Eventos clave de contingencia
    offsets_txt = [(8, 8), (8, -14), (8, 8)]
    for idx, (lbl, ex, ey, col) in enumerate(node.events):
        ax.scatter(ex, ey, color=col, marker="*", s=140, zorder=6, edgecolor="black", linewidth=0.6)
        dx, dy = offsets_txt[idx % len(offsets_txt)]
        ax.annotate(
            lbl,
            xy=(ex, ey),
            xytext=(dx, dy),
            textcoords="offset points",
            fontsize=8.5,
            fontweight="bold",
            color="#222222",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=col, alpha=0.9),
        )

    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-1.6, 2.4)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlabel("Posición X en Gazebo (m)")
    ax.set_ylabel("Posición Y en Gazebo (m)")
    ax.set_title("Trayectorias reales SIL en Gazebo Harmonic (3 robots + 8 pruebas BT)")
    ax.legend(loc="lower right", fontsize=7.8, framealpha=0.92)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    print(f"[PLOT] Figura de trayectorias guardada en: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Suite de pruebas BT en Gazebo Harmonic")
    parser.add_argument("--plot", default="", help="Ruta PNG donde guardar el mapa de trayectorias")
    args = parser.parse_args()

    rclpy.init()
    node = BTGazeboTestSuite()
    try:
        results = node.run_all()
        if args.plot:
            save_trajectory_plot(node, args.plot)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n" + "=" * 74)
    print("   RESULTADOS DE PRUEBAS BEHAVIOR TREES EN GAZEBO HARMONIC (8/8 RAMAS)")
    print("=" * 74)
    passed = 0
    for res in results:
        tag = "PASS" if res.passed else "FAIL"
        if res.passed:
            passed += 1
        print(f" [{tag}] {res.name}")
        print(f"        -> {res.detail}")
    print("=" * 74)
    print(f" Total: {passed}/{len(results)} pruebas superadas.")
    print("=" * 74)
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
