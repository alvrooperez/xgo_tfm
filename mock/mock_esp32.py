#!/usr/bin/env python3
"""
XGO Mini 2S - Gazebo-Style Virtual Hardware & Environment Simulator
Simulates:
1. Virtual ESP32 MCU over pseudo-terminal (PTY) with Luwu serial protocol.
2. 2D Environment with Room Map from Gazebo simulation (room_map.pgm).
3. 360-degree 2D LiDAR sensor raycasting against the map.
4. Animated Gazebo-style robot dog model with quadruped trot gait.
5. ROS 2 Bridge publishing /scan (LaserScan) and /goal_pose (PoseStamped).
"""

import os
import pty
import sys
import time
import math
import struct
import select
import termios
import threading
import argparse

import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Path
    import tf2_ros
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

CLIENT_PORT = "/tmp/tty_robot"
MAP_FILE_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps", "room_map.pgm")


# ==============================================================================
# 1. Environment & Map Representation (Loaded from room_map.pgm)
# ==============================================================================
class RoomMap:
    def __init__(self, pgm_path=MAP_FILE_DEFAULT, resolution=0.050, origin=(-3.953, -3.956)):
        self.resolution = resolution
        self.ox, self.oy = origin
        self.w = 158
        self.h = 158
        self.grid = np.zeros((self.h, self.w), dtype=bool)
        self.wall_runs = []  # [(x1, x2, y)]
        self._load(pgm_path)

    def _load(self, pgm_path):
        if not os.path.exists(pgm_path):
            # Fallback 8m x 8m room with boundaries and 4 central pillars
            self._create_fallback_room()
            return

        try:
            with open(pgm_path, "rb") as f:
                f.readline()  # P5
                dims = f.readline().split()
                while dims[0].startswith(b"#"):
                    dims = f.readline().split()
                self.w, self.h = int(dims[0]), int(dims[1])
                f.readline()  # Maxval
                data = f.read()

            raw_grid = np.array(list(data), dtype=np.uint8).reshape((self.h, self.w))
            # Occupied threshold: black pixels < 50
            self.grid = (raw_grid < 50)
            self._compute_wall_runs()
        except Exception as e:
            print(f"[RoomMap] Error loading {pgm_path}: {e}, using fallback.")
            self._create_fallback_room()

    def _create_fallback_room(self):
        self.w, self.h = 160, 160
        self.grid = np.zeros((self.h, self.w), dtype=bool)
        # Boundaries
        self.grid[0:2, :] = True
        self.grid[-2:, :] = True
        self.grid[:, 0:2] = True
        self.grid[:, -2:] = True
        # Four interior pillars
        self.grid[40:50, 40:50] = True
        self.grid[40:50, 110:120] = True
        self.grid[110:120, 40:50] = True
        self.grid[110:120, 110:120] = True
        self._compute_wall_runs()

    def _compute_wall_runs(self):
        self.wall_runs.clear()
        for y in range(self.h):
            in_run = False
            start_x = 0
            for x in range(self.w):
                if self.grid[y, x]:
                    if not in_run:
                        in_run = True
                        start_x = x
                else:
                    if in_run:
                        in_run = False
                        self.wall_runs.append((start_x, x - 1, y))
            if in_run:
                self.wall_runs.append((start_x, self.w - 1, y))

    def is_occupied(self, wx: float, wy: float) -> bool:
        gx = int((wx - self.ox) / self.resolution)
        gy = (self.h - 1) - int((wy - self.oy) / self.resolution)
        if 0 <= gx < self.w and 0 <= gy < self.h:
            return bool(self.grid[gy, gx])
        return True  # Outside room is occupied

    def is_occupied_footprint(self, wx: float, wy: float, radius: float = 0.17) -> bool:
        if self.is_occupied(wx, wy):
            return True
        for a in (0.0, 0.785, 1.571, 2.356, 3.142, -2.356, -1.571, -0.785):
            if self.is_occupied(wx + radius * math.cos(a), wy + radius * math.sin(a)):
                return True
        return False

    def raycast_lidar(self, rx: float, ry: float, rtheta: float, num_rays=180, max_range=8.0, step=0.04):
        """Simulates 360-degree LiDAR raycasting."""
        angles = rtheta + np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)
        ranges = np.full(num_rays, max_range, dtype=np.float32)
        hits = []

        dists = np.arange(0.08, max_range, step, dtype=np.float32)
        for i in range(num_rays):
            ca, sa = cos_a[i], sin_a[i]
            xs = rx + dists * ca
            ys = ry + dists * sa
            gxs = ((xs - self.ox) / self.resolution).astype(np.int32)
            gys = (self.h - 1 - (ys - self.oy) / self.resolution).astype(np.int32)

            valid = (gxs >= 0) & (gxs < self.w) & (gys >= 0) & (gys < self.h)
            idx = np.where(~valid)[0]
            first_invalid = idx[0] if len(idx) > 0 else len(dists)

            valid_gxs = gxs[:first_invalid]
            valid_gys = gys[:first_invalid]
            coll = self.grid[valid_gys, valid_gxs]
            hit_idx = np.where(coll)[0]
            if len(hit_idx) > 0:
                h_i = hit_idx[0]
                ranges[i] = dists[h_i]
                hits.append((float(xs[h_i]), float(ys[h_i])))
            elif first_invalid < len(dists):
                ranges[i] = dists[first_invalid]
                hits.append((float(xs[first_invalid]), float(ys[first_invalid])))

        return angles, ranges, hits


# ==============================================================================
# 2. Virtual Hardware Emulator (Luwu ESP32 Protocol)
# ==============================================================================
class XgoMockMCU:
    def __init__(self, room_map: RoomMap, port_link=CLIENT_PORT):
        self.map = room_map
        self.port_link = port_link
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)

        # Configure raw mode on master
        mode = termios.tcgetattr(self.master_fd)
        mode[3] = mode[3] & ~termios.ECHO
        termios.tcsetattr(self.master_fd, termios.TCSANOW, mode)

        # Symlink
        if os.path.islink(self.port_link) or os.path.exists(self.port_link):
            os.remove(self.port_link)
        os.symlink(self.slave_name, self.port_link)

        # Hardware State
        self.battery = 88.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # Robot Pose in Map
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_time = time.time()
        self.is_moving = False

        # Trot Gait & LiDAR
        self.gait_phase = 0.0
        self.lidar_rotation = 0.0
        self.trail = []
        self.max_trail_len = 1200

        # Autonomous Navigation Target & Nav2 Path
        self.goal_x = None
        self.goal_y = None
        self.planned_path = []  # [(x, y), ...] from Nav2 /plan
        self.auto_nav_enabled = True

        # Telemetry
        self.rx_packets = 0
        self.tx_packets = 0
        self.checksum_errors = 0
        self.running = True
        self.lock = threading.Lock()

    def calculate_checksum(self, data: bytes) -> int:
        return (255 - (sum(data) % 256)) % 256

    def u8_to_velocity(self, val: int, vmin: float, vmax: float) -> float:
        return vmin + ((vmax - vmin) / 255.0) * val

    def handle_read_request(self, reg: int, read_len: int) -> bytes:
        with self.lock:
            if reg == 0x01:  # BATTERY
                payload = bytes([int(self.battery)])
            elif reg == 0x07:  # FIRMWARE_VERSION
                payload = b"M-5.1.1\x00\x00\x00"
            elif reg == 0x62:  # ROLL
                payload = struct.pack("<f", float(self.roll))
            elif reg == 0x63:  # PITCH
                payload = struct.pack("<f", float(self.pitch))
            elif reg == 0x64:  # YAW
                payload = struct.pack("<f", float(self.yaw))
            else:
                payload = b"\x00" * read_len

        pkt_len = len(payload) + 8
        frame_header = bytes([0x55, 0x00, pkt_len, 0x12, reg])
        chk = self.calculate_checksum(bytes([pkt_len, 0x12, reg]) + payload)
        frame = frame_header + payload + bytes([chk, 0x00, 0xAA])
        with self.lock:
            self.tx_packets += 1
        return frame

    def parse_packet(self, data: bytes):
        if len(data) < 8 or data[0] != 0x55 or data[1] != 0x00:
            return

        pkt_len = data[2]
        mode = data[3]
        reg = data[4]

        # Verify Checksum
        data_bytes = data[2:-3]
        if data[-3] != self.calculate_checksum(data_bytes):
            with self.lock:
                self.checksum_errors += 1
            return

        with self.lock:
            self.rx_packets += 1

        if mode == 0x00:  # Write velocity
            val = data[5]
            with self.lock:
                # Cancel autonomous waypoint seeking when external serial /cmd_vel arrives
                self.goal_x = None
                self.goal_y = None
                if reg == 0x30:
                    self.vx = self.u8_to_velocity(val, -0.25, 0.25)
                    self.last_cmd_time = time.time()
                elif reg == 0x31:
                    self.vy = self.u8_to_velocity(val, -0.18, 0.18)
                    self.last_cmd_time = time.time()
                elif reg == 0x32:
                    self.vyaw = self.u8_to_velocity(val, -1.745, 1.745)
                    self.last_cmd_time = time.time()
                elif reg == 0x3E and val == 255:
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vyaw = 0.0

        elif mode == 0x02:  # Read query
            read_len = data[5] if len(data) > 5 else 1
            resp = self.handle_read_request(reg, read_len)
            try:
                os.write(self.master_fd, resp)
            except OSError:
                pass

    def run_serial_loop(self):
        buffer = bytearray()
        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.02)
            if r:
                try:
                    chunk = os.read(self.master_fd, 128)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                except OSError:
                    break

                while len(buffer) >= 8:
                    idx = buffer.find(b"\x55\x00")
                    if idx == -1:
                        buffer.clear()
                        break
                    if idx > 0:
                        del buffer[:idx]
                    if len(buffer) < 3:
                        break
                    pkt_len = buffer[2]
                    if len(buffer) < pkt_len:
                        break
                    pkt = bytes(buffer[:pkt_len])
                    del buffer[:pkt_len]
                    self.parse_packet(pkt)

    def run_kinematics_loop(self):
        last_t = time.time()
        last_trail_t = 0.0
        while self.running:
            time.sleep(0.02)
            now = time.time()
            dt = now - last_t
            last_t = now
            with self.lock:
                # Clear visual goal pin if robot is within 0.25m
                if self.goal_x is not None and self.goal_y is not None:
                    if math.hypot(self.goal_x - self.x, self.goal_y - self.y) < 0.25:
                        self.goal_x = None
                        self.goal_y = None

                # Watchdog (0.5s timeout for safety)
                if now - self.last_cmd_time > 0.5:
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vyaw = 0.0
                    self.is_moving = False
                else:
                    self.is_moving = (abs(self.vx) > 0.005 or abs(self.vy) > 0.005 or abs(self.vyaw) > 0.02)

                # Odometry Integration & Collision Check
                self.theta += self.vyaw * dt
                self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

                target_dx = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
                target_dy = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
                next_x = self.x + target_dx
                next_y = self.y + target_dy

                # Check map collision with full robot footprint (radius 17cm)
                if not self.map.is_occupied_footprint(next_x, next_y, radius=0.17):
                    self.x = next_x
                    self.y = next_y
                else:
                    # Bumped into wall: halt linear motion
                    self.vx = 0.0
                    self.vy = 0.0

                self.yaw = math.degrees(self.theta)

                # 4. Gait and LiDAR animation update
                if self.is_moving:
                    speed = math.hypot(self.vx, self.vy) + abs(self.vyaw) * 0.1
                    freq = max(1.2, min(4.5, speed * 14.0))
                    self.gait_phase = (self.gait_phase + freq * 2.0 * math.pi * dt) % (2.0 * math.pi)
                    self.battery = max(5.0, self.battery - 0.001 * dt)
                else:
                    self.gait_phase = 0.0

                # LiDAR sensor head spinning at 10 Hz
                self.lidar_rotation = (self.lidar_rotation + 10.0 * 2.0 * math.pi * dt) % (2.0 * math.pi)

                # 5. Trajectory trail
                if now - last_trail_t > 0.08:
                    if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.025:
                        self.trail.append((self.x, self.y))
                        if len(self.trail) > self.max_trail_len:
                            self.trail.pop(0)
                        last_trail_t = now

    def set_goal(self, gx: float, gy: float):
        with self.lock:
            self.goal_x = gx
            self.goal_y = gy

    def reset_pose(self):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.yaw = 0.0
            self.goal_x = None
            self.goal_y = None
            self.planned_path.clear()
            self.trail.clear()

    def set_manual_cmd(self, vx: float, vy: float, vyaw: float):
        with self.lock:
            self.goal_x = None
            self.goal_y = None
            self.vx = vx
            self.vy = vy
            self.vyaw = vyaw
            self.last_cmd_time = time.time()
            self.is_moving = (abs(self.vx) > 0.005 or abs(self.vy) > 0.005 or abs(self.vyaw) > 0.02)

    def close(self):
        self.running = False
        try:
            os.close(self.master_fd)
            os.close(self.slave_fd)
            if os.path.islink(self.port_link):
                os.remove(self.port_link)
        except Exception:
            pass


# ==============================================================================
# 3. Optional ROS 2 Bridge Node (/scan & /goal_pose)
# ==============================================================================
class Ros2Bridge(threading.Thread):
    def __init__(self, mcu: XgoMockMCU, room_map: RoomMap):
        super().__init__(daemon=True)
        self.mcu = mcu
        self.map = room_map
        self.node = None
        self.scan_pub = None
        self.goal_pub = None
        self.tf_broadcaster = None
        self.static_tf_broadcaster = None

    def run(self):
        if not HAS_ROS2:
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node("xgo_mock_bridge")
            self.scan_pub = self.node.create_publisher(LaserScan, "/scan", 10)
            self.goal_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

            # Send static transforms: base_footprint -> base_link -> laser_frame
            t_link = TransformStamped()
            t_link.header.stamp = self.node.get_clock().now().to_msg()
            t_link.header.frame_id = "base_footprint"
            t_link.child_frame_id = "base_link"
            t_link.transform.rotation.w = 1.0

            t_laser = TransformStamped()
            t_laser.header.stamp = self.node.get_clock().now().to_msg()
            t_laser.header.frame_id = "base_link"
            t_laser.child_frame_id = "laser_frame"
            t_laser.transform.translation.x = 0.05
            t_laser.transform.translation.y = 0.0
            t_laser.transform.translation.z = 0.15
            t_laser.transform.rotation.w = 1.0

            self.static_tf_broadcaster.sendTransform([t_link, t_laser])
            self.plan_sub = self.node.create_subscription(Path, "/plan", self._plan_callback, 10)
            print("[ROS 2 Bridge] Active: Streaming /scan, robot TF, /goal_pose, and listening to /plan.")

            # Periodic scan & TF publisher at 10 Hz
            while self.mcu.running and rclpy.ok():
                time.sleep(0.10)
                self._publish_scan_and_tf()
                rclpy.spin_once(self.node, timeout_sec=0.01)
        except Exception as e:
            print(f"[ROS 2 Bridge] Note: Bridge stopped ({e})")

    def _plan_callback(self, msg: Path):
        with self.mcu.lock:
            self.mcu.planned_path = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]

    def _publish_scan_and_tf(self):
        if not self.scan_pub:
            return
        with self.mcu.lock:
            rx, ry, rtheta = self.mcu.x, self.mcu.y, self.mcu.theta

        now_msg = self.node.get_clock().now().to_msg()

        # 1. Dynamic TF: odom -> base_footprint
        t_odom = TransformStamped()
        t_odom.header.stamp = now_msg
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation.x = float(rx)
        t_odom.transform.translation.y = float(ry)
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.z = math.sin(rtheta / 2.0)
        t_odom.transform.rotation.w = math.cos(rtheta / 2.0)
        self.tf_broadcaster.sendTransform(t_odom)

        # 2. LiDAR Scan
        num_rays = 180
        angles, ranges, _ = self.map.raycast_lidar(rx, ry, rtheta, num_rays=num_rays)

        msg = LaserScan()
        msg.header.stamp = now_msg
        msg.header.frame_id = "laser_frame"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / num_rays
        msg.time_increment = 0.0
        msg.scan_time = 0.10
        msg.range_min = 0.08
        msg.range_max = 8.0
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def publish_goal(self, gx: float, gy: float, gtheta: float = 0.0):
        if not self.goal_pub or not self.node:
            return
        try:
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(gx)
            msg.pose.position.y = float(gy)
            msg.pose.position.z = 0.0
            # Quaternion from yaw
            msg.pose.orientation.z = math.sin(gtheta / 2.0)
            msg.pose.orientation.w = math.cos(gtheta / 2.0)
            self.goal_pub.publish(msg)
            print(f"[ROS 2 Bridge] Published Navigation Goal: ({gx:.2f}, {gy:.2f}) -> /goal_pose")
        except Exception as e:
            print(f"[ROS 2 Bridge] Error publishing goal: {e}")


# ==============================================================================
# 4. Gazebo-Style Animated 2D Visualizer GUI (Tkinter)
# ==============================================================================
class XgoGazeboSimGUI:
    def __init__(self, mcu: XgoMockMCU, room_map: RoomMap, bridge: Ros2Bridge):
        self.mcu = mcu
        self.map = room_map
        self.bridge = bridge

        self.root = tk.Tk()
        self.root.title("XGO Mini 2S - Gazebo Simulation Arena (Mock MCU & LiDAR)")
        self.root.geometry("1200x820")
        self.root.minsize(800, 600)
        self.root.configure(bg="#22252a")

        # Viewport params
        self.scale = 110.0   # Pixels per meter
        self.pan_x = 0.0     # World camera center (X)
        self.pan_y = 0.0     # World camera center (Y)
        self.follow_robot = True
        self.show_lidar = True
        self.drag_start = None

        # Build UI layout (clean, Gazebo-like, minimal HUD overlay)
        self._build_ui()
        self._bind_events()

        # Canvas refresh loop (40 FPS)
        self.root.after(25, self._update_loop)

    def _build_ui(self):
        # 1. Main Fullscreen Canvas (Gazebo ground plane theme)
        self.canvas = tk.Canvas(self.root, bg="#2b2e34", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # 2. Minimalist Floating Top Control Bar (Pills)
        self.top_bar = tk.Frame(self.canvas, bg="#1a1c20", bd=1, relief=tk.SOLID)
        self.top_bar.place(relx=0.015, rely=0.015, anchor="nw")

        tk.Label(self.top_bar, text="🤖 XGO Mini 2S", font=("DejaVu Sans", 10, "bold"),
                 fg="#f8fafc", bg="#1a1c20", padx=10, pady=6).pack(side=tk.LEFT)

        self.lbl_hud = tk.Label(self.top_bar, text="Pose: (0.00m, 0.00m) | Bat: 88% 🔋 | IDLE",
                                font=("DejaVu Sans Mono", 9), fg="#94a3b8", bg="#1a1c20", padx=10)
        self.lbl_hud.pack(side=tk.LEFT)

        # Buttons on Top Bar
        self.btn_lidar = tk.Button(self.top_bar, text="📡 LiDAR: ON", font=("DejaVu Sans", 8, "bold"),
                                   bg="#0284c7", fg="white", activebackground="#0369a1", bd=0, padx=10, pady=4,
                                   command=self._toggle_lidar)
        self.btn_lidar.pack(side=tk.LEFT, padx=3, pady=3)

        self.btn_follow = tk.Button(self.top_bar, text="🎥 Cam: Follow", font=("DejaVu Sans", 8),
                                    bg="#334155", fg="white", activebackground="#475569", bd=0, padx=8, pady=4,
                                    command=self._toggle_follow)
        self.btn_follow.pack(side=tk.LEFT, padx=3, pady=3)

        self.btn_clear = tk.Button(self.top_bar, text="🧹 Clear Trail", font=("DejaVu Sans", 8),
                                   bg="#334155", fg="white", activebackground="#475569", bd=0, padx=8, pady=4,
                                   command=self._clear_trail)
        self.btn_clear.pack(side=tk.LEFT, padx=3, pady=3)

        self.btn_reset = tk.Button(self.top_bar, text="🔄 Reset (0,0)", font=("DejaVu Sans", 8, "bold"),
                                   bg="#b91c1c", fg="white", activebackground="#991b1b", bd=0, padx=8, pady=4,
                                   command=self.mcu.reset_pose)
        self.btn_reset.pack(side=tk.LEFT, padx=(3, 6), pady=3)

        # 3. Floating Bottom Instruction Hint
        self.bot_hint = tk.Frame(self.canvas, bg="#1a1c20", bd=1, relief=tk.SOLID)
        self.bot_hint.place(relx=0.015, rely=0.985, anchor="sw")

        hint_text = "📍 Left Click: Set Nav Goal (/goal_pose) | Right Drag: Pan | Scroll: Zoom | WASD: Drive"
        tk.Label(self.bot_hint, text=hint_text, font=("DejaVu Sans", 8),
                 fg="#cbd5e1", bg="#1a1c20", padx=10, pady=4).pack()

    def _bind_events(self):
        # Click on map to set Navigation Goal (publishes to /goal_pose)
        self.canvas.bind("<Button-1>", self._on_map_click)

        # Drag to pan arena
        self.canvas.bind("<ButtonPress-3>", self._on_drag_start)
        self.canvas.bind("<B3-Motion>", self._on_drag_motion)
        self.canvas.bind("<ButtonRelease-3>", self._on_drag_end)

        # Zoom
        self.canvas.bind("<Button-4>", lambda e: self._zoom(1.15, e))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(0.85, e))
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(1.15 if e.delta > 0 else 0.85, e))

        # Direct keyboard teleop
        self.root.bind("<KeyPress>", self._on_key_press)

    def _toggle_lidar(self):
        self.show_lidar = not self.show_lidar
        self.btn_lidar.config(
            text=f"📡 LiDAR: {'ON' if self.show_lidar else 'OFF'}",
            bg="#0284c7" if self.show_lidar else "#334155"
        )

    def _toggle_follow(self):
        self.follow_robot = not self.follow_robot
        self.btn_follow.config(
            text=f"🎥 Cam: {'Follow' if self.follow_robot else 'Free'}",
            bg="#0284c7" if self.follow_robot else "#334155"
        )

    def _clear_trail(self):
        with self.mcu.lock:
            self.mcu.trail.clear()

    def _zoom(self, factor, event):
        self.scale = max(35.0, min(350.0, self.scale * factor))

    def _on_drag_start(self, event):
        self.drag_start = (event.x, event.y)
        self.canvas.focus_set()

    def _on_drag_motion(self, event):
        if self.drag_start:
            dx = event.x - self.drag_start[0]
            dy = event.y - self.drag_start[1]
            self.drag_start = (event.x, event.y)
            self.pan_x += dx / self.scale
            self.pan_y -= dy / self.scale
            if self.follow_robot:
                self.follow_robot = False
                self.btn_follow.config(text="🎥 Cam: Free", bg="#334155")

    def _on_drag_end(self, event):
        self.drag_start = None

    def _on_map_click(self, event):
        # Prevent clicks on the floating top bar from setting a goal
        if event.y < 50 and event.x < 550:
            return

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        cam_x = self.mcu.x if self.follow_robot else self.pan_x
        cam_y = self.mcu.y if self.follow_robot else self.pan_y

        # Screen to world
        wx = cam_x + (event.x - cw / 2.0) / self.scale
        wy = cam_y - (event.y - ch / 2.0) / self.scale

        # Set goal in mock MCU
        self.mcu.set_goal(wx, wy)

        # Publish to ROS 2 topic /goal_pose
        if self.bridge:
            with self.mcu.lock:
                # Target heading pointing from robot to goal
                dx = wx - self.mcu.x
                dy = wy - self.mcu.y
                target_yaw = math.atan2(dy, dx)
            self.bridge.publish_goal(wx, wy, target_yaw)

    def _on_key_press(self, event):
        key = event.keysym.lower()
        vx, vy, vyaw = 0.0, 0.0, 0.0
        if key in ("w", "up"):
            vx = 0.20
        elif key in ("s", "down"):
            vx = -0.20
        elif key == "a":
            vy = 0.15
        elif key == "d":
            vy = -0.15
        elif key in ("q", "left"):
            vyaw = 0.85
        elif key in ("e", "right"):
            vyaw = -0.85
        elif key == "space":
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            return
        self.mcu.set_manual_cmd(vx, vy, vyaw)

    def _world_to_screen(self, wx, wy, cw, ch):
        cam_x = self.mcu.x if self.follow_robot else self.pan_x
        cam_y = self.mcu.y if self.follow_robot else self.pan_y
        sx = cw / 2.0 + (wx - cam_x) * self.scale
        sy = ch / 2.0 - (wy - cam_y) * self.scale
        return sx, sy

    def _draw_gazebo_ground(self, cw, ch):
        # Neutral Gazebo ground plane with tile grid
        cam_x = self.mcu.x if self.follow_robot else self.pan_x
        cam_y = self.mcu.y if self.follow_robot else self.pan_y

        half_w = (cw / 2.0) / self.scale
        half_h = (ch / 2.0) / self.scale

        min_gx = math.floor(cam_x - half_w)
        max_gx = math.ceil(cam_x + half_w)
        min_gy = math.floor(cam_y - half_h)
        max_gy = math.ceil(cam_y + half_h)

        # 1-meter major grid lines (Gazebo style)
        for gx in range(min_gx, max_gx + 1):
            sx, _ = self._world_to_screen(gx, 0, cw, ch)
            color = "#404550" if gx == 0 else "#333740"
            width = 2 if gx == 0 else 1
            self.canvas.create_line(sx, 0, sx, ch, fill=color, width=width)
            self.canvas.create_text(sx + 4, ch - 16, text=f"{gx}m", fill="#64748b",
                                    font=("DejaVu Sans", 8), anchor="sw")

        for gy in range(min_gy, max_gy + 1):
            _, sy = self._world_to_screen(0, gy, cw, ch)
            color = "#404550" if gy == 0 else "#333740"
            width = 2 if gy == 0 else 1
            self.canvas.create_line(0, sy, cw, sy, fill=color, width=width)
            self.canvas.create_text(10, sy - 4, text=f"{gy}m", fill="#64748b",
                                    font=("DejaVu Sans", 8), anchor="sw")

        # Gazebo Origin Axes (Red X, Green Y)
        ox, oy = self._world_to_screen(0, 0, cw, ch)
        self.canvas.create_line(ox, oy, ox + 28, oy, fill="#ef4444", width=3, arrow=tk.LAST)
        self.canvas.create_line(ox, oy, ox, oy - 28, fill="#22c55e", width=3, arrow=tk.LAST)
        self.canvas.create_text(ox + 34, oy, text="X", fill="#ef4444", font=("DejaVu Sans", 8, "bold"))
        self.canvas.create_text(ox, oy - 34, text="Y", fill="#22c55e", font=("DejaVu Sans", 8, "bold"))

    def _draw_map_walls(self, cw, ch):
        # Render the wall runs loaded from room_map.pgm
        res = self.map.resolution
        ox, oy = self.map.ox, self.map.oy
        h = self.map.h

        for x1, x2, y in self.map.wall_runs:
            wx1 = ox + x1 * res
            wx2 = ox + (x2 + 1) * res
            wy2 = oy + (h - y) * res
            wy1 = wy2 - res

            sx1, sy1 = self._world_to_screen(wx1, wy2, cw, ch)
            sx2, sy2 = self._world_to_screen(wx2, wy1, cw, ch)

            # Draw 3D-ish wall box (Gazebo obstacle style)
            self.canvas.create_rectangle(min(sx1, sx2), min(sy1, sy2), max(sx1, sx2), max(sy1, sy2),
                                         fill="#474e58", outline="#1e2227", width=1)

    def _draw_trail(self, cw, ch):
        with self.mcu.lock:
            pts = list(self.mcu.trail)
        if len(pts) < 2:
            return

        screen_pts = []
        for wx, wy in pts:
            sx, sy = self._world_to_screen(wx, wy, cw, ch)
            screen_pts.extend([sx, sy])

        # Subtle dark-cyan navigation trajectory path
        self.canvas.create_line(*screen_pts, fill="#0284c7", width=2, smooth=True, dash=(5, 3))

    def _draw_nav_goal(self, cw, ch):
        with self.mcu.lock:
            gx, gy = self.mcu.goal_x, self.mcu.goal_y
            rx, ry = self.mcu.x, self.mcu.y

        if gx is None or gy is None:
            return

        gx_s, gy_s = self._world_to_screen(gx, gy, cw, ch)
        rx_s, ry_s = self._world_to_screen(rx, ry, cw, ch)

        # Connecting dashed guidance line
        self.canvas.create_line(rx_s, ry_s, gx_s, gy_s, fill="#10b981", width=1, dash=(3, 3))

        # Target Navigation Pin (Gazebo / RViz 2D Goal Pose marker)
        r = 12
        self.canvas.create_oval(gx_s - r, gy_s - r, gx_s + r, gy_s + r,
                                outline="#10b981", fill="", width=2)
        self.canvas.create_oval(gx_s - 4, gy_s - 4, gx_s + 4, gy_s + 4,
                                fill="#22c55e", outline="#10b981")
        # Target Flag Pin
        self.canvas.create_line(gx_s, gy_s, gx_s, gy_s - 22, fill="#f8fafc", width=2)
        self.canvas.create_polygon(gx_s, gy_s - 22, gx_s + 14, gy_s - 16, gx_s, gy_s - 10,
                                   fill="#10b981", outline="#059669")
        self.canvas.create_text(gx_s + 18, gy_s - 16, text=f"GOAL ({gx:.2f}, {gy:.2f})",
                                fill="#22c55e", font=("DejaVu Sans", 8, "bold"), anchor="w")

    def _draw_planned_path(self, cw, ch):
        with self.mcu.lock:
            pts = list(self.mcu.planned_path)
        if len(pts) < 2:
            return

        screen_pts = []
        for wx, wy in pts:
            sx, sy = self._world_to_screen(wx, wy, cw, ch)
            screen_pts.extend([sx, sy])

        # 1. Dark green outer halo for contrast against floor
        self.canvas.create_line(*screen_pts, fill="#064e3b", width=6, capstyle=tk.ROUND, joinstyle=tk.ROUND)
        # 2. Bright emerald line (Nav2 Planned Path)
        self.canvas.create_line(*screen_pts, fill="#00ff88", width=3, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        # 3. Intermediate path dots
        step = max(1, len(pts) // 25)
        for i in range(0, len(pts), step):
            wx, wy = pts[i]
            sx, sy = self._world_to_screen(wx, wy, cw, ch)
            self.canvas.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#ffffff", outline="#059669")

    def _draw_lidar_scan(self, cw, ch):
        if not self.show_lidar:
            return

        with self.mcu.lock:
            rx, ry, rtheta = self.mcu.x, self.mcu.y, self.mcu.theta

        # Compute real-time raycasting
        angles, ranges, hits = self.map.raycast_lidar(rx, ry, rtheta, num_rays=180, max_range=8.0)
        cx, cy = self._world_to_screen(rx, ry, cw, ch)

        # Draw laser impact dots (Gazebo / RViz laser scan appearance)
        for hx, hy in hits:
            sx, sy = self._world_to_screen(hx, hy, cw, ch)
            # Laser hit dot (bright red point)
            self.canvas.create_oval(sx - 2, sy - 2, sx + 2, sy + 2, fill="#ef4444", outline="#b91c1c")

    def _draw_robot_dog(self, cw, ch):
        with self.mcu.lock:
            rx, ry, rtheta = self.mcu.x, self.mcu.y, self.mcu.theta
            vx, vy, vyaw = self.mcu.vx, self.mcu.vy, self.mcu.vyaw
            phase = self.mcu.gait_phase
            is_moving = self.mcu.is_moving
            lidar_rot = self.mcu.lidar_rotation

        cx, cy = self._world_to_screen(rx, ry, cw, ch)

        # XGO Mini 2S physical dimensions (meters)
        body_len = 0.28
        body_wid = 0.16

        # Trot Gait leg swing calculations
        if is_moving:
            step_ampl = 0.045
            reach_1 = step_ampl * math.sin(phase)
            reach_2 = step_ampl * math.sin(phase + math.pi)
            lift_1 = max(0.0, math.cos(phase))
            lift_2 = max(0.0, math.cos(phase + math.pi))
        else:
            reach_1 = 0.0
            reach_2 = 0.0
            lift_1 = 0.0
            lift_2 = 0.0

        legs_def = [
            ("FL", +body_len/2.0 * 0.85, +body_wid/2.0 * 1.15, reach_1, lift_1),
            ("FR", +body_len/2.0 * 0.85, -body_wid/2.0 * 1.15, reach_2, lift_2),
            ("RL", -body_len/2.0 * 0.85, +body_wid/2.0 * 1.15, reach_2, lift_2),
            ("RR", -body_len/2.0 * 0.85, -body_wid/2.0 * 1.15, reach_1, lift_1),
        ]

        def rot(lx, ly):
            cos_t = math.cos(rtheta)
            sin_t = math.sin(rtheta)
            return (lx * cos_t - ly * sin_t) * self.scale, -(lx * sin_t + ly * cos_t) * self.scale

        # 1. Quadruped Robotic Legs (Gazebo metallic/carbon style)
        for name, hx, hy, reach, lift in legs_def:
            hip_sx = cx + rot(hx, hy)[0]
            hip_sy = cy + rot(hx, hy)[1]

            paw_lx = hx + (reach if "F" in name else -reach * 0.85)
            paw_ly = hy + (0.015 if "L" in name else -0.015)
            paw_sx = cx + rot(paw_lx, paw_ly)[0]
            paw_sy = cy + rot(paw_lx, paw_ly)[1]

            knee_lx = (hx + paw_lx) / 2.0 + (0.02 if "F" in name else -0.02)
            knee_ly = (hy + paw_ly) / 2.0 + (0.025 if "L" in name else -0.025)
            knee_sx = cx + rot(knee_lx, knee_ly)[0]
            knee_sy = cy + rot(knee_lx, knee_ly)[1]

            # Thigh & Shin segments (dark graphite industrial metallic)
            self.canvas.create_line(hip_sx, hip_sy, knee_sx, knee_sy, fill="#333842", width=5, capstyle=tk.ROUND)
            self.canvas.create_line(knee_sx, knee_sy, paw_sx, paw_sy, fill="#4b5262", width=3, capstyle=tk.ROUND)

            # Silver joint pivot
            self.canvas.create_oval(knee_sx - 3, knee_sy - 3, knee_sx + 3, knee_sy + 3,
                                   fill="#94a3b8", outline="#1e293b")

            # Paw rubber grip pad
            pr = 5 if lift < 0.2 else 6
            paw_col = "#0f172a" if lift < 0.2 else "#eab308"  # Yellow highlight on lift
            self.canvas.create_oval(paw_sx - pr, paw_sy - pr, paw_sx + pr, paw_sy + pr,
                                   fill=paw_col, outline="#64748b", width=1)

        # 2. Main Body Chassis (Industrial Matte Graphite with Aluminum Top Plate)
        hl = body_len / 2.0
        hw = body_wid / 2.0
        body_corners_local = [
            (+hl, -hw * 0.7),
            (+hl * 0.9, -hw),
            (-hl * 0.9, -hw),
            (-hl, -hw * 0.7),
            (-hl, +hw * 0.7),
            (-hl * 0.9, +hw),
            (+hl * 0.9, +hw),
            (+hl, +hw * 0.7),
        ]
        body_screen_pts = []
        for lx, ly in body_corners_local:
            dx, dy = rot(lx, ly)
            body_screen_pts.extend([cx + dx, cy + dy])

        self.canvas.create_polygon(*body_screen_pts, fill="#1c2024", outline="#525b68", width=2)

        # Brushed aluminum top deck
        top_plate_local = [
            (+hl * 0.7, -hw * 0.65),
            (-hl * 0.7, -hw * 0.65),
            (-hl * 0.7, +hw * 0.65),
            (+hl * 0.7, +hw * 0.65),
        ]
        top_pts = []
        for lx, ly in top_plate_local:
            dx, dy = rot(lx, ly)
            top_pts.extend([cx + dx, cy + dy])
        self.canvas.create_polygon(*top_pts, fill="#2b313a", outline="#64748b", width=1)

        # 3. Head & Vision Sensor
        head_len = 0.075
        head_wid = 0.09
        head_poly_local = [
            (+hl, -head_wid / 2.0),
            (+hl + head_len, -head_wid / 3.0),
            (+hl + head_len * 1.12, 0.0),
            (+hl + head_len, +head_wid / 3.0),
            (+hl, +head_wid / 2.0)
        ]
        head_pts = []
        for lx, ly in head_poly_local:
            dx, dy = rot(lx, ly)
            head_pts.extend([cx + dx, cy + dy])
        self.canvas.create_polygon(*head_pts, fill="#181a1d", outline="#475569", width=2)

        # Dual Camera Eyes
        for ey_sign in [-1, 1]:
            ex, ey = rot(+hl + head_len * 0.5, ey_sign * head_wid * 0.25)
            self.canvas.create_oval(cx + ex - 2, cy + ey - 2, cx + ex + 2, cy + ey + 2,
                                   fill="#38bdf8", outline="#0284c7")

        # 4. Rotating 2D LiDAR Puck on Top Deck (Gazebo LD19 sensor)
        lr = max(6, int(0.04 * self.scale))
        self.canvas.create_oval(cx - lr, cy - lr, cx + lr, cy + lr, fill="#0f1114", outline="#e2e8f0", width=1)

        # Rotating laser slit/emitter inside the puck
        ldx = lr * 0.8 * math.cos(rtheta + lidar_rot)
        ldy = -lr * 0.8 * math.sin(rtheta + lidar_rot)
        self.canvas.create_line(cx, cy, cx + ldx, cy + ldy, fill="#ef4444", width=2)

        # 5. Heading Indicator Arrow
        hx, hy = rot(0.35, 0)
        self.canvas.create_line(cx, cy, cx + hx, cy + hy, fill="#e2e8f0", width=2,
                                arrow=tk.LAST, arrowshape=(8, 10, 4))

        # 6. Commanded Velocity Vector (if non-zero)
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            arrow_scale = 0.55
            v_world_x = (vx * math.cos(rtheta) - vy * math.sin(rtheta)) * arrow_scale
            v_world_y = (vx * math.sin(rtheta) + vy * math.cos(rtheta)) * arrow_scale
            vx_s, vy_s = v_world_x * self.scale, -v_world_y * self.scale
            self.canvas.create_line(cx, cy, cx + vx_s, cy + vy_s,
                                   fill="#f97316", width=3, arrow=tk.LAST, arrowshape=(9, 11, 4))

    def _update_loop(self):
        self.canvas.delete("all")
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        if cw > 50 and ch > 50:
            # 1. Ground plane & Map obstacles
            self._draw_gazebo_ground(cw, ch)
            self._draw_map_walls(cw, ch)

            # 2. Path trajectory trail
            self._draw_trail(cw, ch)

            # 3. Nav2 Global Planned Path
            self._draw_planned_path(cw, ch)

            # 4. Navigation Goal Marker (if active)
            self._draw_nav_goal(cw, ch)

            # 5. LiDAR Laser scan impact points
            self._draw_lidar_scan(cw, ch)

            # 5. Realistic Robot Dog Model
            self._draw_robot_dog(cw, ch)

        # Update minimalist HUD
        with self.mcu.lock:
            x, y, yaw = self.mcu.x, self.mcu.y, self.mcu.yaw
            bat = self.mcu.battery
            is_mov = self.mcu.is_moving
            gx, gy = self.mcu.goal_x, self.mcu.goal_y

        state_str = "NAVIGATING" if gx is not None else ("TROT GAIT" if is_mov else "IDLE")
        hud_text = f"Pose: ({x:+.2f}m, {y:+.2f}m, {yaw:+.1f}°) | Bat: {bat:.0f}% 🔋 | {state_str}"
        self.lbl_hud.config(text=hud_text)

        if self.mcu.running:
            self.root.after(25, self._update_loop)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.mcu.close()
        self.root.destroy()


# ==============================================================================
# 5. Headless Terminal Mode Fallback
# ==============================================================================
def run_headless(mcu: XgoMockMCU):
    sys.stdout.write("\033[2J\033[H")
    while mcu.running:
        status = "MOVING (Trot)" if mcu.is_moving else "IDLE / SAFE"
        dash = f"""
======================================================================
     XGO Mini 2S - VIRTUAL MCU & LIDAR SIMULATOR (HEADLESS MODE)      
======================================================================
  Port:               {mcu.port_link} (Symlink to {mcu.slave_name})
  Pose:               X: {mcu.x:+.3f}m | Y: {mcu.y:+.3f}m | θ: {mcu.yaw:+.1f}°
  Velocities:         Vx: {mcu.vx:+.2f} m/s | Vy: {mcu.vy:+.2f} m/s | W: {mcu.vyaw:+.2f} rad/s
  Battery:            {mcu.battery:.1f}% (7.8V LiPo)
  Packets:            RX: {mcu.rx_packets} | TX: {mcu.tx_packets}
  Status:             {status}
======================================================================
  Press Ctrl+C to stop.
"""
        sys.stdout.write("\033[H" + dash)
        sys.stdout.flush()
        time.sleep(0.1)


# ==============================================================================
# Main Entrypoint
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="XGO Mini 2S Gazebo-Style Mock Simulator")
    parser.add_argument("--port", default=CLIENT_PORT, help="Virtual serial port symlink")
    parser.add_argument("--map", default=MAP_FILE_DEFAULT, help="Path to room_map.pgm")
    parser.add_argument("--headless", action="store_true", help="Force headless terminal mode")
    args = parser.parse_args()

    # 1. Load room map
    room_map = RoomMap(pgm_path=args.map)

    # 2. Initialize Virtual MCU Hardware
    mcu = XgoMockMCU(room_map, port_link=args.port)
    print(f"Starting Gazebo Mock MCU on {args.port} (slave {mcu.slave_name})...")

    t_serial = threading.Thread(target=mcu.run_serial_loop, daemon=True)
    t_kin = threading.Thread(target=mcu.run_kinematics_loop, daemon=True)
    t_serial.start()
    t_kin.start()

    # 3. Optional ROS 2 Bridge (/scan and /goal_pose)
    bridge = None
    if HAS_ROS2:
        bridge = Ros2Bridge(mcu, room_map)
        bridge.start()

    # 4. Launch GUI or Headless
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if has_display and not args.headless:
        try:
            gui = XgoGazeboSimGUI(mcu, room_map, bridge)
            gui.run()
            return
        except Exception as e:
            print(f"Warning: Failed to launch Tkinter GUI ({e}), falling back to headless.")

    try:
        run_headless(mcu)
    except KeyboardInterrupt:
        print("\nStopping Gazebo Mock MCU...")
    finally:
        mcu.close()


if __name__ == "__main__":
    main()
