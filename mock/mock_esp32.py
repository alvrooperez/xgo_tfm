#!/usr/bin/env python3
"""
XGO Mini 2S - Virtual MCU (ESP32) Hardware Simulator & 2D Visual Arena
Simulates the MCU serial protocol over a virtual pseudo-terminal (pty)
and provides an animated 2D graphical visualizer of the quadruped robot dog.
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

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None

CLIENT_PORT = "/tmp/tty_robot"

# ==============================================================================
# Virtual MCU Hardware Simulator
# ==============================================================================
class XgoMockMCU:
    def __init__(self, port_link=CLIENT_PORT):
        self.port_link = port_link
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)

        # Configure raw mode on master
        mode = termios.tcgetattr(self.master_fd)
        mode[3] = mode[3] & ~termios.ECHO  # Disable echo
        termios.tcsetattr(self.master_fd, termios.TCSANOW, mode)

        # Create symlink for client connection
        if os.path.islink(self.port_link) or os.path.exists(self.port_link):
            os.remove(self.port_link)
        os.symlink(self.slave_name, self.port_link)

        # Simulated robot state
        self.battery = 88.0  # Percentage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0       # Degrees
        self.vx = 0.0        # m/s
        self.vy = 0.0        # m/s
        self.vyaw = 0.0      # rad/s

        # World coordinates (meters & radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_time = time.time()
        self.is_moving = False

        # Gait cycle animation
        self.gait_phase = 0.0
        self.gait_frequency = 2.5  # Hz (~trot frequency)

        # Path history for visualizer trail [(x, y)]
        self.trail = []
        self.max_trail_len = 1500

        # Telemetry stats
        self.rx_packets = 0
        self.tx_packets = 0
        self.checksum_errors = 0
        self.running = True
        self.lock = threading.Lock()

    def calculate_checksum(self, data: bytes) -> int:
        sum_val = sum(data) % 256
        return (255 - sum_val) % 256

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

        # Construct response frame: [0x55, 0x00, LEN, 0x12, ADDR, DATA..., CHECKSUM, 0x00, 0xAA]
        pkt_len = len(payload) + 8
        frame_header = bytes([0x55, 0x00, pkt_len, 0x12, reg])
        content_for_chk = bytes([pkt_len, 0x12, reg]) + payload
        chk = self.calculate_checksum(content_for_chk)
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
        chk_received = data[-3]
        chk_calculated = self.calculate_checksum(data_bytes)
        if chk_received != chk_calculated:
            with self.lock:
                self.checksum_errors += 1
            return

        with self.lock:
            self.rx_packets += 1

        if mode == 0x00:  # Write command
            val = data[5]
            with self.lock:
                if reg == 0x30:  # VX
                    self.vx = self.u8_to_velocity(val, -0.25, 0.25)
                    self.last_cmd_time = time.time()
                elif reg == 0x31:  # VY
                    self.vy = self.u8_to_velocity(val, -0.18, 0.18)
                    self.last_cmd_time = time.time()
                elif reg == 0x32:  # VYAW
                    self.vyaw = self.u8_to_velocity(val, -1.745, 1.745)
                    self.last_cmd_time = time.time()
                elif reg == 0x3E:  # ACTION
                    if val == 255:  # Reset
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
                # Watchdog timer (0.5s timeout)
                if now - self.last_cmd_time > 0.5:
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vyaw = 0.0
                    self.is_moving = False
                else:
                    self.is_moving = (abs(self.vx) > 0.005 or abs(self.vy) > 0.005 or abs(self.vyaw) > 0.02)

                # Integrate kinematic odometry
                self.theta += self.vyaw * dt
                self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi  # Normalize [-pi, pi]

                dx = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
                dy = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
                self.x += dx
                self.y += dy
                self.yaw = math.degrees(self.theta)

                # Update gait cycle
                if self.is_moving:
                    speed = math.sqrt(self.vx**2 + self.vy**2) + abs(self.vyaw) * 0.1
                    freq = max(1.0, min(4.0, speed * 12.0))
                    self.gait_phase = (self.gait_phase + freq * 2.0 * math.pi * dt) % (2.0 * math.pi)
                    self.battery = max(5.0, self.battery - 0.001 * dt)
                else:
                    # Gradually return legs to standing stance
                    self.gait_phase = 0.0

                # Record trajectory trail
                if now - last_trail_t > 0.08:
                    if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.02:
                        self.trail.append((self.x, self.y))
                        if len(self.trail) > self.max_trail_len:
                            self.trail.pop(0)
                        last_trail_t = now

    def reset_pose(self):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.yaw = 0.0
            self.trail.clear()

    def set_manual_cmd(self, vx: float, vy: float, vyaw: float):
        with self.lock:
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
# Animated 2D Visualizer GUI (Tkinter)
# ==============================================================================
class XgoVisualSimGUI:
    def __init__(self, mcu: XgoMockMCU):
        self.mcu = mcu

        self.root = tk.Tk()
        self.root.title("XGO Mini 2S - Hardware Mock Simulator & 2D Arena")
        self.root.geometry("1180x760")
        self.root.minsize(900, 600)
        self.root.configure(bg="#14171c")

        # Viewport params
        self.scale = 130.0  # Pixels per meter
        self.pan_x = 0.0    # Offset in meters
        self.pan_y = 0.0
        self.follow_robot = True
        self.drag_start = None

        # Build UI layout
        self._build_ui()
        self._bind_events()

        # Telemetry update loop
        self.root.after(25, self._update_loop)

    def _build_ui(self):
        # Main horizontal split
        self.paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#14171c", bd=0, sashwidth=4)
        self.paned.pack(fill=tk.BOTH, expand=True)

        # ---------------- Left: 2D Arena Canvas ----------------
        self.arena_frame = tk.Frame(self.paned, bg="#1a1d24")
        self.paned.add(self.arena_frame, minsize=550)

        self.canvas = tk.Canvas(self.arena_frame, bg="#181a20", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Canvas overlay header
        self.overlay_header = self.canvas.create_text(
            15, 15, anchor="nw",
            text="2D ARENA SIMULATION (Top-Down Quadruped Model)",
            font=("DejaVu Sans", 11, "bold"), fill="#00d2ff"
        )
        self.overlay_info = self.canvas.create_text(
            15, 38, anchor="nw",
            text="Scale: 1.0m / grid | Mouse Drag: Pan | Scroll: Zoom",
            font=("DejaVu Sans", 9), fill="#8892b0"
        )

        # ---------------- Right: Telemetry & Control Panel ----------------
        self.panel = tk.Frame(self.paned, bg="#1e222b", width=340)
        self.paned.add(self.panel, minsize=320)

        # Title Card
        title_box = tk.Frame(self.panel, bg="#272c38", padx=12, pady=10)
        title_box.pack(fill=tk.X, padx=10, pady=(10, 6))

        tk.Label(title_box, text="XGO Mini 2S Virtual MCU", font=("DejaVu Sans", 13, "bold"),
                 fg="#00d2ff", bg="#272c38").pack(anchor="w")
        tk.Label(title_box, text="Luwu ESP32 Protocol Emulator v1.4", font=("DejaVu Sans", 9),
                 fg="#a0aec0", bg="#272c38").pack(anchor="w")

        # Serial Connection Box
        conn_box = tk.LabelFrame(self.panel, text=" HARDWARE BUS (PTY) ", font=("DejaVu Sans", 9, "bold"),
                                 fg="#00ff88", bg="#1e222b", padx=10, pady=6)
        conn_box.pack(fill=tk.X, padx=10, pady=4)

        self.lbl_port = tk.Label(conn_box, text=f"Port: {self.mcu.port_link}", font=("DejaVu Sans Mono", 9),
                                 fg="#e2e8f0", bg="#1e222b")
        self.lbl_port.pack(anchor="w")

        self.lbl_packets = tk.Label(conn_box, text="RX: 0 | TX: 0 | Errors: 0", font=("DejaVu Sans Mono", 9),
                                    fg="#cbd5e1", bg="#1e222b")
        self.lbl_packets.pack(anchor="w")

        # Battery Telemetry Box
        bat_box = tk.LabelFrame(self.panel, text=" POWER SUBSYSTEM ", font=("DejaVu Sans", 9, "bold"),
                                fg="#ffb84d", bg="#1e222b", padx=10, pady=6)
        bat_box.pack(fill=tk.X, padx=10, pady=4)

        self.lbl_battery = tk.Label(bat_box, text="Battery: 88.0% (7.8V LiPo)", font=("DejaVu Sans", 10, "bold"),
                                    fg="#ffb84d", bg="#1e222b")
        self.lbl_battery.pack(anchor="w")

        self.bat_canvas = tk.Canvas(bat_box, height=14, bg="#2d3748", highlightthickness=0)
        self.bat_canvas.pack(fill=tk.X, pady=(4, 2))

        # Kinematics & Watchdog Box
        kin_box = tk.LabelFrame(self.panel, text=" MOTION & KINEMATICS ", font=("DejaVu Sans", 9, "bold"),
                                fg="#38bdf8", bg="#1e222b", padx=10, pady=6)
        kin_box.pack(fill=tk.X, padx=10, pady=4)

        self.lbl_status = tk.Label(kin_box, text="Status: IDLE / SAFE", font=("DejaVu Sans", 10, "bold"),
                                   fg="#94a3b8", bg="#1e222b")
        self.lbl_status.pack(anchor="w")

        self.lbl_pos = tk.Label(kin_box, text="Pose: X: 0.00m  Y: 0.00m  θ: 0.0°", font=("DejaVu Sans Mono", 9),
                                fg="#e2e8f0", bg="#1e222b")
        self.lbl_pos.pack(anchor="w", pady=(2, 0))

        self.lbl_vel = tk.Label(kin_box, text="Cmd:  Vx: 0.00  Vy: 0.00  W: 0.00", font=("DejaVu Sans Mono", 9),
                                fg="#93c5fd", bg="#1e222b")
        self.lbl_vel.pack(anchor="w")

        # IMU Attitude Box
        imu_box = tk.LabelFrame(self.panel, text=" 6-DOF IMU ATTITUDE ", font=("DejaVu Sans", 9, "bold"),
                                fg="#c084fc", bg="#1e222b", padx=10, pady=6)
        imu_box.pack(fill=tk.X, padx=10, pady=4)

        self.lbl_imu = tk.Label(imu_box, text="Roll: +0.0° | Pitch: +0.0° | Yaw: +0.0°",
                                font=("DejaVu Sans Mono", 9), fg="#e2e8f0", bg="#1e222b")
        self.lbl_imu.pack(anchor="w")

        # Interactive Controls
        ctrl_box = tk.LabelFrame(self.panel, text=" SIMULATOR CONTROLS ", font=("DejaVu Sans", 9, "bold"),
                                 fg="#f43f5e", bg="#1e222b", padx=10, pady=8)
        ctrl_box.pack(fill=tk.X, padx=10, pady=6)

        btn_row1 = tk.Frame(ctrl_box, bg="#1e222b")
        btn_row1.pack(fill=tk.X, pady=2)

        self.btn_follow = tk.Button(btn_row1, text="Follow Robot: ON", font=("DejaVu Sans", 8, "bold"),
                                    bg="#0284c7", fg="white", activebackground="#0369a1", bd=0, padx=8, pady=4,
                                    command=self._toggle_follow)
        self.btn_follow.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_center = tk.Button(btn_row1, text="Center Origin", font=("DejaVu Sans", 8),
                                    bg="#334155", fg="white", activebackground="#475569", bd=0, padx=8, pady=4,
                                    command=self._center_origin)
        self.btn_center.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        btn_row2 = tk.Frame(ctrl_box, bg="#1e222b")
        btn_row2.pack(fill=tk.X, pady=4)

        self.btn_clear = tk.Button(btn_row2, text="Clear Trail", font=("DejaVu Sans", 8),
                                   bg="#334155", fg="white", activebackground="#475569", bd=0, padx=8, pady=4,
                                   command=self._clear_trail)
        self.btn_clear.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_reset = tk.Button(btn_row2, text="Reset Pose", font=("DejaVu Sans", 8, "bold"),
                                   bg="#dc2626", fg="white", activebackground="#b91c1c", bd=0, padx=8, pady=4,
                                   command=self.mcu.reset_pose)
        self.btn_reset.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # Keyboard Manual Teleop Hint
        tele_box = tk.Frame(self.panel, bg="#14171c", padx=10, pady=8)
        tele_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=(4, 10))

        tk.Label(tele_box, text="DIRECT TELEOP (Click arena to focus):", font=("DejaVu Sans", 8, "bold"),
                 fg="#38bdf8", bg="#14171c").pack(anchor="w")
        tk.Label(tele_box, text="  W / S : Forward / Backward\n"
                                "  A / D : Strafe Left / Right\n"
                                "  Q / E : Turn Left / Right\n"
                                "  SPACE : Emergency Stop",
                 font=("DejaVu Sans Mono", 8), justify=tk.LEFT, fg="#94a3b8", bg="#14171c").pack(anchor="w", pady=(2, 0))

    def _bind_events(self):
        # Mouse panning & zooming
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_drag_end)
        self.canvas.bind("<Button-4>", lambda e: self._zoom(1.15, e))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(0.85, e))
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(1.15 if e.delta > 0 else 0.85, e))

        # Direct keyboard teleop
        self.root.bind("<KeyPress>", self._on_key_press)

    def _toggle_follow(self):
        self.follow_robot = not self.follow_robot
        self.btn_follow.config(
            text=f"Follow Robot: {'ON' if self.follow_robot else 'OFF'}",
            bg="#0284c7" if self.follow_robot else "#334155"
        )

    def _center_origin(self):
        self.follow_robot = False
        self.btn_follow.config(text="Follow Robot: OFF", bg="#334155")
        self.pan_x = 0.0
        self.pan_y = 0.0

    def _clear_trail(self):
        with self.mcu.lock:
            self.mcu.trail.clear()

    def _zoom(self, factor, event):
        new_scale = max(30.0, min(500.0, self.scale * factor))
        self.scale = new_scale

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
                self.btn_follow.config(text="Follow Robot: OFF", bg="#334155")

    def _on_drag_end(self, event):
        self.drag_start = None

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
            vyaw = 0.80
        elif key in ("e", "right"):
            vyaw = -0.80
        elif key == "space":
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            return
        self.mcu.set_manual_cmd(vx, vy, vyaw)

    def _world_to_screen(self, wx, wy, cw, ch):
        # Center of screen is (cw/2, ch/2) + pan offset
        cam_x = self.mcu.x if self.follow_robot else self.pan_x
        cam_y = self.mcu.y if self.follow_robot else self.pan_y
        sx = cw / 2.0 + (wx - cam_x) * self.scale
        sy = ch / 2.0 - (wy - cam_y) * self.scale
        return sx, sy

    def _draw_arena(self, cw, ch):
        # Determine visible world coordinates
        cam_x = self.mcu.x if self.follow_robot else self.pan_x
        cam_y = self.mcu.y if self.follow_robot else self.pan_y

        half_w = (cw / 2.0) / self.scale
        half_h = (ch / 2.0) / self.scale

        min_gx = math.floor(cam_x - half_w)
        max_gx = math.ceil(cam_x + half_w)
        min_gy = math.floor(cam_y - half_h)
        max_gy = math.ceil(cam_y + half_h)

        # 1-meter major grid lines
        for gx in range(min_gx, max_gx + 1):
            sx1, _ = self._world_to_screen(gx, cam_y - half_h, cw, ch)
            color = "#334155" if gx == 0 else "#222733"
            width = 2 if gx == 0 else 1
            self.canvas.create_line(sx1, 0, sx1, ch, fill=color, width=width)
            if -half_h + 0.3 < 0:
                self.canvas.create_text(sx1 + 4, ch - 15, text=f"{gx}m", fill="#475569",
                                        font=("DejaVu Sans", 8), anchor="sw")

        for gy in range(min_gy, max_gy + 1):
            _, sy1 = self._world_to_screen(cam_x - half_w, gy, cw, ch)
            color = "#334155" if gy == 0 else "#222733"
            width = 2 if gy == 0 else 1
            self.canvas.create_line(0, sy1, cw, sy1, fill=color, width=width)
            self.canvas.create_text(10, sy1 - 4, text=f"{gy}m", fill="#475569",
                                    font=("DejaVu Sans", 8), anchor="sw")

        # Origin crosshair
        ox, oy = self._world_to_screen(0, 0, cw, ch)
        self.canvas.create_line(ox - 10, oy, ox + 10, oy, fill="#f43f5e", width=2)
        self.canvas.create_line(ox, oy - 10, ox, oy + 10, fill="#10b981", width=2)
        self.canvas.create_text(ox + 8, oy + 8, text="(0,0)", fill="#94a3b8", font=("DejaVu Sans", 8), anchor="nw")

    def _draw_trail(self, cw, ch):
        with self.mcu.lock:
            pts = list(self.mcu.trail)
        if len(pts) < 2:
            return

        screen_pts = []
        for wx, wy in pts:
            sx, sy = self._world_to_screen(wx, wy, cw, ch)
            screen_pts.extend([sx, sy])

        self.canvas.create_line(*screen_pts, fill="#0ea5e9", width=2, smooth=True, dash=(4, 2))

    def _draw_robot_dog(self, cw, ch):
        with self.mcu.lock:
            rx, ry, rtheta = self.mcu.x, self.mcu.y, self.mcu.theta
            vx, vy, vyaw = self.mcu.vx, self.mcu.vy, self.mcu.vyaw
            phase = self.mcu.gait_phase
            is_moving = self.mcu.is_moving

        cx, cy = self._world_to_screen(rx, ry, cw, ch)

        # Dog physical dimensions (meters)
        body_len = 0.28   # 28 cm length
        body_wid = 0.16   # 16 cm width
        leg_len = 0.09    # 9 cm reach

        # Trot gait leg oscillations
        # Diagonal pair 1: FL + RR
        # Diagonal pair 2: FR + RL (phase offset by pi)
        if is_moving:
            step_ampl = 0.045
            reach_1 = step_ampl * math.sin(phase)
            reach_2 = step_ampl * math.sin(phase + math.pi)
            paw_lift_1 = max(0.0, math.cos(phase))
            paw_lift_2 = max(0.0, math.cos(phase + math.pi))
        else:
            reach_1 = 0.0
            reach_2 = 0.0
            paw_lift_1 = 0.0
            paw_lift_2 = 0.0

        # Leg base hip positions in robot coordinate frame (X=forward, Y=left)
        legs_def = [
            ("FL", +body_len/2.0 * 0.85, +body_wid/2.0 * 1.1, reach_1, paw_lift_1),  # Front Left
            ("FR", +body_len/2.0 * 0.85, -body_wid/2.0 * 1.1, reach_2, paw_lift_2),  # Front Right
            ("RL", -body_len/2.0 * 0.85, +body_wid/2.0 * 1.1, reach_2, paw_lift_2),  # Rear Left
            ("RR", -body_len/2.0 * 0.85, -body_wid/2.0 * 1.1, reach_1, paw_lift_1),  # Rear Right
        ]

        def rot(lx, ly):
            # Rotate robot local coords into world screen delta
            cos_t = math.cos(rtheta)
            sin_t = math.sin(rtheta)
            wx = lx * cos_t - ly * sin_t
            wy = lx * sin_t + ly * cos_t
            return wx * self.scale, -wy * self.scale

        # 1. Draw 4 articulated legs & paws
        for name, hx, hy, reach, lift in legs_def:
            hip_sx = cx + rot(hx, hy)[0]
            hip_sy = cy + rot(hx, hy)[1]

            # Paw location with swing
            paw_lx = hx + (reach if "F" in name else -reach * 0.8)
            paw_ly = hy + (0.015 if "L" in name else -0.015)
            paw_sx = cx + rot(paw_lx, paw_ly)[0]
            paw_sy = cy + rot(paw_lx, paw_ly)[1]

            # Upper & lower leg segment
            knee_lx = (hx + paw_lx) / 2.0 + (0.02 if "F" in name else -0.02)
            knee_ly = (hy + paw_ly) / 2.0 + (0.03 if "L" in name else -0.03)
            knee_sx = cx + rot(knee_lx, knee_ly)[0]
            knee_sy = cy + rot(knee_lx, knee_ly)[1]

            # Leg lines
            self.canvas.create_line(hip_sx, hip_sy, knee_sx, knee_sy, fill="#475569", width=4, capstyle=tk.ROUND)
            self.canvas.create_line(knee_sx, knee_sy, paw_sx, paw_sy, fill="#64748b", width=3, capstyle=tk.ROUND)

            # Paw (foot circle)
            paw_r = 5 + int(lift * 2)
            paw_color = "#10b981" if lift > 0.3 else "#f59e0b"
            self.canvas.create_oval(paw_sx - paw_r, paw_sy - paw_r, paw_sx + paw_r, paw_sy + paw_r,
                                   fill=paw_color, outline="#1e293b", width=1)

        # 2. Draw Robot Dog Body Chassis
        hl = body_len / 2.0
        hw = body_wid / 2.0
        body_corners_local = [
            (+hl, -hw * 0.75),
            (+hl * 0.9, -hw),
            (-hl * 0.9, -hw),
            (-hl, -hw * 0.75),
            (-hl, +hw * 0.75),
            (-hl * 0.9, +hw),
            (+hl * 0.9, +hw),
            (+hl, +hw * 0.75),
        ]
        body_screen_pts = []
        for lx, ly in body_corners_local:
            dx, dy = rot(lx, ly)
            body_screen_pts.extend([cx + dx, cy + dy])

        # Main chassis polygon
        self.canvas.create_polygon(*body_screen_pts, fill="#1e293b", outline="#38bdf8", width=2)

        # 3. Luwu XGO decal badge
        bx1, by1 = cx + rot(-0.04, -0.04)[0], cy + rot(-0.04, -0.04)[1]
        bx2, by2 = cx + rot(+0.04, +0.04)[0], cy + rot(+0.04, +0.04)[1]
        self.canvas.create_oval(min(bx1, bx2), min(by1, by2), max(bx1, bx2), max(by1, by2),
                                fill="#0f172a", outline="#0284c7")
        self.canvas.create_text(cx, cy, text="XGO", fill="#38bdf8", font=("DejaVu Sans", 8, "bold"))

        # 4. Dog Head and Sensor Eyes
        head_len = 0.08
        head_wid = 0.10
        head_poly_local = [
            (+hl, -head_wid/2.0),
            (+hl + head_len, -head_wid/3.0),
            (+hl + head_len * 1.15, 0.0),  # Nose
            (+hl + head_len, +head_wid/3.0),
            (+hl, +head_wid/2.0)
        ]
        head_pts = []
        for lx, ly in head_poly_local:
            dx, dy = rot(lx, ly)
            head_pts.extend([cx + dx, cy + dy])
        self.canvas.create_polygon(*head_pts, fill="#0f172a", outline="#00d2ff", width=2)

        # Sensor Eyes
        for ey_sign in [-1, 1]:
            ex, ey = rot(+hl + head_len * 0.5, ey_sign * head_wid * 0.25)
            self.canvas.create_oval(cx + ex - 3, cy + ey - 3, cx + ex + 3, cy + ey + 3,
                                   fill="#00ffcc", outline="#003322")

        # 5. Heading Ray
        hx, hy = rot(0.35, 0)
        self.canvas.create_line(cx, cy, cx + hx, cy + hy, fill="#38bdf8", width=2, arrow=tk.LAST, arrowshape=(8, 10, 4))

        # 6. Commanded Velocity Vector (if non-zero)
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            cmd_mag = math.hypot(vx, vy)
            arrow_scale = 0.6  # Scale for visual arrow
            arrow_wx = vx * arrow_scale
            arrow_wy = vy * arrow_scale
            # Velocity in world frame:
            v_world_x = arrow_wx * math.cos(rtheta) - arrow_wy * math.sin(rtheta)
            v_world_y = arrow_wx * math.sin(rtheta) + arrow_wy * math.cos(rtheta)
            vx_s, vy_s = v_world_x * self.scale, -v_world_y * self.scale
            self.canvas.create_line(cx, cy, cx + vx_s, cy + vy_s,
                                   fill="#f43f5e", width=3, arrow=tk.LAST, arrowshape=(10, 12, 5))

    def _update_loop(self):
        # 1. Clear Canvas
        self.canvas.delete("all")
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        if cw > 50 and ch > 50:
            # Draw layers
            self._draw_arena(cw, ch)
            self._draw_trail(cw, ch)
            self._draw_robot_dog(cw, ch)

            # Re-draw HUD overlay on canvas
            cam_x = self.mcu.x if self.follow_robot else self.pan_x
            cam_y = self.mcu.y if self.follow_robot else self.pan_y
            self.canvas.create_text(
                15, 15, anchor="nw",
                text="XGO MINI 2S - 2D VIRTUAL MCU ARENA",
                font=("DejaVu Sans", 11, "bold"), fill="#00d2ff"
            )
            gait_str = "TROT GAIT (Walking)" if self.mcu.is_moving else "STAND (Idle)"
            color_str = "#10b981" if self.mcu.is_moving else "#94a3b8"
            self.canvas.create_text(
                15, 36, anchor="nw",
                text=f"Pose: ({self.mcu.x:+.2f}m, {self.mcu.y:+.2f}m, {self.mcu.yaw:+.1f}°)  |  Gait: {gait_str}  |  Zoom: {int(self.scale)} px/m",
                font=("DejaVu Sans", 9), fill=color_str
            )

        # 2. Update Side Panel Telemetry
        with self.mcu.lock:
            rx_pkts = self.mcu.rx_packets
            tx_pkts = self.mcu.tx_packets
            chk_errs = self.mcu.checksum_errors
            bat = self.mcu.battery
            vx, vy, vyaw = self.mcu.vx, self.mcu.vy, self.mcu.vyaw
            x, y, yaw = self.mcu.x, self.mcu.y, self.mcu.yaw
            roll, pitch = self.mcu.roll, self.mcu.pitch
            is_mov = self.mcu.is_moving

        self.lbl_packets.config(text=f"RX: {rx_pkts:<5} | TX: {tx_pkts:<5} | Errors: {chk_errs}")
        self.lbl_battery.config(text=f"Battery: {bat:.1f}% ({(6.5 + bat * 0.019):.2f}V LiPo)")

        # Battery Bar
        self.bat_canvas.delete("all")
        bw = self.bat_canvas.winfo_width()
        fill_w = max(2, int((bat / 100.0) * bw))
        bar_color = "#10b981" if bat > 40 else ("#f59e0b" if bat > 20 else "#ef4444")
        self.bat_canvas.create_rectangle(0, 0, fill_w, 14, fill=bar_color, width=0)

        status_text = "Status: MOVING (Trot)" if is_mov else "Status: IDLE / SAFE"
        status_color = "#10b981" if is_mov else "#94a3b8"
        self.lbl_status.config(text=status_text, fg=status_color)
        self.lbl_pos.config(text=f"Pose: X:{x:+.2f}m Y:{y:+.2f}m θ:{yaw:+.1f}°")
        self.lbl_vel.config(text=f"Cmd:  Vx:{vx:+.2f} Vy:{vy:+.2f} W:{vyaw:+.2f}")
        self.lbl_imu.config(text=f"Roll: {roll:+.1f}° | Pitch: {pitch:+.1f}° | Yaw: {yaw:+.1f}°")

        # Schedule next frame (40 FPS)
        if self.mcu.running:
            self.root.after(25, self._update_loop)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.mcu.close()
        self.root.destroy()


# ==============================================================================
# Headless Terminal ASCII Dashboard
# ==============================================================================
def run_headless_dashboard(mcu: XgoMockMCU):
    sys.stdout.write("\033[2J\033[H")
    while mcu.running:
        bat_bar = "█" * int(mcu.battery / 10) + "░" * (10 - int(mcu.battery / 10))
        status_color = "\033[92m" if mcu.is_moving else "\033[93m"
        status_text = "MOVING (Trot)" if mcu.is_moving else "IDLE / WATCHDOG SAFE"

        dog_art = (
            "         / \\__\n"
            "        (    @\\___\n"
            "        /         O\n"
            "       /   (_____/\n"
            "      /_____/   U"
        )

        dashboard = f"""
\033[1;36m======================================================================
         XGO Mini 2S - VIRTUAL MCU (ESP32) HARDWARE SIMULATOR          
======================================================================\033[0m
  \033[1mVirtual Serial Port:\033[0m  \033[94m{mcu.port_link}\033[0m (Symlink to {mcu.slave_name})
  \033[1mLink Status:\033[0m          \033[92mACTIVE / LISTENING\033[0m
  \033[1mPackets Received:\033[0m     {mcu.rx_packets:<8} | \033[1mPackets Replied:\033[0m {mcu.tx_packets:<8}
  \033[1mChecksum Errors:\033[0m      {mcu.checksum_errors}
----------------------------------------------------------------------
  \033[1mSIMULATED HARDWARE TELEMETRY:\033[0m
    Battery Level:      [{bat_bar}] \033[1m{mcu.battery:.1f}%\033[0m (7.8V LiPo)
    Firmware Version:   \033[32mM-5.1.1\033[0m (Luwu MCU Protocol v1.4)
    IMU Orientation:    Roll: {mcu.roll:+.1f}° | Pitch: {mcu.pitch:+.1f}° | Yaw: {mcu.yaw:+.1f}°
----------------------------------------------------------------------
  \033[1mKINEMATICS & COMMAND MONITOR:\033[0m
    Status:             {status_color}\033[1m{status_text}\033[0m
    Commanded Vel:      Vx: {mcu.vx:+.3f} m/s | Vy: {mcu.vy:+.3f} m/s | W: {mcu.vyaw:+.3f} rad/s
    Simulated Position: X:  {mcu.x:+.3f} m   | Y:  {mcu.y:+.3f} m   | θ: {mcu.yaw:+.1f}°
----------------------------------------------------------------------
{dog_art}
\033[1;36m======================================================================\033[0m
  \033[90mPress Ctrl+C to stop the Virtual MCU Simulator.\033[0m
"""
        sys.stdout.write("\033[H" + dashboard)
        sys.stdout.flush()
        time.sleep(0.1)


# ==============================================================================
# Main Entrypoint
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="XGO Mini 2S Virtual MCU Hardware Simulator")
    parser.add_argument("--port", default=CLIENT_PORT, help="Path for virtual serial port symlink")
    parser.add_argument("--headless", action="store_true", help="Force headless terminal ASCII mode")
    args = parser.parse_args()

    mcu = XgoMockMCU(port_link=args.port)
    print(f"Starting Virtual MCU on {args.port} (slave {mcu.slave_name})...")

    t_serial = threading.Thread(target=mcu.run_serial_loop, daemon=True)
    t_kinematics = threading.Thread(target=mcu.run_kinematics_loop, daemon=True)
    t_serial.start()
    t_kinematics.start()

    # Determine whether to launch GUI or Headless
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    launch_gui = has_display and not args.headless

    if launch_gui:
        try:
            gui = XgoVisualSimGUI(mcu)
            gui.run()
            return
        except Exception as e:
            print(f"Warning: Failed to launch Tkinter GUI ({e}), falling back to headless mode.")

    # Headless fallback
    try:
        run_headless_dashboard(mcu)
    except KeyboardInterrupt:
        print("\nStopping Virtual MCU...")
    finally:
        mcu.close()


if __name__ == "__main__":
    main()
