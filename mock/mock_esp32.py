#!/usr/bin/env python3
"""
XGO Mini 2S - Virtual MCU (ESP32) Hardware Simulator & Visual Dashboard
Simulates the MCU serial protocol over a virtual pseudo-terminal (pty).
"""

import os
import pty
import sys
import time
import struct
import select
import termios
import threading

CLIENT_PORT = "/tmp/tty_robot"

class XgoMockMCU:
    def __init__(self, port_link=CLIENT_PORT):
        self.port_link = port_link
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        
        # Configure raw mode on master
        mode = termios.tcgetattr(self.master_fd)
        mode[3] = mode[3] & ~termios.ECHO  # Disable echo
        termios.tcsetattr(self.master_fd, termios.TCSANOW, mode)

        # Create symlink
        if os.path.islink(self.port_link) or os.path.exists(self.port_link):
            os.remove(self.port_link)
        os.symlink(self.slave_name, self.port_link)

        # Robot simulated state
        self.battery = 88.0  # percentage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_time = time.time()
        self.is_moving = False
        
        # Telemetry stats
        self.rx_packets = 0
        self.tx_packets = 0
        self.checksum_errors = 0
        self.running = True

    def calculate_checksum(self, data: bytes) -> int:
        sum_val = sum(data) % 256
        return (255 - sum_val) % 256

    def u8_to_velocity(self, val: int, vmin: float, vmax: float) -> float:
        return vmin + ((vmax - vmin) / 255.0) * val

    def handle_read_request(self, reg: int, read_len: int) -> bytes:
        payload = b""
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
        self.tx_packets += 1
        return frame

    def parse_packet(self, data: bytes):
        if len(data) < 8:
            return
        if data[0] != 0x55 or data[1] != 0x00:
            return

        pkt_len = data[2]
        mode = data[3]
        reg = data[4]

        # Verify Checksum
        data_bytes = data[2:-3]
        chk_received = data[-3]
        chk_calculated = self.calculate_checksum(data_bytes)
        if chk_received != chk_calculated:
            self.checksum_errors += 1
            return

        self.rx_packets += 1

        if mode == 0x00:  # Write command
            val = data[5]
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
            os.write(self.master_fd, resp)

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

                # Process packets starting with 0x55 0x00
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
                        break  # Wait for remaining bytes
                    pkt = bytes(buffer[:pkt_len])
                    del buffer[:pkt_len]
                    self.parse_packet(pkt)

    def run_kinematics_loop(self):
        last_t = time.time()
        while self.running:
            time.sleep(0.05)
            now = time.time()
            dt = now - last_t
            last_t = now

            # Safety Watchdog in MCU (0.5s timeout)
            if now - self.last_cmd_time > 0.5:
                self.vx = 0.0
                self.vy = 0.0
                self.vyaw = 0.0
                self.is_moving = False
            else:
                self.is_moving = (abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.vyaw) > 0.05)

            # Integrate simple odometry
            self.theta += self.vyaw * dt
            # Convert robot local vx, vy to global frame
            import math
            self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            self.yaw = math.degrees(self.theta) % 360.0

            # Battery slow drain
            if self.is_moving:
                self.battery = max(5.0, self.battery - 0.005)

    def render_dashboard(self):
        sys.stdout.write("\033[2J\033[H")  # Clear screen
        while self.running:
            bat_bar = "█" * int(self.battery / 10) + "░" * (10 - int(self.battery / 10))
            status_color = "\033[92m" if self.is_moving else "\033[93m"
            status_text = "MOVING (Trot)" if self.is_moving else "IDLE / WATCHDOG SAFE"

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
  \033[1mVirtual Serial Port:\033[0m  \033[94m{self.port_link}\033[0m (Symlink to {self.slave_name})
  \033[1mLink Status:\033[0m          \033[92mACTIVE / LISTENING\033[0m
  \033[1mPackets Received:\033[0m     {self.rx_packets:<8} | \033[1mPackets Replied:\033[0m {self.tx_packets:<8}
  \033[1mChecksum Errors:\033[0m      {self.checksum_errors}
----------------------------------------------------------------------
  \033[1mSIMULATED HARDWARE TELEMETRY:\033[0m
    Battery Level:      [{bat_bar}] \033[1m{self.battery:.1f}%\033[0m (7.8V LiPo)
    Firmware Version:   \033[32mM-5.1.1\033[0m (Luwu MCU Protocol v1.4)
    IMU Orientation:    Roll: {self.roll:+.1f}° | Pitch: {self.pitch:+.1f}° | Yaw: {self.yaw:+.1f}°
----------------------------------------------------------------------
  \033[1mKINEMATICS & COMMAND MONITOR:\033[0m
    Status:             {status_color}\033[1m{status_text}\033[0m
    Commanded Vel:      Vx: {self.vx:+.3f} m/s | Vy: {self.vy:+.3f} m/s | W: {self.vyaw:+.3f} rad/s
    Simulated Position: X:  {self.x:+.3f} m   | Y:  {self.y:+.3f} m   | θ: {self.yaw:+.1f}°
----------------------------------------------------------------------
{dog_art}
\033[1;36m======================================================================\033[0m
  \033[90mPress Ctrl+C to stop the Virtual MCU Simulator.\033[0m
"""
            sys.stdout.write("\033[H" + dashboard)
            sys.stdout.flush()
            time.sleep(0.1)

    def close(self):
        self.running = False
        try:
            os.close(self.master_fd)
            os.close(self.slave_fd)
            if os.path.islink(self.port_link):
                os.remove(self.port_link)
        except Exception:
            pass

def main():
    mcu = XgoMockMCU()
    print(f"Starting Virtual MCU on {CLIENT_PORT}...")

    t_serial = threading.Thread(target=mcu.run_serial_loop, daemon=True)
    t_kinematics = threading.Thread(target=mcu.run_kinematics_loop, daemon=True)
    t_dash = threading.Thread(target=mcu.render_dashboard, daemon=True)

    t_serial.start()
    t_kinematics.start()
    t_dash.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping Virtual MCU...")
        mcu.close()

if __name__ == "__main__":
    main()
