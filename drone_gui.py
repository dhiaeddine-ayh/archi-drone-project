#!/usr/bin/env python3
"""
drone_gui.py — Complete Drone Control Interface

- Live camera via ROS 2 topic (/usb_cam/image_raw from gazebo_ros_camera)
- Direct PX4 offboard flight control
- Multiple missions + manual direction controls
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
from cv_bridge import CvBridge

import tkinter as tk
from PIL import Image as PILImage, ImageTk
import cv2
import numpy as np
import threading
import math


# ═══════════════════════════════════════════════════
# ROS 2 Node
# ═══════════════════════════════════════════════════

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_gui')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Camera — subscribe to both raw and annotated
        self.create_subscription(Image, '/camera/image_raw', self._raw_cb, 10)
        self.create_subscription(Image, '/road_tracker/annotated_image', self._ann_cb, 10)

        # PX4 state
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._st_cb, qos)

        # Road detection
        self.create_subscription(Bool, '/road_tracker/road_detected', self._road_cb, 10)
        self.create_subscription(Point, '/road_tracker/road_offset', self._off_cb, 10)

        # Publishers
        self.ob_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vc_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.mi_pub = self.create_publisher(String, '/mission/command', 10)

        # State
        self.raw_frame = None
        self.ann_frame = None
        self.frame_count = 0
        self.x = self.y = self.z = self.vx = self.vy = self.heading = 0.0
        self.alt = 0.0
        self.armed = False
        self.nav_state = 0
        self.road_det = False

    def _raw_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.raw_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.frame_count += 1
        except:
            pass

    def _ann_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.ann_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except:
            pass

    def _pos_cb(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        self.alt = -m.z
        self.vx, self.vy = m.vx, m.vy
        self.heading = m.heading

    def _st_cb(self, m):
        self.armed = m.arming_state == 2
        self.nav_state = m.nav_state

    def _road_cb(self, m):
        self.road_det = m.data

    def _off_cb(self, m):
        pass

    def pub_offboard(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.ob_pub.publish(m)

    def pub_sp(self, x, y, z, yaw=float('nan')):
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.yaw = float(yaw)
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(m)

    def vcmd(self, cmd, p1=0., p2=0.):
        m = VehicleCommand()
        m.param1, m.param2 = float(p1), float(p2)
        m.command = cmd
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vc_pub.publish(m)

    def arm(self):
        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def offboard(self):
        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land_cmd(self):
        self.vcmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def send_mi(self, cmd):
        m = String()
        m.data = cmd
        self.mi_pub.publish(m)

    def get_frame(self):
        """Return annotated frame if available, else raw, else None."""
        if self.ann_frame is not None:
            return self.ann_frame
        return self.raw_frame


# ═══════════════════════════════════════════════════
# GUI Application
# ═══════════════════════════════════════════════════

class App:
    ALT = 15.0
    STEP = 2.0

    def __init__(self, node: DroneNode):
        self.node = node

        self.flying = False
        self.preflight_n = 0
        self.tx = self.ty = 0.0
        self.tz = -self.ALT
        self.tyaw = 0.0
        self.flight_timer = None
        self.log_lines = []

        self.root = tk.Tk()
        self.root.title("🛩️ Drone Control Center")
        self.root.configure(bg='#0d1117')
        self.root.geometry("960x750")
        self._build()
        self._keys()

    def _log(self, msg):
        self.log_lines.append(msg)
        if len(self.log_lines) > 5:
            self.log_lines.pop(0)
        self.log_label.config(text="\n".join(self.log_lines))

    def _build(self):
        BG = '#0d1117'
        CARD = '#161b22'
        G = '#2ea043'
        R = '#da3633'
        B = '#388bfd'
        Y = '#d29922'
        P = '#8957e5'
        C = '#3fb950'
        W = '#e6edf3'
        D = '#8b949e'
        BTN = {'font': ('Segoe UI', 10, 'bold'), 'fg': W, 'relief': 'flat',
               'cursor': 'hand2', 'pady': 5, 'bd': 0}

        # ━━━ Top bar ━━━
        top = tk.Frame(self.root, bg='#010409', pady=6)
        top.pack(fill='x')
        tk.Label(top, text="🛩️  DRONE CONTROL CENTER",
                 font=('Segoe UI', 14, 'bold'), fg=W, bg='#010409').pack(side='left', padx=10)
        self.armed_lbl = tk.Label(top, text="● DISARMED", font=('Courier', 10, 'bold'),
                                   fg=R, bg='#010409')
        self.armed_lbl.pack(side='right', padx=10)

        # ━━━ Main: Camera + Controls ━━━
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill='both', expand=True, padx=6, pady=4)
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        # ── Left: Camera ──
        lf = tk.Frame(main, bg='#010409', bd=1, relief='solid')
        lf.grid(row=0, column=0, sticky='nsew', padx=(0, 3))

        self.cam = tk.Label(lf, bg='#010409',
                             text="📷 Waiting for camera...\n\nTopic: /camera/image_raw\nGazebo must be running",
                             fg=D, font=('Segoe UI', 10))
        self.cam.pack(fill='both', expand=True)

        self.cam_info = tk.Label(lf, text="No video", font=('Courier', 9),
                                  fg=D, bg='#010409', anchor='w', padx=5)
        self.cam_info.pack(fill='x')

        # ── Right: Controls ──
        rf = tk.Frame(main, bg=CARD, padx=8, pady=6)
        rf.grid(row=0, column=1, sticky='nsew')

        tk.Label(rf, text="FLIGHT", font=('Segoe UI', 10, 'bold'),
                 fg=C, bg=CARD).pack(anchor='w')

        tk.Button(rf, text="🔓 ARM + TAKEOFF", bg=G, command=self._takeoff, **BTN).pack(fill='x', pady=2)

        bf2 = tk.Frame(rf, bg=CARD)
        bf2.pack(fill='x', pady=2)
        tk.Button(bf2, text="⏹ LAND", bg=Y, command=self._land, **BTN
                  ).pack(side='left', expand=True, fill='x', padx=(0, 2))
        tk.Button(bf2, text="🛑 E-STOP", bg=R, command=self._estop, **BTN
                  ).pack(side='right', expand=True, fill='x', padx=(2, 0))

        tk.Frame(rf, bg='#30363d', height=1).pack(fill='x', pady=4)

        # -- DIRECTION CONTROLS --
        tk.Label(rf, text="MANUAL CONTROL", font=('Segoe UI', 10, 'bold'),
                 fg='#58a6ff', bg=CARD).pack(anchor='w')

        dp = tk.Frame(rf, bg=CARD)
        dp.pack(pady=3)
        DBTN = {'font': ('Segoe UI', 12, 'bold'), 'fg': W, 'relief': 'flat',
                'cursor': 'hand2', 'width': 3, 'height': 1, 'bd': 0}
        tk.Button(dp, text="⬆", bg='#1f6feb', command=lambda: self._move(1, 0, 0, 0), **DBTN).grid(row=0, column=1, padx=1, pady=1)
        tk.Button(dp, text="⬅", bg='#1f6feb', command=lambda: self._move(0, -1, 0, 0), **DBTN).grid(row=1, column=0, padx=1, pady=1)
        tk.Button(dp, text="●", bg='#30363d', **DBTN).grid(row=1, column=1, padx=1, pady=1)
        tk.Button(dp, text="➡", bg='#1f6feb', command=lambda: self._move(0, 1, 0, 0), **DBTN).grid(row=1, column=2, padx=1, pady=1)
        tk.Button(dp, text="⬇", bg='#1f6feb', command=lambda: self._move(-1, 0, 0, 0), **DBTN).grid(row=2, column=1, padx=1, pady=1)

        ay = tk.Frame(rf, bg=CARD)
        ay.pack(fill='x', pady=2)
        tk.Button(ay, text="🔼 UP", bg='#238636', command=lambda: self._move(0, 0, 1, 0), **BTN
                  ).pack(side='left', expand=True, fill='x', padx=1)
        tk.Button(ay, text="🔽 DOWN", bg='#238636', command=lambda: self._move(0, 0, -1, 0), **BTN
                  ).pack(side='left', expand=True, fill='x', padx=1)
        tk.Button(ay, text="↺", bg='#6e40c9', command=lambda: self._move(0, 0, 0, -0.3), **BTN
                  ).pack(side='left', expand=True, fill='x', padx=1)
        tk.Button(ay, text="↻", bg='#6e40c9', command=lambda: self._move(0, 0, 0, 0.3), **BTN
                  ).pack(side='left', expand=True, fill='x', padx=1)

        tk.Frame(rf, bg='#30363d', height=1).pack(fill='x', pady=4)

        # -- MISSIONS --
        tk.Label(rf, text="MISSIONS", font=('Segoe UI', 10, 'bold'),
                 fg=P, bg=CARD).pack(anchor='w')

        missions = [
            ("🛣️ Follow Roads", G, 'START'),
            ("🚗 Follow Cars", '#e85d04', 'FOLLOWCAR'),
            ("🚓 Road Patrol", '#0077b6', 'PATROL'),
            ("🔄 Circle Orbit", B, 'CIRCLE'),
            ("⬜ Square Patrol", B, 'SQUARE'),
            ("✈️ Hover", Y, 'HOVER'),
            ("∞ Figure-8", P, 'FIGURE8'),
            ("📐 Grid Scan", P, 'GRID'),
            ("🏠 Return Home", R, 'RTH'),
        ]
        for txt, col, cmd in missions:
            tk.Button(rf, text=txt, bg=col,
                      command=lambda c=cmd: self._mission(c), **BTN
                      ).pack(fill='x', pady=1)

        tk.Label(rf, text="↑↓←→=move W/X=alt A/D=yaw", font=('Courier', 8),
                 fg=D, bg=CARD).pack(anchor='w', pady=(4, 0))

        # ━━━ Bottom ━━━
        bot = tk.Frame(self.root, bg='#010409', pady=4, padx=8)
        bot.pack(fill='x')

        st = tk.Frame(bot, bg='#010409')
        st.pack(fill='x')
        self.alt_l = tk.Label(st, text="ALT: --m", font=('Courier', 11, 'bold'), fg=Y, bg='#010409')
        self.alt_l.pack(side='left', padx=(0, 12))
        self.spd_l = tk.Label(st, text="SPD: --m/s", font=('Courier', 11, 'bold'), fg=C, bg='#010409')
        self.spd_l.pack(side='left', padx=(0, 12))
        self.road_l = tk.Label(st, text="ROAD: --", font=('Courier', 11, 'bold'), fg=D, bg='#010409')
        self.road_l.pack(side='left', padx=(0, 12))
        self.pos_l = tk.Label(st, text="POS: (0, 0)", font=('Courier', 9), fg=D, bg='#010409')
        self.pos_l.pack(side='right')

        self.log_label = tk.Label(bot, text="Ready — click ARM + TAKEOFF",
                                   font=('Courier', 9), fg=D, bg='#010409',
                                   anchor='w', justify='left')
        self.log_label.pack(fill='x', pady=(4, 0))

    def _keys(self):
        for k, fn in [
            ('<Up>', lambda e: self._move(1, 0, 0, 0)),
            ('<Down>', lambda e: self._move(-1, 0, 0, 0)),
            ('<Left>', lambda e: self._move(0, -1, 0, 0)),
            ('<Right>', lambda e: self._move(0, 1, 0, 0)),
            ('<w>', lambda e: self._move(0, 0, 1, 0)),
            ('<W>', lambda e: self._move(0, 0, 1, 0)),
            ('<x>', lambda e: self._move(0, 0, -1, 0)),
            ('<X>', lambda e: self._move(0, 0, -1, 0)),
            ('<a>', lambda e: self._move(0, 0, 0, -0.3)),
            ('<A>', lambda e: self._move(0, 0, 0, -0.3)),
            ('<d>', lambda e: self._move(0, 0, 0, 0.3)),
            ('<D>', lambda e: self._move(0, 0, 0, 0.3)),
            ('<s>', lambda e: self._mission('START')),
            ('<e>', lambda e: self._land()),
            ('<Escape>', lambda e: self._estop()),
        ]:
            self.root.bind(k, fn)

    # ─── Flight ───

    def _takeoff(self):
        if self.flying:
            self._log("Already flying!")
            return
        self.flying = True
        self.preflight_n = 0
        self.tx = self.node.x
        self.ty = self.node.y
        self.tz = -self.ALT
        self.tyaw = self.node.heading
        self._log("🔓 Arming + Takeoff...")
        self._flight_loop()

    def _flight_loop(self):
        if not self.flying:
            return
        self.node.pub_offboard()

        if self.preflight_n < 10:
            self.node.pub_sp(self.tx, self.ty, self.tz, self.tyaw)
        elif self.preflight_n == 10:
            self.node.offboard()
            self._log("  → OFFBOARD mode")
            self.node.pub_sp(self.tx, self.ty, self.tz, self.tyaw)
        elif self.preflight_n < 20:
            self.node.pub_sp(self.tx, self.ty, self.tz, self.tyaw)
        elif self.preflight_n == 20:
            self.node.arm()
            self._log("  → Armed! Taking off...")
            self.node.pub_sp(self.tx, self.ty, self.tz, self.tyaw)
        else:
            self.node.pub_sp(self.tx, self.ty, self.tz, self.tyaw)

        self.preflight_n += 1
        self.flight_timer = self.root.after(50, self._flight_loop)

    def _move(self, fwd, right, up, yaw):
        if not self.flying:
            return
        self.tx += fwd * self.STEP
        self.ty += right * self.STEP
        self.tz -= up * self.STEP  # NED: up = more negative z
        self.tyaw += yaw

    def _land(self):
        self.flying = False
        if self.flight_timer:
            self.root.after_cancel(self.flight_timer)
        self.node.land_cmd()
        self.node.send_mi('END')
        self._log("⏹ Landing...")

    def _estop(self):
        self.flying = False
        if self.flight_timer:
            self.root.after_cancel(self.flight_timer)
        self.node.disarm()
        self._log("🛑 EMERGENCY STOP!")

    def _mission(self, cmd):
        if not self.flying:
            self._takeoff()
            self.root.after(2000, lambda: self.node.send_mi(cmd))
            self._log(f"🚀 Takeoff → {cmd}")
        else:
            self.node.send_mi(cmd)
            self._log(f"🎯 Mission: {cmd}")

    # ─── Update ───

    def _update(self):
        f = self.node.get_frame()
        if f is not None:
            lw = max(self.cam.winfo_width(), 320)
            lh = max(self.cam.winfo_height(), 240)
            h, w = f.shape[:2]
            s = min(lw / w, lh / h)
            nw, nh = int(w * s), int(h * s)
            pil = PILImage.fromarray(f).resize((nw, nh), PILImage.BILINEAR)
            tki = ImageTk.PhotoImage(image=pil)
            self.cam.config(image=tki, text='')
            self.cam._img = tki
            self.cam_info.config(
                text=f"  {w}×{h} | frames: {self.node.frame_count}", fg='#3fb950')
        else:
            self.cam_info.config(text="  ⏳ Waiting for camera topic...", fg='#d29922')

        spd = math.sqrt(self.node.vx ** 2 + self.node.vy ** 2)
        self.alt_l.config(text=f"ALT: {self.node.alt:.1f}m")
        self.spd_l.config(text=f"SPD: {spd:.1f}m/s")
        self.pos_l.config(text=f"POS: ({self.node.x:.0f}, {self.node.y:.0f})")

        if self.node.road_det:
            self.road_l.config(text="ROAD: ✅", fg='#3fb950')
        else:
            self.road_l.config(text="ROAD: ❌", fg='#da3633')

        if self.node.armed:
            self.armed_lbl.config(text="● ARMED", fg='#3fb950')
        else:
            self.armed_lbl.config(text="● DISARMED", fg='#da3633')

        self.root.after(80, self._update)

    def run(self):
        self.root.after(200, self._update)
        self.root.mainloop()


# ═══════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════

def main():
    rclpy.init()
    node = DroneNode()

    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()

    app = App(node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
