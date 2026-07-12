"""
map_viewer.py — runs on the PC (Windows).

Receives pose + world-frame LiDAR points from the Pi over UDP
and renders a live top-down map:
  - accumulated wall points (white dots)
  - robot trail (cyan line)
  - current robot position + heading arrow (red)

Run this FIRST, then start robot_mapper.py on the Pi.
Press R in the plot window to reset the map.
"""

import socket
import json
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

LISTEN_PORT = 5005

# =============================
# UDP receiver thread
# =============================

state_lock  = threading.Lock()
map_points  = []        # accumulated wall points (world mm)
trail       = []        # robot positions over time
robot_pose  = [0.0, 0.0, 0.0]   # x, y, heading
packet_count = [0]

MAX_MAP_POINTS = 60000  # cap memory; oldest dropped beyond this

def udp_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(1.0)
    while True:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        try:
            packet = json.loads(data.decode())
        except json.JSONDecodeError:
            continue

        with state_lock:
            robot_pose[0] = packet["x"]
            robot_pose[1] = packet["y"]
            robot_pose[2] = packet["h"]
            trail.append((packet["x"], packet["y"]))
            map_points.extend(packet.get("pts", []))
            if len(map_points) > MAX_MAP_POINTS:
                del map_points[: len(map_points) - MAX_MAP_POINTS]
            packet_count[0] += 1

t = threading.Thread(target=udp_thread, daemon=True)
t.start()

# =============================
# Plot setup
# =============================

fig, ax = plt.subplots(figsize=(9, 9), facecolor="black")
ax.set_facecolor("black")
ax.set_aspect("equal")
ax.tick_params(colors="white")
for spine in ax.spines.values():
    spine.set_color("white")
ax.grid(color="gray", linestyle="--", linewidth=0.4, alpha=0.4)

wall_scatter = ax.scatter([], [], s=1.5, c="white", alpha=0.7)
trail_line,  = ax.plot([], [], c="cyan", linewidth=1.2, alpha=0.9)
robot_dot,   = ax.plot([], [], "ro", markersize=8)
heading_line, = ax.plot([], [], c="red", linewidth=2)

ARROW_LEN = 150  # mm

def on_key(event):
    if event.key in ("r", "R"):
        with state_lock:
            map_points.clear()
            trail.clear()
        print("Map reset.")

fig.canvas.mpl_connect("key_press_event", on_key)

def update(frame):
    with state_lock:
        pts   = list(map_points)
        tr    = list(trail)
        x, y, h = robot_pose
        n_packets = packet_count[0]

    if pts:
        arr = np.array(pts)
        wall_scatter.set_offsets(arr)

    if tr:
        tarr = np.array(tr)
        trail_line.set_data(tarr[:, 0], tarr[:, 1])

    robot_dot.set_data([x], [y])
    h_rad = math.radians(h)
    heading_line.set_data(
        [x, x + ARROW_LEN * math.cos(h_rad)],
        [y, y + ARROW_LEN * math.sin(h_rad)],
    )

    # Auto-zoom to fit all data with margin
    if pts or tr:
        all_pts = pts + tr + [(x, y)]
        arr = np.array(all_pts)
        margin = 300
        ax.set_xlim(arr[:, 0].min() - margin, arr[:, 0].max() + margin)
        ax.set_ylim(arr[:, 1].min() - margin, arr[:, 1].max() + margin)

    ax.set_title(
        f"Live Map — packets:{n_packets}  points:{len(pts)}  "
        f"pose:({x:.0f}, {y:.0f})mm  {h:.0f}°   [press R to reset]",
        color="white",
    )
    return wall_scatter, trail_line, robot_dot, heading_line

ani = animation.FuncAnimation(fig, update, interval=100,
                              blit=False, cache_frame_data=False)

print(f"Listening on UDP port {LISTEN_PORT} — start robot_mapper.py on the Pi.")
plt.show()
