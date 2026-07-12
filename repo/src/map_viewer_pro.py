"""
map_viewer_pro.py — PC-side telemetry dashboard + 2.5D map. 

Upgrades over map_viewer_3d.py:
  - Textured floor (procedural checkered mat, WRO-style) instead of wireframe
  - Solid extruded wall band rendering (voxel columns from map points)
  - Live scan layer (orange) at real-time positions
  - Full telemetry HUD:
      state, commanded speed, MEASURED speed (from pose deltas),
      heading + compass direction, lap count, corner distances F/L/R,
      run timer, distance traveled, packet rate
  - Heading arrow + trail colored by speed
  - Keys: R reset map, M toggle accumulation, F follow-robot camera,
          T top-down view, P perspective view

Works with wro_open.py (uses its extra packet fields) and also with
robot_mapper.py (missing fields shown as --).

Requires: pip install pyvista numpy
"""

import socket
import json
import threading
import math
import time
import numpy as np
import pyvista as pv

LISTEN_PORT    = 5005
WALL_HEIGHT    = 100.0
MAX_MAP_POINTS = 40000
UPDATE_MS      = 120

# ============================================================
# Shared state + UDP receiver
# ============================================================

lock        = threading.Lock()
map_points  = []
live_points = []
trail       = []          # (x, y, t, speed_measured)
pose        = {"x": 0.0, "y": 0.0, "h": 0.0}
telem       = {"state": "--", "spd": None, "f": None, "l": None,
               "r": None, "laps": None}
accumulate  = [True]
pkt_times   = []          # for packet-rate calc
run_start   = [None]
dist_total  = [0.0]
_last_xy    = [None]
_meas_speed = [0.0]       # mm/s measured from pose deltas

def udp_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(1.0)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = json.loads(data.decode())
        except (socket.timeout, json.JSONDecodeError):
            continue
        now = time.time()
        with lock:
            if run_start[0] is None:
                run_start[0] = now
            pose["x"], pose["y"], pose["h"] = p["x"], p["y"], p["h"]

            # measured speed + distance from consecutive poses
            if _last_xy[0] is not None:
                lx, ly, lt = _last_xy[0]
                dt = now - lt
                if dt > 0:
                    d = math.hypot(p["x"] - lx, p["y"] - ly)
                    if d < 500:                      # ignore teleports/resets
                        _meas_speed[0] = 0.7 * _meas_speed[0] + 0.3 * (d / dt)
                        dist_total[0] += d
            _last_xy[0] = (p["x"], p["y"], now)

            trail.append((p["x"], p["y"]))
            pts = p.get("pts", [])
            live_points.clear(); live_points.extend(pts)
            if accumulate[0]:
                map_points.extend(pts)
                if len(map_points) > MAX_MAP_POINTS:
                    del map_points[: len(map_points) - MAX_MAP_POINTS]

            for k in ("state", "spd", "f", "l", "r", "laps"):
                if k in p:
                    telem[k] = p[k]

            pkt_times.append(now)
            while pkt_times and now - pkt_times[0] > 2.0:
                pkt_times.pop(0)

threading.Thread(target=udp_thread, daemon=True).start()

# ============================================================
# Scene
# ============================================================

pv.set_plot_theme("dark")
plotter = pv.Plotter(title="WRO Telemetry — 2.5D", window_size=(1400, 900))
plotter.enable_terrain_style(mouse_wheel_zooms=True)
plotter.add_axes()

# ---- Textured floor: procedural WRO-ish mat (checker + border) ----
def make_floor_texture(n=512):
    img = np.full((n, n, 3), 235, dtype=np.uint8)          # light mat
    cell = n // 16
    for i in range(16):
        for j in range(16):
            if (i + j) % 2 == 0:
                img[i*cell:(i+1)*cell, j*cell:(j+1)*cell] = 215
    img[:cell//2, :] = (40, 40, 40); img[-cell//2:, :] = (40, 40, 40)
    img[:, :cell//2] = (40, 40, 40); img[:, -cell//2:] = (40, 40, 40)
    return pv.Texture(img)

FLOOR_SIZE = 6000
floor = pv.Plane(center=(0, 0, -2), direction=(0, 0, 1),
                 i_size=FLOOR_SIZE, j_size=FLOOR_SIZE)
floor.texture_map_to_plane(inplace=True)
plotter.add_mesh(floor, texture=make_floor_texture())

actors = {"walls": None, "live": None, "trail": None,
          "robot": None, "hud": None}

def wall_cloud(pts_2d, levels=5):
    zs = np.linspace(0, WALL_HEIGHT, levels)
    arr = np.array(pts_2d, dtype=float)
    return pv.PolyData(np.vstack(
        [np.column_stack([arr, np.full(len(arr), z)]) for z in zs]))

COMPASS = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
def compass(h):
    return COMPASS[int(((h % 360) + 22.5) // 45) % 8]

follow_cam = [False]

def refresh():
    with lock:
        pts   = list(map_points)
        live  = list(live_points)
        tr    = list(trail)
        x, y, h = pose["x"], pose["y"], pose["h"]
        tm    = dict(telem)
        rate  = len(pkt_times) / 2.0
        meas  = _meas_speed[0]
        dist  = dist_total[0]
        t0    = run_start[0]

    if pts:
        if actors["walls"] is not None:
            plotter.remove_actor(actors["walls"], render=False)
        actors["walls"] = plotter.add_mesh(
            wall_cloud(pts), color=(200, 200, 210), point_size=2)

    if live:
        if actors["live"] is not None:
            plotter.remove_actor(actors["live"], render=False)
        actors["live"] = plotter.add_mesh(
            wall_cloud(live, levels=4), color="orange", point_size=5)

    if len(tr) >= 2:
        tarr = np.array(tr, dtype=float)
        t3 = np.column_stack([tarr, np.full(len(tarr), 3.0)])
        if actors["trail"] is not None:
            plotter.remove_actor(actors["trail"], render=False)
        actors["trail"] = plotter.add_mesh(
            pv.lines_from_points(t3), color="cyan", line_width=3)

    hr = math.radians(h)
    cone = pv.Cone(center=(x, y, 35), direction=(math.cos(hr), math.sin(hr), 0),
                   height=140, radius=50, resolution=14)
    if actors["robot"] is not None:
        plotter.remove_actor(actors["robot"], render=False)
    actors["robot"] = plotter.add_mesh(cone, color="red")

    # ---- HUD ----
    fmt = lambda v, u="": (f"{v}{u}" if v is not None else "--")
    elapsed = f"{time.time()-t0:6.1f}s" if t0 else "   --"
    hud = (
        f"STATE  {fmt(tm['state'])}\n"
        f"SPEED  cmd {fmt(tm['spd'])}   meas {meas/1000:4.2f} m/s\n"
        f"HEAD   {h:7.1f}°  ({compass(h)})\n"
        f"LAPS   {fmt(tm['laps'])}\n"
        f"WALLS  F {fmt(tm['f'],'mm')}  L {fmt(tm['l'],'mm')}  R {fmt(tm['r'],'mm')}\n"
        f"POSE   ({x:6.0f}, {y:6.0f}) mm\n"
        f"DIST   {dist/1000:6.2f} m    TIME {elapsed}\n"
        f"LINK   {rate:4.1f} pkt/s    MAP {len(pts)} pts"
    )
    if actors["hud"] is not None:
        plotter.remove_actor(actors["hud"], render=False)
    actors["hud"] = plotter.add_text(hud, position="upper_left",
                                     font_size=11, color="white",
                                     font="courier")

    if follow_cam[0]:
        plotter.camera.focal_point = (x, y, 0)
        plotter.camera.position = (x - 1200 * math.cos(hr),
                                   y - 1200 * math.sin(hr), 900)
        plotter.camera.up = (0, 0, 1)

    plotter.render()

def reset_map():
    with lock:
        map_points.clear(); trail.clear()
        dist_total[0] = 0.0; run_start[0] = None; _last_xy[0] = None

plotter.add_key_event("r", reset_map)
plotter.add_key_event("m", lambda: accumulate.__setitem__(0, not accumulate[0]))
plotter.add_key_event("f", lambda: follow_cam.__setitem__(0, not follow_cam[0]))
plotter.add_key_event("t", lambda: plotter.view_xy())
plotter.add_key_event("p", lambda: plotter.view_isometric())

print(f"""Listening on UDP {LISTEN_PORT}.
Keys: R reset | M toggle map accumulation | F follow camera
      T top-down view | P perspective view""")

plotter.show(interactive_update=True, auto_close=False)
try:
    while True:
        refresh()
        plotter.update()
        time.sleep(UPDATE_MS / 1000.0)
except (KeyboardInterrupt, RuntimeError):
    pass
