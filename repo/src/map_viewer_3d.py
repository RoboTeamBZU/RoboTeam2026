"""
map_viewer_3d.py — runs on the PC (Windows). 2.5D visualization.

Receives the SAME UDP packets from robot_mapper.py as the 2D viewer.
Renders:
  - floor grid
  - wall points extruded into 100 mm tall columns (WRO wall height)
  - robot trail on the floor
  - robot marker (cone pointing along heading)

Mouse: left-drag = orbit, right-drag = zoom, middle-drag = pan.
Requires:  pip install pyvista
"""

import socket
import json
import threading
import math
import numpy as np
import pyvista as pv

LISTEN_PORT   = 5005
WALL_HEIGHT   = 100.0   # mm — WRO walls are 10 cm
MAX_MAP_POINTS = 40000
UPDATE_MS     = 150     # render refresh interval

# =============================
# Shared state + UDP receiver
# =============================

state_lock  = threading.Lock()
map_points  = []                # accumulated (x, y) wall points, world mm
live_points = []                # ONLY the latest scan — real-time layer
trail       = []                # robot path
robot_pose  = [0.0, 0.0, 0.0]   # x, y, heading deg
accumulate  = [True]            # M key toggles map accumulation

def udp_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(1.0)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            packet  = json.loads(data.decode())
        except (socket.timeout, json.JSONDecodeError):
            continue
        with state_lock:
            robot_pose[0] = packet["x"]
            robot_pose[1] = packet["y"]
            robot_pose[2] = packet["h"]
            trail.append((packet["x"], packet["y"]))
            pts = packet.get("pts", [])
            live_points.clear()
            live_points.extend(pts)          # replaced every packet = real time
            if accumulate[0]:
                map_points.extend(pts)
                if len(map_points) > MAX_MAP_POINTS:
                    del map_points[: len(map_points) - MAX_MAP_POINTS]

threading.Thread(target=udp_thread, daemon=True).start()

# =============================
# PyVista scene
# =============================

pv.set_plot_theme("dark")
plotter = pv.Plotter(title="WRO Live Map — 2.5D")
plotter.add_axes()

# Floor grid (static, generous size; auto-camera handles framing)
floor = pv.Plane(center=(0, 0, -1), direction=(0, 0, 1),
                 i_size=6000, j_size=6000,
                 i_resolution=12, j_resolution=12)
plotter.add_mesh(floor, style="wireframe", color="gray", opacity=0.3)

# Dynamic actors — replaced each refresh
actors = {"walls": None, "live": None, "trail": None, "robot": None}

def make_wall_cloud(pts_2d):
    """
    Extrude 2D wall points into vertical columns by stacking
    the same XY at several Z levels. Cheap and fast — reads as
    solid walls once dense.
    """
    n_levels = 5
    zs = np.linspace(0, WALL_HEIGHT, n_levels)
    arr = np.array(pts_2d, dtype=float)
    stacked = np.vstack([
        np.column_stack([arr, np.full(len(arr), z)]) for z in zs
    ])
    cloud = pv.PolyData(stacked)
    return cloud

def refresh():
    with state_lock:
        pts  = list(map_points)
        live = list(live_points)
        tr   = list(trail)
        x, y, h = robot_pose

    # Walls
    if pts:
        cloud = make_wall_cloud(pts)
        if actors["walls"] is not None:
            plotter.remove_actor(actors["walls"], render=False)
        actors["walls"] = plotter.add_mesh(
            cloud, color="white", point_size=2,
            render_points_as_spheres=False,
        )

    # Live scan — latest packet only, bright orange, drawn on top.
    # This is where moving objects appear at their CURRENT position;
    # the white accumulated map keeps their history/smear if enabled.
    if live:
        lcloud = make_wall_cloud(live)
        if actors["live"] is not None:
            plotter.remove_actor(actors["live"], render=False)
        actors["live"] = plotter.add_mesh(
            lcloud, color="orange", point_size=5,
            render_points_as_spheres=False,
        )

    # Trail
    if len(tr) >= 2:
        tarr = np.array(tr, dtype=float)
        tarr3 = np.column_stack([tarr, np.full(len(tarr), 2.0)])
        line = pv.lines_from_points(tarr3)
        if actors["trail"] is not None:
            plotter.remove_actor(actors["trail"], render=False)
        actors["trail"] = plotter.add_mesh(line, color="cyan", line_width=3)

    # Robot marker — cone pointing along heading
    h_rad = math.radians(h)
    direction = (math.cos(h_rad), math.sin(h_rad), 0.0)
    cone = pv.Cone(center=(x, y, 30), direction=direction,
                   height=120, radius=45, resolution=12)
    if actors["robot"] is not None:
        plotter.remove_actor(actors["robot"], render=False)
    actors["robot"] = plotter.add_mesh(cone, color="red")

    plotter.render()

plotter.add_key_event("r", lambda: (map_points.clear(), trail.clear()))
plotter.add_key_event("m", lambda: accumulate.__setitem__(0, not accumulate[0]))

print(f"Listening on UDP {LISTEN_PORT}. Start robot_mapper.py on the Pi.")
print("Keys: R = reset map, M = toggle map accumulation (live-only mode). Close window to quit.")

# Timed refresh loop via pyvista's interactive update
plotter.show(interactive_update=True, auto_close=False)
import time as _time
try:
    while True:
        refresh()
        plotter.update()
        _time.sleep(UPDATE_MS / 1000.0)
except (KeyboardInterrupt, RuntimeError):
    pass
