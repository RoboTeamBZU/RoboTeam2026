"""
grid_viewer.py — PC-side live grid display for grid_mapper.py.

Draws the occupancy grid exactly like a maze sheet: 50 mm squares,
occupied cells black, robot as a red arrow, axes in mm.
Click any cell to print its (i,j) cell index and mm coordinates.

Run:  python grid_viewer.py        (pip install matplotlib numpy)
Keys: R = reset view to fit map
"""

import socket
import json
import threading
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

LISTEN_PORT = 5005

def bresenham_cells(a, b):
    x0, y0 = a; x1, y1 = b
    dx = abs(x1-x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1-y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    out = []
    while True:
        out.append((x0, y0))
        if x0 == x1 and y0 == y1: break
        e2 = 2*err
        if e2 >= dy: err += dy; x0 += sx
        if e2 <= dx: err += dx; y0 += sy
    return out

lock = threading.Lock()
state = {"x": 0.0, "y": 0.0, "h": 0.0, "res": 50.0, "occ": [],
         "path": [], "goal": None, "pil": [], "vw": []}

def udp_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", LISTEN_PORT))
    s.settimeout(1.0)
    while True:
        try:
            data, _ = s.recvfrom(262144)
            p = json.loads(data.decode())
        except (socket.timeout, json.JSONDecodeError):
            continue
        with lock:
            state["x"] = p.get("x", state["x"])
            state["y"] = p.get("y", state["y"])
            state["h"] = p.get("h", state["h"])
            state["res"] = p.get("res", state["res"])
            if "occ" in p:
                state["occ"] = p["occ"]
            state["path"] = p.get("path", state["path"])
            state["goal"] = p.get("goal", state["goal"])
            state["pil"]  = p.get("pil",  state["pil"])
            state["vw"]   = p.get("vw",   state["vw"])

threading.Thread(target=udp_thread, daemon=True).start()

fig, ax = plt.subplots(figsize=(9, 9))
fig.canvas.manager.set_window_title("Grid Mapper — live")

def on_click(ev):
    if ev.inaxes != ax or ev.xdata is None:
        return
    with lock:
        res = state["res"]
    ci, cj = int(round(ev.xdata / res)), int(round(ev.ydata / res))
    print(f"clicked cell ({ci},{cj})  =  ({ci*res:.0f},{cj*res:.0f}) mm")

fig.canvas.mpl_connect("button_press_event", on_click)

def redraw(_evt=None):
    with lock:
        x, y, h = state["x"], state["y"], state["h"]
        res = state["res"]
        occ = list(state["occ"])
        path = list(state["path"])
        goal = state["goal"]
        pil  = list(state["pil"])
        vw   = list(state["vw"])

    ax.clear()
    ax.set_aspect("equal")
    ax.set_title(f"pose ({x:.0f},{y:.0f})mm  cell "
                 f"({round(x/res)},{round(y/res)})  h {h:.1f}°   "
                 f"occupied cells: {len(occ)}")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")

    # occupied cells as black squares
    for (ci, cj) in occ:
        ax.add_patch(mpatches.Rectangle(
            (ci*res - res/2, cj*res - res/2), res, res,
            facecolor="black", edgecolor="none"))

    # view bounds around data + robot
    xs = [c[0]*res for c in occ] + [x]
    ys = [c[1]*res for c in occ] + [y]
    x0, x1 = min(xs)-300, max(xs)+300
    y0, y1 = min(ys)-300, max(ys)+300
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)

    # 50 mm grid lines (light), 500 mm lines (darker)
    import numpy as _np
    for gx in _np.arange(_np.floor(x0/res)*res, x1, res):
        ax.axvline(gx - res/2, color="0.9", lw=0.4, zorder=0)
    for gy in _np.arange(_np.floor(y0/res)*res, y1, res):
        ax.axhline(gy - res/2, color="0.9", lw=0.4, zorder=0)
    for gx in _np.arange(_np.floor(x0/500)*500, x1, 500):
        ax.axvline(gx, color="0.75", lw=0.8, zorder=0)
    for gy in _np.arange(_np.floor(y0/500)*500, y1, 500):
        ax.axhline(gy, color="0.75", lw=0.8, zorder=0)

    # virtual walls: translucent colored cells on the FORBIDDEN side
    for (ci, cj, col) in vw:
        c = "red" if col == "RED" else "green"
        ax.add_patch(mpatches.Rectangle(
            (ci*res - res/2, cj*res - res/2), res, res,
            facecolor=c, edgecolor=c, alpha=0.25, zorder=2))

    # pillars: solid colored cell at the tracked position
    for (px, py, col) in pil:
        c = "red" if col == "RED" else "green"
        ci, cj = int(round(px/res)), int(round(py/res))
        ax.add_patch(mpatches.Rectangle(
            (ci*res - res/2, cj*res - res/2), res, res,
            facecolor=c, edgecolor="black", lw=1.0, zorder=3))

    # planned path: filled cells (planner corridor) + thin line (follower)
    if len(path) >= 2:
        path_cells = set()
        for k in range(len(path)-1):
            c0 = (int(round(path[k][0]/res)),   int(round(path[k][1]/res)))
            c1 = (int(round(path[k+1][0]/res)), int(round(path[k+1][1]/res)))
            path_cells |= set(bresenham_cells(c0, c1))
        for ci, cj in path_cells:
            ax.add_patch(mpatches.Rectangle(
                (ci*res - res/2, cj*res - res/2), res, res,
                facecolor="gold", edgecolor="none", alpha=0.45, zorder=3))
        ax.plot([p[0] for p in path], [p[1] for p in path],
                color="darkorange", lw=1.2, zorder=4)
    if goal is not None:
        ax.add_patch(mpatches.Rectangle(
            (goal[0]*res - res/2, goal[1]*res - res/2), res, res,
            facecolor="none", edgecolor="limegreen", lw=2.5, zorder=4))

    # robot arrow
    hr = math.radians(h)
    ax.add_patch(mpatches.FancyArrow(
        x, y, 120*math.cos(hr), 120*math.sin(hr),
        width=40, head_width=90, head_length=70,
        facecolor="red", edgecolor="darkred", zorder=5))

    fig.canvas.draw_idle()

timer = fig.canvas.new_timer(interval=300)
timer.add_callback(redraw)
timer.start()
print(f"Listening on UDP {LISTEN_PORT}. Click a cell to print its coordinates.")
redraw()
plt.show()
