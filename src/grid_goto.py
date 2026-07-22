"""
grid_goto.py — drive to a grid cell using A* over the LIVE occupancy map.
Combines grid_mapper (50 mm occupancy grid, blank until seen) with an
A* planner + waypoint follower:
  - pose relative to start: power-on spot = cell (0,0), heading 0
  - grid built continuously from LiDAR while driving
  - UNKNOWN cells are treated as FREE (optimistic); the path is
    replanned every REPLAN_S as the map fills in
  - type a target:   "30 0"      -> go to cell (30,0)   [= 1500,0 mm]
                     "1500 0 mm" -> same target in millimetres
  - optional: preload a hand-pushed map (LOAD_SAVED_MAP = True)
Streams pose + occupied cells + planned path + goal to grid_viewer.py.
"""
import logging
logging.getLogger("rplidar").setLevel(logging.ERROR)
import adafruit_mpu6050
import board
try:
    import numpy as np
    import cv2
    from picamera2 import Picamera2
    _CAM_LIBS = True
except Exception:
    _CAM_LIBS = False
import time
import threading
import math
import json
import socket
import heapq
import os
from gpiozero import AngularServo, PWMOutputDevice, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from rplidar import RPLidar
# ============================================================
# CONFIG
# ============================================================
PC_IP, PC_PORT = "192.168.1.42", 5005
SERVO_PIN = 18
MOTOR_IN1, MOTOR_IN2 = 24, 23
ENCODER_A, ENCODER_B = 17, 27
ENCODER_DIRECTION, COUNTS_PER_REV, WHEEL_D_MM = -1, 165, 45.0
MM_PER_COUNT = math.pi * WHEEL_D_MM / COUNTS_PER_REV
GYRO_DIRECTION, GYRO_DEADZONE = 1, 0.05
SERVO_MAX, SERVO_DIRECTION = 50, 1
HEAD_KP = 1.1
LOOKAHEAD_MM   = 210
WP_REACH_MM    = 100
GOAL_REACH_MM  = 60
TARGET_MMS       = 300.0
TARGET_MMS_TURNY = 180.0
SLOW_ERR_DEG     = 35
DUTY_BASE, DUTY_MIN, DUTY_MAX = 0.60, 0.30, 1.00
SPD_KP, SPD_KI, SPD_I_CLAMP   = 0.0009, 0.0025, 0.25
SPEED_EMA = 0.35
KICK_SPEED, KICK_TIME = 1.0, 0.15
STALL_MMS, STALL_TIME = 40.0, 0.35
REVERSE_ENABLED  = True
STUCK_S          = 1.2
STUCK_REV_MM     = 250.0
REVERSE_DUTY     = 0.55
EMERGENCY_STOP_MM = 130
LIDAR_PORT, LIDAR_BAUD, MAX_DIST = "/dev/ttyUSB0", 115200, 3000
# ---- grid (identical to grid_mapper) ----
GRID_RESOLUTION_MM = 50.0
MAP_MAX_RANGE_MM   = 2500.0
MAP_LIDAR_STRIDE   = 3
FREE_RAY_STRIDE    = 2
OCC_CONFIRM        = 3
LOAD_SAVED_MAP = False
SAVE_PATH      = "grid_map.json"
# ---- pillars ----
PILLAR_CONE_DEG      = 40
PILLAR_MAX_RANGE     = 1200
PILLAR_MIN_RANGE     = 120
CLUSTER_GAP_MM       = 80
CLUSTER_MIN_PTS      = 3
CLUSTER_MAX_SPAN     = 120
ISOLATION_MARGIN_DEG = 8
ISOLATION_DEPTH_MM   = 200
TRACK_MATCH_MM       = 300
PILLAR_CONFIRM       = 3
PENDING_TTL_S        = 1.0
TRACK_FORGET_S       = 20.0
VWALL_CELLS          = 6      # virtual wall length (cells) on forbidden side
VWALL_CAP_CELLS      = 5      # L-cap: forward stroke sealing the wall's tip
                              # when it doesn't reach a known real wall
PILLAR_ACTIVE_MM     = 2000   # pillars farther than this add no constraints
CAM_W, CAM_H, CAM_HFOV_DEG = 640, 480, 62
RED1_LO, RED1_HI = (0, 90, 60),   (10, 255, 255)
RED2_LO, RED2_HI = (170, 90, 60), (180, 255, 255)
GRN_LO,  GRN_HI  = (40, 70, 50),  (90, 255, 255)
COLOR_MIN_PIXELS = 150
# ---- planner ----
OBSTACLE_INFLATION_MM = 130.0     # half robot width + margin
REPLAN_S              = 0.5
A_STAR_MAX_EXPANSIONS = 15000
GOAL_SEARCH_R         = 6         # cells: nudge goal out of a wall zone
# ============================================================
# HARDWARE
# ============================================================
print("pigpio factory...")
factory = PiGPIOFactory()
print("Servo...")
servo = AngularServo(SERVO_PIN, min_angle=-SERVO_MAX, max_angle=SERVO_MAX,
                     min_pulse_width=0.0005, max_pulse_width=0.0025,
                     pin_factory=factory)
servo.angle = 0
print("Motor...")
m1 = PWMOutputDevice(MOTOR_IN1, pin_factory=factory)
m2 = PWMOutputDevice(MOTOR_IN2, pin_factory=factory)
print("Encoder...")
encoder = RotaryEncoder(ENCODER_A, ENCODER_B, max_steps=10000000,
                        pin_factory=factory)
encoder.steps = 0
print("MPU6050...")
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
def calibrate_gyro(n=600):
    print("Calibrating gyro — keep robot STILL...")
    s = 0.0
    for _ in range(n):
        s += mpu.gyro[2]; time.sleep(0.005)
    off = s / n
    print(f"  offset {off:.6f} rad/s")
    return off
gyro_offset = 0.0        # measured after ENTER (warm chip)
print("LiDAR...")
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
lidar.stop(); lidar.reset(); time.sleep(0.5)
latest_scan, scan_lock, running = [], threading.Lock(), [True]
def scan_thread():
    while running[0]:
        try:
            for scan in lidar.iter_scans(max_buf_meas=800):
                if not running[0]:
                    return
                with scan_lock:
                    latest_scan.clear(); latest_scan.extend(scan)
        except Exception:
            if not running[0]:
                return
            try:
                lidar.stop(); lidar.reset(); time.sleep(0.4)
            except Exception:
                pass
threading.Thread(target=scan_thread, daemon=True).start()
print("Camera...")
picam = None
if _CAM_LIBS:
    try:
        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(
            main={"size": (CAM_W, CAM_H)}))
        picam.start()
        time.sleep(1.5)
        print("  camera OK")
    except Exception as e:
        picam = None
if picam is None:
    print("  NO CAMERA — fallback alternates RED/GREEN per pillar")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# ============================================================
# POSE + MOTOR + SPEED (proven pieces)
# ============================================================
def clamp(v, lo, hi): return max(lo, min(hi, v))
heading = 0.0
x_mm = y_mm = 0.0
last_counts = 0
last_t = time.time()
odo_mm = meas_mms = duty = spd_integ = 0.0
stall_since = None
target_mms = 0.0
def update_pose():
    global heading, x_mm, y_mm, last_counts, last_t, meas_mms, odo_mm
    now = time.time(); dt = now - last_t; last_t = now
    gz = (mpu.gyro[2] - gyro_offset) * (180.0 / math.pi)
    if abs(gz) < GYRO_DEADZONE: gz = 0.0
    heading += gz * GYRO_DIRECTION * dt
    c = encoder.steps * ENCODER_DIRECTION
    d = (c - last_counts) * MM_PER_COUNT
    last_counts = c
    odo_mm += d
    h = math.radians(heading)
    x_mm += d * math.cos(h); y_mm += d * math.sin(h)
    if dt > 0:
        meas_mms = (1-SPEED_EMA)*meas_mms + SPEED_EMA*(abs(d)/dt)
def set_duty(d): m1.value = clamp(d,0,1); m2.value = 0.0
def set_duty_rev(d): m1.value = 0.0; m2.value = clamp(d,0,1)
def motor_off(): m1.value = 0.0; m2.value = 0.0
def kick(): set_duty(KICK_SPEED); time.sleep(KICK_TIME)
def kick_rev(): set_duty_rev(KICK_SPEED); time.sleep(KICK_TIME)
def speed_control():
    global duty, spd_integ, stall_since
    if target_mms <= 0:
        duty = 0.0; spd_integ = 0.0; stall_since = None
        motor_off(); return
    err = target_mms - meas_mms
    spd_integ = clamp(spd_integ + err*SPD_KI*0.02, -SPD_I_CLAMP, SPD_I_CLAMP)
    duty = clamp(DUTY_BASE + SPD_KP*err + spd_integ, DUTY_MIN, DUTY_MAX)
    now = time.time()
    if meas_mms < STALL_MMS:
        if stall_since is None: stall_since = now
        elif now - stall_since >= STALL_TIME:
            kick(); spd_integ *= 0.5; stall_since = None; return
    else:
        stall_since = None
    set_duty(duty)
def wrap(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a
def lidar_front(scan):
    d = [dist for q, a, dist in scan
         if q > 0 and 0 < dist <= MAX_DIST and 170 <= a <= 190]
    return min(d) if d else None
# ============================================================
# GRID (identical semantics to grid_mapper)
# ============================================================
occ_hits, free_hits = {}, {}
def world_to_cell(wx, wy):
    return (int(round(wx / GRID_RESOLUTION_MM)),
            int(round(wy / GRID_RESOLUTION_MM)))
def cell_to_world(c):
    return (c[0]*GRID_RESOLUTION_MM, c[1]*GRID_RESOLUTION_MM)
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
def update_grid(scan):
    rc = world_to_cell(x_mm, y_mm)
    h = heading
    for idx, (q, a, dist) in enumerate(scan):
        if idx % MAP_LIDAR_STRIDE or q <= 0 or not (80 < dist <= MAP_MAX_RANGE_MM):
            continue
        wa = math.radians(180.0 - a + h)
        end = world_to_cell(x_mm + dist*math.cos(wa),
                            y_mm + dist*math.sin(wa))
        for c in bresenham_cells(rc, end)[:-1:FREE_RAY_STRIDE]:
            free_hits[c] = min(20, free_hits.get(c, 0) + 1)
            if c in occ_hits:
                occ_hits[c] = max(0, occ_hits[c] - 1)
        occ_hits[end] = min(20, occ_hits.get(end, 0) + 2)
        free_hits[end] = max(0, free_hits.get(end, 0) - 1)
def occupied_cells():
    return [c for c, n in occ_hits.items()
            if n >= OCC_CONFIRM and n > free_hits.get(c, 0)]
if LOAD_SAVED_MAP and os.path.exists(SAVE_PATH):
    with open(SAVE_PATH) as f:
        data = json.load(f)
    for c in data.get("occupied", []):
        occ_hits[tuple(c)] = 6
    for c in data.get("free", []):
        free_hits[tuple(c)] = 6
    print(f"Loaded {SAVE_PATH}: {len(data.get('occupied',[]))} occupied cells")
# ============================================================
# PILLARS: detection, color, tracking (proven machinery)
# ============================================================
def to_robot_frame(lidar_angle):
    b = 180.0 - lidar_angle
    while b > 180:  b -= 360
    while b < -180: b += 360
    return b
def find_pillars(scan):
    ctx = []
    for q, a, dist in scan:
        if q <= 0 or not (PILLAR_MIN_RANGE < dist <= MAX_DIST):
            continue
        b = to_robot_frame(a)
        if abs(b) <= PILLAR_CONE_DEG + ISOLATION_MARGIN_DEG:
            ctx.append((b, dist))
    if not ctx:
        return []
    ctx.sort()
    pts = [(b, d) for b, d in ctx
           if abs(b) <= PILLAR_CONE_DEG and d <= PILLAR_MAX_RANGE]
    if not pts:
        return []
    clusters, cur = [], [pts[0]]
    for p in pts[1:]:
        if abs(p[1] - cur[-1][1]) < CLUSTER_GAP_MM and (p[0] - cur[-1][0]) < 6:
            cur.append(p)
        else:
            clusters.append(cur); cur = [p]
    clusters.append(cur)
    def isolated(c, d_mean):
        b_lo, b_hi = c[0][0], c[-1][0]
        for b, d in ctx:
            near_lo = (b_lo - ISOLATION_MARGIN_DEG) <= b < b_lo
            near_hi = b_hi < b <= (b_hi + ISOLATION_MARGIN_DEG)
            if (near_lo or near_hi) and abs(d - d_mean) < ISOLATION_DEPTH_MM:
                return False
        return True
    out = []
    for c in clusters:
        if len(c) < CLUSTER_MIN_PTS:
            continue
        bearings = [p[0] for p in c]
        dists    = [p[1] for p in c]
        d_mean   = sum(dists) / len(dists)
        span_deg = max(bearings) - min(bearings)
        span_mm  = 2 * d_mean * math.tan(math.radians(span_deg / 2))
        if span_deg < 25 and span_mm <= CLUSTER_MAX_SPAN and isolated(c, d_mean):
            out.append((d_mean, sum(bearings) / len(bearings)))
    return out
_fallback_next = ["RED"]
def get_color(bearing_deg):
    if picam is None:
        col = _fallback_next[0]
        _fallback_next[0] = "GREEN" if col == "RED" else "RED"
        return col
    frame = picam.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    px = int(CAM_W/2 - (bearing_deg/(CAM_HFOV_DEG/2)) * (CAM_W/2))
    half = CAM_W // 6
    x0, x1 = clamp(px-half, 0, CAM_W), clamp(px+half, 0, CAM_W)
    roi = hsv[CAM_H//4: 3*CAM_H//4, x0:x1]
    if roi.size == 0:
        return "UNKNOWN"
    red = cv2.countNonZero(cv2.inRange(roi, RED1_LO, RED1_HI)) \
        + cv2.countNonZero(cv2.inRange(roi, RED2_LO, RED2_HI))
    grn = cv2.countNonZero(cv2.inRange(roi, GRN_LO, GRN_HI))
    if max(red, grn) < COLOR_MIN_PIXELS:
        return "UNKNOWN"
    return "RED" if red > grn else "GREEN"
tracked, pending = [], []
def update_tracks(scan):
    now = time.time()
    for d, b in find_pillars(scan):
        wa = math.radians(heading + b)
        wx = x_mm + d * math.cos(wa)
        wy = y_mm + d * math.sin(wa)
        best, bd = None, TRACK_MATCH_MM
        for t in tracked:
            dd = math.hypot(t["wx"]-wx, t["wy"]-wy)
            if dd < bd: best, bd = t, dd
        if best is not None:
            best["wx"] = 0.7*best["wx"] + 0.3*wx
            best["wy"] = 0.7*best["wy"] + 0.3*wy
            best["seen"] = now
            if best["color"] == "UNKNOWN":
                best["color"] = get_color(b)
            continue
        cand, cd = None, TRACK_MATCH_MM
        for p in pending:
            dd = math.hypot(p["wx"]-wx, p["wy"]-wy)
            if dd < cd: cand, cd = p, dd
        if cand is not None:
            cand["wx"] = 0.6*cand["wx"] + 0.4*wx
            cand["wy"] = 0.6*cand["wy"] + 0.4*wy
            cand["seen"] = now
            cand["hits"] += 1
            if cand["hits"] >= PILLAR_CONFIRM:
                col = get_color(b)
                if col in ("RED", "GREEN"):
                    tracked.append({"wx": cand["wx"], "wy": cand["wy"],
                                    "color": col, "seen": now})
                    pending.remove(cand)
                    if busy[0]:      # never print over the input prompt;
                                     # the viewer shows idle detections
                        print(f"\nPillar {col} @ "
                              f"({cand['wx']:.0f},{cand['wy']:.0f}) "
                              f"cell {world_to_cell(cand['wx'], cand['wy'])}")
        else:
            pending.append({"wx": wx, "wy": wy, "hits": 1, "seen": now})
    pending[:] = [p for p in pending if now - p["seen"] < PENDING_TTL_S]
    # cull tracks that are stale OR clearly passed (well behind the robot):
    # once passed, a pillar must never constrain planning again — the next
    # replan is free of it and detection continues for the next pillar
    hx, hy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    def _along(t):
        return (t["wx"]-x_mm)*hx + (t["wy"]-y_mm)*hy
    tracked[:] = [t for t in tracked
                  if now - t["seen"] < TRACK_FORGET_S and _along(t) > -400]
vwall_cells = []      # [(ci,cj,color)] for the viewer
def pillar_constraints(blocked):
    """Add each active pillar + its VIRTUAL WALL on the forbidden side.
    RED  -> must pass on pillar's RIGHT -> wall extends LEFT of approach
    GREEN-> must pass on pillar's LEFT  -> wall extends RIGHT of approach
    Approach direction = robot -> pillar. The rule becomes geometry;
    A* then has exactly one legal way around."""
    vwall_cells.clear()
    hx, hy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    for t in tracked:
        dx, dy = t["wx"]-x_mm, t["wy"]-y_mm
        d = math.hypot(dx, dy)
        if d > PILLAR_ACTIVE_MM or d < 1:
            continue
        # only pillars AHEAD constrain; a passed pillar's approach vector
        # is reversed and would put the virtual wall on the wrong side
        if dx*hx + dy*hy < -100:
            continue
        ux, uy = dx/d, dy/d
        left  = (-uy, ux)
        right = (uy, -ux)
        side = left if t["color"] == "RED" else right
        # pillar cell inflated like a wall
        pc = world_to_cell(t["wx"], t["wy"])
        r = max(1, int(math.ceil(OBSTACLE_INFLATION_MM / GRID_RESOLUTION_MM)))
        for ox in range(-r, r+1):
            for oy in range(-r, r+1):
                if ox*ox + oy*oy <= r*r:
                    blocked.add((pc[0]+ox, pc[1]+oy))
        # virtual wall: thin blocked line, NOT inflated (legal side stays open).
        # Perpendicular stroke runs toward the forbidden side; if it merges
        # with an already-known wall, done. If it ends in UNSEEN space, an
        # L-cap continues FORWARD (along approach) from its tip so the path
        # cannot wrap around the end through unknown-as-free cells.
        merged = False
        tip = None
        for k in range(1, VWALL_CELLS+1):
            wxk = t["wx"] + side[0]*k*GRID_RESOLUTION_MM
            wyk = t["wy"] + side[1]*k*GRID_RESOLUTION_MM
            c = world_to_cell(wxk, wyk)
            if c in blocked:
                merged = True          # reached real wall zone: sealed
                break
            blocked.add(c)
            vwall_cells.append((c[0], c[1], t["color"]))
            tip = (wxk, wyk)
        if not merged and tip is not None:
            for m in range(1, VWALL_CAP_CELLS+1):
                wxm = tip[0] + ux*m*GRID_RESOLUTION_MM
                wym = tip[1] + uy*m*GRID_RESOLUTION_MM
                c = world_to_cell(wxm, wym)
                if c in blocked:
                    break              # cap reached a real wall
                blocked.add(c)
                vwall_cells.append((c[0], c[1], t["color"]))
    return blocked
# ============================================================
# A* PLANNER (over live grid; unknown = free)
# ============================================================
_infl_offsets = None
def inflated_blocked():
    global _infl_offsets
    if _infl_offsets is None:
        r = max(1, int(math.ceil(OBSTACLE_INFLATION_MM / GRID_RESOLUTION_MM)))
        _infl_offsets = [(dx,dy) for dx in range(-r,r+1) for dy in range(-r,r+1)
                         if dx*dx+dy*dy <= r*r]
    blocked = set()
    for c in occupied_cells():
        for dx,dy in _infl_offsets:
            blocked.add((c[0]+dx, c[1]+dy))
    return blocked
def nearest_free(c, blocked):
    if c not in blocked: return c
    for r in range(1, GOAL_SEARCH_R+1):
        ring = []
        for dx in range(-r, r+1):
            ring += [(c[0]+dx, c[1]-r), (c[0]+dx, c[1]+r)]
        for dy in range(-r+1, r):
            ring += [(c[0]-r, c[1]+dy), (c[0]+r, c[1]+dy)]
        ring.sort(key=lambda p: (p[0]-c[0])**2 + (p[1]-c[1])**2)
        for p in ring:
            if p not in blocked: return p
    return None
def astar(start, goal, blocked):
    # the robot IS at start — carve a bubble the size of the inflation
    # radius so a start near a wall can always escape its own zone
    r = max(1, int(math.ceil(OBSTACLE_INFLATION_MM / GRID_RESOLUTION_MM)))
    blocked = set(blocked)
    for dx in range(-r, r+1):
        for dy in range(-r, r+1):
            if dx*dx + dy*dy <= r*r:
                blocked.discard((start[0]+dx, start[1]+dy))
    moves = [(1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),
             (1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
    g = {start: 0.0}; came = {}
    pq = [(0.0, start)]; closed = set(); exp = 0
    while pq and exp < A_STAR_MAX_EXPANSIONS:
        _, cur = heapq.heappop(pq)
        if cur in closed: continue
        if cur == goal:
            path = [cur]
            while cur in came: cur = came[cur]; path.append(cur)
            return list(reversed(path))
        closed.add(cur); exp += 1
        for dx,dy,cost in moves:
            nb = (cur[0]+dx, cur[1]+dy)
            if nb in blocked: continue
            if dx and dy and ((cur[0]+dx,cur[1]) in blocked or
                              (cur[0],cur[1]+dy) in blocked):
                continue
            ng = g[cur] + cost
            if ng < g.get(nb, 1e18):
                came[nb] = cur; g[nb] = ng
                heapq.heappush(pq, (ng + math.hypot(goal[0]-nb[0], goal[1]-nb[1]), nb))
    return []
def line_free_cells(c0, c1, blocked):
    return all(c not in blocked for c in bresenham_cells(c0, c1))
def simplify(cells, blocked):
    if len(cells) <= 2: return cells
    out = [cells[0]]; i = 0
    while i < len(cells)-1:
        j = len(cells)-1
        while j > i+1 and not line_free_cells(cells[i], cells[j], blocked):
            j -= 1
        out.append(cells[j]); i = j
    return out
# ============================================================
# STREAMING (grid_viewer format + path/goal)
# ============================================================
planned_path = []        # world-mm waypoints
goal_cell = None
armed = [False]          # False until ENTER: viewer shows a raw scan
                         # preview so you can confirm the LiDAR sees
                         # the track before the real map starts
busy  = [False]          # True while a goto is driving (idle thread yields)
def flush_pose():
    """Discard pose deltas accumulated while idle — stale dt and wheel
    movement from handling the robot must never integrate."""
    global last_t, last_counts
    last_t = time.time()
    last_counts = encoder.steps * ENCODER_DIRECTION
def preview_cells():
    """Raw scan endpoints as cells at pose (0,0,0) — no confidence,
    no memory. Purely for pre-start visual confirmation."""
    with scan_lock:
        scan = list(latest_scan)
    cells = set()
    for q, a, dist in scan:
        if q <= 0 or not (80 < dist <= MAP_MAX_RANGE_MM):
            continue
        wa = math.radians(180.0 - a)          # heading = 0 pre-start
        cells.add(world_to_cell(dist * math.cos(wa), dist * math.sin(wa)))
    return list(cells)
def idle_thread():
    """Keeps pose integration and the occupancy grid running while the
    program waits at the coordinate prompt — the map stays live on the
    viewer, and dt never goes stale between gotos."""
    n = 0
    while running[0]:
        if armed[0] and not busy[0]:
            update_pose()
            n += 1
            if n % 2 == 0:
                with scan_lock:
                    scan = list(latest_scan)
                update_grid(scan)
                if n % 4 == 0:
                    update_tracks(scan)
        time.sleep(0.03)
def stream_thread():
    while running[0]:
        occ = occupied_cells() if armed[0] else preview_cells()
        pkt = {"x": round(x_mm), "y": round(y_mm), "h": round(heading, 1),
               "res": GRID_RESOLUTION_MM, "occ": occ[:4000],
               "path": [(round(px), round(py)) for px, py in planned_path],
               "goal": None if goal_cell is None else list(goal_cell),
               "pil": [[round(t["wx"]), round(t["wy"]), t["color"]]
                       for t in tracked],
               "vw": [list(c) for c in vwall_cells]}
        try:
            sock.sendto(json.dumps(pkt).encode(), (PC_IP, PC_PORT))
        except OSError:
            pass
        time.sleep(0.15)
# ============================================================
# GOTO
# ============================================================
def plan(gc):
    """Plan from current pose to goal cell gc. Returns waypoint list (mm)."""
    blocked = pillar_constraints(inflated_blocked())
    start = world_to_cell(x_mm, y_mm)
    goal  = nearest_free(gc, blocked)
    if goal is None:
        return None, None
    cells = astar(start, goal, blocked)
    if not cells:
        return None, goal
    cells = simplify(cells, blocked)
    return [cell_to_world(c) for c in cells], goal
def reverse_out(mm):
    motor_off(); servo.angle = 0
    mark = odo_mm - mm
    kick_rev()
    while odo_mm > mark:
        update_pose(); set_duty_rev(REVERSE_DUTY); time.sleep(0.02)
    motor_off()
def goto_cell(gc):
    global target_mms, planned_path, goal_cell
    busy[0] = True
    time.sleep(0.05)          # let an in-flight idle iteration finish
    flush_pose()              # idle time must not integrate
    wps, goal = plan(gc)
    goal_cell = goal
    if wps is None:
        print("No path (goal unreachable in current map).")
        planned_path = []
        return
    planned_path = wps
    gx, gy = cell_to_world(goal)
    print(f"Goal cell {goal} = ({gx:.0f},{gy:.0f})mm — {len(wps)} waypoints")
    wp_i = 1 if len(wps) > 1 else 0
    stuck_since = None
    last_replan = time.time()
    loop_n = 0
    target_mms = TARGET_MMS
    kick()
    try:
        while True:
            loop_n += 1
            update_pose()
            with scan_lock:
                scan = list(latest_scan)
            if loop_n % 2 == 0:
                update_grid(scan)
            if loop_n % 3 == 0:
                update_tracks(scan)
            front = lidar_front(scan)
            now = time.time()
            # arrived?
            if math.hypot(gx - x_mm, gy - y_mm) < GOAL_REACH_MM:
                target_mms = 0.0; speed_control(); servo.angle = 0
                ci, cj = world_to_cell(x_mm, y_mm)
                print(f"\nREACHED cell {goal}.  pose ({x_mm:.0f},{y_mm:.0f}) "
                      f"= cell ({ci},{cj})  h={heading:.1f}")
                planned_path = []
                return
            # periodic replan on the growing map
            if now - last_replan >= REPLAN_S:
                last_replan = now
                nwps, ngoal = plan(gc)
                if nwps:
                    wps = nwps; planned_path = wps
                    wp_i = 1 if len(wps) > 1 else 0
            # waypoint advance + lookahead
            while wp_i < len(wps)-1 and \
                  math.hypot(wps[wp_i][0]-x_mm, wps[wp_i][1]-y_mm) < WP_REACH_MM:
                wp_i += 1
            lx, ly = wps[wp_i]
            d_wp = math.hypot(lx-x_mm, ly-y_mm)
            if d_wp < LOOKAHEAD_MM and wp_i < len(wps)-1:
                nx, ny = wps[wp_i+1]
                t = clamp((LOOKAHEAD_MM-d_wp)/max(1.0, math.hypot(nx-lx, ny-ly)), 0, 1)
                lx, ly = lx+(nx-lx)*t, ly+(ny-ly)*t
            err = wrap(math.degrees(math.atan2(ly-y_mm, lx-x_mm)) - heading)
            servo.angle = clamp(HEAD_KP*err*SERVO_DIRECTION, -SERVO_MAX, SERVO_MAX)
            target_mms = TARGET_MMS_TURNY if abs(err) > SLOW_ERR_DEG else TARGET_MMS
            # dead end / stuck
            if front is not None and front < EMERGENCY_STOP_MM:
                if REVERSE_ENABLED:
                    print(f"\nBlocked {front:.0f}mm — reversing + replanning")
                    reverse_out(STUCK_REV_MM)
                    return goto_cell(gc)
                else:
                    print(f"\n[warn] blocked {front:.0f}mm (reversing disabled)")
            if meas_mms < STALL_MMS and target_mms > 0:
                if stuck_since is None: stuck_since = now
                elif now - stuck_since > STUCK_S:
                    if REVERSE_ENABLED:
                        print("\nStuck — reversing + replanning")
                        reverse_out(STUCK_REV_MM)
                        return goto_cell(gc)
                    stuck_since = None
            else:
                stuck_since = None
            speed_control()
            if loop_n % 5 == 0:
                ci, cj = world_to_cell(x_mm, y_mm)
                print(f"GOTO {goal} cell:({ci:3d},{cj:3d}) "
                      f"pose:({x_mm:6.0f},{y_mm:6.0f}) h:{heading:6.1f} "
                      f"err:{err:+6.1f} wp:{wp_i}/{len(wps)-1} v:{meas_mms:4.0f}  ",
                      end="\r", flush=True)
            time.sleep(0.02)
    finally:
        target_mms = 0.0
        speed_control()
        busy[0] = False
# ============================================================
# MAIN
# ============================================================
threading.Thread(target=stream_thread, daemon=True).start()
threading.Thread(target=idle_thread, daemon=True).start()
print(f"""
==================================================
GRID GOTO — A* over the live occupancy map
Start = cell (0,0), h=0. Unknown cells count as FREE;
the path replans every {REPLAN_S}s as the map grows.
Targets:  "30 0"       -> cell (30,0)
          "1500 0 mm"  -> millimetres
Empty line quits.
The viewer shows a LIVE SCAN PREVIEW now — check the track
outline looks right, let the gyro warm ~60s, then ENTER
(robot STILL) to calibrate and start the real map.
==================================================""")
input()
gyro_offset = calibrate_gyro()
heading = 0.0; x_mm = y_mm = 0.0
encoder.steps = 0; last_counts = 0
last_t = time.time()
if not LOAD_SAVED_MAP:
    occ_hits.clear(); free_hits.clear()   # preview never pollutes the map
armed[0] = True
try:
    while True:
        raw = input("\ntarget cell i j (or 'x y mm'): ").strip()
        if not raw:
            break
        parts = raw.split()
        try:
            if len(parts) == 3 and parts[2].lower() == "mm":
                gc = world_to_cell(float(parts[0]), float(parts[1]))
            else:
                gc = (int(parts[0]), int(parts[1]))
        except ValueError:
            print("format:  30 0    or    1500 0 mm")
            continue
        goto_cell(gc)
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    running[0] = False
    motor_off(); servo.angle = 0
    time.sleep(0.3)
    try:
        lidar.stop(); lidar.stop_motor(); lidar.disconnect()
    except Exception:
        pass
    try:
        if picam is not None:
            picam.stop()
    except Exception:
        pass
    servo.close(); m1.close(); m2.close(); encoder.close()
    print("Shutdown complete.")
