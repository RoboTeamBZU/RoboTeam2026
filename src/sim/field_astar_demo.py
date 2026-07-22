"""
field_astar_demo.py — offline illustration of the A* planner used in
grid_goto.py, running on a simulated WRO Open-Challenge field.

Builds the same 50 mm occupancy grid the robot builds live (outer 3000x3000
boundary + inner 1000x1000 wall), inflates obstacles by the robot radius,
then runs the SAME A* + path-simplification the vehicle uses to plan a route
from a start cell to a goal cell on the far side of the inner wall.

Saves docs/astar-demo.png. Run: python src/sim/field_astar_demo.py
(pip install matplotlib numpy)
"""
import os, math, heapq
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

RES = 50.0                     # mm per cell (matches grid_goto.py)
INFLATE_MM = 130.0             # OBSTACLE_INFLATION_MM (half robot width + margin)

def w2c(wx, wy): return (int(round(wx/RES)), int(round(wy/RES)))
def c2w(c):      return (c[0]*RES, c[1]*RES)

# ---- build the field occupancy (ring cells of each wall) ----
def ring_cells(x0, y0, x1, y1):
    cells = set()
    ci0, cj0 = w2c(x0, y0); ci1, cj1 = w2c(x1, y1)
    for ci in range(ci0, ci1+1):
        cells.add((ci, cj0)); cells.add((ci, cj1))
    for cj in range(cj0, cj1+1):
        cells.add((ci0, cj)); cells.add((ci1, cj))
    return cells

occ = set()
occ |= ring_cells(-1500, -1500, 1500, 1500)   # outer boundary
occ |= ring_cells(-500,  -500,  500,  500)     # inner wall

# ---- inflate (same idea as inflated_blocked) ----
r = max(1, int(math.ceil(INFLATE_MM / RES)))
offs = [(dx,dy) for dx in range(-r,r+1) for dy in range(-r,r+1) if dx*dx+dy*dy<=r*r]
blocked = set()
for c in occ:
    for dx,dy in offs:
        blocked.add((c[0]+dx, c[1]+dy))

# ---- A* (identical to grid_goto.astar) ----
def astar(start, goal, blocked):
    blocked = set(blocked)
    for dx in range(-r, r+1):
        for dy in range(-r, r+1):
            if dx*dx+dy*dy <= r*r:
                blocked.discard((start[0]+dx, start[1]+dy))
    moves = [(1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),
             (1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
    g={start:0.0}; came={}; pq=[(0.0,start)]; closed=set()
    while pq:
        _,cur=heapq.heappop(pq)
        if cur in closed: continue
        if cur==goal:
            path=[cur]
            while cur in came: cur=came[cur]; path.append(cur)
            return list(reversed(path)), closed
        closed.add(cur)
        for dx,dy,cost in moves:
            nb=(cur[0]+dx,cur[1]+dy)
            if nb in blocked: continue
            if dx and dy and ((cur[0]+dx,cur[1]) in blocked or (cur[0],cur[1]+dy) in blocked):
                continue
            ng=g[cur]+cost
            if ng<g.get(nb,1e18):
                came[nb]=cur; g[nb]=ng
                heapq.heappush(pq,(ng+math.hypot(goal[0]-nb[0],goal[1]-nb[1]),nb))
    return [], closed

start = w2c(-1000, -1000)     # bottom-left lane
goal  = w2c(1000,  1000)      # top-right lane (must route around inner wall)
path, explored = astar(start, goal, blocked)

# ---- plot ----
fig, ax = plt.subplots(figsize=(9,9))
ax.set_aspect("equal")
ax.set_title("A* path planning on the WRO field\n"
             "occupancy grid (50 mm) · obstacle inflation · explored nodes · final path",
             fontsize=12)
ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")

for (ci,cj) in blocked:      # inflated no-go zone
    ax.add_patch(mpatches.Rectangle((ci*RES-RES/2,cj*RES-RES/2),RES,RES,
                 facecolor="#f2c4c4", edgecolor="none", zorder=1))
for (ci,cj) in explored:     # A* frontier expansion
    ax.add_patch(mpatches.Rectangle((ci*RES-RES/2,cj*RES-RES/2),RES,RES,
                 facecolor="#bcd7f0", edgecolor="none", alpha=0.55, zorder=2))
for (ci,cj) in occ:          # real walls
    ax.add_patch(mpatches.Rectangle((ci*RES-RES/2,cj*RES-RES/2),RES,RES,
                 facecolor="black", edgecolor="none", zorder=3))
if path:
    px=[c2w(c)[0] for c in path]; py=[c2w(c)[1] for c in path]
    ax.plot(px,py,color="darkorange",lw=3,zorder=5,label="A* path")
sx,sy=c2w(start); gx,gy=c2w(goal)
ax.plot(sx,sy,"o",color="red",ms=13,zorder=6,label="start")
ax.plot(gx,gy,"*",color="limegreen",ms=22,zorder=6,label="goal")
ax.legend(loc="upper left", fontsize=10)
ax.set_xlim(-1750,1750); ax.set_ylim(-1750,1750)
ax.grid(color="0.9", lw=0.4)

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.normpath(os.path.join(here, "..", "..", "docs", "astar-demo.png"))
plt.tight_layout(); plt.savefig(out, dpi=110)
print("saved", out, "| path cells:", len(path), "| explored:", len(explored))
