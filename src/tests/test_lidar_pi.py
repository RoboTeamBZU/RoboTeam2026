from rplidar import RPLidar
import time
import threading

# =============================
# LiDAR Config
# =============================

PORT     = '/dev/ttyUSB0'
BAUDRATE = 115200
MAX_DIST = 3000   # mm

# =============================
# Direction windows (robot frame)
# =============================
# The LiDAR is mounted rotated 180 degrees:
# its 0 degree mark faces the robot's REAR.
#
# Robot direction -> LiDAR angle window:
#   FRONT -> 170-190
#   LEFT  ->  80-100
#   RIGHT -> 260-280
#   BACK  -> 350-360 and 0-10 (wraps around zero)
#
# Each entry is a list of (min, max) windows so wraparound
# is handled the same way for any direction.

DIRECTION_WINDOWS = {
    "front": [(170, 190)],
    "left":  [(80, 100)],
    "right": [(260, 280)],
    "back":  [(350, 360), (0, 10)],
}

# =============================
# Shared scan data
# =============================

latest_scan = []
scan_lock   = threading.Lock()
running     = [True]

def scan_thread(lidar):
    for scan in lidar.iter_scans(max_buf_meas=500):
        if not running[0]:
            break
        with scan_lock:
            latest_scan.clear()
            latest_scan.extend(scan)

# =============================
# Distance extraction
# =============================

def get_distance(scan, windows):
    """Minimum distance among points falling in any of the angle windows."""
    distances = []
    for quality, angle, distance in scan:
        if quality > 0 and 0 < distance <= MAX_DIST:
            for amin, amax in windows:
                if amin <= angle <= amax:
                    distances.append(distance)
                    break
    return min(distances) if distances else None

def get_wall_distances(scan):
    front = get_distance(scan, DIRECTION_WINDOWS["front"])
    left  = get_distance(scan, DIRECTION_WINDOWS["left"])
    right = get_distance(scan, DIRECTION_WINDOWS["right"])
    back  = get_distance(scan, DIRECTION_WINDOWS["back"])
    return front, left, right, back

# =============================
# Main Test
# =============================

print(f"Connecting to LiDAR on {PORT}...")
lidar = RPLidar(PORT, baudrate=BAUDRATE)

# Clear any stale scan stream from a previous run that didn't shut down
# cleanly — otherwise get_info() reads scan bytes instead of a descriptor
# and raises "Incorrect descriptor starting bytes".
lidar.stop()
lidar.reset()
time.sleep(0.5)

info   = lidar.get_info()
health = lidar.get_health()
print(f"Model: {info['model']}  Firmware: {info['firmware']}  Health: {health[0]}")

print("""
========================================
LIDAR WALL DISTANCE TEST  (robot frame)
========================================
LiDAR mounted 180 deg rotated.
  Front -> 170-190
  Left  ->  80-100
  Right -> 260-280
  Back  -> 350-360 + 0-10

Press CTRL+C to stop.
========================================
""")

t = threading.Thread(target=scan_thread, args=(lidar,), daemon=True)
t.start()

time.sleep(1)

print(f"{'Front':>10} {'Left':>10} {'Right':>10} {'Back':>10} {'L-R Error':>12}")
print("-" * 60)

try:
    while True:
        with scan_lock:
            scan = list(latest_scan)

        if scan:
            front, left, right, back = get_wall_distances(scan)

            if left is not None and right is not None:
                error = left - right
                error_str = f"{error:+8.0f}mm"
            else:
                error_str = "    None  "

            def fmt(val):
                return f"{val:8.0f}mm" if val is not None else "    None  "

            print(f"{fmt(front)} {fmt(left)} {fmt(right)} {fmt(back)} {error_str}", end="\r")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nStopped.")

finally:
    running[0] = False
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Disconnected.")
