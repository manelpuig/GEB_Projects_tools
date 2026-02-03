import socket
import json
import time
import threading
import tkinter as tk

from robodk.robolink import *
from robodk.robomath import *

# -----------------------------
# GLOBAL CONFIG
# -----------------------------
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 2048

TARGET_DEVICE = "G4_Endo"
OBJ_NAME = "Endowrist"

# Acquisition / update rate on the PC side
# Example: 10 Hz => 0.1 s (matches ~100 ms)
ACQ_HZ = 10.0
ACQ_PERIOD_S = 1.0 / ACQ_HZ

# -----------------------------
# RoboDK init
# -----------------------------
RDK = Robolink()
obj = RDK.Item(OBJ_NAME, ITEM_TYPE_OBJECT)
if not obj.Valid():
    raise Exception(f'RoboDK object not found: "{OBJ_NAME}"')

pose0 = obj.Pose()
x0, y0, z0 = pose0[0, 3], pose0[1, 3], pose0[2, 3]

# -----------------------------
# Shared state for GUI
# -----------------------------
latest = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "s3": 1, "s4": 1, "device": "", "hz": 0.0}
lock = threading.Lock()
stop_flag = False


def read_orientation(sock):
    """
    Read one UDP packet, parse JSON, filter by TARGET_DEVICE.
    Returns: (device_id, roll, pitch, yaw, s3, s4) or None
    """
    try:
        data, addr = sock.recvfrom(BUFFER_SIZE)
    except socket.timeout:
        return None

    try:
        pkt = json.loads(data.decode(errors="replace"))
    except json.JSONDecodeError:
        return None

    device_id = pkt.get("device", "")
    if device_id != TARGET_DEVICE:
        return None

    roll  = float(pkt.get("roll", 0.0))
    pitch = float(pkt.get("pitch", 0.0))
    yaw   = float(pkt.get("yaw", 0.0))
    s3    = int(pkt.get("s3", 1))
    s4    = int(pkt.get("s4", 1))

    return device_id, roll, pitch, yaw, s3, s4


def apply_orientation(obj, roll_deg, pitch_deg, yaw_deg):
    """
    Apply orientation to RoboDK object using setPose.
    Keeps translation (x0,y0,z0) fixed.
    """
    r = roll_deg  * pi / 180.0
    p = pitch_deg * pi / 180.0
    y = yaw_deg   * pi / 180.0

    # Rotation order: yaw-pitch-roll (ZYX)
    R = rotz(y) * roty(p) * rotx(r)

    pose = R
    pose[0, 3], pose[1, 3], pose[2, 3] = x0, y0, z0
    obj.setPose(pose)


def udp_thread():
    """
    Loop reading UDP and updating RoboDK + GUI state.
    Rate-limited by ACQ_PERIOD_S.
    """
    global stop_flag

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.5)

    # For rate display
    n = 0
    t_rate0 = time.time()

    # Rate limiting
    t_next = time.time()

    while not stop_flag:

        # Read (can return None)
        res = read_orientation(sock)
        if res is None:
            continue

        device_id, roll, pitch, yaw, s3, s4 = res

        # Rate-limit application/update
        now = time.time()
        if now < t_next:
            # We received data too fast; skip applying this sample
            continue
        t_next = now + ACQ_PERIOD_S

        # Apply to RoboDK
        apply_orientation(obj, roll, pitch, yaw)

        # Update GUI state
        n += 1
        dt = now - t_rate0
        hz = (n / dt) if dt > 0 else 0.0
        if dt >= 1.0:
            n = 0
            t_rate0 = now

        with lock:
            latest["device"] = device_id
            latest["roll"] = roll
            latest["pitch"] = pitch
            latest["yaw"] = yaw
            latest["s3"] = s3
            latest["s4"] = s4
            latest["hz"] = hz

    sock.close()


# -----------------------------
# GUI
# -----------------------------
def gui_update():
    with lock:
        dev = latest["device"]
        r = latest["roll"]
        p = latest["pitch"]
        y = latest["yaw"]
        s3 = latest["s3"]
        s4 = latest["s4"]
        hz = latest["hz"]

    lbl1.config(text=f"Device filter: {TARGET_DEVICE} | rx: {dev}")
    lbl2.config(text=f"RPY (deg): {r:7.2f}  {p:7.2f}  {y:7.2f}")
    lbl3.config(text=f"s3={s3}   s4={s4}")
    lbl4.config(text=f"Update rate: {hz:5.1f} Hz | ACQ_HZ={ACQ_HZ:.1f}")

    root.after(50, gui_update)


def on_close():
    global stop_flag
    stop_flag = True
    root.destroy()


# Start UDP thread
th = threading.Thread(target=udp_thread, daemon=True)
th.start()

root = tk.Tk()
root.title("Endowrist UDP Monitor (RoboDK)")
root.geometry("520x170")

lbl1 = tk.Label(root, text="Device:", font=("Consolas", 11))
lbl1.pack(anchor="w", padx=10, pady=(10, 0))

lbl2 = tk.Label(root, text="RPY:", font=("Consolas", 14))
lbl2.pack(anchor="w", padx=10)

lbl3 = tk.Label(root, text="Buttons:", font=("Consolas", 12))
lbl3.pack(anchor="w", padx=10)

lbl4 = tk.Label(root, text="Rate:", font=("Consolas", 10))
lbl4.pack(anchor="w", padx=10, pady=(6, 0))

root.protocol("WM_DELETE_WINDOW", on_close)
gui_update()
root.mainloop()
