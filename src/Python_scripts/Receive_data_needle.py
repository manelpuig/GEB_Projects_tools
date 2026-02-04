import socket
import json
import time
import threading
import tkinter as tk

from robodk.robolink import *
from robodk.robomath import *

# ============================================================
# CONFIG
# ============================================================
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 2048

TARGET_DEVICE = "G4_Endo"          # Filter only this device
ACQ_HZ = 20.0                      # How often we APPLY pose to RoboDK (Hz)
ACQ_PERIOD_S = 1.0 / ACQ_HZ

# RoboDK items (must exist in your RoboDK station)
OBJECT_NAME = "surgical_needle"             # <-- rename to your real object name
TCP_NAME = "TCP"                   # <-- rename to your real TCP frame name

# Motion along local Z when buttons are pressed
#DZ_STEP_MM = 1.0                   # step per tick (mm). Increase if too small.
DZ_STEP_MM = 1.0 / ACQ_HZ   # => 1 mm/s when button is held

# ============================================================
# RoboDK init
# ============================================================
RDK = Robolink()

obj = RDK.Item(OBJECT_NAME, ITEM_TYPE_OBJECT)
tcp = RDK.Item(TCP_NAME, ITEM_TYPE_FRAME)

if not obj.Valid():
    raise Exception(f'RoboDK object not found: "{OBJECT_NAME}"')
if not tcp.Valid():
    raise Exception(f'RoboDK frame not found: "{TCP_NAME}"')

# Current target to apply orientation/motion
# Press 'c' => Object, press 'o' => TCP
active_item = obj
active_name = "OBJECT"

# ============================================================
# Shared state for GUI
# ============================================================
latest = {
    "device": "",
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "s3": 1,
    "s4": 1,
    "hz": 0.0,
    "status": "waiting",
    "active": active_name
}
lock = threading.Lock()
stop_flag = False


# ============================================================
# UDP + APPLY helpers
# ============================================================
def read_orientation(sock):
    """Read one UDP packet, parse JSON, filter by TARGET_DEVICE. Return dict or None."""
    try:
        data, _addr = sock.recvfrom(BUFFER_SIZE)
    except socket.timeout:
        return None

    try:
        pkt = json.loads(data.decode(errors="replace"))
    except json.JSONDecodeError:
        return None

    device_id = pkt.get("device", "")
    if device_id != TARGET_DEVICE:
        return None

    return {
        "device": device_id,
        "roll": float(pkt.get("roll", 0.0)),
        "pitch": float(pkt.get("pitch", 0.0)),
        "yaw": float(pkt.get("yaw", 0.0)),
        "s3": int(pkt.get("s3", 1)),
        "s4": int(pkt.get("s4", 1)),
        "t": time.time(),
    }


def apply_orientation_to_item(item, roll_deg, pitch_deg, yaw_deg):
    """Apply RPY (deg) to item PoseAbs, keeping its current translation."""
    # RoboDK rotations use radians
    r = roll_deg * pi / 180.0
    p = pitch_deg * pi / 180.0
    y = yaw_deg * pi / 180.0

    # Rotation order (ZYX): yaw-pitch-roll
    R = rotz(y) * roty(p) * rotx(r)

    # Keep current position, update only orientation
    pose_old = item.PoseAbs()
    x, yy, z = pose_old[0, 3], pose_old[1, 3], pose_old[2, 3]
    pose_new = R * transl(x, yy, z)

    item.setPoseAbs(pose_new)


def move_item_local_z(item, dz_mm):
    """Move item along its LOCAL +Z by dz_mm (use negative dz for -Z)."""
    pose = item.PoseAbs()
    pose = pose * transl(0, 0, dz_mm)  # post-multiply => local axis
    item.setPoseAbs(pose)


# ============================================================
# Thread: read UDP continuously, apply at ACQ_HZ
# ============================================================
def udp_thread():
    global stop_flag, active_item, active_name

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.05)  # short timeout => responsive polling

    last_pkt = None

    # Apply-rate measurement (just for display)
    n_apply = 0
    t_rate0 = time.time()

    while not stop_flag:
        # 1) Poll UDP for up to half the apply period to capture the latest packet
        t_poll_end = time.time() + 0.5 * ACQ_PERIOD_S
        while time.time() < t_poll_end:
            pkt = read_orientation(sock)
            if pkt is not None:
                last_pkt = pkt

        # 2) Apply once per cycle (this is what makes the apply rate ~ACQ_HZ)
        if last_pkt is not None:
            roll = last_pkt["roll"]
            pitch = last_pkt["pitch"]
            yaw = last_pkt["yaw"]
            s3 = last_pkt["s3"]
            s4 = last_pkt["s4"]

            # Apply orientation to the currently selected target
            apply_orientation_to_item(active_item, roll, pitch, yaw)

            # Buttons:
            # s3=1 => move +Z, s4=1 => move -Z (on the active target)
            if s3 == 1:
                move_item_local_z(active_item, +DZ_STEP_MM)
            if s4 == 1:
                move_item_local_z(active_item, -DZ_STEP_MM)

            # Compute "applies per second" (for GUI)
            n_apply += 1
            dt = time.time() - t_rate0
            hz = (n_apply / dt) if dt > 0 else 0.0
            if dt >= 1.0:
                n_apply = 0
                t_rate0 = time.time()

            with lock:
                latest["device"] = last_pkt["device"]
                latest["roll"] = roll
                latest["pitch"] = pitch
                latest["yaw"] = yaw
                latest["s3"] = s3
                latest["s4"] = s4
                latest["hz"] = hz
                latest["active"] = active_name
                latest["status"] = "OK"
        else:
            with lock:
                latest["status"] = "waiting (no packets)"

        # 3) Wait until next apply tick (keeps apply frequency controlled)
        time.sleep(ACQ_PERIOD_S)

    sock.close()


# ============================================================
# GUI + Key bindings
# ============================================================
def set_active_object():
    """Key 'c': apply orientation/motion to OBJECT."""
    global active_item, active_name
    active_item = obj
    active_name = "OBJECT"
    with lock:
        latest["active"] = active_name


def set_active_tcp():
    """Key 'o': apply orientation/motion to TCP."""
    global active_item, active_name
    active_item = tcp
    active_name = "TCP"
    with lock:
        latest["active"] = active_name


def gui_update():
    """Refresh GUI labels (does NOT apply motion)."""
    with lock:
        dev = latest["device"]
        r = latest["roll"]
        p = latest["pitch"]
        y = latest["yaw"]
        s3 = latest["s3"]
        s4 = latest["s4"]
        hz = latest["hz"]
        status = latest["status"]
        active = latest["active"]

    lbl1.config(text=f"Device filter: {TARGET_DEVICE} | last rx: {dev}")
    lbl2.config(text=f"Active target: {active}   (press 'c' => Object, 'o' => TCP)")
    lbl3.config(text=f"RPY (deg): {r:7.2f}  {p:7.2f}  {y:7.2f}")
    lbl4.config(text=f"s3={s3}  (+Z)    s4={s4}  (-Z)")
    lbl5.config(text=f"Apply rate: {hz:5.1f} Hz (target {ACQ_HZ:.1f} Hz) | {status}")

    root.after(50, gui_update)  # GUI refresh rate (~20 Hz)


def on_close():
    global stop_flag
    stop_flag = True
    root.destroy()


# Start UDP thread
th = threading.Thread(target=udp_thread, daemon=True)
th.start()

# GUI window
root = tk.Tk()
root.title("Receive_data_RPY (RoboDK)")
root.geometry("700x210")

# Key bindings
root.bind("c", lambda e: set_active_object())
root.bind("o", lambda e: set_active_tcp())

lbl1 = tk.Label(root, text="Device:", font=("Consolas", 11))
lbl1.pack(anchor="w", padx=10, pady=(10, 0))

lbl2 = tk.Label(root, text="Active:", font=("Consolas", 11))
lbl2.pack(anchor="w", padx=10)

lbl3 = tk.Label(root, text="RPY:", font=("Consolas", 14))
lbl3.pack(anchor="w", padx=10)

lbl4 = tk.Label(root, text="Buttons:", font=("Consolas", 12))
lbl4.pack(anchor="w", padx=10)

lbl5 = tk.Label(root, text="Rate:", font=("Consolas", 10))
lbl5.pack(anchor="w", padx=10, pady=(6, 0))

root.protocol("WM_DELETE_WINDOW", on_close)
gui_update()
root.mainloop()
