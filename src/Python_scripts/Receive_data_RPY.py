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
OBJ_NAME = "surgical_needle"

# PC-side update frequency (apply rate)
ACQ_HZ = 10.0
ACQ_PERIOD_S = 1.0 / ACQ_HZ

# -----------------------------
# RoboDK init
# -----------------------------
RDK = Robolink()
obj = RDK.Item(OBJ_NAME, ITEM_TYPE_OBJECT)

# If you want to test without RoboDK, set DRY_RUN=True
DRY_RUN = False

if not DRY_RUN and not obj.Valid():
    raise Exception(f'RoboDK object not found: "{OBJ_NAME}"')

# Save the initial pose (full 4x4 pose)
if not DRY_RUN:
    pose0 = obj.Pose()
    x0, y0, z0 = pose0[0, 3], pose0[1, 3], pose0[2, 3]
else:
    pose0 = None
    x0, y0, z0 = 0.0, 0.0, 0.0

# -----------------------------
# Shared state for GUI
# -----------------------------
latest = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "s3": 1, "s4": 1, "device": "", "hz": 0.0, "status": "waiting"}
lock = threading.Lock()
stop_flag = False


def read_orientation(sock):
    """Reads one UDP packet, returns dict or None. Filters by TARGET_DEVICE."""
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


def apply_orientation(obj, roll_deg, pitch_deg, yaw_deg):
    """Applies RPY to RoboDK object pose, keeping x0,y0,z0 fixed."""
    r = roll_deg  * pi / 180.0
    p = pitch_deg * pi / 180.0
    y = yaw_deg   * pi / 180.0

    # Rotation order: yaw-pitch-roll (ZYX)
    R = rotz(y) * roty(p) * rotx(r)

    pose = R
    pose[0, 3], pose[1, 3], pose[2, 3] = x0, y0, z0

    if not DRY_RUN:
        obj.setPose(pose)


def udp_thread():
    """
    Simple logic:
      - keep the most recent packet (last_pkt)
      - every ACQ_PERIOD_S: apply last_pkt (if exists)
    """
    global stop_flag

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(0.05)  # short timeout so we can poll often

        last_pkt = None

        # for displayed rate = applies per second
        n_apply = 0
        t_rate0 = time.time()

        while not stop_flag:
            # 1) poll UDP a few times quickly to get the latest packet
            t_poll_end = time.time() + 0.5 * ACQ_PERIOD_S
            while time.time() < t_poll_end and not stop_flag:
                pkt = read_orientation(sock)
                if pkt is not None:
                    last_pkt = pkt

            # 2) apply the latest packet once per cycle
            if last_pkt is not None and not stop_flag:
                apply_orientation(obj, last_pkt["roll"], last_pkt["pitch"], last_pkt["yaw"])

                n_apply += 1
                dt = time.time() - t_rate0
                hz = (n_apply / dt) if dt > 0 else 0.0
                if dt >= 1.0:
                    n_apply = 0
                    t_rate0 = time.time()

                with lock:
                    latest["device"] = last_pkt["device"]
                    latest["roll"] = last_pkt["roll"]
                    latest["pitch"] = last_pkt["pitch"]
                    latest["yaw"] = last_pkt["yaw"]
                    latest["s3"] = last_pkt["s3"]
                    latest["s4"] = last_pkt["s4"]
                    latest["hz"] = hz
                    latest["status"] = "OK"
            else:
                with lock:
                    latest["status"] = "waiting (no packets)"

            # 3) wait until next acquisition tick
            time.sleep(ACQ_PERIOD_S)

    finally:
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
        status = latest["status"]

    lbl1.config(text=f"Device filter: {TARGET_DEVICE} | last rx: {dev}")
    lbl2.config(text=f"RPY (deg): {r:7.2f}  {p:7.2f}  {y:7.2f}")
    lbl3.config(text=f"s3={s3}   s4={s4}")
    lbl4.config(text=f"Apply rate: {hz:5.1f} Hz (target {ACQ_HZ:.1f} Hz) | {status}")

    root.after(50, gui_update)


def on_close():
    """
    When closing the Tkinter window:
      1) stop the UDP thread
      2) restore initial pose
      3) close GUI
    """
    global stop_flag
    stop_flag = True

    # Wait a bit so the thread stops applying poses
    try:
        th.join(timeout=1.0)
    except Exception:
        pass

    # Restore initial pose
    if not DRY_RUN and pose0 is not None:
        try:
            obj.setPose(pose0)
        except Exception as e:
            print(f"Warning: could not restore initial pose: {e}")

    root.destroy()


th = threading.Thread(target=udp_thread, daemon=True)
th.start()

root = tk.Tk()
root.title("Endowrist UDP Monitor (RoboDK) - Simple")
root.geometry("560x170")

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
