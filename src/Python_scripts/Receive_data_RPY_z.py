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

OBJ1_NAME = "surgical_needle"   # RoboDK Object
TCP_NAME  = "TCP"               # RoboDK Frame

# Apply/update frequency (how often we apply to RoboDK)
ACQ_HZ = 10.0
ACQ_PERIOD_S = 1.0 / ACQ_HZ

# Translation speed along LOCAL +Z/-Z when s3/s4 are active (mm/s)
Z_SPEED_MM_S = 5.0

# -----------------------------
# RoboDK init
# -----------------------------
RDK = Robolink()

obj = RDK.Item(OBJ1_NAME, ITEM_TYPE_OBJECT)
tcp = RDK.Item(TCP_NAME,  ITEM_TYPE_FRAME)

if not obj.Valid():
    raise Exception(f'RoboDK object not found: "{OBJ1_NAME}"')
if not tcp.Valid():
    raise Exception(f'RoboDK frame not found: "{TCP_NAME}"')

# Save initial poses (absolute). This is what we will restore on exit.
pose0_obj = obj.PoseAbs()
pose0_tcp = tcp.PoseAbs()

# Optional: start TCP at same pose as object (only at startup)
tcp.setPoseAbs(pose0_obj)

# -----------------------------
# Shared state for GUI
# -----------------------------
latest = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "s3": 0, "s4": 0,
    "device": "", "hz": 0.0,
    "status": "waiting"
}
lock = threading.Lock()
stop_flag = False


def read_orientation(sock):
    """Read one UDP packet (JSON), filter by TARGET_DEVICE."""
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
        "roll":  float(pkt.get("roll", 0.0)),
        "pitch": float(pkt.get("pitch", 0.0)),
        "yaw":   float(pkt.get("yaw", 0.0)),
        "s3":    int(pkt.get("s3", 0)),
        "s4":    int(pkt.get("s4", 0)),
    }


def apply_pose_from_rpy_and_buttons(item_obj, item_tcp, roll_deg, pitch_deg, yaw_deg, s3, s4):
    """
    1) Set orientation from RPY (deg) on the OBJECT.
    2) Apply a translation along local Z based on s3/s4 (mm/s).
    3) Keep TCP pose identical to the OBJECT pose.
    """
    # Convert degrees to radians for RoboDK rotations
    r = roll_deg  * pi / 180.0
    p = pitch_deg * pi / 180.0
    y = yaw_deg   * pi / 180.0

    # Rotation order: yaw-pitch-roll (ZYX)
    R = rotz(y) * roty(p) * rotx(r)

    # Keep current absolute position, replace orientation
    pose_old = item_obj.PoseAbs()
    x, yy, z = pose_old[0, 3], pose_old[1, 3], pose_old[2, 3]
    pose_new = R * transl(x, yy, z)

    # Translate along LOCAL Z with a speed (mm/s)
    dz = 0.0
    if s3 == 1:
        dz += Z_SPEED_MM_S * ACQ_PERIOD_S
    if s4 == 1:
        dz -= Z_SPEED_MM_S * ACQ_PERIOD_S

    # Post-multiply => local axes translation
    pose_new = pose_new * transl(0, 0, dz)

    # Apply to object and keep TCP identical
    item_obj.setPoseAbs(pose_new)
    item_tcp.setPoseAbs(pose_new)


def udp_thread():
    """Read UDP and apply to RoboDK at ~ACQ_HZ."""
    global stop_flag

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(0.05)

        last_pkt = None

        # Rate measurement (applies per second, for GUI only)
        n_apply = 0
        t_rate0 = time.time()

        while not stop_flag:
            # Poll for the latest packet for part of the cycle
            t_poll_end = time.time() + 0.5 * ACQ_PERIOD_S
            while time.time() < t_poll_end and not stop_flag:
                pkt = read_orientation(sock)
                if pkt is not None:
                    last_pkt = pkt

            # Apply once per cycle
            if last_pkt is not None and not stop_flag:
                apply_pose_from_rpy_and_buttons(
                    obj, tcp,
                    last_pkt["roll"], last_pkt["pitch"], last_pkt["yaw"],
                    last_pkt["s3"], last_pkt["s4"]
                )

                # Update apply-rate estimate
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
    lbl3.config(text=f"s3={s3} (+Z)   s4={s4} (-Z)   | Z_SPEED={Z_SPEED_MM_S:.1f} mm/s")
    lbl4.config(text=f"Apply rate: {hz:5.1f} Hz (target {ACQ_HZ:.1f} Hz) | {status}")

    root.after(50, gui_update)


def on_close():
    """Stop thread, restore initial poses, then close GUI."""
    global stop_flag
    stop_flag = True

    # Wait a bit so thread stops applying
    try:
        th.join(timeout=1.0)
    except Exception:
        pass

    # Restore initial poses
    try:
        obj.setPoseAbs(pose0_obj)
        tcp.setPoseAbs(pose0_tcp)
    except Exception as e:
        print(f"Warning: could not restore initial poses: {e}")

    root.destroy()


th = threading.Thread(target=udp_thread, daemon=True)
th.start()

root = tk.Tk()
root.title("Receive_data_needle (RoboDK) - Simple")
root.geometry("680x170")

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
