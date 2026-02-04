import socket
import json
import time
import tkinter as tk

# ------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------
DEST_IP = "127.0.0.1"      # receiver IP (same PC => 127.0.0.1)
DEST_PORT = 12345
DEVICE_ID = "G4_Endo"      # must match TARGET_DEVICE in receiver

SEND_HZ = 30.0             # send rate
DT_MS = int(1000.0 / SEND_HZ)

# Slider ranges (degrees)
ROLL_MIN, ROLL_MAX = -90, 90
PITCH_MIN, PITCH_MAX = -90, 90
YAW_MIN, YAW_MAX = -90, 90

# ------------------------------------------------------------
# UDP socket
# ------------------------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ------------------------------------------------------------
# Tkinter GUI
# ------------------------------------------------------------
root = tk.Tk()
root.title("UDP Sender: RPY Sliders + s3/s4 Buttons")
root.geometry("720x360")

# State variables
roll_var = tk.DoubleVar(value=0.0)
pitch_var = tk.DoubleVar(value=0.0)
yaw_var = tk.DoubleVar(value=0.0)

s3_var = tk.IntVar(value=0)
s4_var = tk.IntVar(value=0)

# Labels
lbl_title = tk.Label(root, text=f"Sending to {DEST_IP}:{DEST_PORT} | device={DEVICE_ID} | {SEND_HZ:.1f} Hz",
                     font=("Consolas", 11))
lbl_title.pack(anchor="w", padx=10, pady=(10, 0))

# --- Roll slider ---
frm_roll = tk.Frame(root)
frm_roll.pack(fill="x", padx=10, pady=(10, 0))
tk.Label(frm_roll, text="Roll (deg)", width=12, font=("Consolas", 11)).pack(side="left")
sld_roll = tk.Scale(frm_roll, from_=ROLL_MIN, to=ROLL_MAX, orient="horizontal",
                    resolution=0.1, variable=roll_var, length=520)
sld_roll.pack(side="left", padx=8)
lbl_roll = tk.Label(frm_roll, text="0.0", width=8, font=("Consolas", 11))
lbl_roll.pack(side="left")

# --- Pitch slider ---
frm_pitch = tk.Frame(root)
frm_pitch.pack(fill="x", padx=10, pady=(8, 0))
tk.Label(frm_pitch, text="Pitch (deg)", width=12, font=("Consolas", 11)).pack(side="left")
sld_pitch = tk.Scale(frm_pitch, from_=PITCH_MIN, to=PITCH_MAX, orient="horizontal",
                     resolution=0.1, variable=pitch_var, length=520)
sld_pitch.pack(side="left", padx=8)
lbl_pitch = tk.Label(frm_pitch, text="0.0", width=8, font=("Consolas", 11))
lbl_pitch.pack(side="left")

# --- Yaw slider ---
frm_yaw = tk.Frame(root)
frm_yaw.pack(fill="x", padx=10, pady=(8, 0))
tk.Label(frm_yaw, text="Yaw (deg)", width=12, font=("Consolas", 11)).pack(side="left")
sld_yaw = tk.Scale(frm_yaw, from_=YAW_MIN, to=YAW_MAX, orient="horizontal",
                   resolution=0.1, variable=yaw_var, length=520)
sld_yaw.pack(side="left", padx=8)
lbl_yaw = tk.Label(frm_yaw, text="0.0", width=8, font=("Consolas", 11))
lbl_yaw.pack(side="left")

# --- Buttons s3/s4 ---
frm_btn = tk.Frame(root)
frm_btn.pack(fill="x", padx=10, pady=(14, 0))

def toggle_s3():
    s3_var.set(0 if s3_var.get() == 1 else 1)

def toggle_s4():
    s4_var.set(0 if s4_var.get() == 1 else 1)

btn_s3 = tk.Button(frm_btn, text="s3 = 0", width=12, command=toggle_s3, font=("Consolas", 11))
btn_s3.pack(side="left", padx=(0, 10))

btn_s4 = tk.Button(frm_btn, text="s4 = 0", width=12, command=toggle_s4, font=("Consolas", 11))
btn_s4.pack(side="left")

# Status
lbl_status = tk.Label(root, text="", font=("Consolas", 11))
lbl_status.pack(anchor="w", padx=10, pady=(14, 0))

# Quick controls row
frm_quick = tk.Frame(root)
frm_quick.pack(fill="x", padx=10, pady=(10, 0))

def zero_all():
    roll_var.set(0.0)
    pitch_var.set(0.0)
    yaw_var.set(0.0)

def reset_buttons():
    s3_var.set(0)
    s4_var.set(0)

tk.Button(frm_quick, text="Zero RPY", command=zero_all, width=12, font=("Consolas", 11)).pack(side="left")
tk.Button(frm_quick, text="Reset s3/s4", command=reset_buttons, width=12, font=("Consolas", 11)).pack(side="left", padx=(10, 0))

# ------------------------------------------------------------
# Send loop (Tkinter after)
# ------------------------------------------------------------
def send_loop():
    # Read current GUI values
    roll = float(roll_var.get())
    pitch = float(pitch_var.get())
    yaw = float(yaw_var.get())
    s3 = int(s3_var.get())
    s4 = int(s4_var.get())

    # Update numeric labels
    lbl_roll.config(text=f"{roll:6.1f}")
    lbl_pitch.config(text=f"{pitch:6.1f}")
    lbl_yaw.config(text=f"{yaw:6.1f}")

    # Update button text
    btn_s3.config(text=f"s3 = {s3}")
    btn_s4.config(text=f"s4 = {s4}")

    # Build UDP packet
    pkt = {
        "device": DEVICE_ID,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "s3": s3,
        "s4": s4
    }

    # Send
    try:
        data = json.dumps(pkt).encode("utf-8")
        sock.sendto(data, (DEST_IP, DEST_PORT))
        lbl_status.config(text=f"TX: RPY=({roll:.1f}, {pitch:.1f}, {yaw:.1f}) | s3={s3} s4={s4}")
    except Exception as e:
        lbl_status.config(text=f"ERROR sending UDP: {e}")

    # Schedule next send
    root.after(DT_MS, send_loop)

def on_close():
    try:
        sock.close()
    except Exception:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# Start sending
send_loop()
root.mainloop()
