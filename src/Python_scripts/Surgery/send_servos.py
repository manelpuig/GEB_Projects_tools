import socket
import json
import tkinter as tk

# -----------------------------
# CONFIG
# -----------------------------
UDP_HOST = "127.0.0.1"   # IP del PC receptor (canvia si cal)
UDP_PORT = 12345
RATE_HZ = 50.0
DT_MS = int(1000.0 / RATE_HZ)

DEVICE = "G5_Servos"

# -----------------------------
# UDP socket
# -----------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# -----------------------------
# GUI
# -----------------------------
root = tk.Tk()
root.title("Sim Servos (G5_Servos)")

t_roll1 = tk.DoubleVar(value=0.0)
t_roll2 = tk.DoubleVar(value=0.0)
t_pitch = tk.DoubleVar(value=0.0)
t_yaw   = tk.DoubleVar(value=0.0)

SLIDER_LEN = 320
SLIDER_FROM = -5.0   # Nm (ajusta segons el teu model)
SLIDER_TO   = 5.0

tk.Scale(
    root, from_=SLIDER_FROM, to=SLIDER_TO,
    resolution=0.1, orient="horizontal",
    label="Torque Roll 1 (Nm)", variable=t_roll1,
    length=SLIDER_LEN
).pack(pady=4)

tk.Scale(
    root, from_=SLIDER_FROM, to=SLIDER_TO,
    resolution=0.1, orient="horizontal",
    label="Torque Roll 2 (Nm)", variable=t_roll2,
    length=SLIDER_LEN
).pack(pady=4)

tk.Scale(
    root, from_=SLIDER_FROM, to=SLIDER_TO,
    resolution=0.1, orient="horizontal",
    label="Torque Pitch (Nm)", variable=t_pitch,
    length=SLIDER_LEN
).pack(pady=4)

tk.Scale(
    root, from_=SLIDER_FROM, to=SLIDER_TO,
    resolution=0.1, orient="horizontal",
    label="Torque Yaw (Nm)", variable=t_yaw,
    length=SLIDER_LEN
).pack(pady=4)

# -----------------------------
# UDP send loop
# -----------------------------
def loop_send():
    pkt = {
        "device": DEVICE,
        "t_roll1": float(t_roll1.get()),
        "t_roll2": float(t_roll2.get()),
        "t_pitch": float(t_pitch.get()),
        "t_yaw":   float(t_yaw.get()),
    }

    sock.sendto(json.dumps(pkt).encode(), (UDP_HOST, UDP_PORT))
    root.after(DT_MS, loop_send)

root.after(50, loop_send)
root.mainloop()
