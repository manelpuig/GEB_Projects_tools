import socket, json, time
import tkinter as tk

UDP_HOST = "127.0.0.1"   # o la IP del PC que rep
UDP_PORT = 12345
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ

DEVICE = "G5_Gri"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

root = tk.Tk()
root.title("Sim Gripper (G5_Gri)")

roll = tk.DoubleVar(value=0.0)
pitch = tk.DoubleVar(value=0.0)
yaw = tk.DoubleVar(value=0.0)

# Active-low buttons:
# s1=0 -> open, s2=0 -> close
s1 = tk.IntVar(value=1)
s2 = tk.IntVar(value=1)

tk.Scale(root, from_=-90, to=90, resolution=1, orient="horizontal", label="roll (deg)", variable=roll, length=300).pack()
tk.Scale(root, from_=-90, to=90, resolution=1, orient="horizontal", label="pitch (deg)", variable=pitch, length=300).pack()
tk.Scale(root, from_=-90, to=90, resolution=1, orient="horizontal", label="yaw (deg)", variable=yaw, length=300).pack()

frm = tk.Frame(root)
frm.pack(pady=10)

tk.Checkbutton(frm, text="S1 (OPEN) active", variable=s1, onvalue=0, offvalue=1).grid(row=0, column=0, padx=10)
tk.Checkbutton(frm, text="S2 (CLOSE) active", variable=s2, onvalue=0, offvalue=1).grid(row=0, column=1, padx=10)

def loop_send():
    pkt = {
        "device": DEVICE,
        "roll": float(roll.get()),
        "pitch": float(pitch.get()),
        "yaw": float(yaw.get()),
        "s1": int(s1.get()),
        "s2": int(s2.get()),
    }
    sock.sendto(json.dumps(pkt).encode(), (UDP_HOST, UDP_PORT))
    root.after(int(DT * 1000), loop_send)

root.after(50, loop_send)
root.mainloop()
