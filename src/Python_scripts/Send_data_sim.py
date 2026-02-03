import socket
import json
import time
import math

# Destination (PC running the receiver)
DEST_IP = "127.0.0.1"      # si el receiver és al mateix PC
# DEST_IP = "192.168.1.55" # si vols enviar al PC de l'aula
DEST_PORT = 12345

DEVICE_ID = "G4_Endo"      # ha de coincidir amb TARGET_DEVICE al receiver
SEND_HZ = 50.0             # freqüència d'enviament
DT = 1.0 / SEND_HZ

# Buttons simulation
s3 = 1
s4 = 1
toggle_period_s = 2.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

t0 = time.time()
t_last_toggle = t0

print(f"Sending UDP JSON to {DEST_IP}:{DEST_PORT} as device={DEVICE_ID} at {SEND_HZ:.1f} Hz")

try:
    while True:
        t = time.time() - t0

        # Simple motion pattern (degrees)
        roll = 30.0 * math.sin(2.0 * math.pi * 0.2 * t)      # 0.2 Hz
        pitch = 0.0 #* math.sin(2.0 * math.pi * 0.1 * t + 1) # 0.1 Hz
        yaw = 0.0 #* math.sin(2.0 * math.pi * 0.05 * t + 2)  # 0.05 Hz

        # Toggle buttons every toggle_period_s
        now = time.time()
        if now - t_last_toggle >= toggle_period_s:
            s3 = 0 if s3 == 1 else 1
            s4 = 0 if s4 == 1 else 1
            t_last_toggle = now

        pkt = {
            "device": DEVICE_ID,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "s3": s3,
            "s4": s4
        }

        data = json.dumps(pkt).encode("utf-8")
        sock.sendto(data, (DEST_IP, DEST_PORT))

        time.sleep(DT)

except KeyboardInterrupt:
    print("Stopped sender.")
finally:
    sock.close()
