from robodk.robolink import *
from robodk.robomath import *

import time
import math
import tkinter as tk
import threading
import socket
import json
import os

# ============================================================
# CONFIG
# ============================================================

# RoboDK project
RELATIVE_RDK_PATH = "src/roboDK/SurgeryRobotics.rdk"
ABSOLUTE_RDK_PATH = os.path.abspath(RELATIVE_RDK_PATH)

# UDP
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 2048

# RoboDK item names
ROBOT_NAME = "UR5e"
BASE_NAME = f"{ROBOT_NAME} Base"
INIT_TARGET_NAME = "Init"
ENDOWRIST_NAME = "Endowrist"
GRIPPER_NAME = "Gripper"
NEEDLE_NAME = "Needle"

# Device IDs
DEV_ENDO = "G5_Endo"     # IMU on tool/endowrist
DEV_GRIP = "G5_Gri"      # IMU on gripper
DEV_SERVO = "G5_Servos"  # torques

# Loop rates
STATE_HZ = 50.0         # how often we read latest UDP snapshot and compute targets
ROBOT_CMD_HZ = 5.0      # how often we actually send MoveL commands to the robot
GUI_HZ = 20.0

STATE_DT = 1.0 / STATE_HZ
ROBOT_CMD_DT = 1.0 / ROBOT_CMD_HZ
GUI_DT_MS = int(1000.0 / GUI_HZ)

# Local Z step (mm) per cycle when buttons are active
Z_STEP_MM = 5.0

# Optional safety clamp for ABS angles (deg)
ANGLE_LIMIT_DEG = 90.0

# ============================================================
# SHARED STATE (updated by UDP thread, read by control + GUI)
# ============================================================
shared = {
    "endo": None,    # dict: roll,pitch,yaw,s3,s4
    "grip": None,    # dict: roll,pitch,yaw,s1,s2
    "servo": None,   # dict: t_roll1,t_roll2,t_pitch,t_yaw
    "status": "waiting (no packets)",
    "rx_counts": {DEV_ENDO: 0, DEV_GRIP: 0, DEV_SERVO: 0},
}
lock = threading.Lock()
stop_flag = False


# ============================================================
# UTILS (math + parsing)
# ============================================================
def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def wrap_deg_signed(a):
    """
    Convert degrees to [-180, 180) range (robust for inputs like 0..360).
    """
    a = float(a) % 360.0
    if a >= 180.0:
        a -= 360.0
    return a


def rot_of_pose(pose4):
    """
    Extract a pure rotation 4x4 (homogeneous) matrix from a RoboDK pose.
    """
    return Mat([
        [pose4[0, 0], pose4[0, 1], pose4[0, 2], 0.0],
        [pose4[1, 0], pose4[1, 1], pose4[1, 2], 0.0],
        [pose4[2, 0], pose4[2, 1], pose4[2, 2], 0.0],
        [0.0,         0.0,         0.0,         1.0],
    ])


def pose_with_same_translation(pose_old, R_new):
    """
    Build a new pose with the same XYZ translation as pose_old, but with rotation R_new.
    """
    x, y, z = pose_old[0, 3], pose_old[1, 3], pose_old[2, 3]
    return transl(x, y, z) * R_new


def R_world_from_rpy_deg_extrinsic(roll_deg, pitch_deg, yaw_deg):
    """
    IMU-style Euler (extrinsic/world) convention:
      R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    return rotz(y) * roty(p) * rotx(r)


# ============================================================
# RoboDK INIT
# ============================================================
def initialize_robodk(project_path):
    """
    Load project, bind items, set parents, move robot to Init.
    Returns: (RDK, robot, base, endowrist_item, gripper_item, needle_item)
    """
    RDK = Robolink()
    ##time.sleep(1.5)  # give RoboDK time to fully connect

    ##RDK.AddFile(project_path)

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    base = RDK.Item(BASE_NAME, ITEM_TYPE_FRAME)
    endowrist = RDK.Item(ENDOWRIST_NAME)  # could be a tool/frame/object; we just need Item()
    gripper = RDK.Item(GRIPPER_NAME)
    needle = RDK.Item(NEEDLE_NAME)
    init_target = RDK.Item(INIT_TARGET_NAME, ITEM_TYPE_TARGET)

    if not robot.Valid():
        raise Exception(f'Robot not found: "{ROBOT_NAME}"')
    if not base.Valid():
        raise Exception(f'Base frame not found: "{BASE_NAME}"')
    if not endowrist.Valid():
        raise Exception(f'Item not found: "{ENDOWRIST_NAME}"')
    if not gripper.Valid():
        raise Exception(f'Item not found: "{GRIPPER_NAME}"')
    if not needle.Valid():
        raise Exception(f'Item not found: "{NEEDLE_NAME}"')
    if not init_target.Valid():
        raise Exception(f'Target not found: "{INIT_TARGET_NAME}"')

    # Attach robot base/tool
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)

    # Set initial gripper relative pose (relative to endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)

    # Needle is initially attached to gripper
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)

    # Move robot to Init
    robot.MoveL(init_target)
    robot.setSpeed(50)

    return RDK, robot, base, endowrist, gripper, needle


# ============================================================
# UDP THREAD
# ============================================================
def read_orientation(sock):
    """
    Receive UDP packets continuously and update shared state (thread-safe).
    """
    global stop_flag
    sock.settimeout(0.2)  # allow periodic stop_flag checks

    while not stop_flag:
        try:
            data, _addr = sock.recvfrom(BUFFER_SIZE)
        except socket.timeout:
            continue
        except OSError:
            # socket closed
            break

        try:
            msg = json.loads(data.decode(errors="replace"))
        except json.JSONDecodeError:
            continue

        dev = msg.get("device", "")
        if dev not in (DEV_ENDO, DEV_GRIP, DEV_SERVO):
            continue

        # Normalize angles to signed range and clamp (ABS angles)
        if dev in (DEV_ENDO, DEV_GRIP):
            msg["roll"]  = clamp(wrap_deg_signed(msg.get("roll", 0.0)),  -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)
            msg["pitch"] = clamp(wrap_deg_signed(msg.get("pitch", 0.0)), -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)
            msg["yaw"]   = clamp(wrap_deg_signed(msg.get("yaw", 0.0)),   -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)

        with lock:
            if dev == DEV_ENDO:
                shared["endo"] = msg
            elif dev == DEV_GRIP:
                shared["grip"] = msg
            elif dev == DEV_SERVO:
                shared["servo"] = msg

            shared["rx_counts"][dev] += 1
            shared["status"] = "OK"

# ============================================================
# CONTROL LOOP THREAD (robot + gripper + needle)
# ============================================================
def control_thread(robot, base, gripper, needle,
                   pose0_robot, pose0_gripper, pose0_needle,
                   gripper_init_xyz=(0, 5, -105),
                   get_zero_yaw_tool=lambda: 0.0,
                   get_zero_yaw_gripper=lambda: 0.0):
    """
    Apply IMU absolute orientations:
      - Endowrist IMU controls robot tool orientation in WORLD (extrinsic ZYX)
      - Gripper IMU controls gripper orientation in WORLD (extrinsic ZYX)
    Because gripper is parented to endowrist, we must convert:
      R_grip_rel = inv(R_endo_world) * R_grip_world

    Also handles:
      - s3/s4: local Z translation of the robot TCP (relative motion)
      - s1/s2: open/close (detach/attach needle)
    """
    global stop_flag

    # Reference: initial gripper translation relative to endowrist
    gx0, gy0, gz0 = gripper_init_xyz

    next_robot_cmd = time.time()  # next time we are allowed to send MoveL commands

    while not stop_flag:
        # Snapshot shared state (thread-safe)
        with lock:
            endo = shared["endo"]
            grip = shared["grip"]
            servo = shared["servo"]
            status = shared["status"]

        # -------------------------
        # 1) Apply Endowrist IMU to robot tool pose (absolute orientation in WORLD)
        # -------------------------
        if endo is not None:
            e_roll = endo.get("roll", 0.0)
            e_pitch = endo.get("pitch", 0.0)
            e_yaw = endo.get("yaw", 0.0)
            s3 = endo.get("s3", 1)
            s4 = endo.get("s4", 1)

            # Build absolute world rotation from IMU (extrinsic/world ZYX)
            R_endo_world = rotz(math.radians(get_zero_yaw_tool())) * R_world_from_rpy_deg_extrinsic(
                e_roll, e_pitch, e_yaw
            )

            # Keep current translation, set new orientation
            pose_robot_old = robot.Pose()
            pose_robot_new = pose_with_same_translation(pose_robot_old, R_endo_world)

            # Move robot if reachable
            now = time.time()
            if now >= next_robot_cmd:
                next_robot_cmd = now + ROBOT_CMD_DT

                # 1) orientation command
                jcur = robot.Joints()
                if robot.MoveL_Test(jcur, pose_robot_new) == 0:
                    robot.MoveL(pose_robot_new, True)
                else:
                    with lock:
                        shared["status"] = "Cannot reach the Endowrist orientation"

                # 2) optional Z local step (same rate limit)
                if s3 == 0 or s4 == 0:
                    current_pose = robot.Pose()
                    Tz = transl(0, 0, Z_STEP_MM) if s3 == 0 else transl(0, 0, -Z_STEP_MM)
                    new_pose = current_pose * Tz
                    jcur = robot.Joints()
                    if robot.MoveL_Test(jcur, new_pose) == 0:
                        robot.MoveL(new_pose, True)
                    else:
                        with lock:
                            shared["status"] = "Cannot move further in local Z"

        # -------------------------
        # 2) Apply Gripper IMU
        #    Gripper is parented to endowrist, but IMU is absolute (WORLD),
        #    so convert to relative pose wrt endowrist:
        #       R_rel = inv(R_endo_world) * R_grip_world
        # -------------------------
        if grip is not None and endo is not None:
            g_roll = grip.get("roll", 0.0)
            g_pitch = grip.get("pitch", 0.0)
            g_yaw = grip.get("yaw", 0.0)
            s1 = grip.get("s1", 1)
            s2 = grip.get("s2", 1)

            # Recompute R_endo_world (same as above) to avoid carrying state
            e_roll = endo.get("roll", 0.0)
            e_pitch = endo.get("pitch", 0.0)
            e_yaw = endo.get("yaw", 0.0)
            R_endo_world = rotz(math.radians(get_zero_yaw_tool())) * R_world_from_rpy_deg_extrinsic(
                e_roll, e_pitch, e_yaw
            )

            # Absolute gripper orientation in world
            R_grip_world = rotz(math.radians(get_zero_yaw_gripper())) * R_world_from_rpy_deg_extrinsic(
                g_roll, g_pitch, g_yaw
            )

            # Convert to relative rotation wrt endowrist frame
            R_grip_rel = invH(R_endo_world) * R_grip_world

            # Set gripper pose relative to endowrist:
            # Keep fixed translation (gx0,gy0,gz0) and set relative rotation.
            gripper_pose_new = transl(gx0, gy0, gz0) * R_grip_rel
            gripper.setPose(gripper_pose_new)

            # Needle attach/detach based on buttons
            # Here we interpret:
            #   s1 == 0 -> open (release needle to base)
            #   s2 == 0 -> close (attach needle to gripper)
            if s1 == 0:
                needle.setParentStatic(base)
            elif s2 == 0:
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))

        # -------------------------
        # 3) Save torque info in shared status text (GUI will show it)
        # -------------------------
        if servo is not None:
            # Just keep it in shared for GUI formatting (no heavy processing here)
            pass

        time.sleep(STATE_DT)


# ============================================================
# GUI
# ============================================================
class AppGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Suture Process (IMU ABS RPY - extrinsic/world)")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Two yaw-offset sliders
        self.zero_yaw_tool = tk.DoubleVar(value=0.0)
        self.zero_yaw_gripper = tk.DoubleVar(value=0.0)

        self.text_label = tk.Label(root, text="", font=("Consolas", 11), justify="left")
        self.text_label.pack(padx=20, pady=10)

        tk.Scale(
            root, from_=-180, to=180, orient=tk.HORIZONTAL, length=300,
            label="Tool yaw offset (deg)", variable=self.zero_yaw_tool
        ).pack(pady=4)

        tk.Scale(
            root, from_=-180, to=180, orient=tk.HORIZONTAL, length=300,
            label="Gripper yaw offset (deg)", variable=self.zero_yaw_gripper
        ).pack(pady=4)

        # Kick GUI update loop
        self._gui_update()

    def get_zero_yaw_tool(self):
        return float(self.zero_yaw_tool.get())

    def get_zero_yaw_gripper(self):
        return float(self.zero_yaw_gripper.get())

    def _gui_update(self):
        # Snapshot shared data
        with lock:
            endo = shared["endo"]
            grip = shared["grip"]
            servo = shared["servo"]
            status = shared["status"]
            rx = dict(shared["rx_counts"])

        def fmt_rpy(msg):
            if not msg:
                return "n/a"
            return f'R={msg.get("roll", 0):7.2f}  P={msg.get("pitch", 0):7.2f}  Y={msg.get("yaw", 0):7.2f}'

        def fmt_torque(msg):
            if not msg:
                return "n/a"
            return (
                f'T_R1={msg.get("t_roll1", 0):6.2f}  '
                f'T_R2={msg.get("t_roll2", 0):6.2f}  '
                f'T_P={msg.get("t_pitch", 0):6.2f}  '
                f'T_Y={msg.get("t_yaw", 0):6.2f}'
            )

        # Buttons info (active-low)
        endo_btn = ""
        if endo:
            s3 = endo.get("s3", 1)
            s4 = endo.get("s4", 1)
            endo_btn = f"s3={s3} (up)  s4={s4} (down)"

        grip_btn = ""
        if grip:
            s1 = grip.get("s1", 1)
            s2 = grip.get("s2", 1)
            grip_btn = f"s1={s1} (open)  s2={s2} (close)"

        text = (
            f"Endowrist (ABS RPY world): {fmt_rpy(endo)}\n"
            f"Gripper  (ABS RPY world): {fmt_rpy(grip)}\n"
            f"{endo_btn}   {grip_btn}\n"
            f"Torques: {fmt_torque(servo)}\n"
            f"RX counts: Endo={rx.get(DEV_ENDO,0)}  Grip={rx.get(DEV_GRIP,0)}  Servos={rx.get(DEV_SERVO,0)}\n"
            f"Status: {status}\n"
            f"Offsets: ToolYaw={self.get_zero_yaw_tool():.1f}  GripYaw={self.get_zero_yaw_gripper():.1f}"
        )
        self.text_label.config(text=text)

        self.root.after(GUI_DT_MS, self._gui_update)

    def on_close(self):
        global stop_flag
        stop_flag = True
        self.root.destroy()


# ============================================================
# MAIN
# ============================================================
def main():
    global stop_flag

    # Initialize RoboDK project and items
    RDK, robot, base, endowrist, gripper, needle = initialize_robodk(ABSOLUTE_RDK_PATH)

    # Save initial poses to restore on exit
    pose0_robot = robot.Pose()
    pose0_gripper = gripper.Pose()
    pose0_needle = needle.Pose()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    # Start UDP read orientation
    udp_read = threading.Thread(target=read_orientation, args=(sock,), daemon=True)
    udp_read.start()

    # Create GUI
    root = tk.Tk()
    app = AppGUI(root)

    # Start control loop thread (robot + gripper)
    th_ctrl = threading.Thread(
        target=control_thread,
        args=(
            robot, base, gripper, needle,
            pose0_robot, pose0_gripper, pose0_needle
        ),
        kwargs={
            "gripper_init_xyz": (0, 5, -105),
            "get_zero_yaw_tool": app.get_zero_yaw_tool,
            "get_zero_yaw_gripper": app.get_zero_yaw_gripper,
        },
        daemon=True
    )
    th_ctrl.start()

    # Run GUI loop (blocking)
    root.mainloop()

    # Cleanup on exit
    stop_flag = True
    try:
        sock.close()
    except Exception:
        pass

    # Restore initial poses (best-effort)
    try:
        robot.MoveL(pose0_robot)
        gripper.setPose(pose0_gripper)
        needle.setPose(pose0_needle)
    except Exception:
        pass

    '''try:
        RDK.CloseRoboDK()
    except Exception:
        pass'''


if __name__ == "__main__":
    main()
