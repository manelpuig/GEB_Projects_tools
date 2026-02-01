![University of Barcelona Logo](././Images/Session1/figure1.png)

## Degree in Biomedical Engineering

### Init Hardware-Software setup
The **hardware setup** of the first prototype of the DaVinci surgery system is based on:
- A "Robotics_UB" router: Assigning a fixed IP address to each module (x corresponds to group number)
  - SSID: Robotics_UB
  - Password: 
- Hardware modules:
  - UR5e robot arm with Endowrist tool
  - PC control with roboDK program and python scripts (IP:192.168.1.x5)
  - ESP32 based wireless modules:
    - ESP32 based wireless Gripper control board (IP:192.168.1.x1)
    - ESP32 based wireless Endowrist control board (IP:192.168.1.x2)
    - ESP32 based wireless Servomotors control board (IP:192.168.1.x3)

The **software setup** of the first prototype of the DaVinci surgery system is based on:
- Init_SurgeryRobotics.rdk: program in roboDK virtual environment 
- Python script programs:
    - Read_from_Gripper.py: Reads the PRY Gripper module data
    - Read_from_Endowrist.py: Reads the RPY Endowrist module data
    - Init_SurgeryRobotics_simulation.py: Initial python program frame to read the data from the Gripper and Endowrist modules and send it to the UR5e robot arm in simulated roboDK program environment
    - Init_SurgeryRobotics_real.py: Initial python program frame to read the data from the Gripper and Endowrist modules and send it to the UR5e robot arm using sockeds in real environment
- Arduino programs:
    - Gripper folder: Arduino program for the Gripper module
    - Endowrist folder: Arduino program for the Endowrist module
    - Servomotors folder: Arduino program for the Servomotors module

### Init Hardware-Software setup functionality
The Initial functionality of the first prototype of the DaVinci surgery system is based on:
- The Gripper module reads its RPY (Roll, Pitch, Yaw) orientation and send them to Servomotors module and PC
- The Endowrist module reads its RPY (Roll, Pitch, Yaw) orientation and send them to PC
- The Servomotors module reads the RPY (Roll, Pitch, Yaw) orientation from Gripper module and applies it to the Endowrist tool
- In simulation: The PC reads the RPY (Roll, Pitch, Yaw) orientation from Gripper and Endowrist modules and sends it to the UR5e robot arm in roboDK program environment
- In Real: The PC reads the RPY (Roll, Pitch, Yaw) orientation from Gripper and Endowrist modules and sends it to the UR5e robot arm in roboDK program environment


### Laboratory session 1 Tasks:

The proposed tasks for this first session are:
- Connect properly the Hardware setup
- Save the ESP32 InitialPrograms for the 3 ESP32 modules using PlatformIO. Take care about the proper IP address of each module and PC.
- Run the Init_SurgeryRobotic_simulation.rdk file in the roboDK program to visualize the UR5e robot arm and the Endowrist tool.
- Test the system performances described above 

### Laboratory session 2 Tasks:

The proposed tasks for this second session are:
- Try to perform a suture process in simulation according to the following video:
[![suture process in simulation](Images/Session1/training.png)](https://youtu.be/1t3-Ggcp_Hg?feature=shared)
- IMU library performances:
  - Verify the Endowrist tool Yaw orientation performance. 
    - Are good the Yaw readings?
    - Readings are stable and robust when you are close to the computer or metallic parts?
    - Save the Endowrist_IMU program and verify the improvements in the Yaw orientation readings.
  - Create a new Gripper_IMU program based on the Endowrist_IMU program to improve the Gripper tool Yaw orientation readings.
  - Verify the improvements in Suture process simulation
- Gripper RPY angle corrections:
  - Perform:
    - Move Endowrist Roll mantaining zero gripper roll with respect to the Endowrist tool
    - Move Endowrist Pitch mantaining zero gripper pitch with respect to the Endowrist tool
    - Move Endowrist Yaw mantaining zero gripper yaw with respect to the Endowrist tool
  - Make the necessary corrections in roboDK python program to fix the observed issues

Show and explain the system performances to your teacher.