# 3-DOF Robotic Arm Simulator (FK & IK)

A 3D interactive Kinematics Simulator for a 3-DOF articulated robotic arm built using **Python**, **Open3D**, and **Tkinter**.

## Features

- **Forward Kinematics (FK):** Direct control of joint angles ($\theta_1, \theta_2, \theta_3$) with real-time 3D mesh rendering.
- **Analytical Inverse Kinematics (IK):** Cartesian target input ($X, Y, Z$) to automatically solve for valid joint configurations.
- **Trajectory Animation:** Pre-programmed target sequence execution with smooth step interpolation.
- **Cross-Platform Visualization:** Classic Open3D renderer paired with a responsive Tkinter GUI control panel.

## Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Dharaneesh2612/Robotic_Arm_3d.git
   cd Robotic_Arm_3d
   ```

2. **Create and activate virtual environment:**
   ```bash
   python -m venv venv
   .\venv\Scripts\activate     # On Windows
   # source venv/bin/activate  # On Linux/macOS
   ```

3. **Install dependencies:**
   ```bash
   pip install open3d numpy
   ```

4. **Run the simulation:**
   ```bash
   python robot_arm_3dof.py
   ```

## Requirements
- Python 3.9+
- Open3D
- NumPy
- Tkinter (included with standard Python on Windows)
