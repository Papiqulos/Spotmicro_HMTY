# Design, Implementation and Control of a Quadruped Robot

# Project

Quadruped robot based on the SpotMicro v2 open source design as part of a Graduate Thesis at the University of Patras.

# Stack

## Hardware
- 3d printed parts: Robot body, legs(feet excluded), servo mounts (https://www.thingiverse.com/thing:4155673)
- Feet: Rubber balls cut into hemispheres
- Battery: Gens ace G-Tech 1100mAh 11.1V 3S1P 60C GRP-76 Hardcase Lipo Battery Pack with XT60 Plug
- Microcontroller: Raspberry Pi 5 powered by a 5V/5A type-c power supply
- Servo Motors: 12x Feetech FT5116M
- Servo Drivers: 2x PCA9685 powered by 2x XL4015 step-down converters (one for the front legsand one for the rear legs)
- IMU: GY-85 Gyro-Accel-Mag Sensor
- Lab power supply: for testing purposes only

# Software
- Ubuntu server 24.04 
- Python
- Pybullet for simulation


# Workspace Layout

```
config/
    robot_config.yaml           # Robot physical dimensions
    servo_calib.yaml            # Servo calibration
core/
    kinematics.py               # Kinematics and IK/FK solver
    bezier_curve_gen.py         # Bezier curve generator for swing trajectories
    gait_controller.py          # Trot gait, PID stabilization, CSV logging
hw/
    quad_controller.py          # High-level robot controller (servos + gait)
    imu.py                      # IMU driver (Madgwick/EKF sensor fusion)
    ADXL435.py                  # Accelerometer driver (ADXL345 via adafruit)
    ITG_3200.py                 # Gyroscope driver (I2C)
    QMC5883L.py                 # Magnetometer driver (disabled, too noisy)
    teleop.py                   # Keyboard teleoperation
log/
    roll_pitch/                 # CSV and png logs for roll/pitch PID controller
    log_plotter.py              # Matplotlib plotter for PID CSV logs
    log_plotter.m               # MATLAB plotter for PID CSV logs
    mag.txt                     # Magnetometer data for calibration
sim/
    pybullet_sim.py             # PyBullet simulation
    matplotlib_sim.py           # Matplotlib kinematics visualizer
tests/                          # Hardware-level tests (IMU, PCA9685, servos)
tools/
    utils.py                    # Utility functions
    pid_controller.py           # PID controllers
requirements.txt                # Python packages
requirements_sim.txt            # Python packages for simulation
```

# Common Pitfalls

- The pybullet axis are different than the robot axis as such: X-pybullet-> X-robot, Y-pybullet-> Z-robot, Z-pybullet-> Y-robot
- THe imu axis are differnt than the robot axis as such: X-robot-> -Y-imu, Y-robot-> X-imu, Z-robot-> -Z-imu
- Servos are position controlled
- No encoders present, the only feedback is the IMU
- Ignore file with the _new suffix they are for future development


# Style

- Minimal comments. No "AI-generated" style explanatory comments.
- Direct, technical language. No filler adjectives.
- Package maintainer: Giorgos Papoutsas 