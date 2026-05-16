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
  robot_config.yaml             # Robot physical dimensions
  servo_calib.yaml              # Servo calibration
core/
    kinematics.py               # Kinematics Controller
    bezier_curve_gen.py         # Bezier Curve Generator
    gait_controller.py          # Robot Gait Controller
hw/
    quad_controller.py          # General Robot Controller
    
sim/                            # Simulation files and old code
tests/                          # Test files
tools/
    utils.py                    # Utility functions
    ADXL345.py                  # Accelerometer driver
    ITG_3200.py                 # Gyroscope driver
    QMC5883L.py                 # Magnetometer driver
    imu.py                      # IMU driver
    pid_controller.py           # PID Controllers
    teleop.py                   # Teleoperation
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