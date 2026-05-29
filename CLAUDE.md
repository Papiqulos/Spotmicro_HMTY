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
- Servo Drivers: 2x PCA9685 powered by 2x XL4015 step-down converters (one for front legs and one for rear legs)
- IMU: GY-85 Gyro-Accel-Mag Sensor (ADXL345 + ITG-3200 + QMC5883L)
- Lab power supply: for testing purposes only

# Software
- Ubuntu server 24.04
- Python
- PyBullet for simulation


# Workspace Layout

```
config/
    robot_config.yaml           # Robot physical dimensions (mm), measured on real hardware
    servo_calib.yaml            # Servo calibration offsets
core/
    kinematics.py               # Kinematics: legFK, legIK, bodyIK, robot_IK, robot_FK
    bezier_curve_gen.py         # General n-point Bezier curve via Bernstein polynomials
    gait_controller.py          # Trot gait, PID stabilization, sinusoidal stance, CSV logging
hw/
    quad_controller.py          # High-level robot controller (servos + gait)
    imu.py                      # IMU driver (Madgwick/EKF sensor fusion via ahrs library)
    ADXL435.py                  # Accelerometer driver (ADXL345 via adafruit)
    ITG_3200.py                 # Gyroscope driver (I2C)
    QMC5883L.py                 # Magnetometer driver (disabled, too noisy)
    teleop.py                   # Keyboard teleoperation
log/
    roll_pitch/                 # CSV and png logs for roll/pitch PID controller
    log_plotter.py              # Matplotlib plotter for PID CSV logs (headless)
    log_plotter.m               # MATLAB plotter for PID CSV logs
    mag.txt                     # Magnetometer raw data for calibration
sim/
    pybullet_sim.py             # PyBullet simulation
    matplotlib_sim.py           # Matplotlib kinematics visualizer
tests/
    imu_test.py                 # IMU live output test
    madgwick_filter_test.py     # Madgwick filter standalone test
    pca9685_calibration.py      # PCA9685 calibration helper
    test_pca.py                 # PCA9685 connectivity test
    adxl34x_*.py                # ADXL345 tap/freefall/motion detection tests
tools/
    utils.py                    # to_homogenous and other geometry helpers
    pid_controller.py           # PIDController (single-axis) and PIDControllerRP (roll/pitch)
requirements.txt                # Python packages (hardware)
requirements_sim.txt            # Python packages (simulation)
paper_improvements.txt          # Literature review: improvement items [1]-[49] with file refs
```

# Gait Controller Architecture

`GaitController.trot()` runs one iteration of the trot loop:
1. Cosine velocity ramp (0.5 s startup / deceleration)
2. 30-tap moving average on raw IMU roll/pitch (removes ~2 Hz trot oscillation)
3. PIDControllerRP on filtered roll/pitch -> orientation correction angles
4. Per-leg foot height offset via geometry formula:
   `dy_fl = +(W/2)*tan(roll_corr) + (L/4)*tan(pitch_corr)`  (sign differs per leg)
5. Phase assignment: FL+RR share phase 0.0, FR+RL share phase 0.5 (diagonal trot)
6. Stance: sinusoidal dip trajectory (`stance_sine_trajectory`) with penetration depth δ
7. Swing: 12-point Bezier (`swing_trajectory_control_points`) with triple X-endpoint
   stacking (zero foot velocity at liftoff/touchdown), S/6 touchdown offset (He 2020)
8. Per-leg IK via `Kinematics.legIK` after body transform
9. CSV logging of filtered IMU and PID output each cycle

Control point layout (`_SWING_X_NORM`): `[0,0,0, ..., 1,1,1]` — triple-stacked endpoints.
Height profile (`_SWING_H_NORM`): double-stacked endpoints (velocity zeroed, not acceleration).

# Common Pitfalls

- PyBullet axes differ from robot axes: X-pybullet -> X-robot, Y-pybullet -> Z-robot, Z-pybullet -> Y-robot
- IMU axes differ from robot axes: X-robot -> -Y-imu, Y-robot -> X-imu, Z-robot -> -Z-imu
- Servos are position-controlled. No encoders. Only feedback is IMU.
- `bodyIK` in `gait_controller.trot()` is called with `self.initial_orientation`, not the PID-corrected orientation. The PID correction is applied as a foot Y-offset (dy_dic), not as a body pose rotation.
- `legIK` returns `acos(D)` for theta3 (knee angle), always giving the elbow-down solution. No per-side sign flip needed for SpotMicro's all-elbow topology.
- `QMC5883L` magnetometer is instantiated but disabled due to noise. Do not enable without hardware shielding.
- Ignore files with the `_new` suffix — they are for future development.
- `core/` and `tools/` lack `__init__.py`. Imports rely on `sys.path` manipulation or being run from the project root.

# Paper Improvements — Implementation Status

Items reference `paper_improvements.txt`. Already implemented:

| Item | Description | Location |
|------|-------------|----------|
| [6]  | 30-tap moving average on IMU roll/pitch | `gait_controller.py:67,198` |
| [7]  | Per-leg geometry-aware foot height correction | `gait_controller.py:220` |
| [15/17] | S/6 initial touchdown offset | `gait_controller.py:100` |
| [18] | PID integral anti-windup (max_I clamp) | `pid_controller.py:64` |
| [27/33] | Sinusoidal stance with penetration depth δ | `gait_controller.py:119` |
| [29] | Zero foot velocity at liftoff/touchdown (triple X-stacking) | `gait_controller.py:31` |

Partially implemented:

- **[34]** `_SWING_H_NORM` has double (not triple) endpoint stacking — foot acceleration at touchdown is not zeroed.
- **[8]** PID operates on raw angles then converts to DeltaH; Mori paper recommends PID directly on DeltaH.

Not yet implemented (see `paper_improvements.txt` for full detail):

- [22] Froude number in CSV log
- [24] Static IMU bias calibration at startup (store in servo_calib.yaml)
- [25] Gyro zero-rate bias update during stationary periods
- [26] Asymmetric swing/stance duration (constant swing, variable stance)
- [36] Adaptive Madgwick correction gain during high-acceleration phases
- [37] Raibert heuristic footstep placement
- [40] Exponential smoothing post Madgwick output
- [43] Total tilt fall-detection gate (rho > 45 deg -> halt)
- [19/20] Stuck servo detection via IMU anomaly + safe fallback gait
- [45-49] Turning control (stubs exist: `turn()`, `turn_in_place()`)
- [10/11] Oscillator gait + CMA-ES offline optimization


# Style

- Minimal comments. No "AI-generated" style explanatory comments.
- Direct, technical language. No filler adjectives.
- Package maintainer: Giorgos Papoutsas