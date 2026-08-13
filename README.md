# Quadruped Robot — Design, Implementation and Motion Control

Low-cost 3D-printed quadruped robot with omnidirectional trot gait and IMU-based
attitude stabilisation. Diploma thesis, Department of Electrical and Computer
Engineering, University of Patras.

Built on the open-source [SpotMicro](https://www.thingiverse.com/thing:3445283)
design by Deok-yeon Kim (CC-BY), using the
[SpotMicro v2](https://www.thingiverse.com/thing:4155673) remix by nahueltaibo.

> **Status:** work in progress. Trot gait is fully tuned and validated; the other
> four gaits are implemented but not tuned.

<!-- TODO: demo video / GIF here -->
<!-- TODO: photo of the assembled robot here -->

---

## What it does

- **Omnidirectional gait** — linear translation along an arbitrary heading
  composed with an angular turn rate, resolved per leg through a yaw-arc
  correction
- **Closed-loop attitude stabilisation** — roll/pitch PID acting on fused IMU
  data, applied as per-leg foot-height offsets
- **Five gait patterns** — trot, walk, bound, pace, pronk (phase distribution is
  parametric); only trot is tuned
- **Simulation-first workflow** — controller parameters are tuned in PyBullet
  before being moved to hardware
- **Real-time teleoperation** via a DualSense controller
- **CSV logging** of filtered IMU and PID output for quantitative evaluation

The robot has no joint encoders and no foot contact sensors. The IMU is the only
feedback source — a deliberate consequence of using low-cost position-controlled
servos.

---

## Hardware

| Component | Part |
|---|---|
| Compute | Raspberry Pi 5 (5 V / 5 A USB-C) |
| Servos | 12× Feetech FT5116M |
| Servo drivers | 2× PCA9685 (front `0x40`, rear `0x41`) |
| Power | 2× XL4015 step-down, one per driver board |
| Battery | Gens ace 1100 mAh 11.1 V 3S 60C, XT60 |
| IMU | GY-85 (ADXL345 accel + ITG-3200 gyro + QMC5883L mag) |
| Frame | 3D-printed body, legs and servo mounts |
| Feet | Rubber balls cut into hemispheres |

The magnetometer is present but disabled — too noisy without shielding. Yaw is
therefore not observable; only roll and pitch are stabilised.

Physical dimensions live in [`config/robot_config.yaml`](config/robot_config.yaml),
measured on the real robot.

---

## Repository layout

```
config/     robot_config.yaml (dimensions, joint limits), servo_calib.yaml
core/       kinematics.py, robot_state.py, gait_controller.py, bezier_curve_gen.py
hw/         quad_controller.py, imu.py, sensor drivers, teleop.py
sim/        pybullet_sim.py, matplotlib_sim.py, urdf/ (+ STL meshes)
tools/      pid_controller.py, utils.py
log/        CSV/PNG logs + plotting scripts
thesis/     LaTeX source of the thesis
```

`kinematics.py` reads `config/robot_config.yaml` by relative path, so **run every
script from the repository root**.

## Entry points

| Command | What it does |
|---|---|
| `python sim/pybullet_sim.py` | PyBullet simulation, keyboard-driven |
| `python hw/quad_controller.py` | Hardware controller (on the Pi) |
| `python hw/teleop.py` | DualSense input test |
| `python sim/matplotlib_sim.py` | Static kinematics visualiser |

---

## Setup

### Simulation (any machine)

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements_sim.txt
python sim/pybullet_sim.py
```

### Hardware (Raspberry Pi)

```bash
pip install -r requirements.txt
sudo raspi-config      # enable I2C
i2cdetect -y 1         # expect 0x40 and 0x41 (PCA9685) plus the IMU addresses
```

For the DualSense controller, install the udev rule so it is reachable without
root:

```bash
sudo cp ps5_controller/70-ps5-controller.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## How the gait controller works

`GaitController` exposes four `execute_gait_*` entry points that share one core
(`_compute_ramp` → `_imu_correction` → `_step_legs`) and differ only in how the
swing/stance durations and the phase clock are derived. `execute_gait_fixed_swing_td`
is the most robust for straight lines; the `fixed_stance` variants suit turning.

One control step:

1. **Velocity ramp** — cosine ramp over 0.5 s on both linear and angular velocity
2. **Banked-roll feedforward** — `banked_roll = sign(ω)·atan2(v², g·R)` with
   `R = |v|/|ω|`, added to the measured roll before the PID
3. **Attitude PID** — separate roll and pitch controllers with integral clamping
4. **Foot-height offset** — per leg, `dy = ±(W/2)·tan(roll) ± (L/4)·tan(pitch)`
5. **Phase assignment** — for trot, FL+RR at phase 0.0 and FR+RL at 0.5
6. **Turning composition** — per-leg yaw arc from the nominal foot position;
   linear and angular step vectors are summed, and the result drives both phases
7. **Stance** — sinusoidal dip with penetration depth δ
8. **Swing** — 12-point Bézier with stacked endpoints, giving zero foot velocity
   at lift-off and touchdown, plus an S/6 touchdown offset
9. **Inverse kinematics** per leg, after the body transform
10. **CSV logging** of filtered IMU and PID output

IMU filtering happens in the driver (`hw/imu.py`), not in the gait controller:
Madgwick (or EKF) fusion → exponential low-pass (α = 0.3) → 30-tap moving
average. The moving average removes the ~2 Hz oscillation inherent to trotting,
which would otherwise be fed straight into the PID.

<!-- TODO: diagram of the control loop here -->

---

## Coordinate frames

Three frames disagree, which is the most common source of confusion:

| | Mapping |
|---|---|
| PyBullet → robot | `X→X`, `Y→Z`, `Z→Y` |
| Robot → IMU | `X→−Y`, `Y→X`, `Z→−Z` |

The kinematics frame is **X forward, Y up, Z left**, in millimetres.

---

## Known limitations

- Only the trot gait is tuned. The other four are implemented but not validated.
- Joint limits are loaded from `robot_config.yaml` but not currently enforced in
  `legIK`; an unreachable target returns zero angles instead of raising.
- The magnetometer is disabled, so yaw is not observable — only roll and pitch
  are stabilised.
- The tilt cut-off checks pitch only and does not stop an in-progress run.

---

## Thesis

The LaTeX source is under [`thesis/`](thesis/). Build with:

```bash
cd thesis/first_draft && latexmk main.tex
```

Requires LuaLaTeX and Biber; `.latexmkrc` selects the right engine automatically.

---

## Acknowledgements

- Deok-yeon Kim — original SpotMicro design
- nahueltaibo — SpotMicro v2 remix
- The [SpotMicroAI](https://spotmicroai.readthedocs.io) community
- [spot_mini_mini](https://github.com/moribots/spot_mini_mini) — Bézier gait
  reference
- Previous student implementation at the same department
  ([QuadrupedRobotProject](https://github.com/VagTsiats/QuadrupedRobotProject-ecedk703))

Supervisor: Prof. Charalampos Bechlioulis, University of Patras.

<!-- TODO: pick and add a LICENSE file. Note the SpotMicro design is CC-BY,
     which requires attribution to be preserved. -->
