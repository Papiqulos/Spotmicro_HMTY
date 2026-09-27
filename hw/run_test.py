"""Scripted test run for PID tuning and Chapter 7 measurements.

    python -m hw.run_test F 1                              # gains from robot_config.yaml
    python -m hw.run_test F 1 --kp-r 0.5 --kd-p 0.01       # override single gains
    python -m hw.run_test F 1 --off                        # all gains 0 (F-OFF)
    python -m hw.run_test TW 2 --steps 800
    python -m hw.run_test F 1 --tape                       # ask for tape measurements after the run
"""
import argparse
import json
import numpy as np
import core.kinematics as kinematics
from core.kinematics import LENGTH, WIDTH, L1, L2, L3, L4
from core.gait_controller import _PID_ROLL, _PID_PITCH
from hw.quad_controller import RobotController, trot_params, LIN_VEL, THETA_RESTING
from tools.pid_controller import PIDController

TESTS = {
    "F":  (LIN_VEL, 0.0, "+x"),
    "B":  (LIN_VEL, 0.0, "-x"),
    "R":  (LIN_VEL, 0.0, "+z"),
    "L":  (LIN_VEL, 0.0, "-z"),
    "TR": (0.0, -0.5, "+x"),
    "TW": (LIN_VEL, -0.5, "+x"),
}

THETA_DEFAULT = np.array([0, -45, 60] * 4)

# Tape measurements asked after the run: (key, prompt)
TAPE_LINEAR = [("distance_m", "Απόσταση (m)"), ("lateral_cm", "Πλευρική απόκλιση (cm)"),
               ("heading_deg", "Αλλαγή κατεύθυνσης (deg)")]
TAPE_TURN = [("turn_deg", "Γωνία στροφής (deg)")]


def ask_tape(test):
    fields = TAPE_TURN if test == "TR" else TAPE_LINEAR if test in ("F", "B", "R", "L") else []
    tape = {}
    for key, prompt in fields:
        while True:
            v = input(f"{prompt}, Enter για παράλειψη: ").strip().replace(",", ".")
            if not v:
                break
            try:
                tape[key] = float(v)
                break
            except ValueError:
                print("Δώσε αριθμό")
    return tape


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=TESTS)
    ap.add_argument("rep", type=int)
    ap.add_argument("--steps", type=int, default=760)
    ap.add_argument("--off", action="store_true", help="all PID gains 0")
    ap.add_argument("--tape", action="store_true", help="ask for tape measurements after the run")
    for axis, cfg in (("r", _PID_ROLL), ("p", _PID_PITCH)):
        for k in ("kp", "ki", "kd"):
            ap.add_argument(f"--{k}-{axis}", type=float, default=cfg[k])
    args = ap.parse_args()

    g = vars(args)
    roll = {k: 0.0 if args.off else g[f"{k}_r"] for k in ("kp", "ki", "kd")}
    pitch = {k: 0.0 if args.off else g[f"{k}_p"] for k in ("kp", "ki", "kd")}
    tag = f"{args.test}{'-OFF' if args.off else ''}_{args.rep}"

    kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
    robot = RobotController(kin_solver, init_angles=THETA_DEFAULT, skip_rest=False)
    robot.gait_controller.pid_r = PIDController(**roll)
    robot.gait_controller.pid_p = PIDController(**pitch)
    print(f"{tag}  roll {roll}  pitch {pitch}")
    try:
        robot.move(trot_params(*TESTS[args.test]), args.steps, tag=tag)
    finally:
        robot.gait_controller.smooth_to_target(THETA_RESTING, duration=1.5,
                                               move_callback=robot.apply_angles_robot, unit="deg")
    pid_log = robot.gait_controller.log_path
    print(f"\nPID log: {pid_log}")
    if args.tape and pid_log:
        tape = ask_tape(args.test)
        if tape:
            meta_path = pid_log.replace(".csv", ".json")
            with open(meta_path, encoding="utf-8") as f:
                meta = json.load(f)
            meta["tape"] = tape
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
