"""Scripted simulation runs for Chapter 7, the counterpart of hw/run_test.py.

    python -m sim.run_sim_test all              # F, F-OFF, B, R, L, TR, TW, one run each (tag <test>_1)
    python -m sim.run_sim_test F TW             # selected tests
    python -m sim.run_sim_test all --direct     # headless, faster than real time

Commands and gait parameters are the same as on the robot (hw/quad_controller.trot_params).
Gains come from config/robot_config.yaml; F-OFF runs with all gains 0.
"""
import argparse
import math
import core.kinematics as kinematics
from sim.pybullet_sim import PybulletSim

LIN_VEL = 0.12

TESTS = {
    "F":     (LIN_VEL, 0.0, "+x"),
    "F-OFF": (LIN_VEL, 0.0, "+x"),
    "B":     (LIN_VEL, 0.0, "-x"),
    "R":     (LIN_VEL, 0.0, "+z"),
    "L":     (LIN_VEL, 0.0, "-z"),
    "TR":    (0.0, -0.5, "+x"),
    "TW":    (LIN_VEL, -0.5, "+x"),
}

OFF = dict(kp_r=0.0, ki_r=0.0, kd_r=0.0, kp_p=0.0, ki_p=0.0, kd_p=0.0)


def trot_params(lin_vel, ang_vel, dir):
    """Same values as hw/quad_controller.trot_params."""
    return dict(desired_lin_vel=lin_vel, desired_ang_vel=ang_vel, swing_height=0.035,
                stance_length=0.06, Tswing=0.2, dir=dir, gait_type="trot")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="+", help="test codes or 'all'")
    ap.add_argument("--rep", type=int, default=1)
    ap.add_argument("--steps", type=int, default=1050, help="control steps before deceleration (10 ms each)")
    ap.add_argument("--direct", action="store_true", help="headless, no real-time pacing")
    args = ap.parse_args()
    tests = list(TESTS) if args.tests == ["all"] else args.tests
    for t in tests:
        if t not in TESTS:
            ap.error(f"unknown test {t}, choose from {', '.join(TESTS)} or all")

    sim = PybulletSim(length=kinematics.LENGTH, width=kinematics.WIDTH,
                      l1=kinematics.L1, l2=kinematics.L2, l3=kinematics.L3, l4=kinematics.L4,
                      center=[0, 0, 0.27], orientation=[0, 0, math.pi], center_plane=[0, 0, 0],
                      initial_theta=[0, -30, 60] * 4, angle_unit="deg",
                      gui=not args.direct, interactive=False)
    for t in tests:
        sim.respawn_robot()
        tag = f"{t}_{args.rep}"
        log = sim.run_test(trot_params(*TESTS[t]), args.steps, tag, test=t.split("-")[0],
                           gains=OFF if t == "F-OFF" else None, realtime=not args.direct)
        print(f"{tag}: {log}")
