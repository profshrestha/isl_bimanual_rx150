#!/usr/bin/env python3
"""
Single RX150 — Self-Contained Sequence Script
==============================================
Starts the single arm launch internally, waits for the arm to come up,
runs the motion sequence, then shuts everything down cleanly.

Run with:
  python3 rx150_single_sequence.py          # simulation (default)
  python3 rx150_single_sequence.py --real   # real hardware

Sequence:
  1. arm → HOME
  2. arm → waist +90°  (face left)
  3. arm → waist -90°  (face right)
  4. arm → waist   0°  (return to center)
  5. arm → SLEEP
"""

import sys
import time
import math
import subprocess
import signal
import os
import argparse

import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import create_interbotix_global_node


# ── Config ────────────────────────────────────────────────────────────────────
DEG90   = math.pi / 2.0
T_SLOW  = 2.5
T_MED   = 1.5
T_ACCEL = 0.3

# Joint poses [waist, shoulder, elbow, wrist_angle, wrist_rotate]
HOME = [0.0, -1.80, 1.55, 0.80, 0.0]

ROBOT_NAME  = 'rx150'
ROBOT_MODEL = 'rx150'


# ── Helpers ───────────────────────────────────────────────────────────────────

def go(arm, joints, t=T_MED):
    arm.arm.set_joint_positions(joints, moving_time=t, accel_time=T_ACCEL)
    time.sleep(t + 0.2)


def go_waist(arm, angle_rad, t=T_MED):
    arm.arm.set_single_joint_position('waist', angle_rad, moving_time=t, accel_time=T_ACCEL)
    time.sleep(t + 0.2)


def print_step(n, total, desc):
    print(f'\n{"━"*52}')
    print(f'  Step {n}/{total} — {desc}')
    print(f'{"━"*52}')


# ── Launch management ─────────────────────────────────────────────────────────

def start_launch(sim: bool) -> subprocess.Popen:
    use_sim = 'true' if sim else 'false'

    cmd = [
        'ros2', 'launch',
        'interbotix_xsarm_control', 'xsarm_control.launch.py',
        f'robot_model:={ROBOT_MODEL}',
        f'robot_name:={ROBOT_NAME}',
        f'use_sim:={use_sim}',
        'use_rviz:=false',
    ]

    mode = 'SIMULATION' if sim else 'REAL HARDWARE'
    print(f'\n  Starting single arm launch [{mode}]...\n')

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc


def wait_for_arm(proc: subprocess.Popen, timeout: float = 30.0) -> bool:
    """
    Read launch stdout until the SDK node reports ready, or timeout.

    Sim  output: "Interbotix 'xs_sdk_sim' node is up!"
    Real output: "InterbotixRobotXS is up!"
    """
    print('  Waiting for arm to initialise...')
    deadline = time.time() + timeout

    while time.time() < deadline:
        if proc.poll() is not None:
            print('  Launch process died unexpectedly.')
            return False

        line = proc.stdout.readline()
        if not line:
            time.sleep(0.05)
            continue

        text = line.decode('utf-8', errors='replace').strip()
        print(f'  [launch] {text}')

        is_up = (
            ("xs_sdk_sim' node is up" in text) or
            ("InterbotixRobotXS is up"  in text) or
            ("xs_sdk' node is up"       in text)
        )

        if is_up:
            print('  ARM SDK ready\n')
            return True

    print(f'  Timeout after {timeout}s — proceeding anyway.')
    return True


def stop_launch(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        print('\n  Shutting down launch...')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=6.0)
        except Exception:
            proc.kill()
        print('   Done.')


# ── Motion sequence ───────────────────────────────────────────────────────────

def run_sequence(arm):
    total = 5

    # 1 ── HOME
    print_step(1, total, 'HOME')
    arm.arm.go_to_home_pose()
    time.sleep(1.2)

    # 2 ── waist +90°
    print_step(2, total, 'waist +90°  (face left)')
    go_waist(arm, +DEG90, t=T_MED)

    # 3 ── waist -90°
    print_step(3, total, 'waist -90°  (face right)')
    go_waist(arm, -DEG90, t=T_SLOW)

    # 4 ── waist 0° (center)
    print_step(4, total, 'waist 0°  (return to center)')
    go_waist(arm, 0.0, t=T_MED)

    # 5 ── SLEEP
    print_step(5, total, 'SLEEP')
    arm.arm.go_to_sleep_pose()
    time.sleep(T_SLOW)

    print('\n  Sequence complete.')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--real', action='store_true',
                        help='Run on real hardware (default: simulation)')
    args = parser.parse_args()
    sim  = not args.real

    print('=' * 52)
    print('  Single RX150 Self-Contained Sequence')
    print(f'  Mode: {"SIMULATION" if sim else "REAL HARDWARE"}')
    print('=' * 52)

    launch_proc = None
    arm = None

    try:
        launch_proc = start_launch(sim)
        wait_for_arm(launch_proc, timeout=35.0)

        print('  Settling (3s)...')
        time.sleep(3.0)

        rclpy.init()
        node = create_interbotix_global_node('single_sequence_node')

        arm = InterbotixManipulatorXS(
            robot_model=ROBOT_MODEL,
            robot_name=ROBOT_NAME,
            node=node,
            moving_time=T_MED,
            accel_time=T_ACCEL,
        )

        print('\n  Arm initialised — starting sequence in 2s...')
        time.sleep(2.0)

        run_sequence(arm)

    except KeyboardInterrupt:
        print('\n  Interrupted')

    finally:
        try:
            if arm: arm.shutdown()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop_launch(launch_proc)


if __name__ == '__main__':
    main()
