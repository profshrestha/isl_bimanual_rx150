#!/usr/bin/env python3
"""
Dual RX150 — Self-Contained Sequence Script
============================================
Starts the dual arm launch internally, waits for arms to come up,
runs the motion sequence, then shuts everything down cleanly.

Run with:
  python3 rx150_sequence.py          # simulation (default)
  python3 rx150_sequence.py --real   # real hardware

Sequence:
  1. arm_left  → HOME
  2. arm_left  → waist +90° (faces audience +X)
  3. arm_right → HOME
  4. arm_right → waist -90° (faces audience +X)
  5. arm_left  → HOME → SLEEP
  6. arm_right → HOME → SLEEP

World frame:
  arm_left  (0, +0.3273, 0)  forward=-Y  waist- → faces +X audience
  arm_right (0, -0.3273, 0)  forward=+Y  waist+ → faces +X audience
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
DEG90    = math.pi / 2.0
T_SLOW   = 2.5
T_MED    = 1.5
T_ACCEL  = 0.3

# Joint poses [waist, shoulder, elbow, wrist_angle, wrist_rotate]
HOME  = [ 0.0, -1.80,  1.55,  0.80,  0.0]


# ── Helpers ───────────────────────────────────────────────────────────────────

def go(arm, joints, t=T_MED):
    arm.arm.set_joint_positions(joints, moving_time=t, accel_time=T_ACCEL)
    time.sleep(t + 0.2)


def go_waist(arm, angle_rad, t=T_MED):
    arm.arm.set_single_joint_position('waist', angle_rad, moving_time=t, accel_time=T_ACCEL)
    time.sleep(t + 0.2)


def print_step(n, desc):
    print(f'\n{"━"*52}')
    print(f'  Step {n}/6 — {desc}')
    print(f'{"━"*52}')


# ── Launch management ─────────────────────────────────────────────────────────
MOTOR_CONFIGS_DIR = '/var/isl_robotics_shared/dual-rx150-demo-sparse/motor_configs'


def _arm_cmd(robot_name: str, sim: bool) -> list:
    use_sim = 'true' if sim else 'false'
    cmd = [
        'ros2', 'launch',
        'interbotix_xsarm_control', 'xsarm_control.launch.py',
        'robot_model:=rx150',
        f'robot_name:={robot_name}',
        f'use_sim:={use_sim}',
        'use_rviz:=false',
    ]
    # For real hardware pass custom motor config with the correct port per arm
    if not sim:
        cmd.append(f'motor_configs:={MOTOR_CONFIGS_DIR}/{robot_name}.yaml')
    return cmd


def start_launch(sim: bool):
    mode = 'SIMULATION' if sim else 'REAL HARDWARE'
    print(f'\n  Starting dual arm launch [{mode}]...\n')

    proc_right = subprocess.Popen(
        _arm_cmd('arm_right', sim),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    proc_left = subprocess.Popen(
        _arm_cmd('arm_left', sim),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc_right, proc_left


def _wait_one(proc: subprocess.Popen, label: str, deadline: float) -> bool:
    while time.time() < deadline:
        if proc.poll() is not None:
            print(f'  {label} launch died unexpectedly.')
            return False
        line = proc.stdout.readline()
        if not line:
            time.sleep(0.05)
            continue
        text = line.decode('utf-8', errors='replace').strip()
        print(f'  [{label}] {text}')
        if (
            ("xs_sdk_sim' node is up" in text) or
            ("InterbotixRobotXS is up"  in text) or
            ("xs_sdk' node is up"       in text)
        ):
            print(f'  ✓ {label} SDK ready')
            return True
    return False


def wait_for_arms(procs, timeout: float = 35.0) -> bool:
    import threading
    print('  Waiting for both arms to initialise...')
    results = {}
    deadline = time.time() + timeout

    def wait(proc, label):
        results[label] = _wait_one(proc, label, deadline)

    threads = [
        threading.Thread(target=wait, args=(procs[0], 'arm_right'), daemon=True),
        threading.Thread(target=wait, args=(procs[1], 'arm_left'),  daemon=True),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=timeout + 2)

    if all(results.values()):
        print('  Both arms ready.\n')
        return True
    print(f'  Timeout after {timeout}s — proceeding anyway.')
    return True


def stop_launch(procs):
    for proc in procs:
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=6.0)
            except Exception:
                proc.kill()
    print('  Launch processes stopped.')


# ── Motion sequence ───────────────────────────────────────────────────────────

def run_sequence(L, R):

    # 1 ── arm_left → HOME
    print_step(1, 'arm_left → HOME')
    L.arm.go_to_home_pose()
    time.sleep(1.2)

    # 2 ── arm_left waist +90°
    print_step(2, 'arm_left → waist +90°  (faces +X audience)')
    go_waist(L, +DEG90, t=T_MED)

    # 3 ── arm_right → HOME
    print_step(3, 'arm_right → HOME')
    R.arm.go_to_home_pose()
    time.sleep(1.2)

    # 4 ── arm_right waist -90°
    print_step(4, 'arm_right → waist -90°  (faces +X audience)')
    go_waist(R, -DEG90, t=T_MED)

    # 5 ── arm_left → SLEEP (waist already at +90°, return to center first)
    print_step(5, 'arm_left → HOME then SLEEP')
    L.arm.go_to_home_pose()
    time.sleep(1.2)
    L.arm.go_to_sleep_pose()
    time.sleep(T_SLOW)

    # 6 ── arm_right → SLEEP (waist already at -90°, return to center first)
    print_step(6, 'arm_right → HOME then SLEEP')
    R.arm.go_to_home_pose()
    time.sleep(1.2)
    R.arm.go_to_sleep_pose()
    time.sleep(T_SLOW)

    print('\n✅  Sequence complete.')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--real', action='store_true',
                        help='Run on real hardware (default: simulation)')
    args   = parser.parse_args()
    sim    = not args.real

    print('=' * 52)
    print('  Dual RX150 Self-Contained Sequence')
    print(f'  Mode: {"SIMULATION" if sim else "REAL HARDWARE"}')
    print('=' * 52)

    launch_procs = (None, None)
    L = R = None

    try:
        launch_procs = start_launch(sim)
        wait_for_arms(launch_procs, timeout=35.0)

        # Extra settle time for RViz and topic graph
        print('⏳  Settling (3s)...')
        time.sleep(3.0)

        # Init ROS2 and arm SDK — uses the already-running launch
        rclpy.init()
        node = create_interbotix_global_node('dual_sequence_node')

        L = InterbotixManipulatorXS(
            robot_model='rx150',
            robot_name='arm_left',
            node=node,
            moving_time=T_MED,
            accel_time=T_ACCEL,
        )

        R = InterbotixManipulatorXS(
            robot_model='rx150',
            robot_name='arm_right',
            node=node,
            moving_time=T_MED,
            accel_time=T_ACCEL,
        )

        print('\n🤖  Arms initialised — starting sequence in 2s...')
        time.sleep(2.0)

        run_sequence(L, R)

    except KeyboardInterrupt:
        print('\n⚠️  Interrupted')

    finally:
        for arm in [L, R]:
            try:
                if arm: arm.shutdown()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop_launch(launch_procs)


if __name__ == '__main__':
    main()
