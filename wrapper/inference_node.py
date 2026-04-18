#!/usr/bin/env python3
"""
UMI Diffusion Policy — RX150 Bimanual Inference Node

Runs a trained UMI diffusion policy on two RX150 arms.
robot0 = arm_right (picks object), robot1 = arm_left (places on a target surface).

Usage (inside umi conda env + ROS2 sourced):
  conda activate umi
  source /etc/profile.d/ros2_interbotix.sh

  # Real arms:
  python3 inference_node.py --checkpoint folder_path/checkpoints/run_001/latest.ckpt

  # Sim mode (print actions, do not move arms — useful for pipeline testing):
  python3 inference_node.py --checkpoint <ckpt> --sim

Camera topics (default):
  arm_right GoPro → /gopro_right/image_raw
  arm_left  GoPro → /gopro_left/image_raw

Layout convention (must match training):
  robot0 = arm_right, camera0 = right GoPro
  robot1 = arm_left,  camera1 = left  GoPro
"""

import sys
import os
import argparse
import time
import threading
from collections import deque

import numpy as np
import torch
import scipy.spatial.transform as st

# UMI repo path
UMI_ROOT = '/opt/umi/universal_manipulation_interface'
sys.path.insert(0, UMI_ROOT)

# ─── Policy loading ─────────────────────────────────────────────────────────────

def load_policy(checkpoint_path: str, device: str):
    import dill
    import hydra
    from omegaconf import OmegaConf
    OmegaConf.register_new_resolver("eval", eval, replace=True)

    payload = torch.load(checkpoint_path, pickle_module=dill, map_location='cpu')
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
    workspace = cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    policy = workspace.model
    if hasattr(workspace, 'ema_model') and workspace.ema_model is not None:
        policy = workspace.ema_model
    policy.eval().to(device)
    return policy, cfg


# ─── RX150 arm wrapper ──────────────────────────────────────────────────────────

class RX150Arm:
    """Thin wrapper around InterbotixManipulatorXS for UMI inference."""

    def __init__(self, robot_name: str, node, sim: bool = False):
        self.robot_name = robot_name
        self.sim = sim
        if not sim:
            from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
            self.arm = InterbotixManipulatorXS(
                robot_model='rx150',
                robot_name=robot_name,
                node=node,
            )

    def get_ee_pose(self):
        """Returns (pos [3], axis_angle [3]) in robot base frame (metres, radians)."""
        if self.sim:
            # Return a plausible home-ish pose for testing
            return np.array([0.20, 0.0, 0.15]), np.zeros(3)
        mat = self.arm.arm.get_ee_pose()          # 4×4 homogeneous matrix
        pos = mat[:3, 3]                           # [x, y, z]
        rot = st.Rotation.from_matrix(mat[:3, :3]).as_rotvec()  # axis-angle [3]
        return pos, rot

    def get_gripper_width(self):
        """Returns gripper width in metres. 0.044 = fully open, 0 = closed."""
        if self.sim:
            return 0.044
        # Approximate from finger joint position
        js = self.arm.gripper.core.joint_states
        if js is None:
            return 0.044
        names = list(js.name)
        finger_key = f'{self.robot_name}/left_finger'
        if finger_key in names:
            idx = names.index(finger_key)
            return float(js.position[idx]) * 2  # left + right finger
        return 0.044

    def move(self, x, y, z, roll, pitch, yaw, moving_time=0.1):
        """Move EEF to absolute Cartesian pose. Angles in radians."""
        if self.sim:
            print(f'  [{self.robot_name}] EEF → '
                  f'xyz=({x:.3f},{y:.3f},{z:.3f})  '
                  f'rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})')
            return
        self.arm.arm.set_ee_pose_components(
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            moving_time=moving_time,
            accel_time=moving_time * 0.2,
        )

    def set_gripper(self, width_m: float, moving_time=0.1):
        """Command gripper width in metres. 0 = closed, 0.044 = open."""
        if self.sim:
            print(f'  [{self.robot_name}] gripper → {width_m:.4f} m')
            return
        self.arm.gripper.move_grippers(width_m, delay=moving_time)


# ─── ROS2 image buffer ─────────────────────────────────────────────────────────

class ImageBuffer:
    """Subscribes to two GoPro image topics and keeps the latest frame each."""

    def __init__(self, topic_right: str, topic_left: str):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._imgs = {'right': None, 'left': None}

        # Inline node class so we don't need a separate file
        class _ImgNode(Node):
            def __init__(inner_self):
                super().__init__('umi_image_buffer')
                inner_self.create_subscription(
                    Image, topic_right,
                    lambda m: self._cb(m, 'right'), 10)
                inner_self.create_subscription(
                    Image, topic_left,
                    lambda m: self._cb(m, 'left'), 10)

        self._node = _ImgNode()

    def _cb(self, msg, side: str):
        from cv_bridge import CvBridge
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        with self._lock:
            self._imgs[side] = img  # [H, W, 3] uint8

    @property
    def node(self):
        return self._node

    def get_images(self):
        """Returns (img_right, img_left) as [H,W,3] uint8 arrays."""
        with self._lock:
            return self._imgs['right'], self._imgs['left']

    def ready(self):
        with self._lock:
            return all(v is not None for v in self._imgs.values())


# ─── Helpers ───────────────────────────────────────────────────────────────────

# Transform: robot0 (arm_right) frame → robot1 (arm_left) frame.
#
# Physical layout (both arms face +X):
#   arm_right (robot0) at world [0, -0.3273, 0], facing +X  (yaw = 0°)
#   arm_left  (robot1) at world [0, +0.3273, 0], facing +X  (yaw = 0°)
#
# Both frames have identity rotation (R = I).
# TX_ROBOT1_ROBOT0 = T_left⁻¹ · T_right
#   rotation:    I^T · I = I  (no rotation between frames)
#   translation: I · t_right + (−I · t_left)
#                = [0,-0.3273,0] + [0,-0.3273,0] = [0, -0.6546, 0]
TX_ROBOT1_ROBOT0 = np.array([
    [1, 0, 0,  0.0   ],
    [0, 1, 0, -0.6546],
    [0, 0, 1,  0.0   ],
    [0, 0, 0,  1.0   ],
], dtype=np.float64)


def action_to_euler(action_7: np.ndarray):
    """Convert [x, y, z, rx, ry, rz, gripper] to (x,y,z, roll,pitch,yaw, gripper).
    rx,ry,rz is axis-angle; set_ee_pose_components expects euler XYZ in radians."""
    x, y, z = action_7[:3]
    roll, pitch, yaw = st.Rotation.from_rotvec(action_7[3:6]).as_euler('xyz')
    gripper = float(action_7[6])
    return x, y, z, roll, pitch, yaw, gripper


# ─── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='UMI diffusion policy inference on dual RX150')
    p.add_argument('--checkpoint', '-i', required=True,
                   help='Path to .ckpt file')
    p.add_argument('--frequency', '-f', type=float, default=10.0,
                   help='Control loop frequency in Hz (default: 10)')
    p.add_argument('--steps_per_inference', '-si', type=int, default=6,
                   help='Actions to execute per policy call (default: 6)')
    p.add_argument('--camera_right_topic', default='/gopro_right/image_raw')
    p.add_argument('--camera_left_topic',  default='/gopro_left/image_raw')
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    p.add_argument('--sim', action='store_true',
                   help='Print actions without moving arms (pipeline test)')
    return p.parse_args()


def main():
    args = parse_args()
    dt = 1.0 / args.frequency

    # ── Load policy ──────────────────────────────────────────────────────────
    print(f'Loading policy: {args.checkpoint}')
    policy, cfg = load_policy(args.checkpoint, args.device)
    shape_meta       = cfg.task.shape_meta
    obs_pose_repr    = cfg.task.pose_repr.obs_pose_repr
    action_pose_repr = cfg.task.pose_repr.action_pose_repr
    T_OBS = cfg.task.img_obs_horizon  # typically 2
    print(f'  device={args.device}  T_obs={T_OBS}  '
          f'obs_repr={obs_pose_repr}  act_repr={action_pose_repr}')

    # ── ROS2 init ────────────────────────────────────────────────────────────
    import rclpy
    from diffusion_policy.common.pytorch_util import dict_apply
    from umi.real_world.real_inference_util import get_real_umi_obs_dict, get_real_umi_action
    rclpy.init()

    # ── Arms ─────────────────────────────────────────────────────────────────
    if args.sim:
        arm_right = RX150Arm('arm_right', node=None, sim=True)
        arm_left  = RX150Arm('arm_left',  node=None, sim=True)
    else:
        from interbotix_common_modules.common_robot.robot import \
            create_interbotix_global_node
        interbotix_node = create_interbotix_global_node('umi_inference')
        arm_right = RX150Arm('arm_right', node=interbotix_node)
        arm_left  = RX150Arm('arm_left',  node=interbotix_node)

    # ── Image buffer ─────────────────────────────────────────────────────────
    img_buf = ImageBuffer(args.camera_right_topic, args.camera_left_topic)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(img_buf.node,), daemon=True)
    spin_thread.start()

    print('Waiting for camera images ...')
    while not img_buf.ready():
        time.sleep(0.1)
    print('Both cameras ready.')

    # ── Observation helpers ───────────────────────────────────────────────────
    # Rolling buffer: each entry is one timestep of raw observations
    obs_ring = deque(maxlen=T_OBS)

    def snap():
        """Capture one observation timestep."""
        img_r, img_l = img_buf.get_images()
        pos_r, rot_r = arm_right.get_ee_pose()
        pos_l, rot_l = arm_left.get_ee_pose()
        gw_r = arm_right.get_gripper_width()
        gw_l = arm_left.get_gripper_width()
        return (img_r, img_l,
                pos_r.copy(), rot_r.copy(), gw_r,
                pos_l.copy(), rot_l.copy(), gw_l)

    def build_env_obs():
        """Assemble env_obs dict from the rolling buffer for get_real_umi_obs_dict."""
        buf = list(obs_ring)  # oldest → newest
        return {
            # camera0 = right arm GoPro, camera1 = left arm GoPro
            'camera0_rgb':               np.stack([e[0] for e in buf]),   # [T,H,W,3]
            'camera1_rgb':               np.stack([e[1] for e in buf]),   # [T,H,W,3]
            # robot0 = arm_right
            'robot0_eef_pos':            np.stack([e[2] for e in buf]),   # [T,3]
            'robot0_eef_rot_axis_angle': np.stack([e[3] for e in buf]),   # [T,3]
            'robot0_gripper_width':      np.array([[e[4]] for e in buf]), # [T,1]
            # robot1 = arm_left
            'robot1_eef_pos':            np.stack([e[5] for e in buf]),   # [T,3]
            'robot1_eef_rot_axis_angle': np.stack([e[6] for e in buf]),   # [T,3]
            'robot1_gripper_width':      np.array([[e[7]] for e in buf]), # [T,1]
        }

    # Prime buffer with T_OBS copies of the initial observation
    init_snap = snap()
    for _ in range(T_OBS):
        obs_ring.append(init_snap)

    # Episode start pose (used by get_real_umi_obs_dict for relative obs)
    env_obs_init = build_env_obs()
    episode_start_pose = [
        np.concatenate([env_obs_init['robot0_eef_pos'][-1],
                        env_obs_init['robot0_eef_rot_axis_angle'][-1]]),
        np.concatenate([env_obs_init['robot1_eef_pos'][-1],
                        env_obs_init['robot1_eef_rot_axis_angle'][-1]]),
    ]

    # ── Inference loop ────────────────────────────────────────────────────────
    print(f'Running at {args.frequency} Hz, {args.steps_per_inference} steps/inference.')
    print('Press Ctrl+C to stop.')

    try:
        while True:
            t_loop = time.monotonic()

            # Collect latest observation
            obs_ring.append(snap())
            env_obs = build_env_obs()

            # Build obs dict and run policy
            with torch.no_grad():
                policy.reset()
                obs_dict_np = get_real_umi_obs_dict(
                    env_obs=env_obs,
                    shape_meta=shape_meta,
                    obs_pose_repr=obs_pose_repr,
                    tx_robot1_robot0=TX_ROBOT1_ROBOT0,
                    episode_start_pose=episode_start_pose,
                )
                obs_dict = dict_apply(
                    obs_dict_np,
                    lambda x: torch.from_numpy(x).unsqueeze(0).to(args.device))

                result = policy.predict_action(obs_dict)
                # shape: [1, action_horizon, 10 * n_robots]
                action_seq = result['action_pred'][0].detach().cpu().numpy()

            # Convert to absolute EEF poses: [action_horizon, 7 * n_robots]
            action_seq = get_real_umi_action(action_seq, env_obs, action_pose_repr)

            # Execute steps_per_inference actions
            n_exec = min(args.steps_per_inference, action_seq.shape[0])
            for step_i in range(n_exec):
                t_step = time.monotonic()
                action = action_seq[step_i]  # [14]

                # robot0 = arm_right (action[:7]), robot1 = arm_left (action[7:14])
                x0, y0, z0, r0, p0, yaw0, g0 = action_to_euler(action[:7])
                x1, y1, z1, r1, p1, yaw1, g1 = action_to_euler(action[7:14])

                arm_right.move(x0, y0, z0, r0, p0, yaw0, moving_time=dt)
                arm_left.move( x1, y1, z1, r1, p1, yaw1, moving_time=dt)
                arm_right.set_gripper(g0, moving_time=dt)
                arm_left.set_gripper( g1, moving_time=dt)

                # Update obs after each executed step
                obs_ring.append(snap())

                # Pace to dt
                elapsed = time.monotonic() - t_step
                if elapsed < dt:
                    time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
