"""
Single RX150 Isaac Sim — Setup Script
======================================
Run this ONCE in Script Editor after opening a fresh Isaac Sim stage.

Workflow:
  1. source ~/isaaclab/bin/activate && isaacsim
  2. Window > Script Editor > File > Open > this file > Run
  3. Hit PLAY
  4. python3 /var/isl_robotics_shared/dual-rx150-demo-sparse/arm_move_scripts/rx150_single_sequence.py
     (sim mode, no --real)

  OR with physical arm:
  4. python3 rx150_single_sequence.py --real
     (Isaac Sim mirrors the real arm's joint states)

Pipeline:
  rx150_single_sequence.py (Interbotix SDK)
      ↓
  xs_sdk_sim → /rx150/joint_states (sim)
  OR xs_sdk   → /rx150/joint_states (real hardware)
      ↓
  Isaac Sim MirrorBridge (subscribes)
      ↓
  USD joint drives (Isaac Sim physics)
"""

import omni.kit.commands
import omni.usd
import omni.kit.app
import asyncio
import builtins
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

stage = omni.usd.get_context().get_stage()
RX150_USD = "/var/isl_robotics_shared/dual-rx150-demo-sparse/rx150_usd/rx150.usd"
JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
ARM_PATH = "/World/rx150"

# ── 1. Load arm ───────────────────────────────────────────────────────────────
print("Loading rx150...")
omni.kit.commands.execute("CreateReference",
    usd_context=omni.usd.get_context(),
    path_to=Sdf.Path(ARM_PATH),
    asset_path=RX150_USD, instanceable=False)
xf = UsdGeom.Xformable(stage.GetPrimAtPath(ARM_PATH))
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
print("  rx150 placed at origin")

# ── 2. Physics scene ──────────────────────────────────────────────────────────
physics_scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath('/World/PhysicsScene'))
print("Physics scene created")

# ── 3. Ground plane ───────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Plane', prim_path='/World/GroundPlane')
ground = stage.GetPrimAtPath('/World/GroundPlane')
ground.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.0, 0.0, 0.0))
ground.GetAttribute('xformOp:scale').Set(Gf.Vec3f(5.0, 5.0, 1.0))
UsdPhysics.CollisionAPI.Apply(ground)

# ── 4. Table ──────────────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Cube', prim_path='/World/Table')
table = stage.GetPrimAtPath('/World/Table')
table.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.15, 0.0, 0.015))
table.GetAttribute('xformOp:scale').Set(Gf.Vec3f(0.12, 0.20, 0.03))
UsdPhysics.CollisionAPI.Apply(table)

# ── 5. Target cube ────────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Cube', prim_path='/World/TargetCube')
cube = stage.GetPrimAtPath('/World/TargetCube')
cube.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.15, 0.0, 0.04))
cube.GetAttribute('xformOp:scale').Set(Gf.Vec3f(0.02, 0.02, 0.02))
UsdPhysics.RigidBodyAPI.Apply(cube)
UsdPhysics.CollisionAPI.Apply(cube)
UsdPhysics.MassAPI.Apply(cube).CreateMassAttr().Set(0.1)
print("Ground, table, cube added")

# ── 6. Joint drives ───────────────────────────────────────────────────────────
def apply_drives(arm_path):
    for jname in JOINT_NAMES:
        path = f"{arm_path}/joints/{jname}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  WARNING: joint not found at {path}")
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(10000.0)
        drive.CreateStiffnessAttr(10000.0)
        drive.CreateDampingAttr(500.0)
        drive.CreateTargetPositionAttr().Set(0.0)
    print(f"  drives applied for {arm_path}")

apply_drives(ARM_PATH)

# ── 7. ROS2 Mirror Bridge ─────────────────────────────────────────────────────
import rclpy
import rclpy.node
from sensor_msgs.msg import JointState

def set_joint_cmd(jname, angle_rad):
    prim = stage.GetPrimAtPath(f"{ARM_PATH}/joints/{jname}")
    if not prim.IsValid():
        return
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.GetTargetPositionAttr().Set(float(angle_rad * 57.2958))

class SingleArmBridge(rclpy.node.Node):
    def __init__(self):
        super().__init__('isaac_single_arm_bridge')

        # Mirror xs_sdk (sim or real) joint_states → Isaac Sim drives
        self.create_subscription(JointState,
            '/rx150/joint_states',
            self.mirror_cb, 10)

        self.get_logger().info('SingleArmBridge running')
        self.get_logger().info('  Mirroring /rx150/joint_states → Isaac Sim')

    def mirror_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                set_joint_cmd(name, pos)

# Clean up any existing bridge node
if hasattr(builtins, '_isaac_bridge_node'):
    try:
        builtins._isaac_bridge_node.destroy_node()
    except Exception:
        pass

if not rclpy.ok():
    rclpy.init()

node = SingleArmBridge()
builtins._isaac_bridge_node = node

async def spin_bridge():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await omni.kit.app.get_app().next_update_async()

asyncio.ensure_future(spin_bridge())

print("=" * 50)
print("SETUP COMPLETE")
print("1. Hit PLAY in Isaac Sim")
print("2. python3 /var/isl_robotics_shared/dual-rx150-demo-sparse/arm_move_scripts/rx150_single_sequence.py")
print("   (add --real if physical arm is connected)")
print("=" * 50)
