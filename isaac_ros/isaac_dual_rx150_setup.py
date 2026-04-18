"""
Dual RX150 Isaac Sim — Complete Setup Script
Run this ONCE in Script Editor after opening a fresh Isaac Sim stage.
"""

import omni.kit.commands
import omni.usd
import omni.kit.app
import asyncio
import builtins
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

stage = omni.usd.get_context().get_stage()
RX150_USD = "/var/isl_robotics_shared/dual-rx150-demo-sparse/rx150_usd/rx150.usd"

# ── 1. Place arm_left ──────────────────────────────────────────────────────────
print("Loading arm_left...")
omni.kit.commands.execute("CreateReference",
    usd_context=omni.usd.get_context(),
    path_to=Sdf.Path("/World/arm_left"),
    asset_path=RX150_USD, instanceable=False)
xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/arm_left"))
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, +0.3273, 0.0))
xf.AddRotateZOp().Set(0.0)
print("  arm_left at (0,+0.3273, 0) yaw=0 (faces +X)")

# ── 2. Place arm_right ─────────────────────────────────────────────────────────
print("Loading arm_right...")
omni.kit.commands.execute("CreateReference",
    usd_context=omni.usd.get_context(),
    path_to=Sdf.Path("/World/arm_right"),
    asset_path=RX150_USD, instanceable=False)
xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/arm_right"))
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.3273, 0.0))
xf.AddRotateZOp().Set(0.0)
print("  arm_right at (0, -0.3273, 0) yaw=0 (faces +X)")

# ── 3. Physics scene ───────────────────────────────────────────────────────────
physics_scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath('/World/PhysicsScene'))
print("Physics scene created")

# ── 4. Ground plane ────────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Plane', prim_path='/World/GroundPlane')
ground = stage.GetPrimAtPath('/World/GroundPlane')
ground.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.0, 0.0, 0.0))
ground.GetAttribute('xformOp:scale').Set(Gf.Vec3f(5.0, 5.0, 1.0))
UsdPhysics.CollisionAPI.Apply(ground)

# ── 5. Table ───────────────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Cube', prim_path='/World/Table')
table = stage.GetPrimAtPath('/World/Table')
table.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.15, 0.0, 0.015))
table.GetAttribute('xformOp:scale').Set(Gf.Vec3f(0.12, 0.20, 0.03))
UsdPhysics.CollisionAPI.Apply(table)

# ── 6. Target cube ─────────────────────────────────────────────────────────────
omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
    prim_type='Cube', prim_path='/World/TargetCube')
cube = stage.GetPrimAtPath('/World/TargetCube')
cube.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.15, -0.08, 0.04))
cube.GetAttribute('xformOp:scale').Set(Gf.Vec3f(0.02, 0.02, 0.02))
UsdPhysics.RigidBodyAPI.Apply(cube)
UsdPhysics.CollisionAPI.Apply(cube)
UsdPhysics.MassAPI.Apply(cube).CreateMassAttr().Set(0.1)
print("Ground, table, cube added")

# ── 7. Joint drives ────────────────────────────────────────────────────────────
def apply_drives(arm):
    for jname in ['waist','shoulder','elbow','wrist_angle','wrist_rotate']:
        path = f"/World/{arm}/joints/{jname}"
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
    print(f"  {arm} drives applied")

apply_drives('arm_left')
apply_drives('arm_right')

# ── 8. ROS2 Mirror Bridge ─────────────────────────────────────────────────────
# Mirrors xs_sdk joint_states (sim or real) → Isaac Sim USD drives.
# No separate bridge script needed — same pattern as isaac_single_rx150_setup.py
import rclpy
import rclpy.node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']

def set_joint_cmd(arm, jname, angle_rad):
    prim = stage.GetPrimAtPath(f"/World/{arm}/joints/{jname}")
    if not prim.IsValid():
        return
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.GetTargetPositionAttr().Set(float(angle_rad * 57.2958))

class DualMirrorBridge(rclpy.node.Node):
    def __init__(self):
        super().__init__('isaac_dual_mirror_bridge')
        self.create_subscription(JointState, '/arm_left/joint_states',
            lambda msg: self.mirror(msg, 'arm_left'),  10)
        self.create_subscription(JointState, '/arm_right/joint_states',
            lambda msg: self.mirror(msg, 'arm_right'), 10)
        self.get_logger().info('DualMirrorBridge running')
        self.get_logger().info('  Mirroring /arm_left/joint_states  → Isaac Sim')
        self.get_logger().info('  Mirroring /arm_right/joint_states → Isaac Sim')

    def mirror(self, msg, arm):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                set_joint_cmd(arm, name, pos)

if not rclpy.ok():
    rclpy.init()
if hasattr(builtins, '_isaac_bridge_node'):
    try: builtins._isaac_bridge_node.destroy_node()
    except: pass

node = DualMirrorBridge()
builtins._isaac_bridge_node = node

async def spin_bridge():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await omni.kit.app.get_app().next_update_async()

asyncio.ensure_future(spin_bridge())

print("=" * 50)
print("SETUP COMPLETE — mirror bridge running")
print("1. Hit PLAY in Isaac Sim")
print("2. python3 /var/isl_robotics_shared/dual-rx150-demo-sparse/arm_move_scripts/rx150_dual_sequence.py")
print("   (add --real if both physical arms are connected)")
print("=" * 50)
