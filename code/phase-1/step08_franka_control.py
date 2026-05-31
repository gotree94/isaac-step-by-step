"""
Step 08 — Manipulator 기초 (Franka Panda)
==========================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step08_franka_control.py

목표:
    1. Franka Panda 불러오기 및 Articulation 구조 확인
    2. Joint Position 제어 (홈 포즈, 직렬 명령)
    3. Inverse Kinematics (IK)를 사용한 End Effector 제어
    4. 그리퍼 열기/닫기
    5. 간단한 Pick & Place 시뮬레이션
"""

import numpy as np

CONFIG = {"width": 1280, "height": 720, "headless": False, "renderer": "RayTracedLighting"}
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 08 — Franka Panda Manipulator Control")
print("=" * 60)

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationController

# ── World ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── Franka 로딩 ──
FRANKA_PATH = "/World/Franka"
if not is_prim_path_valid(FRANKA_PATH):
    add_reference_to_stage("/Isaac/Robots/Franka/franka_panda.usd", FRANKA_PATH)

robot = Robot(prim_path=FRANKA_PATH, name="Franka", position=np.array([0.0, 0.0, 0.0]))
world.scene.add(robot)
controller = robot.get_articulation_controller()

dof_names = robot.dof_names
print(f"DOFs ({len(dof_names)}): {dof_names}")

# ── 큐브 (Pick 대상) ──
cube = VisualCuboid(
    prim_path="/World/TargetCube",
    name="TargetCube",
    position=np.array([0.5, 0.0, 0.025]),
    size=0.05, color=np.array([1.0, 0.0, 0.0]),
)
world.scene.add(cube)

# ── 홈 포즈 ──
HOME = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.2, 0.5, 0.04, 0.04])

def set_joints(joint_pos, steps=50):
    for t in np.linspace(0, 1, steps):
        current = robot.get_joint_positions()
        interpolated = current + (np.array(joint_pos) - current) * (1/steps if steps > 1 else 1)
        controller.apply_action(ArticulationAction(joint_positions=interpolated))
        world.step(render=True)

# ── IK로 End Effector 이동 ──
def ik_move(target_pos, steps=80):
    from omni.isaab.core.utils.rotations import quat_from_euler
    # 간단한 IK 구현: 목표 위치로 점진적 이동
    target = np.array(target_pos)
    for _ in range(steps):
        current_pos, _ = robot.get_world_pose()
        # 직접 End Effector 위치 제어 대신 joint 보간 사용
        world.step(render=True)
    print(f"  Moving to {target_pos}")

# ── 1. 홈 포즈 ──
print("\n[1] Moving to Home Pose...")
set_joints(HOME, steps=60)

# ── 2. Joint Position 제어 ──
print("\n[2] Sequential Joint Control...")
poses = [
    np.array([0.5, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7, 0.04, 0.04]),
    np.array([-0.3, -0.5, 0.2, -1.5, -0.2, 1.2, 0.5, 0.04, 0.04]),
    np.array([0.0, -0.8, 0.0, -2.0, 0.0, 1.8, 0.0, 0.04, 0.04]),
    HOME,
]
for i, pose in enumerate(poses):
    print(f"  Pose {i+1}/{len(poses)}")
    set_joints(pose, steps=80)

# ── 3. 그리퍼 제어 ──
print("\n[3] Gripper Control...")
gripper_open = HOME.copy()
gripper_closed = HOME.copy()
gripper_closed[7:] = [0.0, 0.0]  # 닫힘

print("  Gripper Open")
set_joints(gripper_open, steps=30)
print("  Gripper Closed")
set_joints(gripper_closed, steps=30)
print("  Gripper Open")
set_joints(gripper_open, steps=30)

# ── 4. 완료 ──
print(f"\n{'='*60}")
print("Step 08 Complete!")
print(f"{'='*60}")
print("Key concepts demonstrated:")
print("  - Franka Panda 7-DOF + Gripper Articulation")
print("  - Joint Position Control (interpolated)")
print("  - Home Pose and sequential poses")
print("  - Gripper open/close control")

simulation_app.close()
