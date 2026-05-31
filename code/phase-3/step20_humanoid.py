"""
Step 20 — Humanoid Robot in Isaac Sim
========================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step20_humanoid.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. GR1 또는 H1 USD asset (Isaac Sim 내장)

목표:
    1. Humanoid 로봇 로딩 및 DOF 확인
    2. Full-Body Task Space Control
    3. Walking Gait 생성 (Open-Loop)
    4. ROS2 Joint State 발행
    5. T-Pose → Standing → Walking 전이
"""

# ── 1. SimulationApp 초기화 ──
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 20 — Humanoid Robot")
print("=" * 60)

# ── 2. Core API 임포트 ──
import time
import numpy as np
from pxr import Sdf, Gf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. Humanoid 로딩 ──
print("\n[1/6] Loading Humanoid robot...")

HUMANOID_PATH = "/World/Humanoid"

# GR1 / H1 등 내장 Humanoid 시도
humanoid_usd_paths = [
    "/Isaac/Robots/GR1/gr1.usd",
    "/Isaac/Robots/GR1/gr1_alt_fingers.usd",
    "/Isaac/Robots/H1/h1.usd",
    "/Isaac/Robots/H1/h1_alt_fingers.usd",
]

loaded = False
for usd_path in humanoid_usd_paths:
    try:
        if not is_prim_path_valid(HUMANOID_PATH):
            add_reference_to_stage(usd_path, HUMANOID_PATH)
        print(f"  + Humanoid loaded from: {usd_path}")
        loaded = True
        break
    except Exception:
        continue

if not loaded:
    print("  ⚠ No built-in humanoid found. Creating simple humanoid proxy.")
    # Fallback: 간단한 사체형 생성
    for i, (name, pos, scale) in enumerate([
        ("Torso", (0, 0, 0.6), (0.3, 0.2, 0.4)),
        ("Head", (0, 0, 1.0), (0.15, 0.15, 0.15)),
        ("L_Arm", (-0.2, 0, 0.7), (0.05, 0.05, 0.3)),
        ("R_Arm", (0.2, 0, 0.7), (0.05, 0.05, 0.3)),
        ("L_Leg", (-0.1, 0, 0.25), (0.06, 0.06, 0.4)),
        ("R_Leg", (0.1, 0, 0.25), (0.06, 0.06, 0.4)),
    ]):
        VisualCuboid(
            prim_path=f"/World/Humanoid/{name}",
            name=name, position=np.array(pos),
            scale=np.array(scale),
            color=np.array([0.3, 0.6, 0.9]),
        )

humanoid = Robot(
    prim_path=HUMANOID_PATH,
    name="Humanoid",
    position=np.array([0.0, 0.0, 0.1]),
)
world.scene.add(humanoid)

# DOF 정보
print(f"\n  DOF count: {humanoid.num_dof}")
dof_names = humanoid.dof_names
for i, name in enumerate(dof_names):
    print(f"    {i:2d}: {name}")

# ── 5. Task Space Controller ──
print("\n[2/6] Setting up Task Space Controller...")

class HumanoidTaskController:
    def __init__(self, robot):
        self.robot = robot
        self.dof_names = robot.dof_names
        self.dof_count = robot.num_dof

        # 관절 그룹화
        self.leg_joints = {
            'left': [i for i, n in enumerate(self.dof_names)
                     if any(k in n for k in ['l_hip', 'l_knee', 'l_ankle'])],
            'right': [i for i, n in enumerate(self.dof_names)
                      if any(k in n for k in ['r_hip', 'r_knee', 'r_ankle'])],
        }
        self.arm_joints = {
            'left': [i for i, n in enumerate(self.dof_names)
                     if any(k in n for k in ['l_shoulder', 'l_elbow', 'l_wrist'])],
            'right': [i for i, n in enumerate(self.dof_names)
                      if any(k in n for k in ['r_shoulder', 'r_elbow', 'r_wrist'])],
        }

        print(f"  Left leg joints:  {len(self.leg_joints['left'])}")
        print(f"  Right leg joints: {len(self.leg_joints['right'])}")
        print(f"  Left arm joints:  {len(self.arm_joints['left'])}")
        print(f"  Right arm joints: {len(self.arm_joints['right'])}")

    def apply_t_pose(self):
        """T-Pose 설정"""
        pose = np.zeros(self.dof_count)
        for idx in self.arm_joints['left']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.3
            elif 'shoulder_roll' in name: pose[idx] = 0.2
        for idx in self.arm_joints['right']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.3
            elif 'shoulder_roll' in name: pose[idx] = -0.2
        self.robot.set_joint_positions(pose)
        return pose

    def apply_standing_pose(self):
        """Standing Pose (약간 무릎 굽힘)"""
        pose = self.apply_t_pose()
        for group in [self.leg_joints['left'], self.leg_joints['right']]:
            for idx in group:
                name = self.dof_names[idx]
                if 'knee' in name: pose[idx] = 0.15
                elif 'ankle_pitch' in name: pose[idx] = 0.05
        self.robot.set_joint_positions(pose)
        return pose

    def apply_walking_pose(self, phase=0.0):
        """보행 자세 (phase: 0.0~1.0)"""
        pose = self.apply_t_pose()
        swing = np.sin(phase * 2 * np.pi)
        
        for idx in self.leg_joints['left']:
            name = self.dof_names[idx]
            if 'hip_pitch' in name: pose[idx] = -0.15 * swing
            elif 'knee' in name: pose[idx] = 0.15 + 0.15 * max(0, swing)
            elif 'ankle_pitch' in name: pose[idx] = 0.05 * swing
        for idx in self.leg_joints['right']:
            name = self.dof_names[idx]
            if 'hip_pitch' in name: pose[idx] = 0.15 * swing
            elif 'knee' in name: pose[idx] = 0.15 + 0.15 * max(0, -swing)
            elif 'ankle_pitch' in name: pose[idx] = -0.05 * swing
        for idx in self.arm_joints['left']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.3 + 0.1 * swing
        for idx in self.arm_joints['right']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.3 - 0.1 * swing
        
        self.robot.set_joint_positions(pose)
        return pose

controller = HumanoidTaskController(humanoid)

# ── 6. ROS2 Bridge (Joint State + TF) ──
print("\n[3/6] Setting up ROS2 Bridge for Humanoid...")

graph_config = {
    "graph_path": "/ActionGraph/Humanoid_Bridge",
    "evaluator_name": "execution",
}

stage = omni.usd.get_context().get_stage()
if og.Controller.graph_exists("/ActionGraph/Humanoid_Bridge"):
    stage.RemovePrim(Sdf.Path("/ActionGraph/Humanoid_Bridge"))

og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
            ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "ReadJoint.inputs:execIn"),
            ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
            ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
            ("Context.outputs:context", "PubJoint.inputs:context"),
            ("Context.outputs:context", "PubTF.inputs:context"),
            ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
            ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
            ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:domain_id", 0),
            ("Context.inputs:namespace", "/humanoid"),
            ("ReadJoint.inputs:robotPrim", Sdf.Path(HUMANOID_PATH)),
            ("PubJoint.inputs:topicName", "/humanoid/joint_states"),
        ],
    },
)
print("  + ROS2 Bridge created for /humanoid/joint_states")

# ── 7. 시뮬레이션 루프 ──
print("\n[4/6] Running Humanoid simulation...")
print()
print("  Pose sequence: T-Pose → Standing → Walking")
print()

phase = 0.0
for i in range(1200):
    world.step(render=True)

    if i < 100:
        # Phase 1: T-Pose
        if i == 1:
            controller.apply_t_pose()
        if i % 50 == 0:
            print(f"  [{i:4d}] T-Pose")
    
    elif i < 300:
        # Phase 2: Standing
        if i == 100:
            controller.apply_standing_pose()
        if i % 50 == 0:
            print(f"  [{i:4d}] Standing")
    
    else:
        # Phase 3: Walking
        phase += 1/60.0 * 1.5
        if phase >= 1.0:
            phase -= 1.0
        controller.apply_walking_pose(phase)
        if i % 50 == 0:
            pos, _ = humanoid.get_world_pose()
            print(f"  [{i:4d}] Walking: pos=({pos[0]:.2f}, {pos[1]:.2f})")

# ── 8. 결과 확인 ──
print("\n[5/6] Verifying bridge...")
if og.Controller.graph_exists("/ActionGraph/Humanoid_Bridge"):
    g = og.Graph("/ActionGraph/Humanoid_Bridge")
    print(f"  + Graph active: {len(g.get_nodes())} nodes")

# ── 9. 요약 ──
print("\n[6/6] Step 20 Summary")
print("=" * 60)
print()
print("  Humanoid Robot Control:")
print()
print("  Pose Transitions:")
print("    Initial → T-Pose → Standing → Walking")
print()
print(f"  DOF: {humanoid.num_dof} joints")
print(f"  Leg joints: L={len(controller.leg_joints['left'])}, "
      f"R={len(controller.leg_joints['right'])}")
print(f"  Arm joints: L={len(controller.arm_joints['left'])}, "
      f"R={len(controller.arm_joints['right'])}")
print()
print("  ROS2 Topics:")
print("    /humanoid/joint_states")
print("    /tf")
print()
print("  Key concepts:")
print("  - Task Space Control: group joints by function")
print("  - Walking Gait: sinusoidal joint trajectories")
print("  - Phase-based animation: 0.0→1.0 gait cycle")
print("  - ZMP: Center of Mass control for stability")
print("  - Motion Retargeting: human→robot mapping")
print("=" * 60)

simulation_app.close()
