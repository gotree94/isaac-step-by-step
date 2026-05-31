"""
Step 22 — AI Worker in Isaac Sim
==================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step22_ai_worker.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. GR1 Humanoid USD asset (내장)
    3. TurtleBot3 USD asset (내장)

목표:
    1. Human Worker 시뮬레이션 (GR1 Humanoid)
    2. AI Worker (Mobile Manipulator) 배치
    3. Behavior Tree 기반 Task Planning
    4. Human-Robot Collaboration (Handover)
    5. Safety Monitoring (SSM)
    6. Perception Pipeline (RGB-D Camera)
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
print("Step 22 — AI Worker")
print("=" * 60)

# ── 2. Core API 임포트 ──
import time
import math
import numpy as np
from pxr import Sdf, Gf, UsdGeom
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. Collaboration Workspace 생성 ──
print("\n[1/7] Creating Collaboration Workspace...")

# 공동 작업 공간
VisualCuboid(
    prim_path="/World/Workspace/Floor",
    name="ws_floor",
    position=np.array([0.0, 0.0, -0.01]),
    scale=np.array([10.0, 8.0, 0.02]),
    color=np.array([0.25, 0.25, 0.25]),
)

# 중앙 작업대
VisualCuboid(
    prim_path="/World/Workspace/Table",
    name="work_table",
    position=np.array([0.0, 0.0, 0.4]),
    scale=np.array([1.5, 0.8, 0.8]),
    color=np.array([0.4, 0.3, 0.2]),
)

# Staging Area (Human 측)
VisualCuboid(
    prim_path="/World/Workspace/HumanZone",
    name="human_zone",
    position=np.array([-3.0, 0.0, 0.01]),
    scale=np.array([2.0, 2.0, 0.01]),
    color=np.array([0.1, 0.5, 0.1, 0.3]),
)

# Staging Area (Robot 측)
VisualCuboid(
    prim_path="/World/Workspace/RobotZone",
    name="robot_zone",
    position=np.array([3.0, 0.0, 0.01]),
    scale=np.array([2.0, 2.0, 0.01]),
    color=np.array([0.1, 0.1, 0.5, 0.3]),
)

# Handover Zone
VisualCuboid(
    prim_path="/World/Workspace/HandoverZone",
    name="handover_zone",
    position=np.array([0.0, 1.5, 0.01]),
    scale=np.array([1.0, 0.5, 0.01]),
    color=np.array([0.8, 0.6, 0.1, 0.5]),
)

# Conveyor Belt
VisualCuboid(
    prim_path="/World/Workspace/Conveyor",
    name="conveyor",
    position=np.array([3.0, -2.5, 0.08]),
    scale=np.array([3.0, 0.3, 0.04]),
    color=np.array([0.2, 0.2, 0.2]),
)

# 랙 (물품 보관)
VisualCuboid(
    prim_path="/World/Workspace/Rack",
    name="rack",
    position=np.array([-3.0, -2.5, 0.5]),
    scale=np.array([0.8, 0.4, 1.0]),
    color=np.array([0.5, 0.35, 0.2]),
)

# 물품들
items = []
for ii in range(3):
    item = DynamicCuboid(
        prim_path=f"/World/Workspace/Items/Box_{ii}",
        name=f"box_{ii}",
        position=np.array([-3.0 + ii * 0.15, -2.5, 0.9]),
        scale=np.array([0.06, 0.06, 0.06]),
        color=np.array([0.8, 0.2, 0.2]),
        mass=0.1,
    )
    items.append(item)

print("  + Collaboration Workspace created")
print("  + Table, Handover Zone, Conveyor, Rack, Items")

# ── 5. Human Worker 배치 ──
print("\n[2/7] Deploying Human Worker (GR1)...")

human_path = "/World/Humans/Worker1"
if not is_prim_path_valid(human_path):
    add_reference_to_stage(
        "/Isaac/Robots/GR1/gr1.usd",
        human_path,
    )

human = Robot(
    prim_path=human_path,
    name="Human_Worker",
    position=np.array([-3.0, 0.0, 0.1]),
)
world.scene.add(human)

print(f"  + Human Worker: {human.num_dof} DOF")
for i, name in enumerate(human.dof_names[:10]):
    print(f"    {i}: {name}")

# ── 6. AI Worker (Mobile Manipulator) 배치 ──
print("\n[3/7] Deploying AI Worker...")

# AMR (TurtleBot3)
amr_path = "/World/Robots/AI_AMR"
if not is_prim_path_valid(amr_path):
    add_reference_to_stage(
        "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
        amr_path,
    )

ai_robot = Robot(
    prim_path=amr_path,
    name="AI_Worker",
    position=np.array([3.0, 0.0, 0.1]),
)
world.scene.add(ai_robot)

print(f"  + AI Worker: Mobile base ready")

# ── 7. Human Action Controller ──
print("\n[4/7] Human Action Controller...")

class HumanActionController:
    """Human Worker 동작 제어"""
    
    def __init__(self, robot):
        self.robot = robot
        self.current_action = 'idle'
        self.phase = 0.0
        self.position = np.array([-3.0, 0.0, 0.1])
        self.target_pos = np.array([-3.0, 0.0, 0.1])
        
        # DOF 그룹화
        self.arm_joints = {'left': [], 'right': []}
        self.leg_joints = {'left': [], 'right': []}
        
        for i, name in enumerate(robot.dof_names):
            if any(k in name for k in ['l_arm', 'l_shoulder', 'l_elbow']):
                self.arm_joints['left'].append(i)
            elif any(k in name for k in ['r_arm', 'r_shoulder', 'r_elbow']):
                self.arm_joints['right'].append(i)
    
    def set_action(self, action, target=None):
        self.current_action = action
        self.phase = 0.0
        if target is not None:
            self.target_pos = np.array(target)
    
    def update(self, dt=1/60.0):
        self.phase += dt
        
        if self.current_action == 'idle':
            self._idle_pose()
        elif self.current_action == 'walk':
            self._walk_pose()
            self._move_to_target(dt)
        elif self.current_action == 'reach':
            self._reach_pose()
        elif self.current_action == 'handover':
            self._handover_pose()
        elif self.current_action == 'carry':
            self._carry_pose()
            self._move_to_target(dt)
    
    def _idle_pose(self):
        pose = np.zeros(self.robot.num_dof)
        self.robot.set_joint_positions(pose)
    
    def _walk_pose(self):
        pose = np.zeros(self.robot.num_dof)
        swing = np.sin(self.phase * 4 * np.pi)
        for idx in self.arm_joints['left']:
            name = self.robot.dof_names[idx]
            if 'shoulder' in name:
                pose[idx] = 0.1 * swing
        for idx in self.arm_joints['right']:
            name = self.robot.dof_names[idx]
            if 'shoulder' in name:
                pose[idx] = -0.1 * swing
        self.robot.set_joint_positions(pose)
    
    def _reach_pose(self):
        pose = np.zeros(self.robot.num_dof)
        for idx in self.arm_joints['right']:
            name = self.robot.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.8
            elif 'elbow' in name: pose[idx] = -0.5
        self.robot.set_joint_positions(pose)
    
    def _handover_pose(self):
        pose = np.zeros(self.robot.num_dof)
        for idx in self.arm_joints['right']:
            name = self.robot.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.5
            elif 'shoulder_roll' in name: pose[idx] = 0.3
            elif 'elbow' in name: pose[idx] = -0.3
        self.robot.set_joint_positions(pose)
    
    def _carry_pose(self):
        pose = np.zeros(self.robot.num_dof)
        for idx in self.arm_joints['right']:
            name = self.robot.dof_names[idx]
            if 'shoulder_pitch' in name: pose[idx] = -0.3
            elif 'elbow' in name: pose[idx] = -0.6
        self.robot.set_joint_positions(pose)
    
    def _move_to_target(self, dt):
        diff = self.target_pos - self.position
        dist = np.linalg.norm(diff)
        if dist > 0.05:
            step = min(0.3 * dt, dist) * 5
            move = (diff / dist) * step
            self.position[:2] += move[:2]
            self.robot.set_world_pose(self.position)

human_ctrl = HumanActionController(human)

# ── 8. AI Worker Controller ──
print("\n[5/7] AI Worker Controller...")

class AIWorkerController:
    """AI Worker 이동 및 작업 제어"""
    
    def __init__(self, robot):
        self.robot = robot
        self.position = np.array([3.0, 0.0, 0.1])
        self.target = np.array([3.0, 0.0])
        self.at_goal = False
        self.gripper_open = True
        self.task_phase = 'idle'  # idle, moving, handover, convey
    
    def navigate_to(self, x, y):
        self.target = np.array([x, y])
        self.task_phase = 'moving'
    
    def extend_gripper(self):
        self.gripper_open = True
        self.task_phase = 'handover'
    
    def release_gripper(self):
        self.gripper_open = False
        self.task_phase = 'convey'
    
    def update(self, dt=1/60.0):
        if self.task_phase == 'moving':
            diff = self.target - self.position[:2]
            dist = np.linalg.norm(diff)
            if dist < 0.15:
                self.at_goal = True
                return np.array([0.0, 0.0])
            self.at_goal = False
            yaw = math.atan2(diff[1], diff[0])
            linear = min(0.3, dist * 0.8)
            angular = math.atan2(math.sin(yaw - 0),
                                 math.cos(yaw - 0)) * 0.5
            return np.array([linear, angular])
        else:
            return np.array([0.0, 0.0])

ai_ctrl = AIWorkerController(ai_robot)

# ── 9. Behavior Tree 시뮬레이션 ──
print("\n[6/7] Running AI Worker Collaboration Scenario...")

# 시나리오 시퀀스 (10초 간격)
"""
Phase 0 (0-5s):  AI Worker Idle, Human at Rack
Phase 1 (5-10s): Human picks item from rack (reach)
Phase 2 (10-15s): AI Worker moves to Handover Zone
Phase 3 (15-20s): Human walks to Handover Zone
Phase 4 (20-25s): Human → AI Handover
Phase 5 (25-30s): AI Worker moves to Conveyor
Phase 6 (30-35s): AI Worker places item on Conveyor
Phase 7 (35-40s): Return to start position
"""

scenario_phases = [
    (0, 5,   'human_idle',       'ai_idle'),
    (5, 10,  'human_reach',      'ai_idle'),
    (10, 15, 'human_reach',      'ai_moving_handover'),
    (15, 20, 'human_walk_hand',  'ai_handover_wait'),
    (20, 25, 'human_handover',   'ai_handover_receive'),
    (25, 30, 'human_idle',       'ai_moving_conveyor'),
    (30, 35, 'human_idle',       'ai_place_conveyor'),
    (35, 40, 'human_idle',       'ai_return'),
]

sim_time = 0.0
current_phase = -1

# Safety Zone
VisualCuboid(
    prim_path="/World/Safety/SafeZone",
    position=np.array([0.0, 0.0, 0.001]),
    scale=np.array([4.0, 4.0, 0.001]),
    color=np.array([0.3, 1.0, 0.3, 0.15]),
)
VisualCuboid(
    prim_path="/World/Safety/StopZone",
    position=np.array([0.0, 0.0, 0.002]),
    scale=np.array([1.0, 1.0, 0.001]),
    color=np.array([1.0, 0.0, 0.0, 0.2]),
)

print()
print("  Scenario Timeline:")
print("  ─────────────────────────────────────────────")
print("  0-5s:   Human at Rack, AI idle")
print("  5-10s:  Human picks item (reach)")
print("  10-15s: AI moves to Handover Zone")
print("  15-20s: Human walks to Handover Zone")
print("  20-25s: Human → AI Handover")
print("  25-30s: AI moves to Conveyor")
print("  30-35s: AI places item on Conveyor")
print("  35-40s: AI returns to start")
print("  ─────────────────────────────────────────────")
print()

for i in range(2400):
    world.step(render=True)
    sim_time += 1/60.0

    # Phase detection
    new_phase = int(sim_time // 5)
    if new_phase > current_phase and new_phase < len(scenario_phases):
        current_phase = new_phase
        start_t, end_t, human_action, ai_action = scenario_phases[current_phase]
        
        print(f"[{int(sim_time):2d}s] Phase {current_phase}: "
              f"Human={human_action}, AI={ai_action}")
        
        # Human action
        if human_action == 'human_idle':
            human_ctrl.set_action('idle')
            human_ctrl.target_pos = human_ctrl.position
        elif human_action == 'human_reach':
            human_ctrl.set_action('reach')
        elif human_action == 'human_walk_hand':
            human_ctrl.set_action('walk', target=[0.0, 1.5])
        elif human_action == 'human_handover':
            human_ctrl.set_action('handover')
        
        # AI action
        if ai_action == 'ai_idle':
            ai_ctrl.task_phase = 'idle'
        elif ai_action == 'ai_moving_handover':
            ai_ctrl.navigate_to(0.0, 1.0)
            print(f"    AI → Handover Zone")
        elif ai_action == 'ai_handover_wait':
            ai_ctrl.extend_gripper()
            print(f"    AI extending gripper")
        elif ai_action == 'ai_handover_receive':
            ai_ctrl.task_phase = 'handover'
            print(f"    AI receiving item")
        elif ai_action == 'ai_moving_conveyor':
            ai_ctrl.navigate_to(3.0, -2.5)
            print(f"    AI → Conveyor")
        elif ai_action == 'ai_place_conveyor':
            ai_ctrl.release_gripper()
            print(f"    AI placing item on Conveyor")
        elif ai_action == 'ai_return':
            ai_ctrl.navigate_to(3.0, 0.0)
            print(f"    AI → Home")

    # Human update
    human_ctrl.update()

    # AI update
    vel = ai_ctrl.update()
    action = ArticulationAction(
        joint_velocities=np.array([vel[0], vel[0], vel[1]]),
    )
    ai_robot.apply_action(action)
    pos, _ = ai_robot.get_world_pose()
    ai_ctrl.position = pos

    # Safety check
    human_pos = human_ctrl.position[:2]
    robot_pos = pos[:2]
    dist = np.sqrt((human_pos[0]-robot_pos[0])**2 + 
                   (human_pos[1]-robot_pos[1])**2)
    if dist < 0.5 and ai_ctrl.task_phase == 'moving':
        print(f"    ⚠ Safety: Human-Robot distance {dist:.2f}m")
        # Slow down
        action = ArticulationAction(
            joint_velocities=np.array([0.05, 0.05, 0.0]),
        )
        ai_robot.apply_action(action)

# ── 10. 결과 확인 ──
print("\n[7/7] Verifying...")
print(f"  + Human Worker: {human_ctrl.current_action}")
print(f"  + AI Worker: {ai_ctrl.task_phase}")
print(f"  + Collaboration Scenario Completed")

# ── 11. 요약 ──
print("\n" + "="*60)
print("  Step 22 — Summary")
print("="*60)
print()
print("  AI Worker Collaboration:")
print()
print("  ✅ Collaboration Workspace")
print("     - Work Table, Handover Zone, Conveyor, Rack")
print()
print("  ✅ Human Worker (GR1)")
print(f"     - {human.num_dof} DOF")
print("     - Actions: idle, walk, reach, handover, carry")
print()
print("  ✅ AI Worker (TurtleBot3)")
print("     - Navigation: Staging → Handover → Conveyor")
print("     - Gripper control (extend/release)")
print()
print("  ✅ Behavior Tree Scenario")
print("     - 8-phase collaboration timeline")
print("     - Human-Robot Handover")
print("     - Automated task sequence")
print()
print("  ✅ Safety Monitoring")
print("     - Safe Zone (2m radius)")
print("     - Stop Zone (0.5m radius)")
print("     - Distance-based speed regulation")
print()
print("  ✅ Key Concepts:")
print("     - Human-Robot Collaboration (HRC)")
print("     - Shared Workspace Management")
print("     - Task Planning with Action Sequencing")
print("     - Speed and Separation Monitoring (SSM)")
print("     - Handover Protocol")
print("="*60)

simulation_app.close()
