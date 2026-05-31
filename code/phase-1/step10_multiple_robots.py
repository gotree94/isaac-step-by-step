"""
Step 10 — Multiple Robots & Coordination
==========================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step10_multiple_robots.py

목표:
    1. 4대 TurtleBot3를 십자형으로 배치
    2. ArticulationView로 일괄 제어
    3. 거리 기반 충돌 회피
    4. Phase 1 전체 복습 통합
"""

import numpy as np

CONFIG = {"width": 1280, "height": 720, "headless": False, "renderer": "RayTracedLighting"}
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 10 — Multiple Robots & Coordination")
print("Phase 1 Finale")
print("=" * 60)

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView

# ── World ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── Differential Drive 파라미터 ──
WHEEL_DIST = 0.141
WHEEL_RADIUS = 0.033

def wheel_speeds(linear_x=0.0, angular_z=0.0):
    v_l = linear_x - angular_z * WHEEL_DIST / 2
    v_r = linear_x + angular_z * WHEEL_DIST / 2
    return np.array([v_l / WHEEL_RADIUS, v_r / WHEEL_RADIUS])

# ── 4대 TurtleBot3 배치 (십자형) ──
print("\n[1/4] Deploying 4 TurtleBot3 in cross formation...")

ROBOT_USD = "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd"
robot_configs = [
    {"path": "/World/TB3_0", "pos": [0.0, 2.0, 0.1], "yaw": np.pi},     # 하단 → 중앙
    {"path": "/World/TB3_1", "pos": [-2.0, 0.0, 0.1], "yaw": np.pi/2},  # 좌측 → 중앙
    {"path": "/World/TB3_2", "pos": [2.0, 0.0, 0.1], "yaw": -np.pi/2},  # 우측 → 중앙
    {"path": "/World/TB3_3", "pos": [0.0, -2.0, 0.1], "yaw": 0.0},      # 상단 → 중앙
]

robots = []
for cfg in robot_configs:
    add_reference_to_stage(ROBOT_USD, cfg["path"])
    robot = Robot(prim_path=cfg["path"], position=np.array(cfg["pos"]))
    robot.set_world_pose(position=np.array(cfg["pos"]),
                         orientation=[np.cos(cfg["yaw"]/2), 0, 0, np.sin(cfg["yaw"]/2)])
    world.scene.add(robot)
    robots.append(robot)
    print(f"  + {cfg['path']} at {cfg['pos']}")

# ── ArticulationView ──
print("\n[2/4] Creating ArticulationView for batch control...")
robot_view = ArticulationView(
    prim_paths_expr="/World/TB3_[0-3]", name="TB3_View",
)
print(f"  View created with {robot_view.num_dof} DOFs per robot")

# ── 충돌 회피 클래스 ──
class SimpleCollisionAvoidance:
    def __init__(self, min_distance=0.5):
        self.min_distance = min_distance
    
    def get_commands(self, robots):
        """모든 로봇에 대한 회피 명령 생성"""
        commands = []
        for i, robot in enumerate(robots):
            pos, _ = robot.get_world_pose()
            closest_dist = float('inf')
            closest_pos = None
            
            for j, other in enumerate(robots):
                if i == j:
                    continue
                other_pos, _ = other.get_world_pose()
                dist = np.linalg.norm(pos[:2] - other_pos[:2])
                if dist < closest_dist:
                    closest_dist = dist
                    closest_pos = other_pos
            
            if closest_dist < self.min_distance and closest_pos is not None:
                angle = np.arctan2(pos[1] - closest_pos[1], pos[0] - closest_pos[0])
                target_angle = angle - pos[2]  # 로봇 기준 회피 방향
                commands.append((0.08, np.clip(target_angle * 0.5, -0.5, 0.5)))
            else:
                commands.append((0.12, 0.0))
        
        return commands

avoidance = SimpleCollisionAvoidance(min_distance=0.6)

# ── 시뮬레이션 루프 ──
print("\n[3/4] Running multi-robot simulation with collision avoidance...")

FRAMES = 400
for i in range(FRAMES):
    world.step(render=True)
    
    commands = avoidance.get_commands(robots)
    
    # 각 로봇 개별 제어
    for idx, robot in enumerate(robots):
        controller = robot.get_articulation_controller()
        linear_x, angular_z = commands[idx]
        speeds = wheel_speeds(linear_x, angular_z)
        controller.apply_action(
            ArticulationAction(joint_velocities=speeds, joint_indices=[0, 1])
        )
    
    if i % 40 == 0:
        poses = [r.get_world_pose()[0][:2] for r in robots]
        dists = [np.linalg.norm(poses[0] - poses[j]) for j in range(1, 4)]
        print(f"  Frame {i:3d}: Distances from TB3_0: {[f'{d:.3f}' for d in dists]}")

# ── 최종 결과 ──
print("\n[4/4] Final robot positions:")
for idx, robot in enumerate(robots):
    pos, _ = robot.get_world_pose()
    print(f"  TB3_{idx}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

print(f"\n{'='*60}")
print("Phase 1 Complete! 🎉")
print("All 10 Foundation Steps (docs + scripts) finished.")
print(f"{'='*60}")
print()
print("Files created in Phase 1:")
print("  docs/01-phase-1-foundation/")
print("    ├── 01-step-installation.md")
print("    ├── 02-step-gui-basics.md")
print("    ├── 03-step-omnigraph.md")
print("    ├── 04-step-usd-stage.md")
print("    ├── 05-step-python-scripting.md")
print("    ├── 06-step-hello-robot.md")
print("    ├── 07-step-controller.md")
print("    ├── 08-step-manipulator.md")
print("    ├── 09-step-sensors.md")
("    └── 10-step-multiple-robots.md")
print()
print("  code/phase-1/")
print("    ├── step01_verify_installation.py")
print("    ├── step02_gui_basics.py")
print("    ├── step03_omnigraph.py")
print("    ├── step04_usd_stage.py")
print("    ├── step05_python_scripting.py")
print("    ├── step06_hello_robot.py")
print("    ├── step07_turtlebot_control.py")
print("    ├── step08_franka_control.py")
print("    ├── step09_sensors.py")
("    └── step10_multiple_robots.py")

simulation_app.close()
