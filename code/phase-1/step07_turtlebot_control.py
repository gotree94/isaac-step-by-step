"""
Step 07 — TurtleBot3 제어하기 (Controller 기초)
================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step07_turtlebot_control.py

목표:
    1. ArticulationController로 바퀴 속도 명령
    2. Differential Drive 속도 변환
    3. 사각형 Trajectory 주행
    4. Odometry vs Ground Truth 비교
"""

import numpy as np

CONFIG = {"width": 1280, "height": 720, "headless": False, "renderer": "RayTracedLighting"}
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 07 — TurtleBot3 Control Demo")
print("=" * 60)

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# World 생성
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# TurtleBot3 로딩
ROBOT_PATH = "/World/TurtleBot3"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage("/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd", ROBOT_PATH)

robot = Robot(prim_path=ROBOT_PATH, name="TurtleBot3", position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)
controller = robot.get_articulation_controller()

print(f"DOFs: {robot.dof_names}")

# ──────────────────────────────────────────────
# Differential Drive 속도 변환
# ──────────────────────────────────────────────
WHEEL_DIST = 0.141
WHEEL_RADIUS = 0.033

def wheel_speeds(linear_x=0.0, angular_z=0.0):
    v_l = linear_x - angular_z * WHEEL_DIST / 2
    v_r = linear_x + angular_z * WHEEL_DIST / 2
    return np.array([v_l / WHEEL_RADIUS, v_r / WHEEL_RADIUS])

# ──────────────────────────────────────────────
# Odometry
# ──────────────────────────────────────────────
class Odometry:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
    def update(self, left_vel, right_vel, dt):
        v_l = left_vel * WHEEL_RADIUS
        v_r = right_vel * WHEEL_RADIUS
        v = (v_l + v_r) / 2
        omega = (v_r - v_l) / WHEEL_DIST
        self.theta += omega * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        return self.x, self.y, self.theta

odo = Odometry()

# ──────────────────────────────────────────────
# 사각형 Trajectory 주행
# ──────────────────────────────────────────────
print("\nSquare trajectory: drive 2m → turn 90° → ...\n")

waypoints = [
    (0.2, 0.0, 2.0),    # 직진 2m
    (0.0, 0.5, 1.57),   # 90도 회전 (≈π/2 rad)
    (0.2, 0.0, 2.0),
    (0.0, 0.5, 1.57),
    (0.2, 0.0, 2.0),
    (0.0, 0.5, 1.57),
    (0.2, 0.0, 2.0),
    (0.0, 0.5, 1.57),   # 원점 복귀
]

frame = 0
true_positions = []
odo_positions = []

for linear_x, angular_z, duration in waypoints:
    speeds = wheel_speeds(linear_x, angular_z)
    steps = int(duration * 60)
    for _ in range(steps):
        world.step(render=True)
        controller.apply_action(ArticulationAction(joint_velocities=speeds, joint_indices=[0, 1]))

        if frame % 20 == 0:
            true_pos, _ = robot.get_world_pose()
            vel = robot.get_joint_velocities()
            odo.update(vel[0], vel[1], 1/60.0)
            true_positions.append(true_pos[:2].copy())
            odo_positions.append([odo.x, odo.y])
            err = np.linalg.norm(true_pos[:2] - [odo.x, odo.y])
            print(f"  Frame {frame:4d}: True=({true_pos[0]:.3f},{true_pos[1]:.3f}) "
                  f"Odo=({odo.x:.3f},{odo.y:.3f}) err={err:.4f}m")
        frame += 1

# 정지
controller.apply_action(ArticulationAction(joint_velocities=[0, 0]))
for _ in range(10):
    world.step(render=True)

# ──────────────────────────────────────────────
# 최종 결과
# ──────────────────────────────────────────────
true_pos, true_orient = robot.get_world_pose()
print(f"\n{'='*60}")
print(f"Final position (Ground Truth): ({true_pos[0]:.3f}, {true_pos[1]:.3f})")
print(f"Final odometry (Estimated)  : ({odo.x:.3f}, {odo.y:.3f})")
print(f"Final error                 : {np.linalg.norm(true_pos[:2] - [odo.x, odo.y]):.4f}m")
print(f"{'='*60}")

simulation_app.close()
