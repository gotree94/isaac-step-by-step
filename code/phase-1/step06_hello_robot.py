"""
Step 06 — Hello, Robot! 첫 번째 로봇 불러오기
==============================================

실행 방법 (Standalone):
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step06_hello_robot.py

또는 (pip 설치 환경):
    source ~/isaacsim_env/bin/activate
    python ~/isaac-step-curriculum/code/phase-1/step06_hello_robot.py

목표:
    1. TurtleBot3 Waffle USD를 Stage에 Reference로 추가
    2. Robot 객체 생성 및 Articulation 구조 확인
    3. 로봇 관절(Joint) 정보 출력
    4. 간단한 시뮬레이션 실행 (중력 안착)
    5. 여러 로봇을 Scene에 배치
"""

# ──────────────────────────────────────────────
# 1. SimulationApp 초기화 (가장 먼저!)
# ──────────────────────────────────────────────
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 06 — Hello, Robot! (TurtleBot3 Waffle)")
print("=" * 60)

# ──────────────────────────────────────────────
# 2. Core API 임포트
# ──────────────────────────────────────────────
import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path
from omni.isaac.core.utils.stage import (
    get_current_stage,
    add_reference_to_stage,
)
from omni.isaac.core.articulations import Articulation, ArticulationView

# ──────────────────────────────────────────────
# 3. World 생성
# ──────────────────────────────────────────────
print("\n[1/5] Creating World & Ground Plane...")

world = World(
    stage_units_in_meters=1.0,
    physics_dt=1 / 60.0,
    rendering_dt=1 / 60.0,
)
world.scene.add_default_ground_plane()
world.initialize()

stage = get_current_stage()
print("  World initialized.")

# ──────────────────────────────────────────────
# 4. TurtleBot3 불러오기
# ──────────────────────────────────────────────
print("\n[2/5] Loading TurtleBot3 Waffle...")

ROBOT_USD = "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd"
ROBOT_PATH = "/World/TurtleBot3"

# Reference로 USD를 Stage에 추가
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage(
        usd_path=ROBOT_USD,
        prim_path=ROBOT_PATH,
    )
    print(f"  + Added Reference: {ROBOT_USD}")
    print(f"    → {ROBOT_PATH}")
else:
    print(f"  + {ROBOT_PATH} already exists.")

# Robot 객체 생성
robot = Robot(
    prim_path=ROBOT_PATH,
    name="TurtleBot3",
    position=np.array([0.0, 0.0, 0.1]),  # 바닥보다 약간 위
)

# Robot 객체를 World에 등록
world.scene.add(robot)

print(f"  Robot object created: {robot.name}")
print(f"  Position: ({robot.position[0]:.2f}, {robot.position[1]:.2f}, {robot.position[2]:.2f})")

# ──────────────────────────────────────────────
# 5. 로봇 Articulation 구조 분석
# ──────────────────────────────────────────────
print("\n[3/5] Analyzing robot articulation...")

# Articulation 정보
articulation = robot.articulation
dof_names = articulation.dof_names
dof_types = articulation.dof_types
dof_counts = articulation.num_dof

print(f"  DOF (Degrees of Freedom): {dof_counts}")
print(f"  Joint names: {dof_names}")

# 각 관절 상세 정보
for i, name in enumerate(dof_names):
    print(f"\n  Joint[{i}]: {name}")
    print(f"    Type: {dof_types[i]}")
    
    # Joint 위치 읽기
    try:
        pos = articulation.get_joint_positions(joint_indices=i)
        print(f"    Position: {pos}")
    except:
        pass
    
    # Joint 속도 읽기
    try:
        vel = articulation.get_joint_velocities(joint_indices=i)
        print(f"    Velocity: {vel}")
    except:
        pass

# Prim 계층 구조 출력
print("\n  Stage hierarchy under /World/TurtleBot3:")
robot_prim = get_prim_at_path(ROBOT_PATH)
if robot_prim:
    def print_prim_tree(prim, indent=2):
        name = prim.GetName()
        type_name = prim.GetTypeName() or "(Xform)"
        children = prim.GetChildren()
        marker = " 📦" if children else "  "
        print(f"{' ' * indent}├── {name} ({type_name}){marker}")
        for child in children:
            print_prim_tree(child, indent + 4)
    
    for child in robot_prim.GetChildren():
        print_prim_tree(child)

# ──────────────────────────────────────────────
# 6. 여러 로봇 배치 (응용)
# ──────────────────────────────────────────────
print("\n[4/5] Adding more robots...")

robot_positions = [
    (1.5, 0.0, 0.1),
    (0.0, 1.5, 0.1),
    (-1.5, 0.0, 0.1),
]

robots = [robot]  # 첫 번째 로봇 포함
for i, pos in enumerate(robot_positions):
    path = f"/World/TurtleBot3_{i + 1}"
    
    if not is_prim_path_valid(path):
        add_reference_to_stage(
            usd_path=ROBOT_USD,
            prim_path=path,
        )
        
        new_robot = Robot(
            prim_path=path,
            name=f"TurtleBot3_{i + 1}",
            position=np.array(pos),
        )
        world.scene.add(new_robot)
        robots.append(new_robot)
        print(f"  + Added robot at {pos} → {path}")

print(f"\n  Total robots in scene: {len(robots)}")

# ──────────────────────────────────────────────
# 7. 시뮬레이션 실행
# ──────────────────────────────────────────────
print("\n[5/5] Running simulation...")

FRAMES_TO_RUN = 200

for i in range(FRAMES_TO_RUN):
    world.step(render=True)
    
    # 첫 번째 로봇의 상태 모니터링
    if i % 30 == 0:
        pos, orient = robots[0].get_world_pose()
        joint_pos = robots[0].get_joint_positions()
        print(f"  Frame {i:3d}: Robot Z={pos[2]:.3f}m, "
              f"Joints={[f'{jp:.4f}' for jp in joint_pos]}")

# 로봇이 안정화된 후 최종 상태
print(f"\n  Simulation complete ({FRAMES_TO_RUN} frames).")
for idx, r in enumerate(robots):
    pos, _ = r.get_world_pose()
    print(f"  Robot {idx}: position=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

# ──────────────────────────────────────────────
# 8. 정리
# ──────────────────────────────────────────────
print("\n" + "=" * 60)
print("Step 06 Complete!")
print("=" * 60)
print()
print("Key takeaways:")
print("  1. add_reference_to_stage()로 USD 로봇 파일을 Scene에 추가")
print("  2. Robot 객체는 Articulation을 래핑한 고수준 API")
print("  3. TurtleBot3 has Continuous joints for wheels")
print("  4. 동일 USD를 여러 번 Reference → 여러 로봇 배치 가능")
print("  5. World.scene.add(robot)으로 로봇을 시뮬레이션에 등록")
print()
print("Files created in this step: None (read-only USD assets used)")
print("To save the scene: File > Save As... > turtlebot_scene.usd")

# SimulationApp 종료
simulation_app.close()
