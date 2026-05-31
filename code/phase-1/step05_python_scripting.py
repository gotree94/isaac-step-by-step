"""
Step 05 — Python Scripting 기초: Standalone 시뮬레이션 루프
============================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step05_python_scripting.py

또는 (pip 설치 환경):
    source ~/isaacsim_env/bin/activate
    python ~/isaac-step-curriculum/code/phase-1/step05_python_scripting.py

목표:
    1. SimulationApp 초기화 (Standalone 진입점)
    2. World 생성 및 Scene 설정
    3. 시뮬레이션 루프 (world.step)
    4. USD Prim CRUD (Create/Read/Update/Delete)
    5. 동적 객체 물리 시뮬레이션
    6. 데이터 수집 및 출력
"""

# ──────────────────────────────────────────────
# 1. SimulationApp 초기화 (반드시 다른 import보다 먼저!)
# ──────────────────────────────────────────────
# Isaac Sim의 Python 인터프리터로 실행 시, 
# 이 부분이 Kit 애플리케이션을 초기화합니다.
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1280,
    "window_height": 720,
    "headless": False,      # True면 GUI 없이 실행
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 05 — Python Scripting Basics")
print("=" * 60)

# ──────────────────────────────────────────────
# 2. Core API 임포트 (SimulationApp 초기화 후)
# ──────────────────────────────────────────────
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid, DynamicSphere
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path
from omni.isaac.core.utils.stage import get_current_stage

# ──────────────────────────────────────────────
# 3. World 생성
# ──────────────────────────────────────────────
print("\n[1/6] Creating World & Ground Plane...")

world = World(
    stage_units_in_meters=1.0,
    physics_dt=1 / 60.0,
    rendering_dt=1 / 60.0,
)
world.scene.add_default_ground_plane()

# World는 내부적으로 PhysicsScene을 생성합니다.
# Stage를 확인해보세요:
stage = get_current_stage()
print(f"  Stage: {stage}")
print(f"  Physics Scene exists: {is_prim_path_valid('/World/PhysicsScene')}")

# ──────────────────────────────────────────────
# 4. USD Prim CRUD
# ──────────────────────────────────────────────
print("\n[2/6] USD Prim CRUD Demo...")

# 4.1 CREATE - DefinePrim
print("  CREATE:")

# 방법 A: omni.isaac.core.objects (고수준)
cube = DynamicCuboid(
    prim_path="/World/DemoCube",
    name="DemoCube",
    position=np.array([0.0, 0.0, 5.0]),
    scale=np.array([1.0, 1.0, 1.0]),
    mass=1.0,
)
world.scene.add(cube)
print(f"    + /World/DemoCube (DynamicCuboid, mass=1.0)")

# 방법 B: pxr.Usd 직접 (저수준)
sphere_prim = stage.DefinePrim("/World/DemoSphere", "Sphere")
sphere_xform = UsdGeom.Xformable(sphere_prim)
sphere_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 5.0))
sphere_prim.GetAttribute("radius").Set(0.5)
sphere_prim.GetAttribute("primvars:displayColor").Set([(0.0, 0.3, 1.0)])
print(f"    + /World/DemoSphere (Sphere via pxr, r=0.5)")

# 방법 C: UsdGeom API
cylinder = UsdGeom.Cylinder.Define(stage, "/World/DemoCylinder")
cylinder.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 5.0))
cylinder.AddScaleOp().Set(Gf.Vec3d(0.8, 0.8, 1.0))
cylinder.GetHeightAttr().Set(1.0)
cylinder.GetRadiusAttr().Set(0.4)
cylinder.GetPrim().GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.3)])
print(f"    + /World/DemoCylinder (Cylinder via UsdGeom)")

# 4.2 READ
print("  READ:")
prim = get_prim_at_path("/World/DemoCube")
print(f"    /World/DemoCube type: {prim.GetTypeName()}")
for attr in prim.GetAttributes():
    print(f"      - {attr.GetName()} = {attr.Get()}")

# 4.3 UPDATE
print("  UPDATE:")
prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 8.0))
print(f"    Moved DemoCube to Z=8.0")

# 4.4 DELETE
print("  DELETE:")
# (나중에 정리)

# ──────────────────────────────────────────────
# 5. CRUD Helper 함수
# ──────────────────────────────────────────────
print("\n[3/6] CRUD Helper Functions...")

def create_prim(stage, path: str, prim_type: str, 
                position=(0, 0, 0), color=None):
    """USD Prim을 생성하고 Transform + Color를 설정합니다."""
    if is_prim_path_valid(path):
        print(f"    ⚠ {path} already exists. Skipping.")
        return None
    
    prim = stage.DefinePrim(path, prim_type)
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    
    if color:
        prim.GetAttribute("primvars:displayColor").Set([color])
    
    print(f"    + Created {prim_type} at {path}, pos={position}")
    return prim

def read_prim_info(prim_path: str):
    """Prim의 모든 속성을 출력합니다."""
    prim = get_prim_at_path(prim_path)
    if not prim:
        print(f"    ✗ {prim_path} not found!")
        return
    
    print(f"    Prim: {prim_path}")
    print(f"    Type: {prim.GetTypeName()}")
    print(f"    Specifier: {prim.GetSpecifier()}")
    
    for attr in prim.GetAttributes():
        print(f"      {attr.GetName()} = {attr.Get()}")

def update_prim_position(prim_path: str, new_pos):
    """Prim의 위치를 업데이트합니다."""
    prim = get_prim_at_path(prim_path)
    if not prim:
        return False
    
    attr = prim.GetAttribute("xformOp:translate")
    if attr:
        attr.Set(Gf.Vec3d(*new_pos))
        return True
    return False

def delete_prim(prim_path: str):
    """Prim을 Stage에서 제거합니다."""
    stage = get_current_stage()
    if is_prim_path_valid(prim_path):
        stage.RemovePrim(prim_path)
        print(f"    ✗ Removed {prim_path}")
        return True
    return False

# Helper 함수 테스트
create_prim(stage, "/World/Cone", "Cone", position=(0, 2.0, 2.0), color=(1, 1, 0))
create_prim(stage, "/World/Torus", "Torus", position=(0, -2.0, 2.0), color=(1, 0.5, 0))

print()
read_prim_info("/World/Cone")
read_prim_info("/World/Torus")

# ──────────────────────────────────────────────
# 6. 시뮬레이션 루프
# ──────────────────────────────────────────────
print("\n[4/6] Running simulation loop...")

# 데이터 수집용
positions = []
velocities = []
frames = 200

for i in range(frames):
    world.step(render=True)
    
    # 객체 상태 읽기
    pos, orient = cube.get_world_pose()
    vel = cube.get_linear_velocity()
    
    positions.append(pos.copy())
    velocities.append(vel.copy())
    
    # 20프레임마다 진행 상황 출력
    if i % 20 == 0:
        print(f"  Frame {i:3d}: Z={pos[2]:7.3f}m, Vz={vel[2]:7.3f}m/s")
    
    # 종료 조건: 바닥에 안정화
    if pos[2] < 0.1 and abs(vel[2]) < 0.01:
        print(f"  → Object settled at frame {i}")
        break

print(f"\n  Total frames simulated: {len(positions)}")
print(f"  Final position: {positions[-1]}")
print(f"  Impact/rest velocity: {velocities[-1][2]:.3f} m/s")

# ──────────────────────────────────────────────
# 7. 물리 상태 분석
# ──────────────────────────────────────────────
print("\n[5/6] Physics analysis...")

if len(positions) > 1:
    # 최대 낙하 거리
    start_z = positions[0][2]
    min_z = min(p[2] for p in positions)
    fall_distance = start_z - min_z
    print(f"  Drop height: {start_z:.1f}m → {min_z:.3f}m (Δ={fall_distance:.3f}m)")
    
    # 최대 속도
    max_vz = max(abs(v[2]) for v in velocities)
    print(f"  Max vertical speed: {max_vz:.3f} m/s")
    
    # 이론적 충돌 속도: v = sqrt(2 * g * h)
    # g=9.81 (Isaac Sim 기본 중력)
    g = 9.81
    theoretical_v = np.sqrt(2 * g * fall_distance)
    print(f"  Theoretical impact speed: {theoretical_v:.3f} m/s")
    print(f"  (Difference due to damping and collision handling)")

# ──────────────────────────────────────────────
# 8. 정리 및 종료
# ──────────────────────────────────────────────
print("\n[6/6] Cleanup...")

# 생성한 Prim 정리 (데모용)
# delete_prim("/World/DemoCube")
# delete_prim("/World/DemoSphere")
# delete_prim("/World/DemoCylinder")
# delete_prim("/World/Cone")
# delete_prim("/World/Torus")

print()
print("=" * 60)
print("Step 05 Complete!")
print("=" * 60)
print()
print("Key takeaways:")
print("  1. SimulationApp 초기화는 모든 import보다 먼저")
print("  2. World = PhysicsScene + Ground Plane + Timeline")
print("  3. world.step(render=True)로 1프레임 진행")
print("  4. pxr.Usd = 저수준 USD API, omni.isaac.core = 고수준 API")
print("  5. DynamicCuboid는 자동으로 RigidBody + Collision 포함")
print()
print("Try this: change headless=True in CONFIG and re-run.")
print("The simulation will run without the GUI window.")

# SimulationApp 종료
simulation_app.close()
