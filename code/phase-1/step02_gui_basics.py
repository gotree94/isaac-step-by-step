"""
Step 02 — GUI 기본 사용법: Python API로 체험하기
=================================================

실행 방법 (Standalone):
    cd ~/isaac-sim  # or <ISAAC_ROOT>
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step02_gui_basics.py

실행 방법 (GUI Extension):
    Isaac Sim 실행 후 메뉴 File > Open Script > 이 파일 선택
    또는 Script Editor에 붙여넣기 후 Run

목표:
    Step 02에서 GUI로 했던 작업(Cube 생성, Transform, Material, Physics)을
    Python API로 동일하게 수행합니다.
"""

# ──────────────────────────────────────────────
# 1. 임포트
# ──────────────────────────────────────────────
import carb
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema

from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage
from omni.isaac.core.utils.prims import (
    create_prim,
    define_prim,
    is_prim_path_valid,
    get_prim_at_path,
    set_prim_attribute,
)
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, VisualCuboid

# ──────────────────────────────────────────────
# 2. Stage 생성
# ──────────────────────────────────────────────
print("[Step 02] ===== Isaac Sim GUI Basics - Python API Demo =====")

# 새 Stage 생성 (Ctrl+N)
create_new_stage()

# Physics Scene 자동 생성 (World가 담당)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print("[Step 02] Stage and PhysicsScene created.")

# ──────────────────────────────────────────────
# 3. 객체 생성 (Create > Shape > Cube / Sphere / Cylinder)
# ──────────────────────────────────────────────
print("[Step 02] Creating primitives...")

# 3.1 큐브 (VisualCuboid = GUI에서 큐브 생성과 동일)
cube = VisualCuboid(
    prim_path="/World/MyCube",
    name="MyCube",
    position=np.array([0.0, 0.0, 3.0]),
    scale=np.array([1.0, 1.0, 1.0]),
    color=np.array([1.0, 0.0, 0.0]),  # 빨간색
)
world.scene.add(cube)

# 3.2 구체
sphere = DynamicSphere(
    prim_path="/World/MySphere",
    name="MySphere",
    position=np.array([2.0, 0.0, 5.0]),
    radius=0.5,
    color=np.array([0.0, 0.0, 1.0]),  # 파란색
)
world.scene.add(sphere)

# 3.3 원기둥 (UsdGeom API로 직접 생성)
stage = get_current_stage()
cylinder_path = Sdf.Path("/World/MyCylinder")
cylinder_prim = UsdGeom.Cylinder.Define(stage, cylinder_path)
cylinder_prim.AddTranslateOp().Set(Gf.Vec3d([-2.0, 0.0, 4.0]))
cylinder_prim.AddScaleOp().Set(Gf.Vec3d([0.8, 0.8, 0.8]))

print("[Step 02] Cube at (0, 0, 3), Sphere at (2, 0, 5), Cylinder at (-2, 0, 4).")

# ──────────────────────────────────────────────
# 4. Transform 조작 (W/E/R)
# ──────────────────────────────────────────────
print("[Step 02] Applying transforms via Python...")

# 4.1 Move (W) - 큐브 위치 변경
cube_prim = get_prim_at_path("/World/MyCube")
cube_xform = UsdGeom.Xformable(cube_prim)
cube_translate = cube_xform.AddTranslateOp()
cube_translate.Set(Gf.Vec3d([0.0, 1.5, 3.0]))  # Y축 +1.5 이동

# 4.2 Rotate (E) - 구체 회전
sphere_prim = get_prim_at_path("/World/MySphere")
sphere_xform = UsdGeom.Xformable(sphere_prim)
sphere_rotate = sphere_xform.AddRotateXYZOp()
sphere_rotate.Set(Gf.Vec3d([0.0, 0.0, 45.0]))  # Z축 45도 회전

# 4.3 Scale (R) - 원기둥 스케일
cylinder_xform = UsdGeom.Xformable(cylinder_prim)
cylinder_scale = cylinder_xform.AddScaleOp()
cylinder_scale.Set(Gf.Vec3d([1.5, 1.5, 2.0]))  # Z축으로 2배

print("[Step 02] Transforms applied.")

# ──────────────────────────────────────────────
# 5. Material 적용 (Appearance)
# ──────────────────────────────────────────────
print("[Step 02] Applying materials...")

# 5.1 큐브에 빨간 금속성 Material
cube_material_path = "/World/MyCube/Materials/RedMetalMaterial"
cube_material = OmniPBR(
    prim_path=cube_material_path,
    color=np.array([1.0, 0.2, 0.2]),   # 빨간색
    roughness=0.2,                        # 매끈
    metallic=1.0,                         # 금속
)
cube_material.apply(get_prim_at_path("/World/MyCube"))

# 5.2 구체에 파란 유리 Material
sphere_material_path = "/World/MySphere/Materials/BlueGlassMaterial"
sphere_material = OmniPBR(
    prim_path=sphere_material_path,
    color=np.array([0.2, 0.3, 1.0]),   # 파란색
    roughness=0.05,                       # 매우 매끈
    metallic=0.0,                         # 비금속
    opacity=0.6,                          # 반투명
)
sphere_material.apply(get_prim_at_path("/World/MySphere"))

# 5.3 원기둥에 초록 거친 Material
cylinder_material_path = "/World/MyCylinder/Materials/GreenRoughMaterial"
cylinder_material = OmniPBR(
    prim_path=cylinder_material_path,
    color=np.array([0.2, 0.8, 0.2]),   # 초록색
    roughness=0.9,                        # 거침
    metallic=0.0,
)
cylinder_material.apply(get_prim_at_path("/World/MyCylinder"))

print("[Step 02] Materials applied (Red metal, Blue glass, Green rough).")

# ──────────────────────────────────────────────
# 6. 물리 속성 추가 (Rigid Body + Collision)
# ──────────────────────────────────────────────
print("[Step 02] Adding physics properties...")

def add_rigid_body_with_collision(prim_path: str, mass: float = 1.0):
    """
    GUI의 'Rigid Body with Colliders Preset'과 동일한 기능을 수행합니다.
    
    - Rigid Body API 추가
    - Collision API 추가
    - 질량 설정
    """
    prim = get_prim_at_path(prim_path)
    
    # Rigid Body API 적용
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body_api.GetMassAttr().Set(mass)
    rigid_body_api.GetLinearDampingAttr().Set(0.0)
    rigid_body_api.GetAngularDampingAttr().Set(0.0)
    
    # Collision API 적용
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    
    # PhysX Rigid Body (추가 물리 파라미터)
    physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_rigid_api.GetSolverPositionIterationAttr().Set(8)
    physx_rigid_api.GetSolverVelocityIterationAttr().Set(1)
    
    print(f"  + RigidBody (mass={mass}) + Collision added to {prim_path}")

# 각 객체에 물리 속성 추가
add_rigid_body_with_collision("/World/MyCube", mass=2.0)
add_rigid_body_with_collision("/World/MySphere", mass=1.0)
add_rigid_body_with_collision("/World/MyCylinder", mass=5.0)

# ──────────────────────────────────────────────
# 7. 조명 추가 (Create > Lights)
# ──────────────────────────────────────────────
print("[Step 02] Adding lights...")

# Dome Light (환경광)
dome_light_path = Sdf.Path("/World/DomeLight")
dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
dome_light.CreateIntensityAttr().Set(1000.0)

# Distant Light (태양광)
distant_light_path = Sdf.Path("/World/DistantLight")
distant_light = UsdLux.DistantLight.Define(stage, distant_light_path)
distant_light.CreateIntensityAttr().Set(500.0)
distant_light.CreateAngleAttr().Set(1.0)

print("[Step 02] Lights added (Dome 1000, Distant 500).")

# ──────────────────────────────────────────────
# 8. 시뮬레이션 실행
# ──────────────────────────────────────────────
print("[Step 02] =============================================")
print("[Step 02] Scene is ready. Press Play (▶) in the viewport.")
print("[Step 02] Objects will fall under gravity and collide.")
print("[Step 02] =============================================")
print()
print("Objects created:")
print("  /World/MyCube       - Red metal (mass=2.0)")
print("  /World/MySphere     - Blue glass (mass=1.0)")
print("  /World/MyCylinder   - Green rough (mass=5.0)")
print()
print("To save:   File > Save As  -->  step02_scene.usd")
print()
print("USD 정리 (Stage 계층):")
print("  /World")
print("   ├── GroundPlane")
print("   ├── MyCube          <- RigidBody + Collision")
print("   ├── MySphere        <- RigidBody + Collision")
print("   ├── MyCylinder      <- RigidBody + Collision")
print("   ├── DomeLight")
print("   └── DistantLight")
