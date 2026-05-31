"""
Step 04 — USD Stage 구조 이해: Python으로 USD 직접 조작하기
============================================================

실행 방법 (Standalone):
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step04_usd_stage.py

실행 방법 (GUI Script Editor):
    Isaac Sim 실행 후 Script Editor에서 열고 Run.

목표:
    Step 04에서 배운 USD 개념(Prim, Attribute, Specifier, Reference)을
    Python API로 직접 조작합니다.
    
    1. USD Stage 생성 및 Prim 추가
    2. Attribute 읽기/쓰기
    3. Reference를 통한 외부 USD 포함
    4. .usda 파일로 저장하여 텍스트 편집
    5. Layer 조작
"""

# ──────────────────────────────────────────────
# 1. 임포트
# ──────────────────────────────────────────────
import os
import sys
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, Kind

from omni.isaac.core.utils.stage import (
    create_new_stage,
    get_current_stage,
    open_stage,
    save_stage_as_usd,
)
from omni.isaac.core.utils.prims import (
    create_prim,
    define_prim,
    is_prim_path_valid,
    get_prim_at_path,
    get_all_prims_in_stage,
)
from omni.isaac.core.utils.extensions import enable_extension

print("=" * 60)
print("Step 04 — USD Stage Python API Demo")
print("=" * 60)

# ──────────────────────────────────────────────
# 2. 새 Stage 생성
# ──────────────────────────────────────────────
print("\n[1/6] Creating new USD Stage...")
create_new_stage()

stage = get_current_stage()
if not stage:
    print("ERROR: Failed to create stage.")
    sys.exit(1)

print(f"  Stage: {stage}")
print(f"  Layer: {stage.GetRootLayer().realPath}")

# ──────────────────────────────────────────────
# 3. Prim 생성 (def)
# ──────────────────────────────────────────────
print("\n[2/6] Creating Prims (def)...")

# 3.1 Root Xform
world_prim = stage.DefinePrim("/World", "Xform")
print(f"  + /World (Xform)")

# 3.2 Cube
cube_prim = stage.DefinePrim("/World/MyCube", "Cube")
cube_xform = UsdGeom.Xformable(cube_prim)
cube_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))
cube_xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
cube_xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
# 색상
cube_color = cube_prim.GetAttribute("primvars:displayColor")
cube_color.Set([(1.0, 0.0, 0.0)])  # 빨간색
print(f"  + /World/MyCube (Cube) at (0, 0, 1), color=Red")

# 3.3 Sphere
sphere_prim = stage.DefinePrim("/World/MySphere", "Sphere")
sphere_xform = UsdGeom.Xformable(sphere_prim)
sphere_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.5))
sphere_color = sphere_prim.GetAttribute("primvars:displayColor")
sphere_color.Set([(0.0, 0.0, 1.0)])  # 파란색
# 반지름 설정
sphere_prim.GetAttribute("radius").Set(0.5)
print(f"  + /World/MySphere (Sphere) at (2, 0, 0.5), radius=0.5")

# 3.4 Cylinder
cyl_prim = stage.DefinePrim("/World/MyCylinder", "Cylinder")
cyl_xform = UsdGeom.Xformable(cyl_prim)
cyl_xform.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 0.5))
cyl_color = cyl_prim.GetAttribute("primvars:displayColor")
cyl_color.Set([(0.0, 1.0, 0.0)])  # 초록색
cyl_prim.GetAttribute("height").Set(1.0)
cyl_prim.GetAttribute("radius").Set(0.3)
print(f"  + /World/MyCylinder (Cylinder) at (-2, 0, 0.5)")

# 3.5 Light
light_prim = stage.DefinePrim("/World/DistantLight", "DistantLight")
light = UsdLux.DistantLight(light_prim)
light.CreateIntensityAttr().Set(500.0)
light.CreateAngleAttr().Set(1.0)
light.AddTranslateOp().Set(Gf.Vec3d(5.0, 5.0, 10.0))
print(f"  + /World/DistantLight (intensity=500)")

# ──────────────────────────────────────────────
# 4. Attribute 읽기/쓰기
# ──────────────────────────────────────────────
print("\n[3/6] Reading & Writing Attributes...")

# 4.1 Attribute 읽기
prim = get_prim_at_path("/World/MyCube")
translate_attr = prim.GetAttribute("xformOp:translate")
current_pos = translate_attr.Get()
print(f"  Cube position (before): {current_pos}")

# 4.2 Attribute 쓰기
translate_attr.Set(Gf.Vec3d(0.0, 1.5, 2.0))
new_pos = translate_attr.Get()
print(f"  Cube position (after) : {new_pos}")

# 4.3 모든 Attribute 열거
print(f"\n  Attributes of /World/MyCube:")
for attr in prim.GetAttributes():
    print(f"    - {attr.GetName()} = {attr.Get()}")

# 4.4 모든 Prim 열거
print(f"\n  All Prims in Stage:")
all_prims = [p for p in stage.Traverse()]
for p in all_prims:
    specifier = p.GetSpecifier()
    type_name = p.GetTypeName() or "(non)")
    print(f"    [{specifier}] /{str(p.GetPath()).lstrip('/')} ({type_name})")

# ──────────────────────────────────────────────
# 5. Reference를 통한 USD 포함
# ──────────────────────────────────────────────
print("\n[4/6] Adding References...")

# 외부 USD 파일 경로 설정
assets_dir = os.path.expanduser("~/isaac-step-curriculum/assets/scenes")
os.makedirs(assets_dir, exist_ok=True)

# 5.1 참조할 USD 파일 생성
ref_usd_path = os.path.join(assets_dir, "robot_part.usda")

# USD 파일을 직접 작성 (pxr API로 저장)
ref_stage = Usd.Stage.CreateNew(ref_usd_path)
ref_stage.SetDefaultPrim(ref_stage.DefinePrim("/RobotPart", "Xform"))

# 하위 구조
chassis = ref_stage.DefinePrim("/RobotPart/Chassis", "Cube")
chassis_xform = UsdGeom.Xformable(chassis)
chassis_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.3))
chassis_xform.AddScaleOp().Set(Gf.Vec3d(1.0, 0.8, 0.3))
chassis.GetAttribute("primvars:displayColor").Set([(0.5, 0.5, 0.5)])

wheel1 = ref_stage.DefinePrim("/RobotPart/Wheel1", "Cylinder")
wheel1.GetAttribute("height").Set(0.1)
wheel1.GetAttribute("radius").Set(0.15)
wheel1_xform = UsdGeom.Xformable(wheel1)
wheel1_xform.AddTranslateOp().Set(Gf.Vec3d(-0.5, 0, 0.05))
wheel1_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 90, 0))

wheel2 = ref_stage.DefinePrim("/RobotPart/Wheel2", "Cylinder")
wheel2.GetAttribute("height").Set(0.1)
wheel2.GetAttribute("radius").Set(0.15)
wheel2_xform = UsdGeom.Xformable(wheel2)
wheel2_xform.AddTranslateOp().Set(Gf.Vec3d(0.5, 0, 0.05))
wheel2_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 90, 0))

ref_stage.GetRootLayer().Save()
print(f"  + Created reference USD: {ref_usd_path}")

# 5.2 현재 Stage에 Reference 추가
ref_prim = stage.DefinePrim("/World/MyRobot", "Xform")
ref_prim.GetReferences().AddReference(
    assetPath=ref_usd_path,
    primPath=Sdf.Path("/RobotPart")
)
ref_robot_xform = UsdGeom.Xformable(ref_prim)
ref_robot_xform.AddTranslateOp().Set(Gf.Vec3d(3.0, 0.0, 0.0))
print(f"  + Added Reference to /World/MyRobot (from robot_part.usda)")

# 5.3 Override 예시: 참조된 Prim의 속성 덮어쓰기
chassis_override = stage.OverridePrim("/World/MyRobot/Chassis")
chassis_override.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])
print(f"  + Override: MyRobot/Chassis color → Red")

# ──────────────────────────────────────────────
# 6. .usda 파일로 저장
# ──────────────────────────────────────────────
print("\n[5/6] Saving Stage as .usda...")

usda_path = os.path.join(assets_dir, "step04_stage.usda")
stage.Export(usda_path, addSourceFileComment=False)
print(f"  + Saved: {usda_path}")

# 저장된 내용 확인
print(f"\n  File content (first 50 lines):")
with open(usda_path, "r") as f:
    lines = f.readlines()
    for line in lines[:50]:
        print(f"    {line}", end="")
    if len(lines) > 50:
        print(f"    ... ({len(lines) - 50} more lines)")

# ──────────────────────────────────────────────
# 7. 파일 정리 및 완료
# ──────────────────────────────────────────────
print(f"\n[6/6] Cleanup & Summary")
print()
print("=" * 60)
print("Step 04 Complete!")
print("=" * 60)
print()
print("Files created:")
print(f"  {usda_path}")
print(f"  {ref_usd_path}")
print()
print("Stage hierarchy:")
print("  /World")
print("   ├── MyCube (Cube)         - Red, at (0, 1.5, 2)")
print("   ├── MySphere (Sphere)     - Blue, radius=0.5")
print("   ├── MyCylinder (Cylinder) - Green, height=1")
print("   ├── DistantLight          - intensity=500")
print("   ├── MyRobot (Xform)       - Reference from robot_part.usda")
print("   │   ├── Chassis (Cube)    - Override: Red")
print("   │   ├── Wheel1 (Cylinder)")
print("   │   └── Wheel2 (Cylinder)")
print()
print("Key takeaways:")
print("  - stage.DefinePrim() = def specifier (create new)")
print("  - stage.OverridePrim() = over specifier (modify existing)")
print("  - prim.GetReferences().AddReference() = prepend references")
print("  - stage.Export() = save as .usda / .usdc")
print("  - Open saved file in Isaac Sim: File > Open")
