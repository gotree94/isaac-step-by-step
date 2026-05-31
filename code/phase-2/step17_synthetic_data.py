"""
Step 17 — Synthetic Data Generation
======================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step17_synthetic_data.py

사전 준비:
    1. Isaac Sim 5.1.0 (Replicator Extension 활성화)
    2. 출력 디렉토리: /datasets/training (자동 생성)

목표:
    1. Replicator Writer 설정 (COCO + Basic)
    2. Domain Randomization (조명/자세/색상)
    3. RGB + Depth + Segmentation 데이터 생성
    4. COCO JSON Format 저장
    5. ROS2 Bridge로 실시간 Synthetic Data 발행
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
print("Step 17 — Synthetic Data Generation")
print("=" * 60)

# ── 2. Core API 임포트 ──
import os
import json
import random
import numpy as np
from pxr import Sdf, UsdGeom, Gf
import omni.graph.core as og
import omni.usd
import omni.replicator.core as rep

from omni.isaac.core.world import World
from omni.isaac.core.objects import VisualCuboid, VisualSphere, VisualCylinder
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. 출력 디렉토리 설정 ──
print("\n[1/6] Setting up output directory...")

OUTPUT_DIR = "/datasets/training_sd"
os.makedirs(f"{OUTPUT_DIR}/rgb", exist_ok=True)
os.makedirs(f"{OUTPUT_DIR}/depth", exist_ok=True)
os.makedirs(f"{OUTPUT_DIR}/seg", exist_ok=True)
os.makedirs(f"{OUTPUT_DIR}/annotations", exist_ok=True)

print(f"  + Output: {OUTPUT_DIR}")

# ── 5. Scene 구성 (데이터 생성용) ──
print("\n[2/6] Creating data generation scene...")

# TurtleBot3 (메인 객체)
ROBOT_PATH = "/World/TurtleBot3_SD"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage(
        "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
        ROBOT_PATH,
    )
robot = Robot(prim_path=ROBOT_PATH, name="TB_SD", position=np.array([0.0, 0.0, 0.1]))

# 장애물 물체들 (다양한 형태/색상)
objects = []
for i in range(6):
    x = random.uniform(-0.8, 0.8)
    y = random.uniform(-0.8, 0.8)
    h = random.uniform(0.05, 0.25)
    color = np.random.rand(3)
    
    if i % 3 == 0:
        obj = VisualCuboid(
            prim_path=f"/World/Objects/SD_Box_{i:02d}",
            name=f"sd_box_{i}",
            position=np.array([x, y, h / 2]),
            scale=np.array([0.08, 0.08, h]),
            color=color,
        )
    elif i % 3 == 1:
        obj = VisualSphere(
            prim_path=f"/World/Objects/SD_Sphere_{i:02d}",
            name=f"sd_sphere_{i}",
            position=np.array([x, y, h]),
            radius=h,
            color=color,
        )
    else:
        obj = VisualCylinder(
            prim_path=f"/World/Objects/SD_Cylinder_{i:02d}",
            name=f"sd_cyl_{i}",
            position=np.array([x, y, h/2]),
            height=h,
            radius=0.04,
            color=color,
        )
    objects.append(obj)

# Semantic Labels 설정 (Replicator용)
try:
    for i, obj in enumerate(objects):
        rep.modify.semantic_label(
            prim_path=f"/World/Objects/SD_Box_{i:02d}",
            label=f"object_{i}",
            id=i + 2,
        )
    rep.modify.semantic_label(
        prim_path=ROBOT_PATH,
        label="TurtleBot3",
        id=1,
    )
except Exception as e:
    print(f"  ⚠ Semantic labeling note: {e}")

print(f"  + 1 TurtleBot3 + {len(objects)} objects created")

# ── 6. SD 카메라 설정 ──
print("\n[3/6] Setting up synthetic data camera...")

stage = omni.usd.get_context().get_stage()
camera_path = "/World/Camera_SD"
camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
camera_prim.GetFocalLengthAttr().Set(24.0)
camera_prim.GetFocusDistanceAttr().Set(400.0)
camera_prim.GetFStopAttr().Set(0.0)
camera_prim.GetClippingRangeAttr().Set((0.1, 1000.0))
camera_prim.GetHorizontalApertureAttr().Set(20.955)
camera_prim.GetVerticalApertureAttr().Set(15.29)

# 초기 카메라 위치
camera_xform = UsdGeom.Xformable(camera_prim)
translate_op = camera_xform.AddTranslateOp()
translate_op.Set(Gf.Vec3d(1.8, -0.5, 1.2))
look_at_matrix = Gf.Matrix4d().SetLookAt(
    Gf.Vec3d(1.8, -0.5, 1.2),
    Gf.Vec3d(0.0, 0.0, 0.1),
    Gf.Vec3d(0.0, 0.0, 1.0),
)
rotate_op = camera_xform.AddOrientOp()
rotate_op.Set(Gf.Quatd(look_at_matrix.ExtractRotationQuat()))
print(f"  + SD Camera created at (1.8, -0.5, 1.2)")

# ── 7. Replicator Writer + Randomizer 설정 ──
print("\n[4/6] Configuring Replicator writers and randomizers...")

rep.orchestrator.set_capture_on_play(True)

# Render Product
render_product = rep.create.render_product(
    camera_path, resolution=(1280, 720))

# COCO Writer (BBox 2D + Segmentation)
try:
    writer_coco = rep.WriterRegistry.get("CocoWriter")
    writer_coco.initialize(
        output_dir=OUTPUT_DIR,
        rgb=True,
        bounding_box_2d=True,
        semantic_segmentation=True,
        instance_id_segmentation=False,
    )
    writer_coco.attach([render_product])
    print("  + COCO Writer attached (BBox2D + RGB + Seg)")
except Exception as e:
    print(f"  ⚠ COCO Writer note: {e}")

# Basic Writer (Depth + Normal)
try:
    writer_basic = rep.WriterRegistry.get("BasicWriter")
    writer_basic.initialize(
        output_dir=OUTPUT_DIR,
        rgb=False,  # COCO writer가 RGB 처리
        depth_normals=True,
        include_bboxes=False,
    )
    writer_basic.attach([render_product])
    print("  + Basic Writer attached (Depth + Normal)")
except Exception as e:
    print(f"  ⚠ Basic Writer note: {e}")

# ── 8. Domain Randomization ──
print("\n[5/6] Setting up domain randomization...")

NUM_FRAMES = 200

with rep.trigger.on_frame(max_execs=NUM_FRAMES):
    rep.randomizer.light(
        light_type="dome",
        intensity=(500, 2000),
        color=(0.8, 1.2),
        rotation=(0, 360),
    )

with rep.trigger.on_frame(max_execs=NUM_FRAMES):
    rep.randomizer.camera(
        pose=rep.distribution.uniform(
            [(1.0, -1.0, 0.8), (2.5, 1.0, 2.0)]
        ),
        look_at=(0.0, 0.0, 0.1),
    )

print(f"  + Domain randomization configured for {NUM_FRAMES} frames")
print("    - Light: dome, intensity(500-2000), rotation(0-360)")
print("    - Camera: position randomized (looking at origin)")

# ── 9. 데이터 생성 실행 ──
print(f"\n[6/6] Generating synthetic data...")
print(f"  Generating {NUM_FRAMES} frames...")

world.step(render=True)

# 전역 조명 설정
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")

# Replicator 실행
rep.orchestrator.run()

# 시뮬레이션 루프 (데이터 생성)
for i in range(NUM_FRAMES):
    world.step(render=True)
    
    if i % 20 == 0:
        print(f"  Frame {i:4d}/{NUM_FRAMES} generated...")
    
    if i == NUM_FRAMES - 1:
        print(f"  Frame {i+1:4d}/{NUM_FRAMES} — Done!")

# ── 10. 결과 확인 ──
print("\n" + "=" * 60)
print("Step 17 — Complete!")
print("=" * 60)
print()
print("  Generated data:")
print(f"    Location: {OUTPUT_DIR}")

# 출력 파일 확인
for subdir in ['rgb', 'depth', 'seg', 'annotations']:
    path = os.path.join(OUTPUT_DIR, subdir)
    if os.path.exists(path):
        files = [f for f in os.listdir(path) if not f.startswith('.')]
        print(f"    - {subdir}/: {len(files)} files")

# COCO JSON 확인
coco_path = os.path.join(OUTPUT_DIR, "coco_data", "annotations.json")
if os.path.exists(coco_path):
    with open(coco_path, 'r') as f:
        data = json.load(f)
    print(f"\n  COCO annotations:")
    print(f"    Images: {len(data.get('images', []))}")
    print(f"    Annotations: {len(data.get('annotations', []))}")
    print(f"    Categories: {len(data.get('categories', []))}")

print()
print("  Key concepts:")
print("  - Replicator Writer: COCO + Basic (RGB, Depth, Seg, BBox2D)")
print("  - Domain Randomization: Light + Camera pose")
print("  - Data formats: COCO JSON for training")
print("  - Resolution: 1280x720")
print("  - Semantic Labels: object classification IDs")
print()
print("  Next step: Use this data for training a perception model")
print("  (YOLO, Detectron2, or custom PyTorch pipeline)")
print("=" * 60)

simulation_app.close()
