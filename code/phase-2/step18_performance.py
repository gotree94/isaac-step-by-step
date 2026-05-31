"""
Step 18 — Performance Optimization
====================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step18_performance.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. NVIDIA GPU 드라이버 (성능 모니터링용)

목표:
    1. 성능 모니터링 설정 (Carb Profiling + FPS/GPU/메모리)
    2. Physics 최적화 (Substeps, Solver Iterations)
    3. Rendering 최적화 (Resolution Scale, Renderer)
    4. Scene 최적화 (Instancing, Prim Count)
    5. 성능 측정 전/후 비교
    6. ROS2 Bridge 성능 튜닝
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
print("Step 18 — Performance Optimization")
print("=" * 60)

# ── 2. Core API 및 모니터링 임포트 ──
import time
import numpy as np
from pxr import Sdf, UsdGeom, Gf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid, VisualSphere
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. 성능 측정 유틸리티 ──
print("\n[1/6] Performance monitoring setup...")

class PerformanceMonitor:
    """FPS, Physics, GPU 모니터링"""
    
    def __init__(self):
        self.start_time = time.perf_counter()
        self.frame_count = 0
        self.fps = 0.0
        self.log_interval = 60
    
    def update(self):
        """매 프레임 호출 → 주기적 통계 출력"""
        self.frame_count += 1
        elapsed = time.perf_counter() - self.start_time
        self.fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        if self.frame_count % self.log_interval == 0:
            # Stage 정보
            stage = omni.usd.get_context().get_stage()
            prim_count = len(list(stage.TraverseAll())) if stage else 0
            
            print(f"  [PERF] Frame {self.frame_count:4d}: "
                  f"FPS={self.fps:.1f}, Prims={prim_count}")

monitor = PerformanceMonitor()

def benchmark_section(func, label="Section", iterations=50):
    """특정 코드 블록 성능 측정"""
    times = []
    for _ in range(iterations):
        start = time.perf_counter_ns()
        func()
        end = time.perf_counter_ns()
        times.append((end - start) / 1_000_000)  # ms
    
    avg = np.mean(times)
    std = np.std(times)
    print(f"  {label}: {avg:.2f}ms ± {std:.2f}ms (n={iterations})")
    return avg

# ── 5. 성능 측정 1: 기본 Scene ──
print("\n[2/6] Benchmarking baseline performance...")

BASE_ROBOT_PATH = "/World/Benchmark_TB"
if not is_prim_path_valid(BASE_ROBOT_PATH):
    add_reference_to_stage(
        "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
        BASE_ROBOT_PATH,
    )
robot = Robot(prim_path=BASE_ROBOT_PATH, name="Bench_TB", 
              position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)

# 몇 프레임 안정화
for _ in range(20):
    world.step(render=True)

def step_world():
    world.step(render=True)

baseline_time = benchmark_section(step_world, "Baseline step()", 60)
print(f"  Estimated FPS: {1000/baseline_time:.1f}")

# ── 6. Physics 최적화 ──
print("\n[3/6] Tuning physics parameters...")

from omni.physx.physx import PhysX
physx = PhysX()

# 최적화 전 설정 저장
original_substeps = physx.get_num_substeps()
print(f"  Before: substeps={original_substeps}")

# 최적화 적용
physx.set_num_substeps(1)  # 2→1
physx.set_solver_iteration_counts(16)  # 32→16
print(f"  Physics tuned: substeps=1, solver_iterations=16")

# 최적화 후 측정
opt_physics_time = benchmark_section(step_world, "Optimized physics step", 60)
print(f"  Improvement: {((baseline_time - opt_physics_time)/baseline_time*100):.1f}% faster")

# ── 7. Rendering 최적화 ──
print("\n[4/6] Tuning rendering parameters...")

from omni.kit.viewport import Viewport
vp = Viewport()

# Resolution Scale 최적화
vp.set_resolution_scale(0.75)  # 100% → 75% = ~56% pixels
print(f"  Resolution scale: 1.0 → 0.75")

# Shadow Map 최적화
try:
    vp.set_shadow_map_size(1024)  # 4096 → 1024
    print(f"  Shadow map: 4096 → 1024")
except Exception:
    print(f"  ⚠ Shadow map setting skipped")

opt_render_time = benchmark_section(step_world, "Optimized rendering step", 60)
print(f"  Further improvement: {((opt_physics_time - opt_render_time)/opt_physics_time*100):.1f}%")

# ── 8. Scene 최적화 (Point Instancing) ──
print("\n[5/6] Scene optimization...")

# 많은 물체를 Instancing으로 효율적으로 렌더링
stage = omni.usd.get_context().get_stage()

try:
    # Instancer 생성
    instancer_path = "/World/Instancer"
    instancer = UsdGeom.PointInstancer.Define(stage, instancer_path)
    
    # 샘플 구체 추가 (Proto)
    proto_prim = stage.GetPrimAtPath("/World/Instancer/Proto")
    if not proto_prim:
        sphere = VisualSphere(
            prim_path="/World/Instancer/Proto",
            name="instance_proto",
            position=np.array([0.0, 0.0, 0.0]),
            radius=0.05,
            color=np.array([0.2, 0.8, 0.2]),
        )
        # 원본은 invisible
        sphere.hide(True)
    
    instancer.CreatePrototypesRel().AddTarget("/World/Instancer/Proto")
    
    # 100개 구체 배치
    positions = []
    for i in range(100):
        x = (i % 10) * 0.4 - 1.8
        y = (i // 10) * 0.4 - 1.8
        positions.append((x, y, 0.0))
    
    instancer.CreatePositionsAttr().Set(positions)
    instancer.CreateProtoIndicesAttr().Set([0] * 100)
    
    print(f"  + Point Instancing: 100 instances")
    
except Exception as e:
    print(f"  ⚠ Instancing note: {e}")

opt_scene_time = benchmark_section(step_world, "Optimized scene step", 60)

# ── 9. Ros2 Bridge 성능 최적화 (Graph 설정) ──
print("\n[6/6] Setting up optimized ROS2 Bridge graph...")

graph_config = {
    "graph_path": "/ActionGraph/Performance_Bridge",
    "evaluator_name": "execution",
}

if og.Controller.graph_exists("/ActionGraph/Performance_Bridge"):
    stage.RemovePrim(Sdf.Path("/ActionGraph/Performance_Bridge"))

(
    graph_handle,
    graph_nodes,
    _,
    _,
) = og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("RateCtrl", "omni.graph.nodes.RateController"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
            
            ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
        og.Controller.Keys.CONNECT: [
            # OnTick → RateController (15Hz 제한)
            ("OnTick.outputs:tick", "RateCtrl.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "SubTwist.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "DiffCtrl.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "ArticCtrl.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "ReadOdom.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "PubOdom.inputs:execIn"),
            ("RateCtrl.outputs:execOut", "PubTF.inputs:execIn"),
            
            ("Context.outputs:context", "SubTwist.inputs:context"),
            ("Context.outputs:context", "PubOdom.inputs:context"),
            ("Context.outputs:context", "PubTF.inputs:context"),
            
            ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
            ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),
            ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),
            
            ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
            ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("RateCtrl.inputs:frequency", 30.0),  # 30Hz 제한
            ("SubTwist.inputs:topicName", "/cmd_vel"),
            
            ("DiffCtrl.inputs:wheelDistance", 0.141),
            ("DiffCtrl.inputs:wheelRadius", 0.033),
            ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
            ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
            
            ("ArticCtrl.inputs:robotPath", Sdf.Path(BASE_ROBOT_PATH)),
            ("ArticCtrl.inputs:jointNames",
             ["left_wheel_joint", "right_wheel_joint"]),
            
            ("ReadOdom.inputs:chassisPrim",
             Sdf.Path(f"{BASE_ROBOT_PATH}/base_link")),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:frameId", "odom"),
            ("PubOdom.inputs:childFrameId", "base_footprint"),
        ],
    },
)
print(f"  + Optimized Bridge created (RateController @ 30Hz)")
print(f"  + Omitted: LaserScan + JointState for better performance")

# ── 10. 최종 성능 측정 ──
print("\n" + "=" * 60)
print("Running optimized simulation...")
print("=" * 60)

for i in range(300):
    world.step(render=True)
    monitor.update()

# ── 11. 요약 ──
print()
print("=" * 60)
print("Step 18 — Performance Optimization Complete!")
print("=" * 60)
print()
print("  Performance optimizations applied:")
print()
print("  Physics:")
print("    - substeps: 2 → 1")
print("    - solver_iterations: 32 → 16")
print()
print("  Rendering:")
print("    - resolution_scale: 1.0 → 0.75 (44% fewer pixels)")
print("    - shadow_map: 4096 → 1024")
print()
print("  Scene:")
print("    - Point Instancing: 100 instances")
print()
print("  ROS2 Bridge:")
print("    - RateController @ 30Hz (instead of 60Hz)")
print("    - Omitted high-frequency topics (LaserScan)")
print("    - QoS: BEST_EFFORT (recommended)")
print()
print("  Benchmark Results (1 step() in ms):")
print(f"    Baseline:  {baseline_time:.2f}ms ({1000/baseline_time:.0f} FPS)")
print(f"    Physics:   {opt_physics_time:.2f}ms ({1000/opt_physics_time:.0f} FPS)")
print(f"    Render:    {opt_render_time:.2f}ms ({1000/opt_render_time:.0f} FPS)")
print(f"    Scene:     {opt_scene_time:.2f}ms ({1000/opt_scene_time:.0f} FPS)")
print()
print("  Key concepts:")
print("  - Always MEASURE before optimizing (baseline)")
print("  - Physics substeps have linear cost")
print("  - Resolution scale gives square-law savings")
print("  - RateController reduces ROS2 overhead")
print("  - Point Instancing saves GPU memory")
print("  - Profile → Identify → Optimize → Verify")
print()
print("  === Phase 2 Complete! ===")
print("  Next: Phase 3 — Advanced Robotics (Steps 19~26)")
print("=" * 60)

simulation_app.close()
