# Step 18 — Performance Optimization

> **소요 시간**: 90분
> **난이도**: ★★★★☆ (고급)
> **선수 조건**: Phase 1 완료, Phase 2 경험

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Isaac Sim 성능 모니터링** 도구를 사용한다 (FPS, 메모리, GPU)
2. **Stage 규모 최적화** (Instancing, LOD, Culling)를 적용한다
3. **Physics 설정**을 튜닝한다 (Substeps, Solver Iterations)
4. **Rendering 품질 vs 속도 트레이드오프**를 이해하고 최적점을 찾는다
5. **Carb Profiling Tools**로 병목 지점을 찾는다
6. **Multi-GPU 설정**과 **Distributed Computing**을 이해한다
7. **ROS2 Bridge 통신 성능**을 최적화한다
8. **OmniGraph Execution** 효율을 높인다

---

## 1. 성능 메트릭 개요

### 1.1 주요 성능 지표

| 지표 | 측정 대상 | 목표 값 |
|------|----------|---------|
| **FPS** | 렌더링 + 물리 | ≥ 30 FPS (실시간) |
| **Physics DT** | 물리 시뮬레이션 | 1/60 sec (16.67ms) |
| **GPU 메모리** | VRAM 사용량 | < 80% |
| **CPU 사용률** | Physics + Script | < 70% |
| **ROS2 메시지 지연** | Bridge 통신 | < 10ms |
| **OmniGraph 실행 시간** | Graph 노드 처리 | < 5ms |

### 1.2 성능 병목 유형

```
[Scene Complexity] → [Physics] → [Rendering] → [ROS2 Bridge]
       │                │             │              │
       ▼                ▼             ▼              ▼
  너무 많은 Prim    Solver 속도    Pixel 수/    메시지 직렬화/
  /Texture/LOD     Substeps       Shader 품질    발행 주기
```

---

## 2. 성능 모니터링 도구

### 2.1 Carb Profiling

Carb는 NVIDIA Omniverse의 프로파일링 프레임워크입니다.

```python
import carb
from omni.isaac.core.utils.extensions import enable_extension

# Profiling Extension 활성화
enable_extension("omni.kit.profiler")

# Carb Profiler 설정
carb.profiler.begin_frame("main_frame")

# 측정할 코드 블록
carb.profiler.begin("physics_step")
world.step(render=True)
carb.profiler.end("physics_step")

carb.profiler.end_frame("main_frame")

# 통계 출력 (선택)
stats = carb.profiler.get_statistics()
for name, data in stats.items():
    if data["avg"] > 5:  # 5ms 이상 걸리는 항목 출력
        print(f"  ⚠ {name}: avg={data['avg']:.2f}ms, "
              f"max={data['max']:.2f}ms")
```

### 2.2 Isaac Sim Built-in Stats

```python
def get_performance_stats():
    """Isaac Sim 내장 성능 통계"""
    
    stats = {}
    
    # FPS
    from omni.kit.window.stats import get_instance
    perf = get_instance()
    if perf:
        stats['fps'] = perf.fps
    
    # Physics
    from omni.physx.physx import PhysX
    physx = PhysX()
    stats['physics_fps'] = physx.get_physics_fps()
    stats['physics_dt'] = physx.get_physics_dt()
    
    # Stage Statistics
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if stage:
        stats['prim_count'] = len(stage.TraverseAll())
    
    # GPU Memory (NVIDIA-SMI)
    import subprocess
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=memory.used,memory.total,utilization.gpu',
             '--format=csv,noheader,nounits'],
            capture_output=True, text=True, timeout=2,
        )
        if result.returncode == 0:
            mem_used, mem_total, gpu_util = map(float, result.stdout.strip().split(', '))
            stats['gpu_memory_used_mb'] = mem_used
            stats['gpu_memory_total_mb'] = mem_total
            stats['gpu_util_pct'] = gpu_util
    except Exception:
        pass
    
    return stats
```

### 2.3 실시간 모니터링

```python
class PerformanceMonitor:
    """실시간 성능 모니터 (오버레이 표시)"""
    
    def __init__(self, window_name="Performance"):
        from omni.kit.window.stats import get_instance
        self.stats = get_instance()
        
        self.fps_history = []
        self.physics_history = []
        self.log_interval = 100  # frames
    
    def update(self, frame_count):
        """매 프레임 호출"""
        
        if frame_count % self.log_interval == 0:
            stats = get_performance_stats()
            
            print(f"[PERF] Frame {frame_count}:")
            print(f"  FPS: {stats.get('fps', 'N/A'):.1f}")
            print(f"  Physics FPS: {stats.get('physics_fps', 'N/A')}")
            print(f"  Prims: {stats.get('prim_count', 'N/A')}")
            print(f"  GPU Mem: {stats.get('gpu_memory_used_mb', 'N/A'):.0f}/"
                  f"{stats.get('gpu_memory_total_mb', 'N/A'):.0f} MB")
            print(f"  GPU Util: {stats.get('gpu_util_pct', 'N/A'):.1f}%")
```

---

## 3. Stage 최적화

### 3.1 Prim Count 관리

```python
def optimize_stage():
    """Stage 규모 최적화"""
    
    stage = omni.usd.get_context().get_stage()
    
    # 1. Prim 수 확인
    prim_count = len(list(stage.TraverseAll()))
    print(f"  Current prims: {prim_count}")
    
    if prim_count > 10000:
        print("  ⚠ High prim count! Consider:")
        print("    - Point instancing")
        print("    - LOD (Level of Detail)")
        print("    - Remove hidden prims")
    
    # 2. 중복/불필요 Prim 제거
    for prim in stage.TraverseAll():
        # Visibility OFF prims 제거
        vis_attr = prim.GetAttribute("visibility")
        if vis_attr and vis_attr.Get() == "invisible":
            stage.RemovePrim(prim.GetPath())
            print(f"  - Removed invisible prim: {prim.GetPath()}")
    
    # 3. Active 여부 확인
    for prim in stage.TraverseAll():
        if not prim.IsActive():
            stage.RemovePrim(prim.GetPath())
    
    return prim_count
```

### 3.2 Point Instancing

```python
def setup_point_instancing(mesh_path, count=100):
    """Point Instancing으로 중복 메모리 최소화"""
    
    from pxr import UsdGeom
    
    stage = omni.usd.get_context().get_stage()
    
    # Instancer 생성
    instancer_path = "/World/Instancer"
    instancer = UsdGeom.PointInstancer.Define(stage, instancer_path)
    
    # Instance할 Proto (원본 Mesh)
    proto_path = instancer_path + "/Proto"
    instancer.CreatePrototypesRel().AddTarget(proto_path)
    
    # 위치 배열 (GPT 점)
    positions = []
    for i in range(count):
        x = (i % 10) * 0.3 - 1.5
        y = (i // 10) * 0.3 - 1.5
        positions.append((x, y, 0.0))
    
    # Instance 위치 설정
    positions_attr = instancer.CreatePositionsAttr()
    positions_attr.Set(positions)
    
    # Instance 수
    instancer.CreateProtoIndicesAttr().Set([0] * count)
    
    print(f"  + Point instancing: {count} instances of {mesh_path}")
```

### 3.3 LOD (Level of Detail)

```python
def setup_lod(mesh_paths, distances):
    """LOD 설정 (거리에 따른 상세도)"""
    
    from pxr import UsdLux, UsdGeom
    
    stage = omni.usd.get_context().get_stage()
    
    # LOD Group 생성
    lod_path = "/World/LOD_Group"
    lod_prim = UsdGeom.Xform.Define(stage, lod_path)
    
    # LOD 모델 추가
    for i, (mesh_path, distance) in enumerate(zip(mesh_paths, distances)):
        # LOD 자식 Prim
        child_path = f"{lod_path}/LOD_{i}"
        
        if is_prim_path_valid(mesh_path):
            # Reference 복사
            from omni.isaac.core.utils.stage import add_reference_to_stage
            add_reference_to_stage(mesh_path, child_path)
        
        # LOD Switch 설정
        lod_prim_prim = stage.GetPrimAtPath(child_path)
        if lod_prim_prim:
            switch = lod_prim_prim.CreateAttribute(
                "lodVisibility", 
                Sdf.ValueTypeNames.Float3Array,
            )
            switch.Set([(distance, distance * 2, 0)])  # visible_range
    
    print(f"  + LOD Group with {len(mesh_paths)} levels")
```

---

## 4. Physics 최적화

### 4.1 Physics 파라미터 튜닝

```python
def tune_physics(enable_ccd=False, substeps=2, solver_iterations=32):
    """Physics 성능 튜닝"""
    
    from omni.physx.physx import PhysX
    
    physx = PhysX()
    
    # Physics DT (60Hz = 16.67ms)
    physx.set_physics_dt(1.0 / 60.0)
    
    # Substeps (1 = 저품질/고속, 4 = 고품질/저속)
    physx.set_num_substeps(substeps)
    
    # Solver Iterations (4=빠름, 32=정확)
    physx.set_solver_iteration_counts(solver_iterations)
    
    # CCD (Continuous Collision Detection)
    # 빠른 물체 충돌 정확도 향상, CPU 부하 증가
    physx.set_ccd_enabled(enable_ccd)
    
    # GPU 가속 Physics (대규모 Scene)
    physx.set_gpu_enabled(True)
    
    print(f"  Physics tuned:")
    print(f"    DT: 1/{int(1/physx.get_physics_dt())}s")
    print(f"    Substeps: {substeps}")
    print(f"    Solver Iterations: {solver_iterations}")
    print(f"    CCD: {enable_ccd}")
```

### 4.2 최적화 추천 값

| 시나리오 | Substeps | Solver Iterations | CCD | FPS 목표 |
|----------|----------|-------------------|-----|---------|
| 간단한 Scene (1-2 로봇) | 1-2 | 16-32 | OFF | 60 |
| 복잡한 Scene (3+ 로봇) | 1 | 8-16 | OFF | 30-60 |
| 충돌 정밀도 필요 | 2-4 | 32-64 | ON | 30 |
| SLAM/Nav2 (실시간) | 1 | 8 | OFF | 60 |
| JSON 데이터 생성 | 1 | 16 | OFF | 30-60 |

### 4.3 Articulation 최적화

```python
def optimize_articulation(robot_path):
    """Articulation 설정 최적화"""
    
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_path)
    
    if not robot_prim:
        return
    
    # Articulation 설정
    art = robot_prim.GetAttribute("physxArticulation")
    if art:
        # Solver Position Iterations
        art.Set(32)
    
    # 각 Joint 최적화
    for prim in stage.TraverseAll():
        if prim.GetPath().pathString.startswith(robot_path):
            # Motor Damping (진동 감소, 성능 향상)
            damping = prim.GetAttribute("drive:damping")
            if damping and damping.Get() < 0.1:
                damping.Set(0.1)  # 최소 댐핑
            
            # Stiffness (강성)
            stiffness = prim.GetAttribute("drive:stiffness")
            if stiffness and stiffness.Get() > 10000:
                stiffness.Set(5000)  # 적정 수준으로 감소
    
    print(f"  + Articulation optimized: {robot_path}")
```

---

## 5. Rendering 최적화

### 5.1 Renderer 선택

```python
from omni.kit.renderer import Renderer

def optimize_renderer(mode="balanced"):
    """Renderer 설정 최적화"""
    
    if mode == "performance":
        # 최고 성능 (RT off)
        config = {
            "renderer": "Performance",
            "shadows": False,
            "ambient_occlusion": False,
            "anti_aliasing": 0,
            "texture_quality": "low",
            "material_quality": "low",
        }
    elif mode == "balanced":
        # 균형
        config = {
            "renderer": "RayTracedLighting",
            "shadows": True,
            "ambient_occlusion": True,
            "anti_aliasing": 2,
            "texture_quality": "medium",
            "material_quality": "medium",
        }
    elif mode == "quality":
        # 최고 품질
        config = {
            "renderer": "PathTracing",
            "shadows": True,
            "ambient_occlusion": True,
            "anti_aliasing": 4,
            "texture_quality": "high",
            "material_quality": "high",
            "path_tracing_samples": 8,
        }
    
    # Renderer 변경
    renderer = Renderer()
    renderer.set_renderer(config["renderer"])
    renderer.set_quality_level(config["material_quality"])
    
    print(f"  Renderer set to '{mode}' mode:")
    for k, v in config.items():
        print(f"    {k}: {v}")

# 사용 예
optimize_renderer("performance")  # 최고 성능
optimize_renderer("balanced")     # 기본 추천
```

### 5.2 렌더링 튜닝 파라미터

```python
def tune_rendering_parameters():
    """세부 렌더링 파라미터 튜닝"""
    
    from omni.kit.viewport import Viewport
    
    vp = Viewport()
    
    vp.set_resolution_scale(1.0)  # 1.0=100%, 0.5=50%
    
    # Resolution Scale vs FPS Trade-off:
    # 1.0 = 1920x1080 (기준)
    # 0.75 = 1440x810 (35% 성능 향상)
    # 0.5 = 960x540 (70% 성능 향상)
    
    # Shadow Map Size
    vp.set_shadow_map_size(1024)  # 1024=빠름, 4096=고품질
    
    # Draw Distance
    vp.set_draw_distance(100.0)  # 먼 거리 culling
    
    # Backface Culling
    vp.set_backface_culling(True)
    
    print(f"  Rendering tuned:")
    print(f"    Resolution: 1920x1080")
    print(f"    Shadow Map: 1024")
    print(f"    Draw Distance: 100m")
```

---

## 6. ROS2 Bridge 성능 최적화

### 6.1 발행 주기 조정

```python
def optimize_ros2_bridge(publish_rate=30):
    """ROS2 Bridge 성능 최적화"""
    
    # Tick Rate 조정
    # OnPlaybackTick의 기본 rate는 physics dt (60Hz)
    # 하지만 ROS2 publish rate는 더 낮아도 됨
    
    # OmniGraph에서 publish interval 제어
    # RateController 노드 추가
    import omni.graph.core as og
    
    graph_config = {
        "graph_path": "/ActionGraph/OptimizedBridge",
        "evaluator_name": "execution",
    }
    
    og.Controller.edit(
        graph_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("RateCtrl", "omni.graph.nodes.RateController"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "RateCtrl.inputs:execIn"),
                ("RateCtrl.outputs:execOut", "PubOdom.inputs:execIn"),
                ("Context.outputs:context", "PubOdom.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RateCtrl.inputs:frequency", publish_rate),
                ("PubOdom.inputs:topicName", "/odom"),
            ],
        },
    )
    
    print(f"  + ROS2 Bridge optimized: publish rate = {publish_rate}Hz")
    
    # 데이터 압축 (선택)
    # ROS2 메시지 압축 설정
    # export ROS2_MESSAGE_COMPRESSION=lz4 (환경 변수)
```

### 6.2 QoS 프로파일 최적화

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 QoS 최적화
def get_optimized_qos():
    """성능 최적화 QoS 설정"""
    
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,  # 손실 허용 (더 빠름)
        history=HistoryPolicy.KEEP_LAST,             # 최신 메시지만 유지
        depth=1,                                     # 1개만 유지 (메모리 절약)
        durability=None,                              # Volatile
    )
    
    return qos

# 사용 예
qos = get_optimized_qos()
publisher = node.create_publisher(
    LaserScan, '/scan', qos_profile=qos)
```

### 6.3 대규모 LiDAR 데이터 최적화

```python
def optimize_lidar_publishing():
    """LiDAR 스캔 발행 최적화"""
    
    # LiDAR 포인트 수 감소
    # 원본: 360° / 0.5° = 720 points
    # 최적화: 360° / 1° = 360 points (50% 감소)
    
    stage = omni.usd.get_context().get_stage()
    lidar_prim = stage.GetPrimAtPath(
        "/World/TurtleBot3/base_scan/Lidar")
    
    if lidar_prim:
        # LiDAR 분해능 조정
        lidar_prim.GetAttribute("horizontalFov").Set(360.0)
        lidar_prim.GetAttribute("rotationRate").Set(5.0)  # 10→5Hz
        
        # Range 제한 (불필요한 먼 거리 데이터 감소)
        lidar_prim.GetAttribute("range").Set(3.5)  # 3.5m
        
        print("  + LiDAR optimized: 5Hz, 3.5m range")
```

### 6.4 ROS2 통신 성능 측정

```python
class Ros2LatencyMonitor(Node):
    """ROS2 Bridge 통신 지연 측정"""
    
    def __init__(self):
        super().__init__('latency_monitor')
        
        self.cmd_vel_latency = []
        self.odom_latency = []
        self.scan_latency = []
        
        self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10)
    
    def on_cmd_vel(self, msg):
        now = self.get_clock().now()
        stamp = msg.header.stamp
        delay = (now - stamp).nanoseconds / 1e6  # ms
        self.cmd_vel_latency.append(delay)
    
    def print_stats(self):
        for name, data in [
            ('cmd_vel', self.cmd_vel_latency),
            ('odom', self.odom_latency),
            ('scan', self.scan_latency),
        ]:
            if data:
                avg = sum(data) / len(data)
                mx = max(data)
                print(f"  {name}: avg={avg:.1f}ms, max={mx:.1f}ms")
```

---

## 7. OmniGraph Execution 최적화

### 7.1 Graph 평가 모드

```python
def optimize_graph_evaluation(graph_path):
    """OmniGraph 평가 모드 최적화"""
    
    import omni.graph.core as og
    
    graph = og.Graph(graph_path)
    
    # 실행 모드 설정
    # "execution": 매 틱마다 실행 (기본)
    # "simulation": 시뮬레이션 루프에서만 실행 (성능 우선)
    graph.set_evaluator_type("execution")  # 그대로 유지
    
    # 병렬 실행 활성화
    try:
        graph.set_execution_parallel_enabled(True)
        print(f"  + Graph parallel execution enabled: {graph_path}")
    except Exception as e:
        print(f"  ⚠ Parallel execution not supported: {e}")
```

### 7.2 필요한 노드만 유지

```python
def prune_unnecessary_nodes(graph_path):
    """불필요한 OmniGraph 노드 제거"""
    
    import omni.graph.core as og
    
    graph = og.Graph(graph_path)
    nodes = graph.get_nodes()
    
    print(f"  Current nodes: {len(nodes)}")
    
    # ROS2 Bridge 최소 구성 권장:
    # 필요한 노드만 유지하고 불필요한 시각화 노드 제거
    
    essential_types = [
        "omni.graph.action.OnPlaybackTick",
        "omni.isaac.ros2_bridge.ROS2Context",
        "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
        "omni.isaac.core_nodes.IsaacDifferentialController",
        "omni.isaac.core_nodes.IsaacArticulationController",
        "omni.isaac.core_nodes.IsaacReadOdometry",
        "omni.isaac.ros2_bridge.ROS2PublishOdometry",
        "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
        "omni.isaac.ros2_bridge.ROS2PublishLaserScan",
        "omni.isaac.core_nodes.IsaacReadLaserScan",
    ]
    
    return len(nodes)
```

---

## 8. Multi-GPU 설정

### 8.1 NVIDIA Omniverse Multi-GPU

```python
def setup_multi_gpu():
    """Multi-GPU 설정"""
    
    from omni.kit.gpu_affinity import GpuAffinity
    
    # GPU 개수 확인
    gpu_affinity = GpuAffinity()
    gpu_count = gpu_affinity.get_gpu_count()
    print(f"  Available GPUs: {gpu_count}")
    
    if gpu_count > 1:
        # GPU 할당
        # GPU 0: Physics + Rendering
        # GPU 1: Synthetic Data Generation
        # GPU 2: ROS2 Bridge + Inference
        
        gpu_affinity.set_primary_gpu(0)  # Main rendering GPU
        
        print(f"  Multi-GPU configured:")
        print(f"    GPU 0: Primary (Rendering + Physics)")
        if gpu_count > 1:
            print(f"    GPU 1: Secondary")
        if gpu_count > 2:
            print(f"    GPU 2: Tertiary")
```

### 8.2 Scalability Options

```python
def configure_scalability(mode="single_gpu"):
    """확장성 설정"""
    
    config = {
        "single_gpu": {
            "use_gpu": True,
            "gpu_count": 1,
            "physics_gpu": 0,
        },
        "multi_gpu_shared": {
            "use_gpu": True,
            "gpu_count": 2,
            "physics_gpu": 0,
        },
        "multi_gpu_dedicated": {
            "use_gpu": True,
            "gpu_count": 2,
            "physics_gpu": 0,
        },
    }
    
    settings = config.get(mode, config["single_gpu"])
    
    # PhysX GPU 설정
    from omni.physx.physx import PhysX
    physx = PhysX()
    physx.set_gpu_enabled(True)
    
    print(f"  Scalability: {mode}")
    return settings
```

---

## 9. 성능 체크리스트

### 9.1 Scene 최적화

- [ ] Prim 수 < 10,000
- [ ] Point Instancing 사용 (중복 메시)
- [ ] 불필요한 Prim 제거 (invisible, inactive)
- [ ] Texture 크기 제한 (2048x2048 이하)
- [ ] LOD 설정 (멀리 있는 객체 저해상도)

### 9.2 Physics 최적화

- [ ] Substeps = 1 (가능하면)
- [ ] Solver Iterations = 16-32
- [ ] CCD = OFF (고속 물체 없으면)
- [ ] Articulation 댐핑 최소값 설정
- [ ] Physics DT = 1/60 (실시간)

### 9.3 Rendering 최적화

- [ ] Renderer = "Performance" 또는 "RayTracedLighting"
- [ ] Resolution Scale = 0.75-1.0
- [ ] Shadow Map = 1024
- [ ] Anti-aliasing OFF
- [ ] Headless mode (데이터 생성 시)

### 9.4 ROS2 Bridge 최적화

- [ ] Publish Rate = 15-30Hz (불필요하게 높지 않게)
- [ ] QoS = BEST_EFFORT
- [ ] 필요한 토픽만 발행
- [ ] LiDAR Rate = 5-10Hz
- [ ] 메시지 크기 최적화

---

## 10. 실습: 성능 최적화 실행

### 10.1 전체 실행

```bash
# 성능 측정 및 최적화
cd ~/isaac-sim
./python.sh ~/isaac-step-curriculum/code/phase-2/step18_performance.py

# NVIDIA-SMI 모니터링 (별도 터미널)
watch -n 0.5 nvidia-smi

# ROS2 통신 대역폭 모니터링
source /opt/ros/humble/setup.bash
ros2 topic bw /odom
ros2 topic bw /scan
```

### 10.2 최적화 전/후 비교

```python
def benchmark_performance(func, label="Benchmark"):
    """성능 측정"""
    
    import time
    
    # Warm-up
    for _ in range(10):
        func()
    
    # 측정
    times = []
    for _ in range(100):
        start = time.perf_counter()
        func()
        end = time.perf_counter()
        times.append((end - start) * 1000)  # ms
    
    avg = sum(times) / len(times)
    print(f"  {label}: {avg:.2f}ms avg")
    return avg
```

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 성능 모니터링 | Carb Profiling, FPS, GPU 메모리 |
| ✅ Stage 최적화 | Prim Count, Instancing, LOD |
| ✅ Physics 튜닝 | Substeps, Solver, CCD |
| ✅ Rendering 튜닝 | Renderer 선택, Resolution Scale |
| ✅ ROS2 최적화 | Publish Rate, QoS, LiDAR 설정 |
| ✅ Multi-GPU | GPU 할당, 분산 처리 |
| ✅ OmniGraph | 병렬 실행, 노드 최적화 |
| ✅ Benchmarking | 전/후 성능 비교 |

### 최적화 체계

```
1. MEASURE (현재 성능 측정)
   ↓
2. IDENTIFY (병목 지점 파악)
   ↓
3. OPTIMIZE (최적화 적용)
   ↓
4. MEASURE (다시 측정, 개선 확인)
   ↓
5. REPEAT (필요시 반복)
```

---

## 12. Phase 2 완료

**축하합니다! Phase 2 (ROS2)를 완료했습니다.**

이제 다음을 할 수 있습니다:
| Step | 주제 | 역량 |
|------|------|------|
| 11 | ROS2 Bridge | Isaac Sim ↔ ROS2 통신 |
| 12 | TurtleBot3 Teleop | /cmd_vel, /odom, /tf |
| 13 | SLAM | LiDAR Mapping |
| 14 | Nav2 | 자율 주행 |
| 15 | MoveIt2 | 로봇팔 모션 계획 |
| 16 | Multi-Robot | Namespace, 충돌 회피 |
| 17 | Synthetic Data | Replicator, Domain Randomization |
| 18 | Performance | 최적화, Profiling |

**다음: Phase 3 — Advanced Robotics (Steps 19~26)**
- Digital Twin, Humanoid, Warehouse, AI Worker
- 실제 산업용 프로젝트로 확장

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim 성능 가이드 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/performance/index.html |
| Carb Profiling | https://docs.omniverse.nvidia.com/prod_kit/prod/kit/prod_kit/profiling.html |
| PhysX Settings | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/settings.html |
| Rendering 설정 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/rendering/ |
| ROS2 QoS | https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html |
| Multi-GPU | https://docs.omniverse.nvidia.com/kit/docs/omni.kit.gpu_affinity/latest/ |
