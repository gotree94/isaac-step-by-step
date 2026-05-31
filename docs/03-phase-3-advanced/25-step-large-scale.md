# Step 25 — Large-Scale Simulation

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 24 (ROS2 Advanced), Step 21 (Warehouse)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **대규모 로봇 Fleet**을 Isaac Sim에서 동시 운용한다 (10+ 로봇)
2. **Distributed Simulation** 아키텍처를 이해한다
3. **ROS2 Multi-Host** 통신을 구성한다
4. **Performance Profiling**으로 병목을 식별한다
5. **Omniverse Farm**으로 클라우드 렌더링을 설정한다
6. **Kubernetes**에서 Isaac Sim 컨테이너를 오케스트레이션한다
7. **대규모 Warehouse** 시뮬레이션을 최적화한다

---

## 1. Large-Scale 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Large-Scale Isaac Sim                              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │  Sim Node 1   │  │  Sim Node 2   │  │  Sim Node 3   │  ... (N)    │
│  │  (GPU 0)      │  │  (GPU 1)      │  │  (GPU 2)      │             │
│  │  ┌────────┐   │  │  ┌────────┐   │  │  ┌────────┐   │             │
│  │  │Robots  │   │  │  │Robots  │   │  │  │Robots  │   │             │
│  │  │1-5     │   │  │  │6-10    │   │  │  │11-15   │   │             │
│  │  └────────┘   │  │  └────────┘   │  │  └────────┘   │             │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘             │
│         │                 │                 │                        │
│  ┌──────▼─────────────────▼─────────────────▼──────────────┐        │
│  │              ROS2 Distributed Network                    │        │
│  │  /robot1/cmd_vel  /robot2/odom  /robot3/joint_states     │        │
│  └──────────────────────────────────────────────────────────┘        │
│                                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              Orchestration Layer                                │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │   │
│  │  │Kubernetes│  │Omniverse │  │Monitoring│                    │   │
│  │  │          │  │Farm      │  │(Grafana) │                    │   │
│  │  └──────────┘  └──────────┘  └──────────┘                    │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.1 확장 전략

| 전략 | 설명 | 한계 |
|------|------|------|
| **단일 GPU Fleet** | 1개 GPU에 여러 로봇 | GPU 메모리 (보통 16-80 로봇) |
| **Multi-GPU** | GPU별 로봇 분산 | NVLink/InfiniBand 필요 |
| **Distributed** | 여러 머신에 분산 | 네트워크 지연 |
| **Cloud (K8s)** | 컨테이너 오케스트레이션 | GPU 비용 |

### 1.2 성능 벤치마크

| 설정 | 로봇 수 | FPS | GPU 메모리 |
|------|---------|-----|------------|
| TurtleBot3 단순 | 100 | 30+ | 4GB |
| Franka Panda + 물리 | 32 | 30 | 8GB |
| Humanoid (GR1) | 16 | 30 | 12GB |
| Mixed Fleet | 50 | 20 | 16GB |

---

## 2. Multi-GPU Fleet

### 2.1 GPU별 로봇 분산

```python
class MultiGPUIsland:
    """GPU별 로봇 Fleet 할당"""
    
    def __init__(self, gpu_id, island_size=5):
        self.gpu_id = gpu_id
        self.island_size = island_size
        self.robots = []
        
        # GPU별 Isaac Sim Instance (별도 프로세스)
        import os
        os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
        
        print(f"  + GPU Island {gpu_id}: ready for {island_size} robots")
    
    def deploy_robots(self, start_id=0):
        """Island 내 로봇 배치"""
        for i in range(self.island_size):
            robot_id = start_id + i
            x = (i % 5) * 2.0 - 4.0
            y = (i // 5) * 2.0 - 2.0
            
            robot_path = f"/World/Robots/Robot_{robot_id:03d}"
            if not is_prim_path_valid(robot_path):
                add_reference_to_stage(
                    "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
                    robot_path,
                )
            
            robot = Robot(
                prim_path=robot_path,
                name=f"Robot_{robot_id:03d}",
                position=np.array([x, y, 0.1]),
            )
            world.scene.add(robot)
            self.robots.append(robot)
        
        print(f"  + {self.island_size} robots deployed on GPU {self.gpu_id}")

# 사용 예
# islands = [MultiGPUIsland(0, 5), MultiGPUIsland(1, 5), MultiGPUIsland(2, 5)]
```

### 2.2 Fleet Manager

```python
class FleetManager:
    """로봇 Fleet 전체 관리"""
    
    def __init__(self):
        self.robots = []
        self.robot_count = 0
        self.max_robots = 50
        self.fleet_status = {}
    
    def spawn_fleet(self, count, robot_type="turtlebot3"):
        """Fleet 생성"""
        actual_count = min(count, self.max_robots)
        
        for i in range(actual_count):
            robot_id = len(self.robots)
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, 5)
            
            robot = self._create_robot(robot_id, robot_type, x, y)
            self.robots.append(robot)
        
        self.robot_count = len(self.robots)
        print(f"  + Fleet: {self.robot_count} robots spawned")
        return self.robot_count
    
    def _create_robot(self, robot_id, robot_type, x, y):
        """개별 로봇 생성"""
        robot_path = f"/World/Fleet/Robot_{robot_id:04d}"
        
        usd_map = {
            "turtlebot3": "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
            "franka": "/Isaac/Robots/Franka/franka_alt_fingers.usd",
            "carter": "/Isaac/Robots/Carter/carter.usd",
        }
        
        usd_path = usd_map.get(robot_type, usd_map["turtlebot3"])
        
        if not is_prim_path_valid(robot_path):
            add_reference_to_stage(usd_path, robot_path)
        
        robot = Robot(
            prim_path=robot_path,
            name=f"Robot_{robot_id:04d}",
            position=np.array([x, y, 0.1]),
        )
        world.scene.add(robot)
        
        # Fleet 상태 등록
        self.fleet_status[robot_id] = {
            'type': robot_type,
            'position': (x, y),
            'status': 'idle',
            'battery': 100.0,
        }
        
        return robot
    
    def random_walk(self, dt=1/60.0):
        """Fleet 전체 Random Walk"""
        for i, robot in enumerate(self.robots):
            # Random velocity
            vx = np.random.uniform(-0.3, 0.3)
            vy = np.random.uniform(-0.3, 0.3)
            
            action = ArticulationAction(
                joint_velocities=np.array([vx, vx, vy * 2]),
            )
            robot.apply_action(action)
            
            # Status 업데이트
            pos, _ = robot.get_world_pose()
            self.fleet_status[i]['position'] = (pos[0], pos[1])
    
    def get_fleet_stats(self):
        """Fleet 통계"""
        stats = {
            'total': self.robot_count,
            'active': sum(1 for s in self.fleet_status.values() 
                         if s['status'] == 'active'),
            'idle': sum(1 for s in self.fleet_status.values() 
                       if s['status'] == 'idle'),
            'avg_battery': np.mean([s['battery'] 
                                   for s in self.fleet_status.values()]),
        }
        return stats
```

---

## 3. Distributed Simulation

### 3.1 Multi-Host ROS2

```bash
# ════════════════════════════════════════════════════════
# ROS2 Distributed Setup (Multi-Host)
# ════════════════════════════════════════════════════════

# Host 1 (192.168.1.10) — Isaac Sim Master
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop; ros2 daemon start
./python.sh ~/isaac-step-curriculum/code/phase-3/step25_large_scale.py

# Host 2 (192.168.1.11) — Isaac Sim Worker
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop; ros2 daemon start
./python.sh ~/isaac-step-curriculum/code/phase-3/step25_large_scale.py \
  --worker --worker-id 1

# Host 3 (192.168.1.12) — Monitoring
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 topic list
ros2 topic hz /robot_001/odom
```

### 3.2 Fast DDS Configuration

```xml
<!-- fastdds.xml — DDS 최적화 -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <!-- UDP Multicast 비활성화 (대규모 환경) -->
        <transport_descriptor>
            <transport_id>UDPTransport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>65536</sendBufferSize>
            <receiveBufferSize>65536</receiveBufferSize>
            <maxMessageSize>65000</maxMessageSize>
            <TTL>1</TTL>
        </transport_descriptor>
        
        <!-- Shared Memory (동일 호스트 최적화) -->
        <transport_descriptor>
            <transport_id>ShmTransport</transport_id>
            <type>SHM</type>
            <segment_size>2097152</segment_size>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="isaac_fleet_participant" 
                 is_default_profile="true">
        <rtps>
            <builtin>
                <domainId>42</domainId>
                <leaseDuration>5.0</leaseDuration>
                <leaseAnnouncement>
                    <period>1.0</period>
                </leaseAnnouncement>
                <metatrafficUnicastLocatorList>
                    <locator/>
                </metatrafficUnicastLocatorList>
                <initialAnnouncements>
                    <count>1</count>
                </initialAnnouncements>
            </builtin>
            <userTransports>
                <transport_id>ShmTransport</transport_id>
                <transport_id>UDPTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### 3.3 Distributed Sync

```python
class DistributedSync(Node):
    """분산 시뮬레이션 동기화"""
    
    def __init__(self, worker_id=0, total_workers=3):
        super().__init__('distributed_sync')
        
        self.worker_id = worker_id
        self.total_workers = total_workers
        self.sim_time = 0.0
        self.sync_interval = 0.1  # 100ms sync
        
        # Sync service
        self.sync_srv = self.create_service(
            SimSync, f'/worker_{worker_id}/sync',
            self.handle_sync)
        
        # Clock publisher (Master only)
        if worker_id == 0:
            self.clock_pub = self.create_publisher(
                Clock, '/clock', 10)
            self.sync_timer = self.create_timer(
                self.sync_interval, self.broadcast_clock)
        
        # Worker registration
        self.registration_client = self.create_client(
            RegisterWorker, '/master/register_worker')
        
        self.get_logger().info(
            f'DistributedSync: Worker {worker_id}/{total_workers}')
    
    def handle_sync(self, request, response):
        """동기화 요청 처리"""
        response.sim_time = self.sim_time
        response.worker_count = self.total_workers
        response.paused = request.paused
        return response
    
    def broadcast_clock(self):
        """Master → Workers 클록 동기화"""
        msg = Clock()
        msg.clock = rclpy.time.Time(
            seconds=self.sim_time).to_msg()
        self.clock_pub.publish(msg)
        self.sim_time += self.sync_interval
    
    def register_with_master(self, master_ip="192.168.1.10"):
        """Master 등록"""
        req = RegisterWorker.Request()
        req.worker_id = self.worker_id
        req.ip_address = self.get_ip()
        req.capabilities = {
            'gpu_memory': 24,
            'max_robots': 10,
            'has_franka': True,
        }
        
        while not self.registration_client.wait_for_service(
                timeout_sec=1.0):
            self.get_logger().warn('Waiting for master...')
        
        future = self.registration_client.call_async(req)
```

---

## 4. Performance Optimization

### 4.1 성능 프로파일링

```python
class IsaacProfiler:
    """Isaac Sim Performance Profiler"""
    
    def __init__(self):
        self.metrics = {}
        self.start_time = time.time()
        self.frame_count = 0
        
        # Profiling hooks
        self.profiler = omni.kit.profiler.Profiler()
    
    def start_profile(self, name):
        """프로파일링 시작"""
        self.metrics[name] = {
            'start': time.perf_counter(),
            'count': 0,
        }
    
    def end_profile(self, name):
        """프로파일링 종료"""
        if name in self.metrics:
            elapsed = time.perf_counter() - self.metrics[name]['start']
            self.metrics[name]['elapsed'] = elapsed
            self.metrics[name]['count'] += 1
    
    def get_fps(self):
        """FPS 계산"""
        elapsed = time.time() - self.start_time
        return self.frame_count / elapsed if elapsed > 0 else 0
    
    def report(self):
        """프로파일링 리포트"""
        print("\n  ═══════ Performance Report ═══════")
        print(f"  FPS: {self.get_fps():.1f}")
        print(f"  Total frames: {self.frame_count}")
        
        for name, data in sorted(self.metrics.items()):
            if 'elapsed' in data and data['count'] > 0:
                avg = data['elapsed'] / data['count']
                print(f"  {name}: {data['elapsed']*1000:.1f}ms "
                      f"({data['count']} calls, avg {avg*1000:.3f}ms)")
    
    def frame_tick(self):
        """프레임마다 호출"""
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.report()
```

### 4.2 최적화 기법

```python
class LargeScaleOptimizer:
    """대규모 시뮬레이션 최적화"""
    
    @staticmethod
    def disable_rendering_for_distant(robot_pos, camera_pos, max_dist=20):
        """카메라에서 먼 로봇 렌더링 비활성화"""
        for robot_path, pos in robot_pos.items():
            dist = np.linalg.norm(pos - camera_pos)
            prim = stage.GetPrimAtPath(robot_path)
            if prim:
                vis_attr = prim.GetAttribute("visibility")
                if vis_attr:
                    if dist > max_dist:
                        vis_attr.Set("invisible")
                    else:
                        vis_attr.Set("inherited")
    
    @staticmethod
    def reduce_physics_rate_for_idle(fleet_status):
        """유휴 로봇 물리 속도 감소"""
        for robot_id, status in fleet_status.items():
            if status['status'] == 'idle':
                # Skeletal animation 비활성화 등
                pass
    
    @staticmethod
    def use_instanced_meshes():
        """Instanced Mesh 사용 (동일 USD 공유)"""
        import omni.usd
        from pxr import UsdGeom
        
        # USD Referencing: 동일 에셋 참조
        stage = omni.usd.get_context().get_stage()
        # Use Sdf.Reference for identical robot meshes
```

### 4.3 GPU 메모리 관리

```python
class GPUMemoryManager:
    """GPU 메모리 관리"""
    
    def __init__(self, total_gb=24, warning_threshold=0.85):
        self.total_gb = total_gb
        self.warning_threshold = warning_threshold
        self.allocations = {}
    
    def track_allocation(self, name, size_gb):
        """메모리 할당 추적"""
        self.allocations[name] = size_gb
        total = sum(self.allocations.values())
        
        if total / self.total_gb > self.warning_threshold:
            print(f"  ⚠ GPU memory warning: {total:.1f}/{self.total_gb}GB")
        
        return total
    
    def get_available(self):
        """사용 가능한 GPU 메모리 (simulated)"""
        used = sum(self.allocations.values())
        return self.total_gb - used
    
    def optimize(self):
        """메모리 최적화"""
        # 불필요한 텍스처 언로드
        # Cache flushing
        # Mipmap 조정
        freed = 0
        for name in list(self.allocations.keys()):
            if self.allocations[name] < 0.1:
                freed += self.allocations.pop(name, 0)
        
        if freed > 0:
            print(f"  + Freed {freed:.2f}GB GPU memory")
        
        return freed
```

---

## 5. Omniverse Farm

### 5.1 Farm Job 설정

```python
class OmniverseFarmJob:
    """Omniverse Farm 시뮬레이션 Job"""
    
    def __init__(self, job_name="warehouse_sim"):
        self.job_name = job_name
        self.tasks = []
        self.status = "pending"
    
    def add_task(self, task_name, robot_count=5, duration=60):
        """Farm Task 추가"""
        task = {
            'name': task_name,
            'robot_count': robot_count,
            'duration': duration,
            'gpu_required': 16,  # GB
            'status': 'pending',
        }
        self.tasks.append(task)
        return task
    
    def distribute_to_workers(self, workers):
        """Worker에 Task 분배"""
        for i, task in enumerate(self.tasks):
            worker = workers[i % len(workers)]
            task['worker'] = worker
            task['status'] = 'running'
            print(f"  + Task '{task['name']}' → {worker}")
    
    def collect_results(self):
        """결과 수집"""
        results = []
        for task in self.tasks:
            results.append({
                'task': task['name'],
                'robots': task['robot_count'],
                'duration': task['duration'],
                'metrics': {
                    'collisions': np.random.randint(0, 10),
                    'orders_completed': np.random.randint(10, 100),
                    'avg_latency': np.random.uniform(5, 50),
                },
            })
        return results
```

### 5.2 Farm Worker

```python
class FarmWorker:
    """Omniverse Farm Worker Node"""
    
    def __init__(self, worker_id, gpu_id=0):
        self.worker_id = worker_id
        self.gpu_id = gpu_id
        self.status = "idle"
        self.current_job = None
        self.capabilities = {
            'gpu': f'NVIDIA RTX {np.random.choice(["A6000", "A5000", "4090"])}',
            'vram_gb': 24,
            'max_robots': 10,
        }
    
    def accept_job(self, job):
        """Job 수락"""
        self.current_job = job
        self.status = "running"
        print(f"\n  Worker {self.worker_id} accepting job: {job['name']}")
        print(f"    Robots: {job['robot_count']}, Duration: {job['duration']}s")
        return True
    
    def execute_job(self, progress_callback=None):
        """Job 실행"""
        start_time = time.time()
        
        for step in range(self.current_job['duration'] * 60):
            # Simulate work
            elapsed = time.time() - start_time
            
            if progress_callback and step % 60 == 0:
                progress = step / (self.current_job['duration'] * 60)
                progress_callback(self.worker_id, progress)
        
        self.status = "completed"
        return {
            'worker_id': self.worker_id,
            'job': self.current_job['name'],
            'execution_time': time.time() - start_time,
        }
```

---

## 6. Kubernetes 배포

### 6.1 Docker Image

```dockerfile
# Dockerfile.isaac-sim-worker
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# ROS2 Humble 설치
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip

# Curriculum 복사
COPY code/phase-3/step25_large_scale.py /workspace/
COPY config/ /workspace/config/

# Fast DDS 설정
COPY config/fastdds.xml /workspace/

# Entry point
ENTRYPOINT ["/opt/nvidia/isaac-sim/python.sh", \
            "/workspace/step25_large_scale.py"]
```

### 6.2 Kubernetes Manifest

```yaml
# k8s-isaac-fleet.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: isaac-sim
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: isaac-sim-master
  namespace: isaac-sim
spec:
  replicas: 1
  selector:
    matchLabels:
      app: isaac-master
  template:
    metadata:
      labels:
        app: isaac-master
    spec:
      containers:
      - name: isaac-sim
        image: isaac-sim-worker:5.1.0
        args: ["--master"]
        resources:
          limits:
            nvidia.com/gpu: 1
            memory: "32Gi"
            cpu: "8"
        env:
        - name: ROS_DOMAIN_ID
          value: "42"
        - name: ROS_LOCALHOST_ONLY
          value: "0"
        volumeMounts:
        - name: dds-config
          mountPath: /workspace/config
      volumes:
      - name: dds-config
        configMap:
          name: fastdds-config
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: isaac-sim-worker
  namespace: isaac-sim
spec:
  replicas: 3  # 3개 Worker
  selector:
    matchLabels:
      app: isaac-worker
  template:
    metadata:
      labels:
        app: isaac-worker
    spec:
      containers:
      - name: isaac-sim
        image: isaac-sim-worker:5.1.0
        args: ["--worker", "--worker-id", "$(WORKER_ID)"]
        resources:
          limits:
            nvidia.com/gpu: 1
            memory: "24Gi"
            cpu: "4"
        env:
        - name: ROS_DOMAIN_ID
          value: "42"
        - name: WORKER_ID
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
---
apiVersion: v1
kind: Service
metadata:
  name: isaac-master-service
  namespace: isaac-sim
spec:
  selector:
    app: isaac-master
  ports:
  - port: 11311
    name: ros-master
```

### 6.3 Kubectl 명령

```bash
# 배포
kubectl apply -f k8s-isaac-fleet.yaml

# 상태 확인
kubectl get pods -n isaac-sim -w
kubectl logs -n isaac-sim deployment/isaac-sim-master

# Scale Worker
kubectl scale deployment isaac-sim-worker --replicas=5

# 리소스 모니터링
kubectl top pods -n isaac-sim
```

---

## 7. 모니터링 & 로깅

### 7.1 ROS2 Monitoring Stack

```yaml
# prometheus-ros2-exporter.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: ros2-exporter-config
data:
  metrics.yaml: |
    exporters:
      - name: robot_status
        topic: /fleet/status
        fields:
          - name: active_robots
            path: active_count
          - name: avg_battery
            path: average_battery
      
      - name: system_metrics
        type: system
        metrics:
          - cpu_percent
          - memory_mb
          - gpu_utilization
```

### 7.2 Grafana Dashboard

```json
{
  "dashboard": {
    "title": "Isaac Sim Fleet Monitor",
    "panels": [
      {
        "title": "Active Robots",
        "type": "stat",
        "targets": [
          {"expr": "ros2_robot_status_active_robots"}
        ]
      },
      {
        "title": "GPU Utilization",
        "type": "gauge",
        "targets": [
          {"expr": "nvidia_gpu_utilization{job=\"isaac-sim\"}"}
        ]
      },
      {
        "title": "Simulation FPS",
        "type": "graph",
        "targets": [
          {"expr": "isaac_sim_fps"}
        ]
      },
      {
        "title": "ROS2 Topic Latency",
        "type": "heatmap",
        "targets": [
          {"expr": "ros2_latency_ms"}
        ]
      }
    ]
  }
}
```

---

## 8. 실행 절차

### 8.1 Single Machine Fleet

```bash
# Terminal 1: Isaac Sim Fleet
cd ~/isaac-sim
./python.sh ~/isaac-step-curriculum/code/phase-3/step25_large_scale.py \
  --fleet 10 --robot-type turtlebot3

# Terminal 2: Fleet Monitor
ros2 topic echo /fleet/status
```

### 8.2 Distributed Multi-Host

```bash
# ════════════════════════════════════════════════════════
# Distributed Setup (3 Hosts)
# ════════════════════════════════════════════════════════

# Host 1 — Master (GPU 0, Robots 1-5)
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=~/config/fastdds.xml
./python.sh step25_large_scale.py --master --robots 5 --gpu 0

# Host 2 — Worker 1 (GPU 1, Robots 6-10)
./python.sh step25_large_scale.py --worker 1 --robots 5 --gpu 1

# Host 3 — Worker 2 (GPU 2, Robots 11-15)
./python.sh step25_large_scale.py --worker 2 --robots 5 --gpu 2
```

### 8.3 Profiling

```bash
# Performance Profile
./python.sh -m cProfile -o profile.stats \
  step25_large_scale.py --fleet 10

# Snakeviz 시각화
pip install snakeviz
snakeviz profile.stats
```

---

## 9. 문제 해결

### 문제 1: DDS 패킷 손실 (대규모 환경)

**해결:**
- `ROS_LOCALHOST_ONLY=0`으로 설정
- Fast DDS UDP buffer 크기 증가 (65536)
- Topic 분할 (Namespace + Robot ID)

### 문제 2: GPU OOM

**해결:**
```bash
# 메모리 단편화 해결
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128

# 로봇 수 감소
./python.sh step25_large_scale.py --fleet 5
```

### 문제 3: Worker 간 시간 불일치

**해결:**
- Master Clock /clock topic 사용
- NTP 시간 동기화
- `use_sim_time:=True` 설정

---

## 10. 정리

| 항목 | 내용 |
|------|------|
| ✅ Multi-GPU Fleet | GPU별 Island 배치 |
| ✅ Distributed Simulation | Multi-Host ROS2 |
| ✅ DDS Optimization | Fast DDS Tuning |
| ✅ Performance Profiling | FPS, Latency 측정 |
| ✅ GPU Memory | 할당/최적화/모니터링 |
| ✅ Omniverse Farm | Job 분배 및 실행 |
| ✅ Kubernetes | Container Orchestration |
| ✅ Monitoring | Prometheus + Grafana |

---

## 11. 다음 Step 예고

**Step 26 — Final Integration**에서는:
- 26 Step 전체 Recap
- 8개 Final Project Preview
- Curriculum 디렉토리 구조 정리
- Makefile / Launch Scripts 생성
- Troubleshooting Guide 작성
- 전체 시스템 End-to-End 통합

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Multi-GPU Isaac Sim | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/tutorial_multi_gpu.html |
| Omniverse Farm | https://docs.omniverse.nvidia.com/farm/latest/ |
| Kubernetes GPU | https://kubernetes.io/docs/tasks/manage-gpus/scheduling-gpus/ |
| Fast DDS Tuning | https://fast-dds.docs.eprosima.com/en/latest/ |
| Prometheus ROS2 | https://github.com/ros2/metrics |
