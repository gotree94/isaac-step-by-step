# Step 19 — Digital Twin with Isaac Sim

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Phase 1 + Phase 2 완료

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Digital Twin** 개념과 산업 응용을 이해한다
2. **실제 센서 데이터**를 Isaac Sim으로 스트리밍한다
3. **실시간 동기화** (Real-Time Sync)를 구현한다
4. **Digital Twin 대시보드**를 구축한다
5. **IoT 데이터**를 ROS2 Bridge로 수집하고 시뮬레이션에 반영한다
6. **What-If 시나리오**를 Digital Twin에서 시뮬레이션한다

---

## 1. Digital Twin 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    Physical World                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
│  │ 센서     │  │ 로봇     │  │ IoT      │                  │
│  │ (LiDAR   │  │ (AMR)    │  │ (온도/전력)│                 │
│  │  Camera) │  │          │  │          │                  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                  │
│       │             │             │                         │
└───────┼─────────────┼─────────────┼─────────────────────────┘
        │             │             │
        ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Bridge                               │
│  /scan  /odom  /joint_states  /temperature  /power          │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                   Isaac Sim (Digital Twin)                   │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ 3D Scene     │  │ Simulation   │  │ Dashboard        │  │
│  │ (Factory)    │  │ (Physics +   │  │ (Web/ROS2)       │  │
│  │              │  │  Rendering)  │  │                  │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ Sync Engine  │  │ What-If      │  │ Data Logger      │  │
│  │ (Real-Time   │  │ Simulator    │  │ (History +       │  │
│  │  Matching)   │  │              │  │  Analytics)      │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 1.1 Digital Twin 구성 요소

| 구성 요소 | 역할 | 구현 |
|-----------|------|------|
| **Physical Asset** | 실제 장비/로봇 | TurtleBot3, Franka, 센서 |
| **Virtual Asset** | 3D 모델 + 물리 | Isaac Sim USD Scene |
| **Data Connection** | 실시간 데이터 동기화 | ROS2 Bridge, MQTT, OPC-UA |
| **Sync Engine** | 상태 동기화 | Pose, Joint, Sensor 업데이트 |
| **Dashboard** | 시각화 + 제어 | Web Dashboard, rviz2, Grafana |
| **Analytics** | 데이터 분석 + 예측 | Python, ML 모델 |
| **What-If Engine** | 가상 시나리오 | Parameter 변경, 시뮬레이션 |

### 1.2 사용 사례

| 산업 | 응용 | Isaac Sim 활용 |
|------|------|---------------|
| **제조** | 공장 디지털 트윈 | 생산 라인 시뮬레이션 |
| **물류** | 창고 디지털 트윈 | AMR 경로 최적화 |
| **로보틱스** | 로봇 셀 디지털 트윈 | Pick & Place 최적화 |
| **건설** | 건설 현장 모니터링 | 장비 위치 추적 |
| **에너지** | 발전소 디지털 트윈 | 설비 상태 모니터링 |

---

## 2. Digital Twin Scene 구축

### 2.1 Factory Scene 생성

```python
def create_factory_digital_twin():
    """공장 Digital Twin Scene 생성"""
    
    from omni.isaac.core.objects import VisualCuboid, VisualCylinder
    
    # 바닥
    floor = VisualCuboid(
        prim_path="/World/Factory/Floor",
        name="factory_floor",
        position=np.array([0.0, 0.0, -0.01]),
        scale=np.array([10.0, 8.0, 0.02]),
        color=np.array([0.3, 0.3, 0.3]),
    )
    
    # 벽
    walls = [
        ("Wall_N", (5.0, 0.0, 1.5), (0.1, 8.0, 3.0)),
        ("Wall_S", (-5.0, 0.0, 1.5), (0.1, 8.0, 3.0)),
        ("Wall_E", (0.0, 4.0, 1.5), (10.0, 0.1, 3.0)),
        ("Wall_W", (0.0, -4.0, 1.5), (10.0, 0.1, 3.0)),
    ]
    for name, pos, scale in walls:
        VisualCuboid(
            prim_path=f"/World/Factory/{name}",
            name=name,
            position=np.array(pos),
            scale=np.array(scale),
            color=np.array([0.5, 0.5, 0.5]),
        )
    
    # 설비 (Conveyor Belt, Machine 등)
    equipment = [
        ("Machine_1", (-2.0, 1.5, 0.6), (0.8, 0.6, 1.2), [0.2, 0.4, 0.8]),
        ("Machine_2", (2.0, -1.5, 0.6), (0.8, 0.6, 1.2), [0.2, 0.4, 0.8]),
        ("Conveyor", (0.0, 0.0, 0.15), (3.0, 0.4, 0.3), [0.3, 0.3, 0.3]),
        ("Table_1", (-1.0, -2.0, 0.3), (0.6, 0.4, 0.6), [0.6, 0.4, 0.2]),
        ("Table_2", (1.0, 2.0, 0.3), (0.6, 0.4, 0.6), [0.6, 0.4, 0.2]),
        ("Rack_1", (3.5, 0.0, 1.0), (0.5, 2.0, 2.0), [0.4, 0.3, 0.2]),
        ("Rack_2", (-3.5, 0.0, 1.0), (0.5, 2.0, 2.0), [0.4, 0.3, 0.2]),
    ]
    
    for name, pos, scale, color in equipment:
        VisualCuboid(
            prim_path=f"/World/Factory/{name}",
            name=name,
            position=np.array(pos),
            scale=np.array(scale),
            color=np.array(color),
        )
    
    print("  + Factory Digital Twin scene created")
    print(f"    - 1 floor, 4 walls, {len(equipment)} equipment")
```

### 2.2 AMR 로봇 배치

```python
def deploy_amr_robots():
    """Digital Twin에 AMR 로봇 배치"""
    
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    
    amr_configs = [
        {"name": "AMR_1", "pos": np.array([-3.0, -2.0, 0.1]),
         "path": "/World/Robots/AMR_1"},
        {"name": "AMR_2", "pos": np.array([3.0, 2.0, 0.1]),
         "path": "/World/Robots/AMR_2"},
        {"name": "AMR_3", "pos": np.array([-1.0, 3.0, 0.1]),
         "path": "/World/Robots/AMR_3"},
    ]
    
    amrs = []
    for cfg in amr_configs:
        if not is_prim_path_valid(cfg["path"]):
            add_reference_to_stage(
                "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
                cfg["path"],
            )
        
        robot = Robot(
            prim_path=cfg["path"],
            name=cfg["name"],
            position=cfg["pos"],
        )
        world.scene.add(robot)
        amrs.append(robot)
        print(f"  + {cfg['name']} deployed at {cfg['pos']}")
    
    return amrs
```

---

## 3. 실시간 동기화 (Sync Engine)

### 3.1 ROS2 상태 구독 → 씬 업데이트

```python
class DigitalTwinSync(Node):
    """실제 센서 데이터를 받아 씬 동기화"""
    
    def __init__(self, namespace=''):
        super().__init__('digital_twin_sync')
        self.ns = namespace
        
        # 실제 로봇 위치 구독
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{namespace}/odom',
            self.on_odom,
            10,
        )
        
        # 실제 조인트 상태 구독
        self.joint_sub = self.create_subscription(
            JointState,
            f'{namespace}/joint_states',
            self.on_joint_state,
            10,
        )
        
        # IoT 센서 데이터 구독
        self.temp_sub = self.create_subscription(
            Float32,
            f'{namespace}/temperature',
            self.on_temperature,
            10,
        )
        
        # 시뮬레이션 속도 조정 Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            f'{namespace}/cmd_vel',
            10,
        )
        
        # Last known state
        self.current_pose = None
        self.current_joints = None
        
        # Timer for sync update
        self.timer = self.create_timer(0.033, self.sync_update)  # 30Hz
    
    def on_odom(self, msg):
        """Odometry 수신 → 현재 위치 저장"""
        self.current_pose = msg.pose.pose
    
    def on_joint_state(self, msg):
        """Joint State 수신"""
        self.current_joints = msg
    
    def on_temperature(self, msg):
        """온도 센서 데이터"""
        if msg.data > 60.0:
            self.get_logger().warn(
                f'⚠ High temperature: {msg.data}°C')
    
    def sync_update(self):
        """씬 상태 업데이트"""
        if self.current_pose:
            # Isaac Sim에서 Twin 로봇 위치 업데이트
            twin_path = f"/World/Robots/AMR_Twin"
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(twin_path)
            
            if prim:
                xform = UsdGeom.Xformable(prim)
                translate = xform.ClearXformOpOrder()
                translate = xform.AddTranslateOp()
                translate.Set(Gf.Vec3d(
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z,
                ))
```

### 3.2 OpenUSD 직접 조작 (고속 동기화)

```python
def update_twin_pose_fast(prim_path, position, orientation):
    """OpenUSD API로 빠른 위치 업데이트"""
    
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim:
        return False
    
    xform = UsdGeom.Xformable(prim)
    
    # 위치 업데이트
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    
    if translate_op:
        translate_op.Set(Gf.Vec3d(*position))
    
    # 회전 업데이트
    orient_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
            break
    
    if orient_op and orientation:
        orient_op.Set(Gf.Quatd(*orientation))
    
    return True
```

---

## 4. What-If 시뮬레이션

### 4.1 시나리오 엔진

```python
class WhatIfEngine:
    """Digital Twin What-If 시나리오 엔진"""
    
    def __init__(self, world):
        self.world = world
        self.scenarios = {}
        self.results = {}
    
    def register_scenario(self, name, setup_fn, teardown_fn=None):
        """시나리오 등록"""
        self.scenarios[name] = {
            'setup': setup_fn,
            'teardown': teardown_fn,
        }
        print(f"  + Scenario registered: {name}")
    
    def run_scenario(self, name, duration_seconds=10.0):
        """시나리오 실행"""
        if name not in self.scenarios:
            print(f"  ⚠ Unknown scenario: {name}")
            return None
        
        print(f"\n  Running scenario: {name}")
        
        # Setup
        self.scenarios[name]['setup']()
        
        # Simulate
        num_steps = int(duration_seconds / self.world.get_physics_dt())
        start_time = time.time()
        
        metrics = {
            'distances': [],
            'speeds': [],
            'collisions': [],
            'completion': 0.0,
        }
        
        for step in range(num_steps):
            self.world.step(render=True)
            
            # 매트릭 수집
            if step % 10 == 0:
                metrics['distances'].append(
                    self._measure_distance_to_goal())
                metrics['speeds'].append(
                    self._measure_average_speed())
        
        metrics['execution_time'] = time.time() - start_time
        self.results[name] = metrics
        
        # Teardown
        if self.scenarios[name]['teardown']:
            self.scenarios[name]['teardown']()
        
        print(f"  Scenario complete: {name}")
        return metrics
    
    def compare_scenarios(self, scenario_names):
        """시나리오 비교"""
        print("\n=== Scenario Comparison ===")
        for name in scenario_names:
            if name in self.results:
                r = self.results[name]
                print(f"  {name}:")
                print(f"    Time: {r.get('execution_time', 0):.2f}s")
                print(f"    Avg Speed: {np.mean(r.get('speeds', [0])):.2f} m/s")
                print(f"    Min Distance: {min(r.get('distances', [0])):.2f}m")
```

### 4.2 예제 시나리오

```python
# 시나리오 1: 기본 경로
def scenario_default():
    robot.apply_action(ArticulationAction(
        joint_velocities=np.array([5.0, 5.0]),
        joint_indices=[0, 1],
    ))

# 시나리오 2: 장애물 회피
def scenario_avoidance():
    # 장애물이 있을 때 경로 변경
    robot.apply_action(ArticulationAction(
        joint_velocities=np.array([3.0, 5.0]),  # 우회전
        joint_indices=[0, 1],
    ))

# 시나리오 3: 최적화 경로
def scenario_optimized():
    # PID 제어 기반 최적 경로
    target_speed = 0.3
    left = (target_speed / 0.033)
    right = (target_speed / 0.033)
    robot.apply_action(ArticulationAction(
        joint_velocities=np.array([left, right]),
        joint_indices=[0, 1],
    ))

# 등록
engine = WhatIfEngine(world)
engine.register_scenario("Default Path", scenario_default)
engine.register_scenario("Obstacle Avoidance", scenario_avoidance)
engine.register_scenario("Optimized Path", scenario_optimized)

# 실행 및 비교
engine.run_scenario("Default Path", 5.0)
engine.run_scenario("Optimized Path", 5.0)
engine.compare_scenarios(["Default Path", "Optimized Path"])
```

---

## 5. Digital Twin Dashboard

### 5.1 ROS2 기반 Dashboard

```python
class DigitalTwinDashboard(Node):
    """Web/CLI Digital Twin Dashboard"""
    
    def __init__(self):
        super().__init__('dt_dashboard')
        
        self.metrics = {
            'fps': 0.0,
            'sync_delay_ms': 0.0,
            'robot_count': 0,
            'active_scenarios': [],
            'alerts': [],
        }
        
        # Dashboard 업데이트 타이머
        self.timer = self.create_timer(1.0, self.update_dashboard)
    
    def update_dashboard(self):
        """1초마다 Dashboard 업데이트"""
        
        print("\033[2J\033[H")  # Clear screen
        print("=" * 60)
        print("   ISAAC SIM DIGITAL TWIN DASHBOARD")
        print("=" * 60)
        print()
        print(f"  Runtime:          {self.get_clock().now().to_msg().sec}s")
        print(f"  Active Robots:    {self.metrics['robot_count']}")
        print(f"  Sync Delay:       {self.metrics['sync_delay_ms']:.1f}ms")
        print(f"  Active Scenario:  {self.metrics['active_scenarios']}")
        print()
        
        if self.metrics['alerts']:
            print("  ⚠ ALERTS:")
            for alert in self.metrics['alerts'][-3:]:
                print(f"    - {alert}")
            print()
        
        print(f"  Connected Topics:")
        # ROS2 토픽 목록 출력
        topics = self.get_topic_names_and_types()
        for topic, types in topics:
            if any(kw in topic for kw in ['/odom', '/scan', '/cmd_vel']):
                print(f"    {topic:30s} [{types[0]}]")
```

### 5.2 Web Dashboard (Flask 기반)

```python
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO
import threading
import json

class WebDashboard:
    """Web 기반 Digital Twin Dashboard"""
    
    def __init__(self, host='0.0.0.0', port=5000):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.host = host
        self.port = port
        
        self.setup_routes()
        
        # 백그라운드 데이터 업데이트
        self.data = {
            'robots': [],
            'alerts': [],
            'metrics': {},
        }
        
        self.update_thread = threading.Thread(
            target=self._update_loop, daemon=True)
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('dashboard.html')
        
        @self.app.route('/api/status')
        def api_status():
            return jsonify(self.data)
        
        @self.app.route('/api/scenarios')
        def api_scenarios():
            return jsonify(list(engine.scenarios.keys()))
    
    def _update_loop(self):
        while True:
            time.sleep(0.5)
            self.socketio.emit('update', self.data)
    
    def start(self):
        print(f"  + Web Dashboard: http://{self.host}:{self.port}")
        self.socketio.run(self.app, 
                         host=self.host, 
                         port=self.port,
                         allow_unsafe_werkzeug=True)
```

---

## 6. IoT 데이터 통합

### 6.1 MQTT Bridge (ROS2 ↔ MQTT)

```python
import paho.mqtt.client as mqtt

class MqttIotBridge(Node):
    """MQTT IoT 데이터 ↔ ROS2 Bridge"""
    
    def __init__(self):
        super().__init__('mqtt_iot_bridge')
        
        # MQTT 클라이언트
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.subscribe("factory/+/sensor/+")
        self.mqtt_client.loop_start()
        
        # ROS2 Publishers
        self.temp_pub = self.create_publisher(
            Float32, '/factory/temperature', 10)
        self.power_pub = self.create_publisher(
            Float32, '/factory/power_consumption', 10)
        self.vibration_pub = self.create_publisher(
            Float32, '/factory/vibration', 10)
        
        self.get_logger().info('MQTT IoT Bridge started')
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 → ROS2 발행"""
        topic = msg.topic
        payload = json.loads(msg.payload.decode())
        
        if 'temperature' in topic:
            msg = Float32()
            msg.data = payload['value']
            self.temp_pub.publish(msg)
        elif 'power' in topic:
            msg = Float32()
            msg.data = payload['value']
            self.power_pub.publish(msg)
```

### 6.2 시뮬레이션 데이터 기록

```python
class DataLogger(Node):
    """Digital Twin 데이터 로깅"""
    
    def __init__(self, output_file="dt_log.h5"):
        super().__init__('dt_data_logger')
        
        self.output_file = output_file
        self.buffer = []
        
        # 다양한 토픽 구독
        self.subs = []
        for topic, msg_type in [
            ('/odom', Odometry),
            ('/scan', LaserScan),
            ('/joint_states', JointState),
            ('/factory/temperature', Float32),
        ]:
            sub = self.create_subscription(
                msg_type, topic, 
                lambda msg, t=topic: self.on_data(msg, t),
                10)
            self.subs.append(sub)
        
        # 저장 타이머
        self.save_timer = self.create_timer(10.0, self.flush_buffer)
        
        self.get_logger().info(
            f'Data Logger started → {output_file}')
    
    def on_data(self, msg, topic):
        self.buffer.append({
            'timestamp': time.time(),
            'topic': topic,
            'data': msg,
        })
    
    def flush_buffer(self):
        """버퍼를 HDF5 파일에 저장"""
        if not self.buffer:
            return
        
        import h5py
        with h5py.File(self.output_file, 'a') as f:
            group = f.create_group(
                f'batch_{int(time.time())}')
            
            for item in self.buffer[-100:]:  # 최근 100개
                topic_clean = item['topic'].strip('/').replace('/', '_')
                ds = group.create_dataset(
                    topic_clean, data=str(item['data']))
        
        self.get_logger().info(
            f'Saved {len(self.buffer)} records')
        self.buffer = []
```

---

## 7. 실행 절차

### 7.1 Terminal Setup

```bash
# ════════════════════════════════════════════════════════
# Digital Twin — 4 Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim Digital Twin
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-3/step19_digital_twin.py

# 터미널 2: Sync Engine + Dashboard
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run dt_sync dt_dashboard_node

# 터미널 3: Data Logger
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run dt_logger data_logger_node

# 터미널 4: Web Dashboard (선택)
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
python ~/isaac-step-curriculum/code/phase-3/dt_web_dashboard.py
```

### 7.2 What-If 시나리오 실행

```bash
# ROS2 서비스로 시나리오 실행
ros2 service call /run_scenario std_srvs/srv/Trigger \
  "{data: 'Obstacle Avoidance'}"

# 시나리오 비교
ros2 service call /compare_scenarios std_srvs/srv/Trigger \
  "{data: 'Default Path,Optimized Path'}"
```

---

## 8. 문제 해결 (Troubleshooting)

### 문제 1: 동기화 지연이 큽니다.

**해결:**
- ROS2 QoS를 BEST_EFFORT로 설정
- 발행 주기를 30Hz로 제한
- USD 직접 조작 (Xformable) 사용

### 문제 2: What-If 시나리오 결과가 불일치합니다.

**해결:**
- 시작 전에 모든 로봇을 초기 위치로 리셋
- 동일한 Random Seed 사용
- Physics 설정 일치 확인

### 문제 3: MQTT 연결이 불안정합니다.

**해결:**
```python
mqtt_client.connect_async("broker_address", 1883, 60)
mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
```

---

## 9. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Digital Twin Scene | Factory + AMR 배치 |
| ✅ Sync Engine | ROS2 → USD 실시간 업데이트 |
| ✅ What-If Engine | 시나리오 등록/실행/비교 |
| ✅ Dashboard | CLI + Web Dashboard |
| ✅ IoT Bridge | MQTT → ROS2 데이터 통합 |
| ✅ Data Logger | HDF5 기반 기록 |

### Digital Twin 핵심

```
Physical World ─── ROS2 ───→ Virtual World (Isaac Sim)
                                   │
                          ┌────────┼────────┐
                          │        │        │
                          ▼        ▼        ▼
                     Sync      What-If   Dashboard
                    Engine    Simulator
                          │        │        │
                          └────────┼────────┘
                                   ▼
                             Decision Support
```

---

## 10. 다음 Step 예고

**Step 20 — Humanoid Robot**에서는:
- Isaac Sim에서 Humanoid 로봇 (예: GR1, H1) 로딩
- Full-Body IK and Motion Control
- Walking Gait Generation
- Task Space Control
- ROS2와 Humanoid 연동

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Digital Twin 개요 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ |
| Isaac Sim Digital Twin | https://developer.nvidia.com/industries/digital-twins |
| ROS2 Industrial | https://rosindustrial.org/ |
| MQTT + ROS2 | https://github.com/grocid/ros2-mqtt-bridge |
| What-If Analysis | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ext_omni_what_if.html |
