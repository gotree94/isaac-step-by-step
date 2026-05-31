# Step 22 — AI Worker

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 21 (Warehouse), Step 17 (Synthetic Data)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **AI Worker 개념**을 이해하고 Isaac Sim에 구현한다
2. **Human-Robot Collaboration** 시나리오를 구성한다
3. **Perception Pipeline**을 구축한다 (RGB-D + Object Detection)
4. **Behavior Tree** 기반 Task Planning을 구현한다
5. **Human Pose Estimation**과 **Action Recognition**을 시뮬레이션한다
6. **Human Digital Twin**과 **Robot**을 동시 운용한다
7. **Warehouse AI Worker** 통합 시나리오를 완성한다

---

## 1. AI Worker 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│                     AI Worker System                          │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐     │
│  │              Perception Pipeline                      │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │     │
│  │  │ RGB-D    │→│ Object   │→│ Scene             │  │     │
│  │  │ Camera   │  │ Detection│  │ Understanding    │  │     │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │     │
│  └─────────────────────────────────────────────────────┘     │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐     │
│  │              Planning & Execution                     │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │     │
│  │  │ Behavior │→│ Task     │→│ Motion           │  │     │
│  │  │ Tree     │  │ Scheduler│  │ Controller       │  │     │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │     │
│  └─────────────────────────────────────────────────────┘     │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐     │
│  │              Collaboration Layer                      │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │     │
│  │  │ Human    │→│ Shared  │→│ Joint Action     │  │     │
│  │  │ Tracking │  │ Workspace│  │ Coordination    │  │     │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │     │
│  └─────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────┘
```

### 1.1 AI Worker 정의

| 요소 | 설명 | Isaac Sim 구현 |
|------|------|---------------|
| **Human Worker** | 시뮬레이션된 인간 작업자 | Humanoid USD + Animation |
| **AI Assistant** | 협업 로봇 (Mobile Manipulator) | TurtleBot3 + Franka |
| **Shared Workspace** | 인간-로봇 공동 작업 공간 | Warehouse Staging Area |
| **Perception** | 환경 인식 | ROS2 RGB-D + YOLO |
| **Task Planning** | 행동 결정 | Behavior Tree (py_trees) |
| **Safety** | 충돌 방지 | Speed Separation Monitoring |

### 1.2 Human-Robot Collaboration 시나리오

```
시나리오: AI Worker가 인간과 협업하여 창고에서 물품을 집어 컨베이어에 싣는다.

1. [Human] 랙에서 물품을 꺼낸다
2. [Perception] AI가 인간의 동작을 인식한다
3. [AI] 물품을 받기 위해 Staging Area로 이동한다
4. [Human] 물품을 AI Worker에게 전달한다 (Handover)
5. [AI] 물품을 Conveyor Belt에 싣는다
6. [Human] 다음 작업으로 이동한다
7. [Coordination] 동시에 같은 공간에서 충돌 없이 작업한다
```

---

## 2. Human Worker Simulation

### 2.1 Humanoid Human Worker

```python
class HumanWorker:
    """시뮬레이션된 인간 작업자"""
    
    ACTIONS = {
        'idle': {'pose': 'standing', 'speed': 0},
        'walk': {'pose': 'walking', 'speed': 0.5},
        'reach': {'pose': 'reaching', 'speed': 0},
        'carry': {'pose': 'carrying', 'speed': 0.3},
        'handover': {'pose': 'extending', 'speed': 0},
    }
    
    def __init__(self, world, name="Human_Worker"):
        self.world = world
        self.name = name
        self.current_action = 'idle'
        self.position = np.array([2.0, 3.0, 0.0])
        self.action_timer = 0
        
        # Humanoid 로딩
        human_path = "/World/Humans/Worker1"
        if not is_prim_path_valid(human_path):
            add_reference_to_stage(
                "/Isaac/Robots/GR1/gr1.usd",
                human_path,
            )
        
        self.robot = Robot(
            prim_path=human_path,
            name=name,
            position=self.position,
        )
        world.scene.add(self.robot)
        print(f"  + Human Worker loaded at {self.position}")
    
    def set_action(self, action_name, target_pos=None):
        """작업자 동작 설정"""
        if action_name in self.ACTIONS:
            self.current_action = action_name
            self.action_timer = 0
            if target_pos is not None:
                self.target_pos = target_pos
            print(f"  Human: {action_name}")
    
    def update(self, dt=1/60.0):
        """작업자 상태 업데이트"""
        self.action_timer += dt
        
        action = self.ACTIONS[self.current_action]
        
        # 이동 (walk, carry)
        if action['speed'] > 0 and hasattr(self, 'target_pos'):
            diff = self.target_pos - self.position
            dist = np.linalg.norm(diff)
            if dist > 0.1:
                step = action['speed'] * dt
                self.position += (diff / dist) * min(step, dist)
                self.robot.set_world_pose(self.position)
        
        # Action-specific joint poses
        if self.current_action == 'reach':
            self._apply_reach_pose()
        elif self.current_action == 'handover':
            self._apply_handover_pose()
    
    def _apply_reach_pose(self):
        """물건 집기 자세"""
        pose = np.zeros(self.robot.num_dof)
        for i, name in enumerate(self.robot.dof_names):
            if 'r_shoulder_pitch' in name:
                pose[i] = -0.8  # 팔 뻗기
            elif 'r_elbow' in name:
                pose[i] = -0.5  # 팔꿈치 굽힘
            elif 'torso' in name:
                pose[i] = 0.2   # 상체 숙임
        self.robot.set_joint_positions(pose)
    
    def _apply_handover_pose(self):
        """물건 건네기 자세"""
        pose = np.zeros(self.robot.num_dof)
        for i, name in enumerate(self.robot.dof_names):
            if 'r_shoulder_pitch' in name:
                pose[i] = -0.5
            elif 'r_shoulder_roll' in name:
                pose[i] = 0.3
            elif 'r_elbow' in name:
                pose[i] = -0.3
            elif 'r_wrist' in name:
                pose[i] = 0.2  # 손목 회전 (건네기)
        self.robot.set_joint_positions(pose)
```

### 2.2 Human Action Recognition

```python
class HumanActionRecognizer(Node):
    """인간 작업자 동작 인식 (시뮬레이션)"""
    
    def __init__(self):
        super().__init__('human_action_recognizer')
        
        # Recognize joint states → action
        self.joint_sub = self.create_subscription(
            JointState,
            '/human/joint_states',
            self.on_joint_states,
            10,
        )
        
        # Action publisher
        self.action_pub = self.create_publisher(
            String,
            '/human/current_action',
            10,
        )
        
        # Current action
        self.current_action = 'unknown'
        self.action_history = []
    
    def on_joint_states(self, msg):
        """관절 상태로 동작 분류"""
        # 특징 추출 (간소화)
        shoulder_pitch = msg.position[
            msg.name.index('r_shoulder_pitch')
        ] if 'r_shoulder_pitch' in msg.name else 0
        
        elbow = msg.position[
            msg.name.index('r_elbow')
        ] if 'r_elbow' in msg.name else 0
        
        # Rule-based action classification
        if shoulder_pitch < -0.6 and elbow < -0.3:
            action = 'reaching'
        elif shoulder_pitch < -0.3 and elbow < -0.2:
            action = 'handover'
        elif abs(shoulder_pitch) < 0.1:
            action = 'idle'
        else:
            action = 'walking'
        
        if action != self.current_action:
            self.get_logger().info(f'Human action: {action}')
            self.current_action = action
            msg_out = String()
            msg_out.data = action
            self.action_pub.publish(msg_out)
```

---

## 3. Behavior Tree 기반 Task Planning

### 3.1 Behavior Tree 구조

```python
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.decorators import Timeout, Condition
from py_trees import logging as py_trees_logging

# ── 기본 Action Nodes ──

class DetectHuman(Behaviour):
    """인간 작업자 감지"""
    def __init__(self, name="DetectHuman", recognizer=None):
        super().__init__(name)
        self.recognizer = recognizer
    
    def update(self):
        if self.recognizer and self.recognizer.current_action != 'unknown':
            return Status.SUCCESS
        return Status.RUNNING

class MoveToStaging(Behaviour):
    """Staging Area로 이동"""
    def __init__(self, name="MoveToStaging", controller=None):
        super().__init__(name)
        self.controller = controller
    
    def update(self):
        if self.controller:
            self.controller.navigate_to(-6.0, 0.0)
            if self.controller.at_goal():
                return Status.SUCCESS
        return Status.RUNNING

class ReceiveObject(Behaviour):
    """인간으로부터 물건 수령"""
    def __init__(self, name="ReceiveObject", controller=None):
        super().__init__(name)
        self.controller = controller
        self.received = False
    
    def update(self):
        if self.received:
            return Status.SUCCESS
        if self.controller:
            self.controller.extend_gripper()
            # Human action이 handover인지 확인
            return Status.RUNNING
        return Status.FAILURE

class PlaceOnConveyor(Behaviour):
    """Conveyor Belt에 물건 배치"""
    def __init__(self, name="PlaceOnConveyor", controller=None):
        super().__init__(name)
        self.controller = controller
    
    def update(self):
        if self.controller:
            self.controller.navigate_to(-6.0, 3.5)
            if self.controller.at_goal():
                self.controller.release_gripper()
                return Status.SUCCESS
        return Status.RUNNING

# ── Task Tree 구성 ──

def create_ai_worker_bt(recognizer, controller):
    """AI Worker Behavior Tree"""
    
    root = Sequence("AI_Worker_Routine", memory=True)
    
    # Phase 1: 대기 및 감지
    wait_and_detect = Sequence("Wait_for_Human", memory=True)
    wait_and_detect.add_child(DetectHuman(recognizer=recognizer))
    root.add_child(wait_and_detect)
    
    # Phase 2: 협업 Sequence
    collaboration = Sequence("Collaboration", memory=True)
    collaboration.add_child(MoveToStaging(controller=controller))
    
    # Timeout decorator for handover
    handover = Timeout(
        name="Receive_Timeout",
        child=ReceiveObject(controller=controller),
        duration=10.0,
    )
    collaboration.add_child(handover)
    collaboration.add_child(PlaceOnConveyor(controller=controller))
    root.add_child(collaboration)
    
    return root
```

### 3.2 Behavior Tree 실행

```python
class BehaviorTreeRunner:
    """Behavior Tree 실행기"""
    
    def __init__(self, tree):
        self.tree = tree
        self.tick_count = 0
        py_trees_logging.level = py_trees_logging.Level.WARN
    
    def setup(self):
        self.tree.setup()
    
    def tick(self):
        self.tree.tick_once()
        self.tick_count += 1
        return self.tree.status
    
    def get_status(self):
        """Tree 상태 문자열"""
        def describe(node, indent=0):
            prefix = "  " * indent
            status = node.status
            if status == Status.SUCCESS:
                status_str = "✓"
            elif status == Status.FAILURE:
                status_str = "✗"
            elif status == Status.RUNNING:
                status_str = "▶"
            else:
                status_str = "○"
            yield f"{prefix}{status_str} {node.name}"
            for child in node.children:
                yield from describe(child, indent + 1)
        
        return "\n".join(describe(self.tree))
```

---

## 4. Perception Pipeline

### 4.1 RGB-D Camera Setup

```python
def setup_perception_camera():
    """AI Worker Perception Camera 설정"""
    
    from omni.isaac.sensor import Camera
    
    camera = Camera(
        prim_path="/World/Perception/Camera",
        name="ai_camera",
        position=np.array([-6.0, 0.0, 1.2]),
        frequency=30,
        resolution=(640, 480),
        orientation=np.array([0.5, 0.5, -0.5, 0.5]),  # Staging Area 바라봄
    )
    
    camera.initialize()
    
    # ROS2 Bridge for camera
    og.Controller.edit(
        {"graph_path": "/ActionGraph/PerceptionBridge", 
         "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("CameraHelper", "omni.isaac.sensor.IsaacReadCameraInfo"),
                ("PubRGB", "omni.isaac.ros2_bridge.ROS2PublishImage"),
                ("PubDepth", "omni.isaac.ros2_bridge.ROS2PublishImage"),
                ("PubInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CameraHelper.inputs:cameraPrim", 
                 Sdf.Path("/World/Perception/Camera")),
                ("PubRGB.inputs:topicName", "/ai_worker/camera/rgb"),
                ("PubDepth.inputs:topicName", "/ai_worker/camera/depth"),
                ("PubInfo.inputs:topicName", "/ai_worker/camera/info"),
            ],
        },
    )
    
    return camera
```

### 4.2 Object Detection (YOLO 연동)

```python
class AIWorkerPerception(Node):
    """AI Worker Perception Pipeline"""
    
    def __init__(self):
        super().__init__('ai_worker_perception')
        
        # Image subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/ai_worker/camera/rgb',
            self.on_rgb_image, 10)
        self.depth_sub = self.create_subscription(
            Image, '/ai_worker/camera/depth',
            self.on_depth_image, 10)
        
        # Detection publisher
        self.detect_pub = self.create_publisher(
            Detection2DArray, '/ai_worker/detections', 10)
        
        # YOLO (simulated)
        self.detection_labels = ['human', 'box', 'rack', 'forklift']
        self.frame_count = 0
    
    def on_rgb_image(self, msg):
        """RGB 이미지 처리 (simplified)"""
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:  # 1초마다
            # Simulated detection
            detections = Detection2DArray()
            detections.header = msg.header
            
            # 가상의 박스 위치 (실제로는 YOLO 추론)
            mock_boxes = [
                ((200, 150), (300, 300), 'human', 0.95),
                ((350, 200), (400, 350), 'box', 0.88),
                ((100, 100), (250, 400), 'rack', 0.92),
            ]
            
            for (x1, y1), (x2, y2), label, conf in mock_boxes:
                box = Detection2D()
                box.bbox.center.x = (x1 + x2) / 2
                box.bbox.center.y = (y1 + y2) / 2
                box.bbox.size_x = x2 - x1
                box.bbox.size_y = y2 - y1
                box.results[0].hypothesis.class_id = label
                box.results[0].hypothesis.score = conf
                detections.detections.append(box)
            
            self.detect_pub.publish(detections)
            self.get_logger().info(
                f'Detected {len(mock_boxes)} objects')
    
    def on_depth_image(self, msg):
        """Depth 이미지로 거리 계산"""
        # Depth → PointCloud 변환 (simplified)
        pass
```

---

## 5. Human-Robot Safety System

### 5.1 Speed and Separation Monitoring (SSM)

```python
class SafetyMonitor(Node):
    """인간-로봇 안전 모니터링"""
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Human position
        self.human_pose = None
        self.create_subscription(
            Odometry, '/human/odom',
            lambda msg: setattr(self, 'human_pose', msg), 10)
        
        # Robot state
        self.robot_state = None
        self.create_subscription(
            JointState, '/franka/joint_states',
            lambda msg: setattr(self, 'robot_state', msg), 10)
        
        # Speed control
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/franka/cmd_vel_safe', 10)
        
        # Safety zones
        self.SAFE_ZONE = 1.0    # m: 안전
        self.WARNING_ZONE = 0.5 # m: 경고
        self.STOP_ZONE = 0.25   # m: 정지
        
        self.timer = self.create_timer(0.05, self.check_safety)
    
    def check_safety(self):
        """인간-로봇 거리 체크 및 속도 제어"""
        if self.human_pose is None:
            return
        
        px = self.human_pose.pose.pose.position.x
        py = self.human_pose.pose.pose.position.y
        
        # Robot assumed at (-6, 0)
        dist = np.sqrt((px + 6)**2 + py**2)
        
        twist = Twist()
        
        if dist < self.STOP_ZONE:
            self.get_logger().warn(f'🛑 EMERGENCY STOP (d={dist:.2f}m)')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif dist < self.WARNING_ZONE:
            self.get_logger().warn(f'⚠ WARNING (d={dist:.2f}m)')
            twist.linear.x = 0.1  # Slow mode
        elif dist < self.SAFE_ZONE:
            twist.linear.x = 0.3  # Reduced speed
        else:
            twist.linear.x = 0.5  # Full speed
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f'Safety: d={dist:.2f}m, v={twist.linear.x:.1f}')
```

### 5.2 Safety Zone Visualization

```python
def create_safety_zones():
    """안전 구역 시각화"""
    
    from omni.isaac.core.objects import VisualCuboid
    
    zones = [
        ("Safe_Zone", 1.0, np.array([0.3, 1.0, 0.3, 0.2])),      # 녹색
        ("Warning_Zone", 0.5, np.array([1.0, 0.8, 0.0, 0.3])),    # 황색
        ("Stop_Zone", 0.25, np.array([1.0, 0.0, 0.0, 0.4])),      # 적색
    ]
    
    for name, radius, color in zones:
        VisualCuboid(
            prim_path=f"/World/Safety/{name}",
            position=np.array([-6.0, 0.0, 0.01]),
            scale=np.array([radius*2, radius*2, 0.01]),
            color=color[:3],
        )
    
    print("  + Safety zones created (Green/Yellow/Red)")
```

---

## 6. 실행 절차

### 6.1 Terminal Setup

```bash
# ════════════════════════════════════════════════════════
# AI Worker — 4 Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim AI Worker
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-3/step22_ai_worker.py

# 터미널 2: Perception Node
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run ai_worker perception.py

# 터미널 3: Behavior Tree
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run ai_worker behavior_tree.py

# 터미널 4: Safety Monitor
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run ai_worker safety_monitor.py
```

### 6.2 Monitoring

```bash
# Perception 확인
ros2 topic echo /ai_worker/detections --once

# Action 인식
ros2 topic echo /human/current_action

# 안전 상태
ros2 topic echo /franka/cmd_vel_safe
```

---

## 7. 문제 해결

### 문제 1: YOLO 탐지가 되지 않습니다.

**해결:**
```bash
# CUDA 확인
python -c "import torch; print(torch.cuda.is_available())"

# YOLO 설치 확인
pip install ultralytics
python -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt')"
```

### 문제 2: Behavior Tree가 영원히 Running 상태입니다.

**해결:**
- 각 Behaviour의 update()가 SUCCESS/FAILURE를 반환하는지 확인
- Timeout decorator로 제한 시간 설정
- py_trees.logging.level = DEBUG로 상세 로그 확인

### 문제 3: 인간-로봇 충돌이 발생합니다.

**해결:**
- Safety Monitor의 Zone 거리 확인
- Stop Zone을 0.5m로 증가
- Robot 속도를 0.3 m/s로 제한

---

## 8. 실습 과제

### 과제 1: AI Worker 시나리오 확장
- 2명의 Human Worker + 1대의 AI Worker
- 물품 분류 (좌/우 Conveyor Belt 분기)
- Pick-and-Place with Franka Panda

### 과제 2: 고급 Perception
- YOLOv8 실제 추론 연동
- 3D Bounding Box + Pose Estimation
- Multi-Object Tracking (SORT/DeepSORT)

### 과제 3: Safety System 고도화
- ISO/TS 15066 Speed Monitoring 구현
- Predictive Collision Avoidance
- 작업자 위험 동작 감지

---

## 9. 정리

| 항목 | 내용 |
|------|------|
| ✅ AI Worker | 개념 이해 및 구현 |
| ✅ Human Worker | Humanoid 시뮬레이션 |
| ✅ Behavior Tree | Task Planning |
| ✅ Perception | RGB-D + Detection |
| ✅ Safety | SSM Monitor |
| ✅ Collaboration | Human-Robot Handover |

---

## 10. 다음 Step 예고

**Step 23 — Deep Learning in Isaac Sim**에서는:
- Isaac Sim + PyTorch/TensorFlow Integration
- Reinforcement Learning with Isaac Gym
- Imitation Learning from Human Demonstration
- Domain Randomization
- Training → Export → Deployment Pipeline

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Behavior Tree (py_trees) | https://py-trees.readthedocs.io/ |
| YOLOv8 | https://docs.ultralytics.com/ |
| ISO/TS 15066 | https://www.iso.org/standard/62996.html |
| Human-Robot Collaboration | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/tutorial_human_robot.html |
