# Step 20 — Humanoid Robot in Isaac Sim

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 08 (Franka Control), Phase 1 완료

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Humanoid 로봇** (GR1, H1, or custom)을 Isaac Sim에 로딩한다
2. **Full-Body Inverse Kinematics (IK)**를 설정한다
3. **Walking Gait** 생성 및 실행한다
4. **Task Space Control**로 상체/하체를 개별 제어한다
5. **ROS2 Bridge**로 Humanoid 상태를 발행한다
6. **Motion Retargeting** 개념을 이해한다

---

## 1. Humanoid Robot 아키텍처

```
┌──────────────────────────────────────────────┐
│              Humanoid Robot                   │
│                                              │
│  ┌─── Head (Camera/LiDAR) ─────────────────┐ │
│  │                                          │ │
│  ├── Left Arm (7-DOF)  ├── Right Arm (7-DOF) │
│  │  shoulder_pitch     │  shoulder_pitch     │
│  │  shoulder_roll      │  shoulder_roll      │
│  │  elbow              │  elbow              │
│  │  wrist              │  wrist              │
│  └─────────────────────┴──────────────────── ┘ │
│                                              │
│  ┌─── Torso (1-DOF: waist) ────────────────┐ │
│                                              │
│  ├── Left Leg (6-DOF)  ├── Right Leg (6-DOF) │
│  │  hip_yaw            │  hip_yaw            │
│  │  hip_roll           │  hip_roll           │
│  │  hip_pitch          │  hip_pitch          │
│  │  knee               │  knee               │
│  │  ankle_pitch        │  ankle_pitch        │
│  │  ankle_roll         │  ankle_roll         │
│  └─────────────────────┴──────────────────── ┘ │
└──────────────────────────────────────────────┘
```

### 1.1 Humanoid DOF 구성

| 부위 | 관절 수 | 관절 목록 |
|------|---------|----------|
| **Head** | 2 | neck_pitch, neck_yaw |
| **Torso** | 1 | waist_yaw |
| **Left Arm** | 7 | l_shoulder_p/r, l_elbow, l_wrist_p/r/y |
| **Right Arm** | 7 | r_shoulder_p/r, r_elbow, r_wrist_p/r/y |
| **Left Leg** | 6 | l_hip_y/r/p, l_knee, l_ankle_p/r |
| **Right Leg** | 6 | r_hip_y/r/p, r_knee, r_ankle_p/r |
| **Total** | **29** | |

### 1.2 Humanoid USD 구조

Isaac Sim의 Humanoid USD는 다음과 같은 계층 구조를 가집니다:

```
/World/Humanoid
  ├── body (RigidBody)
  │     ├── pelvis
  │     ├── torso
  │     ├── head
  │     ├── l_upper_arm → l_lower_arm → l_hand
  │     ├── r_upper_arm → r_lower_arm → r_hand
  │     ├── l_thigh → l_shin → l_foot
  │     └── r_thigh → r_shin → r_foot
  └── Articulation (PhysX Articulation)
```

---

## 2. Humanoid 로딩

### 2.1 Isaac Sim 내장 Humanoid 로딩

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import numpy as np

# Isaac Sim 내장 Humanoid
HUMANOID_PATH = "/World/Humanoid"
add_reference_to_stage(
    "/Isaac/Robots/GR1/gr1.usd",  # 또는 H1, custom
    HUMANOID_PATH,
)

humanoid = Robot(
    prim_path=HUMANOID_PATH,
    name="Humanoid",
    position=np.array([0.0, 0.0, 0.95]),  # 키 ≈ 0.95m
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
)
world.scene.add(humanoid)

# DOF 정보 출력
print(f"DOF count: {humanoid.num_dof}")
for i, name in enumerate(humanoid.dof_names):
    print(f"  {i:2d}: {name}")
```

### 2.2 URDF → USD 변환

```python
def convert_urdf_to_usd(urdf_path, output_usd_path):
    """URDF 파일을 USD로 변환"""
    
    from omni.isaac.core.utils.stage import create_stage
    
    # 새 Stage 생성
    temp_stage = create_stage()
    
    # URDF Importer
    from omni.isaac.urdf.urdf_importer import UrdfImporter
    
    importer = UrdfImporter()
    results = importer.import_urdf(
        urdf_path=urdf_path,
        destination_path=output_usd_path,
        fix_joints=True,
        make_articulation=True,
    )
    
    if results:
        print(f"  + URDF converted: {output_usd_path}")
        return results
    
    print(f"  ⚠ URDF conversion failed")
    return None
```

---

## 3. Full-Body IK 설정

### 3.1 Isaac Sim IK Solver

```python
def setup_fullbody_ik(humanoid_path="/World/Humanoid"):
    """인체 전신 IK 설정"""
    
    from omni.isaac.core.utils.ik_solver import IKSolver
    
    # 왼발 IK
    l_foot_ik = IKSolver(
        prim_path=f"{humanoid_path}/l_foot",
        end_effector_frame="l_foot",
        solver_type="dls",  # Damped Least Squares
        max_iterations=100,
        tolerance=0.01,
    )
    
    # 오른발 IK
    r_foot_ik = IKSolver(
        prim_path=f"{humanoid_path}/r_foot",
        end_effector_frame="r_foot",
        solver_type="dls",
        max_iterations=100,
        tolerance=0.01,
    )
    
    # 오른손 IK
    r_hand_ik = IKSolver(
        prim_path=f"{humanoid_path}/r_hand",
        end_effector_frame="r_hand",
        solver_type="dls",
        max_iterations=100,
        tolerance=0.01,
    )
    
    print(f"  + IK Solvers initialized (l_foot, r_foot, r_hand)")
    return l_foot_ik, r_foot_ik, r_hand_ik
```

### 3.2 Task Space Control

```python
class HumanoidTaskSpaceController:
    """Humanoid Task Space Control"""
    
    def __init__(self, robot):
        self.robot = robot
        self.dof_names = robot.dof_names
        self.dof_count = robot.num_dof
        
        # 관절 인덱스 그룹
        self.leg_indices = {
            'left': [i for i, n in enumerate(self.dof_names) 
                     if n.startswith('l_hip') or n.startswith('l_knee') or n.startswith('l_ankle')],
            'right': [i for i, n in enumerate(self.dof_names) 
                      if n.startswith('r_hip') or n.startswith('r_knee') or n.startswith('r_ankle')],
        }
        
        self.arm_indices = {
            'left': [i for i, n in enumerate(self.dof_names) 
                     if n.startswith('l_shoulder') or n.startswith('l_elbow') or n.startswith('l_wrist')],
            'right': [i for i, n in enumerate(self.dof_names) 
                      if n.startswith('r_shoulder') or n.startswith('r_elbow') or n.startswith('r_wrist')],
        }
        
        # 초기 자세 (T-Pose)
        self.t_pose = self._get_t_pose()
    
    def _get_t_pose(self):
        """T-Pose 관절 각도"""
        pose = [0.0] * self.dof_count
        
        # Left Arm: T-pose (slightly down)
        for idx in self.arm_indices['left']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name:
                pose[idx] = -0.3
            elif 'shoulder_roll' in name:
                pose[idx] = 0.2
            elif 'elbow' in name:
                pose[idx] = 0.0
        
        # Right Arm: T-pose
        for idx in self.arm_indices['right']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name:
                pose[idx] = -0.3
            elif 'shoulder_roll' in name:
                pose[idx] = -0.2
        
        return np.array(pose)
    
    def set_standing_pose(self):
        """Standing Pose"""
        standing = self._get_t_pose().copy()
        
        # 약간의 무릎 굽힘 (안정성)
        for idx in self.leg_indices['left'] + self.leg_indices['right']:
            name = self.dof_names[idx]
            if 'knee' in name:
                standing[idx] = 0.15
        
        self.robot.set_joint_positions(standing)
        return standing
    
    def set_walking_pose(self, phase=0.0):
        """Walking Pose 생성 (phase: 0.0~1.0 보행 사이클)"""
        pose = self._get_t_pose().copy()
        
        # 보행 사이클
        swing = np.sin(phase * 2 * np.pi)
        
        # Left Leg
        for idx in self.leg_indices['left']:
            name = self.dof_names[idx]
            if 'hip_pitch' in name:
                pose[idx] = -0.2 * swing
            elif 'knee' in name:
                pose[idx] = 0.15 + 0.2 * max(0, swing)
            elif 'ankle_pitch' in name:
                pose[idx] = 0.1 * swing
        
        # Right Leg
        for idx in self.leg_indices['right']:
            name = self.dof_names[idx]
            if 'hip_pitch' in name:
                pose[idx] = 0.2 * swing
            elif 'knee' in name:
                pose[idx] = 0.15 + 0.2 * max(0, -swing)
            elif 'ankle_pitch' in name:
                pose[idx] = -0.1 * swing
        
        # Arm swing (counter-phase)
        for idx in self.arm_indices['left']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name:
                pose[idx] = -0.3 + 0.1 * swing
        
        for idx in self.arm_indices['right']:
            name = self.dof_names[idx]
            if 'shoulder_pitch' in name:
                pose[idx] = -0.3 - 0.1 * swing
        
        self.robot.set_joint_positions(pose)
        return pose
```

---

## 4. Walking Gait 생성

### 4.1 간단한 Open-Loop Gait

```python
class SimpleGaitGenerator:
    """간단한 직진 보행 패턴 생성기"""
    
    def __init__(self, humanoid_controller, step_length=0.05, step_height=0.03):
        self.controller = humanoid_controller
        self.step_length = step_length
        self.step_height = step_height
        self.phase = 0.0
    
    def update(self, dt=1/60.0):
        """매 프레임 호출"""
        self.phase += dt * 1.5  # 걸음 속도 (1.5 steps/sec)
        self.phase %= 1.0
        
        self.controller.set_walking_pose(self.phase)
    
    def get_velocity(self):
        """예상 이동 속도"""
        return self.step_length * 1.5  # m/s
```

### 4.2 ZMP (Zero Moment Point) 기반 Gait

```python
class ZmpGaitGenerator:
    """ZMP 기반 안정적 보행"""
    
    def __init__(self, robot, com_height=0.85):
        self.robot = robot
        self.com_height = com_height
        self.g = 9.81
        
        # 보행 파라미터
        self.double_support_ratio = 0.2  # 양발 지지 비율
        self.step_time = 0.6  # 한 걸음 시간 (s)
        self.step_length = 0.15  # 보폭 (m)
        
        # 궤적 데이터
        self.foot_trajectory = None
        self.com_trajectory = None
    
    def plan_footstep(self, start_pos, goal_pos):
        """Footstep 계획"""
        dx = goal_pos[0] - start_pos[0]
        dy = goal_pos[1] - start_pos[1]
        num_steps = max(2, int(abs(dx) / self.step_length))
        
        footsteps = []
        for i in range(num_steps + 1):
            t = i / num_steps
            x = start_pos[0] + dx * t
            y = start_pos[1] + dy * t
            foot = 'left' if i % 2 == 0 else 'right'
            footsteps.append((foot, x, y))
        
        return footsteps
    
    def generate_com_trajectory(self, footsteps):
        """Center of Mass 궤적 생성"""
        # Capture Point 기반 CoM 궤적
        com_traj = []
        for i in range(len(footsteps)):
            foot, fx, fy = footsteps[i]
            next_foot = footsteps[min(i+1, len(footsteps)-1)]
            
            # CoM은 두 발 사이
            cx = (fx + next_foot[1]) / 2
            cy = (fy + next_foot[2]) / 2
            
            com_traj.append((cx, cy, self.com_height))
        
        return com_traj
```

---

## 5. ROS2 Humanoid Interface

### 5.1 Joint State + Twist 명령

```python
class HumanoidRos2Bridge(Node):
    """Humanoid ↔ ROS2 통신 브릿지"""
    
    def __init__(self, robot, namespace='/humanoid'):
        super().__init__('humanoid_bridge')
        self.robot = robot
        self.ns = namespace
        
        # Joint State Publisher
        self.joint_pub = self.create_publisher(
            JointState, f'{namespace}/joint_states', 10)
        
        # Odometry Publisher
        self.odom_pub = self.create_publisher(
            Odometry, f'{namespace}/odom', 10)
        
        # 속도 명령 Subscriber (걷기 속도)
        self.cmd_sub = self.create_subscription(
            Twist, f'{namespace}/cmd_vel', self.on_cmd_vel, 10)
        
        # 타이머
        self.timer = self.create_timer(0.05, self.publish_state)  # 20Hz
    
    def on_cmd_vel(self, msg):
        """속도 명령 수신 → 걷기 파라미터 조정"""
        self.get_logger().info(
            f'Command: linear_x={msg.linear.x:.2f}, '
            f'angular_z={msg.angular.z:.2f}')
        # 보행 속도/방향 업데이트
        self.update_gait_parameters(
            speed=msg.linear.x,
            turn_rate=msg.angular.z,
        )
    
    def publish_state(self):
        """Humanoid 상태 발행"""
        # Joint State
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.robot.dof_names
        joint_msg.position = self.robot.get_joint_positions().tolist()
        joint_msg.velocity = self.robot.get_joint_velocities().tolist()
        self.joint_pub.publish(joint_msg)
        
        # Odometry (간이)
        pos, orient = self.robot.get_world_pose()
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[1]
        odom_msg.pose.pose.position.z = pos[2]
        self.odom_pub.publish(odom_msg)
    
    def update_gait_parameters(self, speed=0.0, turn_rate=0.0):
        """보행 파라미터 업데이트"""
        pass  # Gait generator와 연결
```

### 5.2 rviz2 Humanoid Visualization

```bash
# rviz2에서 Humanoid 표시
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2

# Displays > Add > RobotModel
#   Robot Description: robot_description
#   TF Prefix: humanoid_

# Displays > Add > TF
# Displays > Add > Odometry (topic: /humanoid/odom)
```

---

## 6. Motion Retargeting

### 6.1 개념

Motion Retargeting은 사람의 동작을 Humanoid 로봇에 전이하는 기술입니다:

```
사람 (MoCap/Video) → Skeleton 추출 → Retargeting → Humanoid Joints
```

### 6.2 간단한 Retargeting

```python
class SimpleMotionRetargeter:
    """사람 동작 → 휴머노이드 관절 매핑"""
    
    # 사람 → 휴머노이드 관절 매핑
    JOINT_MAP = {
        'left_shoulder': 'l_shoulder_pitch',
        'right_shoulder': 'r_shoulder_pitch',
        'left_elbow': 'l_elbow',
        'right_elbow': 'r_elbow',
        'left_hip': 'l_hip_pitch',
        'right_hip': 'r_hip_pitch',
        'left_knee': 'l_knee',
        'right_knee': 'r_knee',
        'left_ankle': 'l_ankle_pitch',
        'right_ankle': 'r_ankle_pitch',
        'neck': 'neck_pitch',
    }
    
    def __init__(self, dof_names):
        self.mapping = {}
        for human_joint, robot_joint in self.JOINT_MAP.items():
            if robot_joint in dof_names:
                self.mapping[human_joint] = dof_names.index(robot_joint)
    
    def retarget(self, human_poses, scale_factor=1.0):
        """사람 포즈를 로봇 관절 각도로 변환"""
        robot_pose = [0.0] * max(self.mapping.values()) + [0.0]
        
        for human_joint, robot_idx in self.mapping.items():
            if human_joint in human_poses:
                # 스케일링 + 제한 적용
                angle = human_poses[human_joint] * scale_factor
                robot_pose[robot_idx] = np.clip(
                    angle, -np.pi, np.pi)
        
        return np.array(robot_pose)
```

---

## 7. 실행 절차

### 7.1 Terminal Setup

```bash
# ════════════════════════════════════════════════════════
# Humanoid — 3 Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim Humanoid
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-3/step20_humanoid.py

# 터미널 2: ROS2 Monitor
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list | grep /humanoid
ros2 topic echo /humanoid/joint_states --once | head -30

# 터미널 3: rviz2 (Humanoid 시각화)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2
```

### 7.2 수동 제어

```bash
# Humanoid 걷기 속도 명령
ros2 topic pub /humanoid/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# 정지
ros2 topic pub /humanoid/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10
```

---

## 8. 문제 해결

### 문제 1: Humanoid가 넘어집니다.

**해결:**
- CoM (Center of Mass)를 발 안에 유지
- ZMP 기반 안정화 사용
- 무릎을 약간 굽혀 충격 흡수
- 보폭을 줄이고 속도를 낮춤

### 문제 2: IK가 수렴하지 않습니다.

**해결:**
```python
# DLS IK 설정
solver = IKSolver(
    prim_path="/World/Humanoid/r_foot",
    solver_type="dls",
    max_iterations=200,     # 증가
    tolerance=0.05,         # 완화
    damping=0.5,            # 댐핑 추가
)
```

### 문제 3: GR1 USD를 찾을 수 없습니다.

**해결:**
```bash
# Isaac Sim Asset Browser에서 확인
# 또는 다른 휴머노이드 사용 (H1, custom URDF)
# 또는 간단한 휴머노이드 직접 생성
```

---

## 9. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Humanoid 로딩 | GR1/H1 USD 로딩, DOF 확인 |
| ✅ Full-Body IK | DLS IK Solver 설정 |
| ✅ Walking Gait | Open-Loop + ZMP 기반 |
| ✅ Task Space Control | 상체/하체 분리 제어 |
| ✅ ROS2 Interface | Joint State, Odometry, Cmd |
| ✅ Motion Retargeting | 사람 → 로봇 매핑 |

---

## 10. 다음 Step 예고

**Step 21 — Warehouse Automation**에서는:
- 창고 시뮬레이션 환경 구축
- 다중 AMR + Manipulator 협업
- Order Fulfillment Pipeline
- Path Planning in Warehouse
- ROS2 Multi-Robot Warehouse System

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim Humanoid | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/robot_setup.html |
| GR1 Robot | https://www.youtube.com/watch?v=demo_gr1 |
| ZMP Walking | https://en.wikipedia.org/wiki/Zero_moment_point |
| Motion Retargeting | https://thegradient.pub/motion-retargeting/ |
| ROS2 + Humanoid | https://docs.ros.org/en/humble/Tutorials/Intermediate/Humanoid.html |
