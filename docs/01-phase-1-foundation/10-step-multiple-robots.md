# Step 10 — Multiple Robots & Coordination

> **소요 시간**: 60분
> **난이도**: ★★★☆☆ (중급)
> **선수 조건**: Step 07~09 완료

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **여러 로봇**을 하나의 Scene에서 동시에 제어한다
2. **ArticulationView**로 여러 Articulation을 일괄 제어한다
3. Robot 간 **충돌 회피** 기본 로직을 구현한다
4. **서로 다른 타입의 로봇**을 함께 배치한다
5. 간단한 **Fleet Coordination**(군집) 시나리오를 구현한다
6. 전체 Phase 1의 학습 내용을 **종합 복습**한다

---

## 1. 여러 로봇 배치

### 1.1 동일 로봇 여러 대

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

ROBOT_USD = "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd"

# 여러 TurtleBot3 배치
robot_configs = [
    {"path": "/World/Robot_0", "pos": [0.0, 0.0, 0.1]},
    {"path": "/World/Robot_1", "pos": [1.5, 0.0, 0.1]},
    {"path": "/World/Robot_2", "pos": [-1.5, 0.0, 0.1]},
    {"path": "/World/Robot_3", "pos": [0.0, 1.5, 0.1]},
]

robots = []
for cfg in robot_configs:
    add_reference_to_stage(ROBOT_USD, cfg["path"])
    robot = Robot(prim_path=cfg["path"], position=np.array(cfg["pos"]))
    world.scene.add(robot)
    robots.append(robot)
```

### 1.2 서로 다른 로봇

```python
# TurtleBot3 + Franka 함께 배치
add_reference_to_stage("/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd", "/World/TB3")
add_reference_to_stage("/Isaac/Robots/Franka/franka_panda.usd", "/World/Franka")

tb3 = Robot(prim_path="/World/TB3", position=np.array([1.0, 0.0, 0.1]))
franka = Robot(prim_path="/World/Franka", position=np.array([-1.0, 0.0, 0.0]))

world.scene.add(tb3)
world.scene.add(franka)
```

---

## 2. ArticulationView로 일괄 제어

### 2.1 ArticulationView 개념

**ArticulationView**는 **여러 개의 Articulation을 하나의 배열처럼 관리**하는 클래스입니다.

```python
from omni.isaac.core.articulations import ArticulationView

# ArticulationView 생성
robot_view = ArticulationView(
    prim_paths_expr="/World/Robot_[0-9]",  # 정규식으로 여러 Prim 지정
    name="RobotView",
)

# 모든 로봇의 관절 위치 읽기 (한 번에)
all_joint_pos = robot_view.get_joint_positions()  # shape: (N, num_dof)

# 모든 로봇에 동일한 명령 적용
robot_view.apply_action(
    ArticulationAction(joint_velocities=np.array([5.0, 5.0]))
)
```

### 2.2 개별/일괄 제어

```python
# 일괄: 모든 로봇 직진
robot_view.apply_action(
    ArticulationAction(joint_velocities=np.array([
        [6.0, 6.0],  # Robot_0
        [6.0, 6.0],  # Robot_1
        [6.0, 6.0],  # Robot_2
    ]))
)

# 개별: Robot_0만 회전
robot_view.apply_action(
    ArticulationAction(
        joint_velocities=np.array([
            [3.0, 3.0],   # Robot_0
            [6.0, 6.0],   # Robot_1
            [6.0, 6.0],   # Robot_2
        ]),
        joint_indices=np.array([
            [0, 1],
            [0, 1],
            [0, 1],
        ]),
    )
)
```

---

## 3. 충돌 회피 (Collision Avoidance)

### 3.1 거리 기반 회피

가장 간단한 충돌 회피: 로봇 간 거리가 임계값보다 가까워지면 회전합니다.

```python
class SimpleCollisionAvoidance:
    def __init__(self, min_distance=0.3):
        self.min_distance = min_distance
    
    def get_avoidance_command(self, robot, all_robots):
        """로봇과 가장 가까운 이웃과의 거리를 기준으로 회피 명령 생성"""
        pos, _ = robot.get_world_pose()
        
        closest_dist = float('inf')
        for other in all_robots:
            if other == robot:
                continue
            other_pos, _ = other.get_world_pose()
            dist = np.linalg.norm(pos[:2] - other_pos[:2])
            if dist < closest_dist:
                closest_dist = dist
                closest_pos = other_pos
        
        if closest_dist < self.min_distance:
            # 가까운 로봇과 반대 방향으로 회전
            angle = np.arctan2(pos[1] - closest_pos[1], 
                               pos[0] - closest_pos[0])
            # 충돌 위험 → 회피 회전
            return 0.1, 0.5  # (linear_x, angular_z)
        else:
            # 안전 → 직진
            return 0.15, 0.0
```

### 3.2 벽 충돌 회피 (LiDAR 활용)

```python
class LidarCollisionAvoidance:
    def __init__(self, lidar, min_distance=0.3):
        self.lidar = lidar
        self.min_distance = min_distance
    
    def get_command(self):
        pc = self.lidar.get_pointcloud()
        if pc is None or pc.shape[0] == 0:
            return 0.15, 0.0  # 기본 직진
        
        # 전방 영역 포인트 필터링 (x > 0, 일정 각도 내)
        front_points = pc[(pc[:, 0] > 0) & (abs(pc[:, 1]) < 0.2)]
        
        if len(front_points) == 0:
            return 0.15, 0.0
        
        min_front_dist = np.min(np.linalg.norm(front_points, axis=1))
        
        if min_front_dist < self.min_distance:
            # 장애물 발견 → 좌회전
            return 0.0, 0.3
        else:
            return 0.15, 0.0
```

---

## 4. 실습: TurtleBot3 Fleet 주행

### 4.1 시나리오

```
4대의 TurtleBot3가 십자형으로 배치되어 출발:
    
         R2           각 로봇이 중앙을 향해 직진
         |            중앙에서 충돌 직전 회피
    R1───┼───R3       회피 후 계속 주행
         |
         R4
```

### 4.2 구현

```python
import numpy as np

# 로봇 배치 (십자형)
positions = [
    (0.0, 2.0, 0.1),    # R1 (아래에서 위로)
    (-2.0, 0.0, 0.1),   # R2 (오른쪽에서 왼쪽으로)
    (2.0, 0.0, 0.1),    # R3 (왼쪽에서 오른쪽으로)
    (0.0, -2.0, 0.1),   # R4 (위에서 아래로)
]

robots = []
for i, pos in enumerate(positions):
    path = f"/World/TB3_{i}"
    add_reference_to_stage(ROBOT_USD, path)
    robot = Robot(prim_path=path, position=np.array(pos))
    
    # 각 로봇의 초기 방향 설정
    if i == 0:  # R1: 위 → 아래 (180도 회전)
        robot.set_world_pose(position=np.array(pos), 
                             orientation=quat_from_euler([0, 0, np.pi]))
    elif i == 1:  # R2: 오른쪽 → 왼쪽
        robot.set_world_pose(position=np.array(pos),
                             orientation=quat_from_euler([0, 0, np.pi/2]))
    elif i == 2:  # R3: 왼쪽 → 오른쪽
        robot.set_world_pose(position=np.array(pos),
                             orientation=quat_from_euler([0, 0, -np.pi/2]))
    # R4: 위 → 아래 (기본 방향)
    
    world.scene.add(robot)
    robots.append(robot)

# 충돌 회피
avoidance = SimpleCollisionAvoidance(min_distance=0.5)

for _ in range(500):
    world.step(render=True)
    
    for robot in robots:
        controller = robot.get_articulation_controller()
        linear_x, angular_z = avoidance.get_avoidance_command(robot, robots)
        speeds = compute_wheel_speeds(linear_x, angular_z)
        controller.apply_action(
            ArticulationAction(joint_velocities=speeds, joint_indices=[0, 1])
        )
```

---

## 5. Multi-Robot GUI 제어

### 5.1 Scene View에서 개별 선택

여러 로봇이 Scene에 있으면 Stage Window에서 개별 선택 후:
- Property Panel에서 개별 Transform/속성 조정
- Ctrl+클릭으로 여러 개 선택 → 일괄 속성 변경

### 5.2 여러 로봇의 Stage 구조

```
/World
├── TB3_0 (TurtleBot3)
│   ├── base_link
│   ├── wheel_left_link
│   └── wheel_right_link
├── TB3_1 (TurtleBot3)
│   ├── base_link
│   ├── wheel_left_link
│   └── wheel_right_link
├── Franka (Franka Panda)
│   ├── panda_link0
│   ├── panda_link1
│   └── ...
├── Ground Plane
└── PhysicsScene
```

---

## 6. Phase 1 종합 복습

### 6.1 지금까지 배운 내용

| Step | 주제 | 핵심 내용 |
|------|------|-----------|
| 01 | 설치 | pip/OV/Container 4가지 방법 |
| 02 | GUI | 7개 UI 패널, Create/Transform/Material/Physics |
| 03 | OmniGraph | Action Graph, OnPlaybackTick, WritePrimProperty |
| 04 | USD Stage | Prim/Attribute/Reference/Layer/Specifier |
| 05 | Python Scripting | Standalone/Extension/Notebook, World, Sim Loop |
| 06 | Hello Robot | TurtleBot3 로딩, Articulation 구조 |
| 07 | Controller | ArticulationController, Differential Drive |
| 08 | Manipulator | Franka Panda, IK, Pick & Place |
| 09 | Sensor | Camera/LiDAR/IMU/Contact, ROS2 Bridge |
| 10 | Multiple Robots | ArticulationView, 충돌 회피, Fleet |

### 6.2 Phase 1 → Phase 2 연결

Phase 1에서 배운 기초(Foundation)를 바탕으로
Phase 2에서는 **ROS2 Integration**과 **실제 애플리케이션**에 집중합니다:

```
Phase 1 (Foundation)      →      Phase 2 (ROS2 + Application)
─────────────────────────────────────────────────────
USD Stage                 →      ROS2 Bridge
Articulation Controller   →      ROS2 Control
Sensor Data Collection    →      ROS2 Topic Pub/Sub
OmniGraph                 →      ROS2 Action Graph
Multiple Robots           →      Multi-Robot ROS2 System
```

---

## 7. 문제 해결 (Troubleshooting)

### 문제 1: 여러 로봇을 추가했는데 성능이 느립니다.

**원인**: 로봇 수가 많을수록 물리/렌더링 부하 증가
**해결**:
```python
# 렌더링 DT 증가
world = World(rendering_dt=1/30.0)  # 30 FPS로 낮춤

# 불필요한 센서 비활성화
# headless 모드 고려
```

### 문제 2: ArticulationView에서 Prim 경로를 찾을 수 없습니다.

**원인**: prim_paths_expr 정규식이 잘못됨
**해결**: `is_prim_path_valid`로 각 경로 먼저 확인
```python
for i in range(4):
    path = f"/World/Robot_{i}"
    assert is_prim_path_valid(path), f"{path} not found!"
```

### 문제 3: 로봇들이 서로 밀어내며 물리 충돌이 발생합니다.

**원인**: Articulation 간 기본 물리 충돌 처리
**해결**: Collision API 설정 조정 또는 충돌 회피 로직 적용

### 문제 4: 동일 USD를 여러 번 Reference했는데 모두 같은 위치에 나타납니다.

**원인**: 각 Reference Prim의 position이 모두 (0, 0, 0)으로 초기화됨
**해결**: Robot 생성 시 `position` 파라미터를 각각 다르게 설정

---

## 8. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 여러 로봇 배치 | 동일/다른 로봇을 한 Scene에 |
| ✅ ArticulationView | 여러 Articulation 일괄 제어 |
| ✅ 충돌 회피 | 거리 기반 + LiDAR 기반 |
| ✅ Multi-Robot Fleet | 십자형 교차 주행 |
| ✅ Phase 1 종합 복습 | 10개 Step 전체 요약 |

---

## 9. Phase 1 수료

🎉 축하합니다! Isaac Sim 기초(Foundation) Phase 1을 완료했습니다.

### 수료 기준

- [ ] Isaac Sim 5.1 설치 완료
- [ ] GUI를 자유롭게 사용 가능
- [ ] USD Stage 구조 이해
- [ ] Python Standalone/Extension 스크립트 작성 가능
- [ ] TurtleBot3 제어 (Differential Drive)
- [ ] Franka Panda 제어 (Position/IK)
- [ ] 센서 데이터 수집 (Camera/LiDAR/IMU)
- [ ] 여러 로봇 동시 제어

### Phase 1 학습 로드맵

```
초보자
  │
  ▼
Step 01-02: Isaac Sim 설치 + GUI 적응
  │
  ▼
Step 03-05: OmniGraph + USD + Python (핵심 개념)
  │
  ▼
Step 06-07: Mobile Robot (TurtleBot3 제어)
  │
  ▼
Step 08-09: Manipulator + Sensor (Franka, Camera/LiDAR)
  │
  ▼
Step 10: Multi-Robot (종합)
  │
  ▼
Phase 2: ROS2 Integration + 실전 애플리케이션
```

### 다음을 시도해보세요

1. 지금까지 만든 Scene USD를 정리하여 포트폴리오로 저장
2. TurtleBot3 + Franka + Sensor가 모두 있는 복합 Scene 구성
3. 각 Step의 코드 스크립트를 실행하여 정상 작동 확인
4. 궁금한 점을 IsaacSimKR 네이버 카페/유튜브에서 찾아보기

---

## Phase 2 Preview

**Phase 2 — ROS2 Integration & Application**에서는:

| Step | 주제 |
|------|------|
| 11 | ROS2 Bridge 설치 및 설정 |
| 12 | ROS2 TurtleBot3 Teleop |
| 13 | ROS2 SLAM in Isaac Sim |
| 14 | ROS2 Navigation2 (Nav2) |
| 15 | ROS2 Manipulator Control (MoveIt2) |
| 16 | Multi-Robot ROS2 System |
| 17 | Synthetic Data Generation |
| 18 | Performance Optimization |

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| ArticulationView API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.core.html#omni-isaac-core-articulations |
| Multiple Robots 예제 | (Standalone Examples) |
| ROS2 Multi-Robot | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_robot.html |
| Isaac Sim Examples | (설치 경로)/standalone_examples/ |
