# Step 07 — TurtleBot3 제어하기 (Controller 기초)

> **소요 시간**: 75분
> **난이도**: ★★☆☆☆ (초급~중급)
> **선수 조건**: Step 06 완료 (로봇 불러오기)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Articulation Controller**로 로봇 관절에 속도 명령을 보낸다
2. **Differential Drive** 모델을 이해하고 파라미터를 설정한다
3. Python으로 TurtleBot3를 **직진/회전/정지**시킨다
4. **Odometry**를 계산하여 로봇의 위치를 추정한다
5. **Keyboard Teleop**으로 실시간 로봇 제어를 한다
6. 간단한 **경로(Trajectory)** 를 따라 주행시킨다

---

## 1. 로봇 제어 개요

### 1.1 제어 체인

로봇을 움직이기 위한 명령 체인은 다음과 같습니다:

```
사용자 입력 (키보드/조이스틱/코드)
    │
    ▼
목표 속도 (linear_x, angular_z)
    │
    ▼
Differential Controller
    │ (좌/우 바퀴 속도로 변환)
    ▼
Articulation Controller
    │ (Joint Velocity 명령)
    ▼
PhysX Articulation
    │ (물리 엔진 실행)
    ▼
로봇 움직임
```

### 1.2 두 가지 제어 방식

| 방식 | 설명 | 사용처 |
|------|------|--------|
| **OmniGraph** (GUI) | 노드 연결로 제어 로직 구성 | ROS2 연동, Keyboard Teleop |
| **Python API** (코드) | `ArticulationController` 직접 호출 | 자율 주행, 경로 계획 |

> 이 Step에서는 **Python API**로 집중 학습합니다.
> OmniGraph 방식은 Step 03에서 이미 다루었습니다.

---

## 2. Articulation Controller

### 2.1 Articulation Controller란?

관절(Joint)에 직접 힘/속도/위치 명령을 보내는 Isaac Sim의 컨트롤러입니다.

```python
from omni.isaac.core.articulations import ArticulationController

# Robot 객체에서 컨트롤러 가져오기
controller = robot.get_articulation_controller()
```

### 2.2 지원 명령 타입

| 명령 타입 | 설명 | 사용 예 |
|-----------|------|---------|
| **Velocity** | 관절 속도 제어 | 바퀴 회전 속도 |
| **Position** | 관절 위치 제어 | 로봇 팔 각도 |
| **Effort** | 관절 토크/힘 제어 | 그리퍼 파지력 |
| **None** | 명령 없음 (자유 회전) | |

### 2.3 바퀴 속도 명령

```python
# 좌/우 바퀴에 속도 명령 (rad/s)
joint_velocity_cmd = np.array([left_wheel_speed, right_wheel_speed])
controller.apply_action(joint_velocity_cmd)
```

> **참고**: TurtleBot3의 DOF 순서는 `[left_wheel_joint, right_wheel_joint]`입니다.

---

## 3. Differential Drive 모델

### 3.1 Differential Drive란?

두 개의 바퀴 속도를 독립적으로 제어하여 로봇을 움직이는 방식입니다.

```
          ▲ Y (전진 방향)
          │
    ┌────┴────┐
    │         │
  ──┤  Robot  ├── → X (좌우)
    │         │
    └─────────┘
   왼바퀴  오른바퀴
   v_left   v_right
```

### 3.2 속도 변환 공식

목표 `linear_x` (전진 속도)와 `angular_z` (회전 속도)를 좌/우 바퀴 속도로 변환:

```
v_left  = linear_x  -  angular_z * wheelDistance / 2
v_right = linear_x  +  angular_z * wheelDistance / 2
```

| 변수 | 설명 | TurtleBot3 Waffle 값 |
|------|------|----------------------|
| `linear_x` | 전진 속도 (m/s) | 목표값 (예: 0.2) |
| `angular_z` | 회전 속도 (rad/s) | 목표값 (예: 0.5) |
| `wheelDistance` | 좌우 바퀴 사이 거리 (m) | **0.141** |
| `wheelRadius` | 바퀴 반지름 (m) | **0.033** |
| `v_left`, `v_right` | 좌/우 바퀴 각속도 (rad/s) | 계산 결과 |

> **주의**: 위 공식은 바퀴 **각속도(rad/s)** 가 아닌 **선속도(m/s)** 입니다.
> `ArticulationController`는 **각속도(rad/s)** 를 입력받으므로,
> `v_left / wheelRadius`로 나누어 각속도로 변환해야 합니다.

### 3.3 최종 변환 코드

```python
def compute_wheel_speeds(linear_x: float, angular_z: float):
    """
    Differential Drive 모델: 목표 속도를 좌/우 바퀴 각속도로 변환
    
    Args:
        linear_x: 전진 속도 (m/s)
        angular_z: 회전 속도 (rad/s)
    
    Returns:
        (left_wheel_speed, right_wheel_speed) in rad/s
    """
    wheel_distance = 0.141   # TurtleBot3 Waffle: 0.141m
    wheel_radius = 0.033     # TurtleBot3 Waffle: 0.033m
    
    # 바퀴 선속도 (m/s)
    v_left = linear_x - angular_z * wheel_distance / 2
    v_right = linear_x + angular_z * wheel_distance / 2
    
    # 바퀴 각속도 (rad/s)
    w_left = v_left / wheel_radius
    w_right = v_right / wheel_radius
    
    return np.array([w_left, w_right])
```

---

## 4. Python으로 TurtleBot3 제어

### 4.1 기본 제어 루프

```python
import numpy as np
from omni.isaac.core.utils.types import ArticulationAction

# Articulation Controller 가져오기
controller = robot.get_articulation_controller()

# 시뮬레이션 루프
for i in range(500):
    world.step(render=True)
    
    # 100프레임까지 직진, 이후 정지
    if i < 100:
        wheel_speeds = compute_wheel_speeds(linear_x=0.2, angular_z=0.0)
    else:
        wheel_speeds = compute_wheel_speeds(linear_x=0.0, angular_z=0.0)
    
    # 속도 명령 적용 (Joint Velocity 방식)
    action = ArticulationAction(
        joint_velocities=wheel_speeds,
        joint_indices=[0, 1],  # left_wheel=0, right_wheel=1
    )
    controller.apply_action(action)
```

### 4.2 직진 (Go Straight)

```python
# 직진: linear_x > 0, angular_z = 0
# 양쪽 바퀴 같은 속도
wheel_speeds = compute_wheel_speeds(linear_x=0.2, angular_z=0.0)
# 결과: [6.06, 6.06] rad/s
```

### 4.3 회전 (Turn)

```python
# 제자리 회전: linear_x = 0, angular_z > 0
# 좌/우 바퀴 반대 방향
wheel_speeds = compute_wheel_speeds(linear_x=0.0, angular_z=0.5)
# 결과: [-2.14, 2.14] rad/s

# 좌회전: angular_z > 0 (시계 반대 방향)
# 우회전: angular_z < 0 (시계 방향)
```

### 4.4 곡선 주행

```python
# 우측 앞으로 곡선 주행
wheel_speeds = compute_wheel_speeds(linear_x=0.2, angular_z=-0.3)
# 좌측 바퀴가 더 빠름
```

---

## 5. Odometry (위치 추정)

### 5.1 Odometry란?

**Odometry**는 바퀴의 회전을 기반으로 로봇의 위치를 추정하는 방법입니다.

> **한계**: 바퀴 슬립이 발생하면 실제 위치와 차이가 발생합니다.
> 실제 로봇에서는 LiDAR나 카메라로 보정합니다.

### 5.2 Odometry 계산

```python
class SimpleOdometry:
    def __init__(self, wheel_distance=0.141, wheel_radius=0.033):
        self.wheel_distance = wheel_distance
        self.wheel_radius = wheel_radius
        self.x = 0.0   # X 위치 (m)
        self.y = 0.0   # Y 위치 (m)
        self.theta = 0.0  # 방향 (rad)
    
    def update(self, left_vel, right_vel, dt):
        """바퀴 속도로 위치 업데이트"""
        # 바퀴 선속도
        v_left = left_vel * self.wheel_radius
        v_right = right_vel * self.wheel_radius
        
        # 로봇 속도
        v = (v_left + v_right) / 2          # 선속도
        omega = (v_right - v_left) / self.wheel_distance  # 각속도
        
        # 위치 업데이트 (Euler 적분)
        self.theta += omega * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        
        return self.x, self.y, self.theta
```

### 5.3 시뮬레이션 Ground Truth와 비교

Isaac Sim에서 Ground Truth 위치는 `get_world_pose()`로 얻을 수 있습니다:

```python
# Ground Truth (시뮬레이터)
true_pos, true_orient = robot.get_world_pose()

# Odometry 추정 위치
odo = SimpleOdometry()
odo.update(left_vel, right_vel, dt=1/60.0)
est_pos = (odo.x, odo.y)

# 오차
error = np.linalg.norm(np.array(true_pos[:2]) - np.array(est_pos[:2]))
print(f"Odometry error: {error:.4f}m")
```

---

## 6. Keyboard Teleop

### 6.1 OmniGraph 키보드 제어 (GUI)

Step 03에서 배운 OmniGraph를 사용하여 키보드로 TurtleBot3를 제어할 수 있습니다.

**Action Graph 구성**:

```
On Playback Tick
    │
    ├──→ Keyboard Input (W/S) → Multiply(linear_speed: 0.2)
    │         → Differential Controller → Articulation Controller
    │
    └──→ Keyboard Input (A/D) → Multiply(angular_speed: 0.5)
              → Differential Controller → Articulation Controller
```

**단축키 사용**:
```
Tools > Robotics > Omnigraph Controllers > Differential Controller
```
1. **Articulation Root**: `/World/TurtleBot3`
2. **wheelDistance**: `0.141`
3. **wheelRadius**: `0.033`
4. **Use Keyboard Control (WASD)**: 체크
5. **OK** 버튼

> 그러면 OmniGraph가 자동 생성되고, WASD로 TurtleBot3를 제어할 수 있습니다.

### 6.2 Python Keyboard Teleop

```python
from omni.isaac.core.utils.keyboard import KeyboardCommandReader

keyboard_reader = KeyboardCommandReader()

for i in range(1000):
    world.step(render=True)
    
    # 키보드 입력 읽기
    cmd = keyboard_reader.advance()
    
    if cmd is not None:
        linear_x = cmd.x  # 전/후 (W/S)
        angular_z = cmd.z  # 좌/우 회전 (A/D)
    else:
        linear_x = 0.0
        angular_z = 0.0
    
    wheel_speeds = compute_wheel_speeds(linear_x, angular_z)
    action = ArticulationAction(joint_velocities=wheel_speeds)
    controller.apply_action(action)
```

---

## 7. 경로 주행 (Trajectory Following)

### 7.1 간단한 경로: 사각형 주행

```python
# 사각형 경로: 직진 2m → 우회전 90° → 직진 2m → ...
waypoints = [
    (0.2, 0.0, 1.0),    # 직진 (1초)
    (0.0, 0.5, 1.57),   # 우회전 90도 (1.57 rad ≈ π/2)
    (0.2, 0.0, 1.0),    # 직진
    (0.0, 0.5, 1.57),   # 우회전
    (0.2, 0.0, 1.0),    # 직진
    (0.0, 0.5, 1.57),   # 우회전
    (0.2, 0.0, 1.0),    # 직진
    (0.0, 0.5, 1.57),   # 우회전 (원점 복귀)
]

for linear_x, angular_z, duration in waypoints:
    wheel_speeds = compute_wheel_speeds(linear_x, angular_z)
    frames = int(duration * 60)  # 60fps 기준
    
    for _ in range(frames):
        world.step(render=True)
        action = ArticulationAction(joint_velocities=wheel_speeds)
        controller.apply_action(action)
```

### 7.2 원 주행

```python
# 반지름 1m 원을 그리며 주행
# v = r * ω → linear_x = radius * angular_z
radius = 1.0
linear_x = 0.2
angular_z = linear_x / radius  # = 0.2 rad/s

wheel_speeds = compute_wheel_speeds(linear_x, angular_z)

for _ in range(600):  # 약 10초 (1바퀴 ≈ 31.4s)
    world.step(render=True)
    action = ArticulationAction(joint_velocities=wheel_speeds)
    controller.apply_action(action)
```

---

## 8. 실습: TurtleBot3 자율 주행

### 목표

TurtleBot3를 일정 시간 직진시키고, 바퀴 속도 데이터와 Odometry를 수집하여 실제 위치와 비교합니다.

### 시나리오

```
1. TurtleBot3를 Scene에 배치 (Step 06)
2. 2초간 직진 (linear_x=0.2, angular_z=0)
3. 1초간 제자리 회전 (linear_x=0, angular_z=0.5)
4. 2초간 직진
5. 정지 후 Odometry vs Ground Truth 비교
```

---

## 9. 문제 해결 (Troubleshooting)

### 문제 1: 바퀴가 회전하지 않습니다.

**확인 사항**:
- [ ] `ArticulationController`를 가져왔는가?
- [ ] `apply_action`을 매 프레임 호출하는가?
- [ ] 바퀴 속도가 너무 작지 않은가? (예: 0.01 rad/s)
- [ ] `joint_indices`가 올바른가? (left=0, right=1)

### 문제 2: 로봇이 명령과 반대 방향으로 움직입니다.

**원인**: 바퀴 인덱스가 반대
**해결**: `joint_indices=[1, 0]`으로 변경 (순서 스왑)
```python
dof_names = robot.dof_names  # DOF 순서 확인
print(dof_names)  # ['left_wheel_joint', 'right_wheel_joint']
```

### 문제 3: 로봇이 명령을 멈춰도 계속 움직입니다.

**원인**: 마찰이 없어 관성이 유지됨
**해결**: 명령을 중단할 때 반드시 0 속도 명령 전송
```python
# 정지 명령
controller.apply_action(ArticulationAction(joint_velocities=[0, 0]))
```

### 문제 4: 로봇이 바닥을 뚫고 떨어집니다.

**원인**: Articulation과 Ground Plane 충돌 문제
**해결**: 로봇의 Translate Z를 충분히 높게 설정 (0.05~0.1)

### 문제 5: Odometry 오차가 너무 큽니다.

**원인**: 
- 바퀴 슬립 (급가속/급회전)
- 관성에 의한 미끄러짐
**해결**: 
- `linear_x`를 0.1 이하로 낮춤
- `angular_z`를 0.3 이하로 낮춤
- 감속/가속을 부드럽게 (ramp-up/ramp-down)

---

## 10. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Articulation Controller | `apply_action()`으로 관절 속도/위치/토크 명령 |
| ✅ Differential Drive | 좌/우 바퀴 속도로 직진/회전 제어 |
| ✅ 속도 변환 공식 | v = (v_l + v_r)/2, ω = (v_r - v_l)/wheelDistance |
| ✅ Odometry | 바퀴 회전 기반 위치 추정 |
| ✅ Keyboard Teleop | WASD로 실시간 제어 |
| ✅ 경로 주행 | 사각형/원 Trajectory 생성 |

### 제어 파라미터 요약 (TurtleBot3 Waffle)

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `wheelDistance` | 0.141 m | 좌우 바퀴 사이 거리 |
| `wheelRadius` | 0.033 m | 바퀴 반지름 |
| 최대 권장 `linear_x` | 0.3 m/s | 너무 빠르면 슬립 발생 |
| 최대 권장 `angular_z` | 1.0 rad/s | 너무 빠르면 전복 위험 |

---

## 11. 다음 Step 예고

**Step 08 — Manipulator 기초 (Franka Panda)** 에서는:
- Manipulator(로봇 팔)의 Articulation 구조
- Inverse Kinematics(IK) 개념과 Isaac Sim의 IK Solver
- Franka Panda 불러오기
- End Effector 위치 제어
- Pick & Place 기본 동작

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Articulation Controller | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.core.html#omni-isaac-core-articulations |
| Differential Drive OmniGraph | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/commonly_used_omnigraph_shortcuts.html |
| TurtleBot3 제원 | https://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/ |
| ROS2 Control | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_control.html |
| Keyboard Input | https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/nodes/omni-graph-action/keyboardinput.html |
