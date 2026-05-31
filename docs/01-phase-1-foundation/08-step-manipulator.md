# Step 08 — Manipulator 기초 (Franka Panda)

> **소요 시간**: 75분
> **난이도**: ★★★☆☆ (중급)
> **선수 조건**: Step 07 완료 (TurtleBot3 제어)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Manipulator(로봇 팔)** 의 구조와 Articulation 차이를 이해한다
2. **Franka Panda Research 3** 로봇을 Scene에 불러온다
3. **Joint Position 제어**로 각 관절의 목표 각도를 설정한다
4. **Inverse Kinematics (IK)** 의 기본 개념을 이해한다
5. Isaac Sim의 **IK Solver**를 사용하여 End Effector를 제어한다
6. **그리퍼(Gripper)** 를 열고 닫는다
7. 간단한 **Pick & Place** 동작을 시뮬레이션한다

---

## 1. Manipulator vs Mobile Robot

### 1.1 차이점

| 항목 | Mobile Robot (TurtleBot3) | Manipulator (Franka Panda) |
|------|--------------------------|---------------------------|
| 주된 제어 | 바퀴 속도 (Joint Velocity) | 관절 각도 (Joint Position) |
| DOF | 2 (좌/우 바퀴) | 7 (팔) + 2 (그리퍼) |
| 기준 좌표계 | Base (몸체 중심) | End Effector (손끝) |
| 제어 모드 | Velocity | Position (주로) |
| 물리 특징 | Differential Drive | Articulation + Kinematics |

### 1.2 Franka Panda Research 3 제원

| 항목 | 값 |
|------|-----|
| 자유도 (DOF) | 7 (팔) + 2 (그리퍼) |
| 최대 도달 거리 | 855 mm |
| 최대 페이로드 | 3 kg |
| 관절 타입 | 7x Revolute (회전 관절) |
| 그리퍼 타입 | 2x Prismatic (직선 이동) |
| 무게 | 약 18 kg |

### 1.3 Franka Panda 관절 구조

```
panda_joint1 (J1) ─── 기저 회전 (좌/우)
    │
panda_joint2 (J2) ─── 숄더 (상/하)
    │
panda_joint3 (J3) ─── 상완 회전
    │
panda_joint4 (J4) ─── 엘보 (상/하)
    │
panda_joint5 (J5) ─── 전완 회전
    │
panda_joint6 (J6) ─── 손목 (상/하)
    │
panda_joint7 (J7) ─── 손목 회전
    │
panda_hand ─────────── 그리퍼 베이스
    ├── panda_finger_joint1 (좌)
    └── panda_finger_joint2 (우)
```

---

## 2. Franka Panda 불러오기

### 2.1 GUI로 불러오기

```
Content Browser > Isaac > Robots > Franka > franka_panda.usd
→ Viewport로 드래그 앤 드롭
```

또는:
```
Create > Isaac > Robots > Franka > franka_panda.usd
```

### 2.2 Python으로 불러오기

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Franka USD 경로
FRANKA_USD = "/Isaac/Robots/Franka/franka_panda.usd"
FRANKA_PATH = "/World/Franka"

add_reference_to_stage(usd_path=FRANKA_USD, prim_path=FRANKA_PATH)

robot = Robot(
    prim_path=FRANKA_PATH,
    name="Franka",
    position=np.array([0.0, 0.0, 0.0]),
)
```

### 2.3 Stage 계층 확인

```
/World
├── Ground Plane
├── PhysicsScene
└── franka_panda (Reference)
    ├── panda_link0         ← 베이스 (Articulation Root)
    ├── panda_link1
    ├── panda_link2
    ├── ...                 ← 7개 링크
    ├── panda_hand
    │   ├── panda_left_finger
    │   └── panda_right_finger
    └── panda_joint1 ~ panda_joint7  ← 관절 (Revolute)
```

---

## 3. Joint Position 제어

### 3.1 Position 제어 모드

Manipulator는 주로 **Position 제어**를 사용합니다.
각 관절의 목표 각도를 설정하면 Articulation Controller가 해당 위치로 이동시킵니다.

```python
# 관절 목표 각도 설정 (라디안)
joint_target = np.array([
    0.0,    # panda_joint1
    -0.5,   # panda_joint2
    0.0,    # panda_joint3
    -1.5,   # panda_joint4
    0.0,    # panda_joint5
    1.2,    # panda_joint6
    0.5,    # panda_joint7
])

# Joint Position 명령 전송
controller.apply_action(
    ArticulationAction(joint_positions=joint_target)
)
```

### 3.2 관절 한계 (Joint Limits)

각 관절에는 회전 가능한 범위가 있습니다. 범위를 벗어나면 명령이 무시되거나 물리 충돌이 발생합니다.

| 관절 | 최소 (rad) | 최대 (rad) | 설명 |
|------|-----------|-----------|------|
| J1 | -2.897 | 2.897 | 기저 회전 |
| J2 | -1.763 | 1.763 | 숄더 |
| J3 | -2.897 | 2.897 | 상완 회전 |
| J4 | -3.072 | -0.069 | 엘보 (주의: 음수만!) |
| J5 | -2.897 | 2.897 | 전완 회전 |
| J6 | -0.017 | 3.752 | 손목 (주의: 양수만!) |
| J7 | -2.897 | 2.897 | 손목 회전 |

> **⚠️ 중요**: J4와 J6은 비대칭 범위입니다.
> J4는 항상 음수, J6는 항상 양수로 유지해야 현실적인 자세가 됩니다.

### 3.3 홈 포즈 (Home Pose)

Franka의 기본 홈 포즈:

```python
HOME_POSE = np.array([
    0.0,      # J1
    -0.5,     # J2
    0.0,      # J3
    -1.5,     # J4
    0.0,      # J5
    1.2,      # J6
    0.5,      # J7
])
```

---

## 4. Inverse Kinematics (IK)

### 4.1 IK란?

**Inverse Kinematics**는 "End Effector(손끝)의 목표 위치/방향"으로부터
"각 관절의 각도"를 계산하는 문제입니다.

```
Forward Kinematics (FK):
  관절 각도 (J1~J7) → End Effector 위치 (x, y, z)

Inverse Kinematics (IK):
  End Effector 위치 (x, y, z) → 관절 각도 (J1~J7)
```

### 4.2 Isaac Sim의 IK Solver

Isaac Sim은 `PhysX` 기반의 IK Solver를 제공합니다.

**GUI로 IK 사용하기:**

1. Window > **IK Solver** 열기
2. Franka 선택
3. End Effector: `panda_hand` 선택
4. 목표 위치를 Gizmo로 드래그
5. Solver가 자동으로 관절 각도 계산

**Python으로 IK 사용하기:**

```python
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.rotations import euler_to_quat
from omni.isaac.manipulators import SingleManipulator

# Manipulator 객체 생성 (IK 포함)
manipulator = SingleManipulator(
    prim_path=FRANKA_PATH,
    name="Franka",
    end_effector_prim_name="panda_hand",
    robot_articulation=robot,
)

# End Effector 목표 위치 설정
target_position = np.array([0.5, 0.0, 0.3])   # x, y, z (m)
target_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z (Quaternion)

# IK 계산 및 적용
manipulator.apply_action(
    ArticulationAction(
        joint_positions=manipulator.get_ik_solver().compute_ik(
            target_position=target_position,
            target_orientation=target_orientation,
        )
    )
)
```

### 4.3 IK Solver 파라미터

| 파라미터 | 설명 | 기본값 |
|----------|------|--------|
| `max_iterations` | 최대 반복 횟수 | 100 |
| `tolerance` | 허용 오차 (m) | 0.001 |
| `solver_type` | Solver 알고리즘 | "LMA" (Levenberg-Marquardt) |
| `weight` | 각 관절 가중치 | 모두 1.0 |

---

## 5. 그리퍼(Gripper) 제어

### 5.1 그리퍼 관절

Franka Panda의 그리퍼는 2개의 Prismatic Joint로 구성됩니다:

```python
# 그리퍼 관절 이름
dof_names = robot.dof_names
print(dof_names)
# ['panda_joint1', ..., 'panda_joint7', 
#  'panda_finger_joint1', 'panda_finger_joint2']
```

**DOF 인덱스:**
- 0~6: 팔 관절 (J1~J7)
- 7~8: 그리퍼 (좌/우)

### 5.2 그리퍼 열기/닫기

```python
# 그리퍼 열기 (0.04 = 최대 개방)
action = ArticulationAction(
    joint_positions=np.array([
        0.0, -0.5, 0.0, -1.5, 0.0, 1.2, 0.5,  # 팔 (홈 포즈)
        0.04, 0.04        # 그리퍼 (열림)
    ]),
    joint_indices=[0, 1, 2, 3, 4, 5, 6, 7, 8],
)

# 그리퍼 닫기 (0.0 = 완전 밀착)
action = ArticulationAction(
    joint_positions=np.array([
        0.0, -0.5, 0.0, -1.5, 0.0, 1.2, 0.5,  # 팔 (홈 포즈)
        0.0, 0.0          # 그리퍼 (닫힘)
    ]),
)
```

---

## 6. 실습: Pick & Place 기본

### 6.1 시나리오

```
1. Franka가 홈 포즈에서 시작
2. 테이블 위 빨간 큐브를 향해 이동 (IK)
3. 그리퍼 닫기 (큐브 파지)
4. 큐브를 들어 올리기
5. 목적지로 이동
6. 그리퍼 열기 (큐브 놓기)
7. 홈 포즈로 복귀
```

### 6.2 구현

```python
import numpy as np
from omni.isaac.core.objects import VisualCuboid

# 큐브 생성
cube = VisualCuboid(
    prim_path="/World/Cube",
    name="TargetCube",
    position=np.array([0.5, 0.0, 0.05]),  # 테이블 위
    size=0.05,
    color=np.array([1.0, 0.0, 0.0]),
)

def move_to(target_pos, target_orient=None, steps=100):
    """IK로 End Effector를 목표 위치로 이동"""
    if target_orient is None:
        target_orient = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion identity
    
    ik_result = manipulator.get_ik_solver().compute_ik(
        target_position=target_pos,
        target_orientation=target_orient,
    )
    
    if ik_result is None:
        print("IK failed! Target position unreachable.")
        return False
    
    # 부드러운 이동을 위해 보간
    start_joints = robot.get_joint_positions()[:7]
    
    for t in np.linspace(0, 1, steps):
        interpolated = start_joints + (ik_result[:7] - start_joints) * t
        action = ArticulationAction(joint_positions=interpolated)
        controller.apply_action(action)
        world.step(render=True)
    
    return True

# Pick & Place 실행
# 1. 큐브 위로 이동
move_to(np.array([0.5, 0.0, 0.25]), steps=80)

# 2. 그리퍼 열기
open_gripper()

# 3. 큐브 위치로 하강
move_to(np.array([0.5, 0.0, 0.08]), steps=40)

# 4. 그리퍼 닫기 (파지)
close_gripper()

# 5. 들어 올리기
move_to(np.array([0.5, 0.0, 0.3]), steps=40)

# 6. 목적지로 이동
move_to(np.array([-0.3, 0.5, 0.3]), steps=100)

# 7. 하강
move_to(np.array([-0.3, 0.5, 0.08]), steps=40)

# 8. 그리퍼 열기 (놓기)
open_gripper()

# 9. 홈 포즈 복귀
move_to_home()
```

---

## 7. Manipulator 관련 Extension

### 7.1 사용 가능한 Extension

Isaac Sim은 Manipulator 제어를 위한 여러 Extension을 제공합니다:

| Extension | 기능 |
|-----------|------|
| **omni.isaac.manipulators** | SingleManipulator, MultiManipulator |
| **omni.isaac.manipulators.ik_solver** | IK Solver 인터페이스 |
| **omni.kit.manipulator** | GUI Manipulator 제어 |
| **omni.isaac.motion_generation** | RRT, LPA* 등 경로 계획 |

### 7.2 Motion Generation (RRT)

복잡한 환경에서는 충돌을 회피하는 경로 계획이 필요합니다.

```python
from omni.isaac.motion_generation import LulaKinematicsSolver, RRTMotionGenerator

# RRT 경로 계획기
motion_gen = RRTMotionGenerator(
    robot_description=FrankaDescription(),
    kinematics_solver=LulaKinematicsSolver(),
)

# 시작/목표 관절 설정
start_joints = robot.get_joint_positions()[:7]
goal_joints = np.array([0.5, -0.3, 0.2, -1.8, 0.1, 1.5, 0.8])

# 경로 계산
path = motion_gen.compute_path(start_joints, goal_joints)

# 경로 따라 이동
for joints in path:
    controller.apply_action(ArticulationAction(joint_positions=joints))
    world.step(render=True)
```

---

## 8. GUI Manipulator 제어

### 8.1 IK Solver Window

1. **Window > IK Solver** 열기
2. Franka 선택
3. End Effector: `panda_hand` 선택
4. 목표 위치 Gizmo를 드래그하여 Franka 손끝 이동
5. IK Solver가 실시간으로 관절 각도 계산

### 8.2 Manual Joint Control

1. Stage Window에서 Franka 선택
2. Property Panel > **Articulation** 섹션
3. 각 관절의 값을 직접 슬라이더로 조정
4. 시뮬레이션이 Play 상태여야 적용됨

---

## 9. 문제 해결 (Troubleshooting)

### 문제 1: Franka가 Play 후 비정상적으로 떨립니다.

**원인**: Joint Drive Gain 부적합
**해결**: Property Panel에서 각 관절의 Stiffness/Gain 확인

| 파라미터 | 권장값 | 설명 |
|----------|--------|------|
| Stiffness | 10000 | 위치 유지 강도 |
| Damping | 100 | 진동 억제 |
| Max Force | 1000 | 최대 힘 제한 |

### 문제 2: IK가 실패합니다 (position unreachable).

**원인**: 목표 위치가 로봇의 작업 반경(855mm)을 벗어남
**해결**: Franka 기준 0.85m 이내의 목표 위치로 설정

### 문제 3: 그리퍼가 물체를 잡지 못합니다.

**원인**: 
- 그리퍼 속도가 너무 빠름
- 충분히 닫히지 않음
- 물체와 그리퍼 사이 마찰 부족
**해결**:
- `close_gripper()`를 천천히 수행 (여러 프레임에 걸쳐)
- 그리퍼 `max_force` 증가

### 문제 4: 관절이 목표 위치에 도달하지 못합니다.

**원인**: 관절 한계(Joint Limit)에 도달
**해결**: `robot.get_joint_limits()`로 각 관절의 범위 확인

### 문제 5: 로봇이 움직일 때 바닥과 충돌합니다.

**원인**: Franka가 너무 낮은 위치에 있음
**해결**: Franka를 Z=0 (바닥)에 배치하고, 목표 위치를 충분히 높게 설정

---

## 10. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Manipulator 구조 | 7-DOF Revolute + 2-DOF Prismatic (Gripper) |
| ✅ Franka Panda 불러오기 | Reference + Robot 객체 |
| ✅ Joint Position 제어 | 각 관절의 목표 각도 설정 |
| ✅ IK 개념 | End Effector 위치 → 관절 각도 변환 |
| ✅ IK Solver | Isaac Sim 내장 IK (LMA 알고리즘) |
| ✅ 그리퍼 제어 | 열기/닫기 (Prismatic Joint) |
| ✅ Pick & Place | 이동 → 파지 → 들어올림 → 이동 → 놓음 |
| ✅ Motion Generation | RRT 기반 경로 계획 (충돌 회피) |

---

## 11. 다음 Step 예고

**Step 09 — Sensor 기초**에서는:
- Isaac Sim의 센서 종류 (Camera, LiDAR, IMU, Contact)
- Camera RGB/Depth 데이터 Python으로 수집
- LiDAR Point Cloud 데이터 수집
- Contact Sensor로 충돌 감지
- ROS2 Bridge를 통한 센서 데이터 발행

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Franka Panda 제원 | https://franka.de/research |
| Manipulator API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.manipulators.html |
| IK Solver | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulator/ik_solver.html |
| Motion Generation | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/motion_generation/index.html |
| SingleManipulator | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.manipulators.html#omni-isaac-manipulators-single-manipulator |
