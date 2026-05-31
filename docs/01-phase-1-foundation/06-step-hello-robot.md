# Step 06 — Hello, Robot! 첫 번째 로봇 불러오기

> **소요 시간**: 60분
> **난이도**: ★★☆☆☆ (초급~중급)
> **선수 조건**: Step 05 완료 (Python Scripting 기초)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim의 **내장 로봇 에셋**을 탐색한다
2. GUI의 **Create 메뉴와 Content Browser**로 로봇을 Scene에 추가한다
3. Python을 사용하여 **로봇을 프로그래밍 방식**으로 불러온다
4. **URDF 파일**을 Isaac Sim으로 가져온다 (Import)
5. 로봇의 **관절(Joint) 구조**를 Stage Window에서 확인한다
6. 불러온 로봇의 **위치(Pose)를 조정**한다
7. **Articulation** 개념을 이해하고 시뮬레이션을 실행한다

---

## 1. Isaac Sim의 내장 로봇 에셋

### 1.1 지원하는 로봇 목록

Isaac Sim 5.1은 다양한 로봇을 내장 에셋으로 제공합니다:

| 로봇 | 타입 | 설명 |
|------|------|------|
| **TurtleBot3** (Waffle / Burger) | 2륜 모바일 로봇 | ROS2 튜토리얼용, 저렴한 교육용 |
| **Franka Research 3** | 7-DOF Manipulator | 협동로봇, Pick & Place |
| **Universal Robots UR10e** | 6-DOF Manipulator | 산업용 협동로봇 |
| **Nova Carter** | 4륜 모바일 로봇 | NVIDIA 자체 개발, Isaac Perceptor |
| **JetBot** | 2륜 모바일 로봇 | NVIDIA AI 교육용 |
| **G1** / **H1** | 휴머노이드 | Unitree 로봇 (Phase 3에서 사용) |
| **Spot** | 4족 보행 로봇 | Boston Dynamics |

> **이 커리큘럼**에서는 TurtleBot3를 중심으로 시작합니다.
> Phase 3에서 Franka, UR10e, G1, H1 등으로 확장합니다.

### 1.2 로봇 에셋 위치

내장 로봇 에셋은 다음 경로에 USD 파일로 저장되어 있습니다:

```
Isaac Sim 설치 경로 (pip/env):
  <python-env>/Lib/site-packages/isaacsim/...
  또는
  <isaac-sim-root>/apps/...

Content Browser에서:
  Isaac > Robots > TurtleBot3  (또는 Franka, UR10 등)
```

---

## 2. GUI로 로봇 불러오기 (Content Browser)

### 2.1 Content Browser 사용

1. Isaac Sim 실행
2. **Content Browser** 탭 열기 (기본 우측 영역)
3. **Isaac** > **Robots** > **TurtleBot3** 선택
4. `turtlebot3_waffle.usd` 파일을 Viewport로 **드래그 앤 드롭**

**또는**:
```
Create > Isaac > Robots > TurtleBot3 > turtlebot3_waffle.usd
```

### 2.2 Stage Window 확인

드래그 앤 드롭 후 Stage Window에서 로봇의 계층 구조를 확인합니다:

```
/World
├── Ground Plane
├── PhysicsScene
└── turtlebot3_waffle        ← Reference로 불러온 로봇
    ├── base_link             ← 로봇 베이스 (Articulation Root)
    ├── base_footprint
    ├── wheel_left_link       ← 좌측 바퀴
    ├── wheel_right_link      ← 우측 바퀴
    ├── caster_back_wheel_link
    └── lidar_link            ← LiDAR 센서
        └── lidar_sensor
```

> **중요**: TurtleBot3는 기본적으로 **Articulation**으로 설정되어 있습니다.
> Articulation = 여러 Rigid Body가 Joint로 연결된 최적화된 물리 구조.

### 2.3 로봇 위치 조정

1. Stage Window에서 `turtlebot3_waffle` 선택
2. **W 키** (Move)로 로봇을 Ground Plane 위로 이동
3. **Property Panel**에서 Translate Z를 `0.1`로 설정 (바닥보다 약간 위)

---

## 3. Python으로 로봇 불러오기

### 3.1 기본 코드

```python
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# 로봇 USD 파일 경로
robot_usd_path = "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd"

# Stage에 Reference 추가
if not is_prim_path_valid("/World/TurtleBot"):
    add_reference_to_stage(
        usd_path=robot_usd_path,
        prim_path="/World/TurtleBot"
    )

# Robot 객체 생성
robot = Robot(
    prim_path="/World/TurtleBot",
    name="MyTurtleBot",
    position=np.array([0.0, 0.0, 0.1]),
)
```

### 3.2 Robot API 주요 기능

```python
# 관절 이름 목록
joint_names = robot.dof_names
print(f"Joints: {joint_names}")

# 관절 위치 읽기
joint_positions = robot.get_joint_positions()
print(f"Joint positions: {joint_positions}")

# 관절 위치 설정
robot.set_joint_positions(np.array([0.5, -0.3]))

# 베이스 위치
world_pose = robot.get_world_pose()
print(f"Robot position: {world_pose[0]}")
print(f"Robot orientation: {world_pose[1]}")
```

---

## 4. URDF 파일 가져오기

### 4.1 URDF란?

**URDF (Unified Robot Description Format)**는 ROS에서 사용하는 XML 기반 로봇 모델 포맷입니다.

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### 4.2 GUI로 URDF 가져오기

1. **Utilities > URDF Importer** 실행
2. URDF 파일 선택
3. Import 옵션 설정:
   - **Fix Base**: 로봇 베이스를 고정할지 (모바일 로봇은 해제)
   - **Articulation**: Articulation으로 변환 (권장)
   - **Make default prim**: 자동 선택
4. **Import** 버튼 클릭

### 4.3 Python으로 URDF 가져오기

```python
from omni.isaac.examples.urdf_importer import import_urdf

# URDF 파일 경로
urdf_path = "/path/to/my_robot.urdf"

# USD 출력 경로
output_path = "/path/to/my_robot.usd"

# Import 실행
robot_prim_path = import_urdf(
    urdf_path=urdf_path,
    output_path=output_path,
    import_to_stage=True,
    make_default_prim=False,
    fix_base=False,          # 모바일 로봇이므로 베이스 고정 안 함
    joint_limit_offset=0.0,  # 관절 제한 오프셋
)

print(f"Robot imported to: {robot_prim_path}")
```

### 4.4 URDF Import 시 주의사항

| 문제 | 해결 |
|------|------|
| 중력 방향이 Y축 | URDF는 Y-Up, Isaac Sim은 Z-Up → 자동 변환 |
| 텍스처 누락 | URDF의 mesh 파일 경로를 절대 경로로 변경 |
| 관절이 너무 뻣뻣함 | Import 후 Joint Drive Gain 조정 |
| Base가 공중에 떠 있음 | Import 후 Translate Z 조정 |

---

## 5. Articulation 이해

### 5.1 Articulation이란?

**Articulation**은 여러 Rigid Body가 **Joint**로 연결된 구조를 하나의 최적화된 시스템으로 시뮬레이션하는 PhysX 기능입니다.

```
Articulation Root (base_link)
├── Revolute Joint → link1
│   └── Revolute Joint → link2
│       └── Revolute Joint → link3 (end effector)
└── Continuous Joint → wheel_left
└── Continuous Joint → wheel_right
```

**Articulation의 장점**:
- 개별 Rigid Body보다 **성능 우수** (내부 제약 조건 최적화)
- **관절 피드백**(힘/토크) 정확
- **역운동학(IK)** 지원
- ROS2 **Joint State** 인터페이스와 호환

### 5.2 Articulation 확인

Stage Window에서 로봇 Prim을 선택하고 **Property Panel**에서 확인:

```
Physics:
  Articulation Root: ✅ (초록색 체크)
    └─ base_link이 Articulation Root로 설정됨
  
  Joint:
    └─ 각 링크가 Revolute/Continuous/Fixed Joint로 연결
```

### 5.3 Robot vs Articulation

| 클래스 | 설명 | 사용처 |
|--------|------|--------|
| `Robot` | 고수준 API (Articulation을 래핑) | 일반적인 로봇 제어 |
| `Articulation` | 저수준 API (직접 Joint 제어) | 세밀한 관절 제어 필요 시 |
| `ArticulationView` | 여러 Articulation을 묶어서 관리 | Isaac Lab, RL 학습 |

---

## 6. 실습: TurtleBot3 불러오고 시뮬레이션

### 6.1 GUI 실습

1. **Create > Isaac > Robots > TurtleBot3 > turtlebot3_waffle** 선택
2. Stage Window에서 계층 구조 확인
3. Property Panel에서 Joint 설정 확인
   - `left_wheel_joint`: Continuous 타입 (무한 회전)
   - `right_wheel_joint`: Continuous 타입
4. **Play (▶)** 클릭 → 로봇이 중력에 의해 Ground Plane에 안착
5. **Stop (■)**

### 6.2 Transform 조정

로봇의 Transform을 Property Panel에서 직접 조정:

```
turtlebot3_waffle:
  Translate: X=0, Y=0, Z=0.05  (바닥에서 약간 위)
  Rotate:    X=0, Y=0, Z=0
  Scale:     X=1, Y=1, Z=1
```

### 6.3 저장

```
File > Save As... > turtlebot_scene.usd
```

---

## 7. 여러 로봇 Scene에 배치

### 7.1 여러 로봇 추가

같은 USD 파일을 여러 번 Reference하여 여러 로봇을 배치할 수 있습니다.

**GUI 방식**:
1. Content Browser에서 turtlebot3_waffle.usd를 Viewport로 드래그
2. 위치 이동 (첫 번째 로봇)
3. 다시 드래그 (두 번째 로봇, 자동으로 다른 이름 할당)
4. 두 번째 로봇 위치 이동

**Python 방식**:
```python
robots = []
for i, pos in enumerate([(0, 0, 0.1), (2, 0, 0.1), (-1, 1.5, 0.1)]):
    path = f"/World/Robot_{i}"
    add_reference_to_stage(robot_usd_path, path)
    robot = Robot(prim_path=path, name=f"Robot{i}", position=np.array(pos))
    robots.append(robot)
```

### 7.2 한계

모든 TurtleBot3이 동일한 USD 파일을 Reference하므로:
- **개별 색상 변경 불가** (같은 Material 공유)
- 각 로봇의 **물리 상태는 독립적** (각자 Articulation)

---

## 8. 로봇 에셋 파일 구조

### 8.1 USD 로봇 패키지 구조

내장 로봇 USD의 일반적인 구조:

```
<로봇.usd>                 ← 메인 USD (Articulation Root 설정)
├── <로봇>.usd             ← Geometry/Visual Mesh
├── <로봇>_collision.usd   ← Collision Mesh (선택 사항)
├── textures/              ← 텍스처 이미지
│   ├── base_color.png
│   └── metallic.png
└── materials/             ← Material 정의
    └── robot.mdl
```

### 8.2 TurtleBot3 예시

```
turtlebot3_waffle.usd
├── meshes/
│   ├── base_link.stl
│   ├── wheel_left.stl
│   └── ...
└── turtlebot3_waffle.usd (self)
```

---

## 9. 로봇 관절 구조 분석

### 9.1 TurtleBot3 관절

| 관절 이름 | 타입 | 부모 링크 | 자식 링크 | 설명 |
|-----------|------|-----------|-----------|------|
| `base_link_to_base_footprint` | Fixed | base_footprint | base_link | 고정 연결 |
| `left_wheel_joint` | Continuous | base_link | wheel_left_link | 무한 회전 (바퀴) |
| `right_wheel_joint` | Continuous | base_link | wheel_right_link | 무한 회전 (바퀴) |
| `caster_back_wheel_joint` | Fixed | base_link | caster_back_wheel_link | 고정 캐스터 |

### 9.2 Franka Panda 관절

| 관절 이름 | 타입 | 설명 |
|-----------|------|------|
| `panda_joint1` | Revolute | 베이스 회전 |
| `panda_joint2` | Revolute | 숄더 |
| `panda_joint3` | Revolute | 엘보 |
| `panda_joint4` | Revolute | 전완 회전 |
| `panda_joint5` | Revolute | 손목 |
| `panda_joint6` | Revolute | 손목 회전 |
| `panda_joint7` | Revolute | 말단 회전 |
| `panda_finger_joint1` | Prismatic | 그리퍼 (좌) |
| `panda_finger_joint2` | Prismatic | 그리퍼 (우) |

### 9.3 관절 타입

| 타입 | 설명 | 자유도 | 예시 |
|------|------|--------|------|
| **Revolute** | 회전 (제한 있음) | 1 | 팔꿈치, 무릎 |
| **Continuous** | 무한 회전 | 1 | 바퀴 |
| **Prismatic** | 직선 이동 | 1 | 그리퍼 |
| **Fixed** | 고정 (움직임 없음) | 0 | 구조적 연결 |
| **Spherical** | 3축 회전 | 3 | 볼 조인트 |
| **D6** | 6자유도 | 6 | 완전 자유 |

---

## 10. 문제 해결 (Troubleshooting)

### 문제 1: Content Browser에 로봇이 보이지 않습니다.

**해결**:
1. **Window > Content Browser**가 열려 있는지 확인
2. Isaac 탭이 아닌 **Isaac** 폴더가 있는지 확인
3. Isaac Sim 재시작 또는 Content Browser 새로고침 (F5)

### 문제 2: 로봇을 Viewport에 드래그했는데 아무것도 안 보입니다.

**원인**: 로봇이 Ground Plane 아래에 있거나 너무 먼 거리에 있음
**해결**:
1. **F 키**로 로봇에 포커스
2. Property Panel에서 Translate Z를 0.1~1.0 사이로 설정
3. Viewport를 확대/축소하여 확인

### 문제 3: URDF Import 후 관절이 비정상적입니다.

**원인**: URDF의 joint limit, axis 설정이 Isaac Sim과 호환되지 않음
**해결**:
1. Import 후 Property Panel에서 각 Joint의 Limit 확인
2. 필요시 **USD Edit**으로 직접 수정
3. URDF 원본의 joint limit 단위 확인 (라디안 vs 도)

### 문제 4: 로봇이 Play 직후 바닥을 뚫고 떨어집니다.

**원인**: 
- Ground Plane이 없음
- 또는 로봇이 너무 높은 위치에 있어 충돌 속도가 너무 빠름
**해결**:
- **Create > Physics > Ground Plane** 추가
- 로봇 Translate Z를 0.05~0.1로 낮춤

### 문제 5: 로봇 바퀴가 Play 후에도 회전하지 않습니다.

**원인**: 바퀴에 Joint Drive가 설정되지 않음
**해결**: (다음 Step에서 다룰 내용)
- Articulation Controller로 바퀴 속도 명령 필요
- Step 07에서 학습

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 내장 로봇 에셋 | TurtleBot3, Franka, UR10e 등 10+ 로봇 |
| ✅ GUI로 로봇 배치 | Content Browser → 드래그 앤 드롭 |
| ✅ Python으로 로봇 배치 | add_reference_to_stage + Robot() |
| ✅ URDF Import | Utilities > URDF Importer |
| ✅ Articulation 개념 | Joint로 연결된 최적화된 물리 구조 |
| ✅ 관절 타입 | Revolute, Continuous, Prismatic, Fixed |
| ✅ Robot API | get_joint_positions(), set_joint_positions() |
| ✅ Stage 계층 | 로봇의 링크/관절 구조 파악 |

---

## 12. 다음 Step 예고

**Step 07 — TurtleBot3 제어하기 (Controller 기초)**에서는:
- Articulation Controller로 바퀴 속도 명령
- Differential Drive 모델 이해 (wheelDistance, wheelRadius)
- Python으로 TurtleBot3 직진/회전 제어
- Odometry 계산 (위치 추정)
- Keyboard Teleop 연결

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Robot USD Assets | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/robots.html |
| URDF Importer | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/urdf_importer.html |
| Articulation 개요 | https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/articulations.html |
| TurtleBot3 공식 | https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ |
| Robot API 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.core.html#omni-isaac-core-robots |
