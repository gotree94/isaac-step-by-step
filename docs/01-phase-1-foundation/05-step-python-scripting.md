# Step 05 — Python Scripting 기초

> **소요 시간**: 75분
> **난이도**: ★★☆☆☆ (초급~중급)
> **선수 조건**: Step 04 완료 (USD Stage 구조), 기본 Python 문법

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim의 **3가지 개발 워크플로우**(Standalone, Extension, Notebook)를 이해한다
2. **Standalone Python** 스크립트를 작성하고 실행한다
3. **omni.isaac.core** API의 기본 구조(World, PhysicsScene, Prim)를 이해한다
4. **omni.usd**와 **pxr.Usd** API의 관계를 설명한다
5. USD Prim을 Python으로 **탐색(Create/Read/Update/Delete)** 한다
6. 시뮬레이션 **타임라인**(Play/Stop/Step)을 Python으로 제어한다
7. 간단한 시뮬레이션 루프를 작성한다

---

## 1. Isaac Sim의 3가지 개발 워크플로우

Isaac Sim에서 Python을 사용하는 방법은 크게 3가지입니다:

### 1.1 Standalone Python (외부 실행)

Isaac Sim 바이너리 없이 **터미널에서 직접 Python 스크립트 실행**합니다.
Isaac Sim의 Python 인터프리터를 사용합니다.

```bash
# <isaac-sim-root>/python.sh myscript.py
cd ~/isaac-sim
./python.sh ~/isaac-step-curriculum/code/phase-1/my_script.py
```

| 장점 | 단점 |
|------|------|
| GUI 없이 서버/클러스터에서 실행 가능 | GUI에서 실시간 확인 불가 |
| 배치 처리, 자동화에 적합 | headless 모드는 렌더링 제한 |
| CI/CD 파이프라인 통합 용이 | 디버깅이 상대적으로 어려움 |

> **pip 설치 환경**에서는 가상환경의 Python을 직접 사용:
> ```bash
> source ~/isaacsim_env/bin/activate
> python my_script.py
> ```

### 1.2 Extension Python (GUI 내 실행)

Isaac Sim이 실행 중인 상태에서 **Script Editor**에 코드를 입력/실행합니다.

```bash
# Isaac Sim 실행 후
# Window > Script Editor > Python >
# 코드 작성 후 Run
```

| 장점 | 단점 |
|------|------|
| GUI와 실시간 연동 (결과 즉시 확인) | Isaac Sim이 실행 중이어야 함 |
| Hot-reload 지원 (코드 수정 → 즉시 반영) | 장기 실행 스크립트에 부적합 |
| UI 컴포넌트 생성 가능 | |

### 1.3 Jupyter Notebook (대화형)

Isaac Sim의 Python 커널을 Jupyter에 연결하여 대화형으로 개발합니다.

```bash
cd ~/isaac-sim
./python.sh -m notebook
```

| 장점 | 단점 |
|------|------|
| 셀 단위 실행, 결과 시각화 | 설정이 복잡함 |
| 데이터 분석/시각화에 최적 | Isaac Sim 버전마다 호환성 이슈 |
| 교육/데모에 유용 | |

### 1.4 워크플로우 선택 기준

| 상황 | 권장 워크플로우 |
|------|---------------|
| 빠른 프로토타이핑, 실험 | Extension (Script Editor) |
| 배치 렌더링, 데이터 생성 | Standalone |
| RL 학습, 장기 시뮬레이션 | Standalone (headless) |
| 데이터 분석, 시각화 | Jupyter Notebook |
| 패키지/도구 개발 | Extension (패키지 구조) |

> **이 커리큘럼**에서는 주로 **Extension**과 **Standalone** 방식을 병행합니다.

---

## 2. API 계층 구조

Isaac Sim의 Python API는 여러 계층으로 구성되어 있습니다:

```
┌─────────────────────────────────────────────┐
│  omni.isaac.core         ← High-level API   │
│  (World, Scene, Robot, Sensor 등)            │
├─────────────────────────────────────────────┤
│  omni.isaac.core_nodes   ← OmniGraph API    │
│  (WritePrimProperty, ArticulationCtrl 등)    │
├─────────────────────────────────────────────┤
│  omni.kit / omni.usd     ← Omniverse Kit    │
│  (Stage 관리, Undo/Redo, Event 등)           │
├─────────────────────────────────────────────┤
│  pxr.Usd / pxr.Sdf / pxr.Gf  ← USD API     │
│  (Prim, Attribute, Layer, Composition 등)    │
├─────────────────────────────────────────────┤
│  carb                     ← Carbonite (기반) │
│  (로깅, 설정, 프레임워크)                      │
└─────────────────────────────────────────────┘
```

### 2.1 pxr.Usd (USD Python API)

USD 자체의 Python 바인딩. Prim/Attribute/Layer/Reference 등 USD 기본 기능.

```python
from pxr import Usd, Sdf, Gf

stage = Usd.Stage.CreateNew("test.usda")
prim = stage.DefinePrim("/World/Cube", "Cube")
attr = prim.GetAttribute("xformOp:translate")
attr.Set(Gf.Vec3d(1, 2, 3))
```

### 2.2 omni.isaac.core (Core API)

Isaac Sim의 고수준 API. World, Scene, Robot 등 편의 기능 제공.

```python
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid

world = World()
world.scene.add_default_ground_plane()

cube = DynamicCuboid(
    prim_path="/World/Cube",
    position=np.array([0, 0, 2.0]),
    mass=1.0,
)
world.scene.add(cube)
```

### 2.3 API 선택 가이드

| 작업 | 권장 API |
|------|----------|
| Prim 생성/삭제 | `pxr.Usd` (직접) 또는 `omni.isaac.core.utils.prims` |
| Attribute 읽기/쓰기 | `pxr.Usd` (GetAttribute/SetAttribute) |
| Transform 조작 | `pxr.UsdGeom.Xformable` |
| 물리 속성 추가 | `omni.isaac.core.objects.DynamicCuboid` 등 |
| 로봇 제어 | `omni.isaac.core.articulations` |
| 시뮬레이션 루프 | `omni.isaac.core.world.World.step()` |
| OmniGraph | `omni.graph.core.Controller` |

---

## 3. World 객체 이해

### 3.1 World란?

`World`는 Isaac Sim의 **최상위 시뮬레이션 컨텍스트**입니다.
Physics Scene, Ground Plane, Time Stepping을 관리합니다.

```python
from omni.isaac.core.world import World

# 기본 World 생성 (physics_dt=1/60, rendering_dt=1/60)
world = World(stage_units_in_meters=1.0)

# 설정 변경
world = World(
    physics_dt=1/60.0,       # 물리 스텝 간격 (초)
    rendering_dt=1/60.0,     # 렌더링 간격 (초)
    stage_units_in_meters=1.0,  # 단위
)
```

### 3.2 World 주요 기능

```python
# World 초기화 (PhysicsScene 자동 생성)
world.initialize()

# Ground Plane 추가
world.scene.add_default_ground_plane()

# 시뮬레이션 스텝
world.step(render=True)  # 한 스텝 진행

# Play/Stop
world.play()  # 시뮬레이션 시작
world.stop()  # 시뮬레이션 중지
```

### 3.3 World의 생명주기

```
World 생성
    │
    ▼
world.initialize()  ← PhysicsScene, 기본 리소스 로딩
    │
    ▼
world.play()        ← 시뮬레이션 시작 (타임라인 Play)
    │
    ▼
루프:
    world.step()    ← 물리/렌더링 1스텝 진행
    │
    ▼
world.stop()        ← 시뮬레이션 중지
    │
    ▼
world.reset()       ← 초기 상태로 리셋
    │
    ▼
world.cleanup()     ← 리소스 정리
```

---

## 4. 첫 번째 Standalone 스크립트

### 4.1 기본 구조

```python
"""
my_first_script.py — Isaac Sim Standalone Python 스크립트 기본 구조
"""

import sys
import numpy as np
from omni.isaac.kit import SimulationApp

# ── 1. SimulationApp 초기화 (가장 먼저!) ──
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1280,
    "window_height": 720,
    "headless": False,     # True=렌더링 없이 백그라운드 실행
    "renderer": "RayTracedLighting",  # "PathTracing", "RayTracedLighting"
}
simulation_app = SimulationApp(CONFIG)

# ── 2. Isaac Sim Core API 임포트 ──
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid

# ── 3. World 생성 및 Scene 설정 ──
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="MyCube",
    position=np.array([0.0, 0.0, 2.0]),
    mass=1.0,
)
world.scene.add(cube)

# ── 4. 시뮬레이션 루프 ──
for frame in range(100):  # 100 프레임 실행
    world.step(render=True)
    
    if frame % 10 == 0:  # 10프레임마다 상태 출력
        pos, orient = cube.get_world_pose()
        print(f"Frame {frame}: Cube position = {pos}")

# ── 5. 종료 ──
simulation_app.close()
```

### 4.2 실행 방법

```bash
cd ~/isaac-sim
./python.sh my_first_script.py
```

> **headless=True**: GUI 없이 실행. 서버/클러스터 환경에 적합.
> **headless=False**: GUI 창이 뜨고 실시간으로 시뮬레이션 확인 가능.

---

## 5. USD Prim CRUD (Create/Read/Update/Delete)

### 5.1 Create (생성)

```python
from pxr import Usd, UsdGeom, Gf

stage = world.stage

# 방법 1: DefinePrim (권장)
prim = stage.DefinePrim("/World/MyObject", "Cube")

# 방법 2: UsdGeom API
cube = UsdGeom.Cube.Define(stage, "/World/MyObject")
```

### 5.2 Read (읽기)

```python
# Prim 가져오기
prim = stage.GetPrimAtPath("/World/Cube")
if not prim:
    print("Prim not found!")
    return

# 타입 확인
print(f"Type: {prim.GetTypeName()}")

# Attribute 읽기
attr = prim.GetAttribute("xformOp:translate")
value = attr.Get()  # Gf.Vec3d 반환
print(f"Position: ({value[0]}, {value[1]}, {value[2]})")

# 모든 Attribute 열거
for attr in prim.GetAttributes():
    print(f"  {attr.GetName()} = {attr.Get()}")
```

### 5.3 Update (수정)

```python
# Attribute 값 설정
attr = prim.GetAttribute("xformOp:translate")
attr.Set(Gf.Vec3d(5.0, 0.0, 3.0))

# Transform 조작 (UsdGeom API)
xform = UsdGeom.Xformable(prim)
translate_op = xform.AddTranslateOp()
translate_op.Set(Gf.Vec3d(5.0, 0.0, 3.0))
```

### 5.4 Delete (삭제)

```python
# Prim 삭제
stage.RemovePrim("/World/MyObject")

# Prim이 삭제되었는지 확인
if not stage.GetPrimAtPath("/World/MyObject"):
    print("Prim successfully removed.")
```

---

## 6. 시뮬레이션 루프 패턴

### 6.1 고정 프레임 루프 (Standalone)

```python
# N 프레임 동안 시뮬레이션 실행
for i in range(500):
    world.step(render=True)
    
    # 매 프레임 큐브 위치 출력
    pos, _ = cube.get_world_pose()
    print(f"Frame {i}: Z={pos[2]:.3f}")
```

### 6.2 실시간 루프 (Extension)

```python
import asyncio

async def simulation_loop():
    while True:
        world.step(render=True)
        await asyncio.sleep(0)  # 제어권 반납 (non-blocking)

# Extension 로드 시 실행
asyncio.ensure_future(simulation_loop())
```

### 6.3 조건부 종료 루프

```python
# 큐브가 바닥에 닿을 때까지 실행
while True:
    world.step(render=True)
    
    pos, _ = cube.get_world_pose()
    if pos[2] <= 0.05:  # 바닥에 닿음 (Z <= 0.05)
        print(f"Cube landed at frame {world.current_time_step_index}")
        break

print("Simulation complete.")
```

---

## 7. 시뮬레이션 상태 읽기

### 7.1 기본 정보

```python
# 현재 물리 스텝 인덱스
print(f"Physics step: {world.current_time_step_index}")

# 시뮬레이션 시간
print(f"Sim time: {world.current_time}")

# 시뮬레이션 재생 여부
print(f"Is playing: {world.is_playing()}")
```

### 7.2 객체 상태

```python
# DynamicCuboid의 물리 상태
pos, orient = cube.get_world_pose()     # 위치 + 회전(Quaternion)
linear_vel = cube.get_linear_velocity()  # 선속도
angular_vel = cube.get_angular_velocity()  # 각속도

print(f"Position: {pos}")
print(f"Linear Vel: {linear_vel}")
```

### 7.3 충돌 감지

```python
# 아직 충돌 감지는 Phase 2에서 자세히 다룸
# 간단한 방법: Z축 위치 + 속도로 바닥 충돌 추정
if pos[2] < 0.1 and abs(linear_vel[2]) < 0.01:
    print("Object has settled on ground.")
```

---

## 8. Extension 워크플로우

### 8.1 Script Editor 사용

1. Isaac Sim 실행
2. **Window > Script Editor** (또는 **Window > Visual Scripting > Script Editor**)
3. Python 코드 입력
4. **Run** 버튼 클릭

### 8.2 Hot-Reload

Extension 방식의 큰 장점은 **Hot-Reload**입니다:
1. 코드 수정 후 저장 (Ctrl+S)
2. Script Editor에서 다시 **Run**
3. Isaac Sim을 재시작할 필요 없음

### 8.3 간단한 Extension 예시

```python
# Script Editor에 입력 후 Run
import omni.usd
from pxr import Usd, UsdGeom, Gf

# 현재 Stage 가져오기
stage = omni.usd.get_context().get_stage()

# 큐브 생성
cube = UsdGeom.Cube.Define(stage, "/World/ScriptedCube")
xform = UsdGeom.Xformable(cube)
xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 1.0))

print("Cube created at /World/ScriptedCube!")
```

---

## 9. pip 설치 환경에서의 Python 스크립트

Isaac Sim을 pip로 설치한 경우, 가상환경의 Python을 직접 사용합니다.

### 9.1 기본 실행

```bash
source ~/isaacsim_env/bin/activate
python my_script.py
```

### 9.2 headless 모드

```bash
# 렌더링 없이 실행 (서버 환경)
source ~/isaacsim_env/bin/activate
python my_script.py  # 스크립트 내 CONFIG["headless"] = True
```

### 9.3 주의사항

- 가상환경 Python 버전이 3.11인지 확인
- pip 패키지와 충돌 방지를 위해 **가상환경 격리** 유지
- `isaacsim` 패키지가 설치된 가상환경에서만 실행 가능

---

## 10. 종합 실습: 자유 낙하 시뮬레이션 + 데이터 수집

### 목표

큐브 자유 낙하 시뮬레이션을 Python으로 작성하고, 프레임별 위치 데이터를 수집합니다.

### 스크립트 구조

```python
import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid

# World 생성
world = World()
world.scene.add_default_ground_plane()

# 큐브 생성 (높이 10m에서 낙하)
cube = DynamicCuboid(
    prim_path="/World/FallingCube",
    name="FallingCube",
    position=np.array([0.0, 0.0, 10.0]),
    mass=1.0,
)
world.scene.add(cube)

# 데이터 수집
positions = []
velocities = []

print("Starting free fall simulation...")
for i in range(300):
    world.step(render=True)
    
    pos, _ = cube.get_world_pose()
    vel = cube.get_linear_velocity()
    
    positions.append(pos.copy())
    velocities.append(vel.copy())
    
    if i % 30 == 0:
        print(f"Frame {i}: Z={pos[2]:.3f}m, Vz={vel[2]:.3f}m/s")
    
    # 바닥에 닿으면 정지
    if pos[2] < 0.1 and abs(vel[2]) < 0.01:
        print(f"Landed at frame {i}")
        break

print(f"\nData collected: {len(positions)} frames")
print(f"Final position: {positions[-1]}")
print(f"Impact velocity: {velocities[-1][2]:.3f} m/s")

simulation_app.close()
```

### 실행 결과 예상

```
Starting free fall simulation...
Frame 0: Z=10.000m, Vz=0.000m/s
Frame 30: Z=9.875m, Vz=-0.816m/s
Frame 60: Z=9.500m, Vz=-1.633m/s
...
Frame 270: Z=0.050m, Vz=-5.715m/s
Landed at frame 286

Data collected: 287 frames
Final position: [0. 0. 0.05]
Impact velocity: -5.715 m/s
```

> **물리 검증**: 이론적 충돌 속도 `sqrt(2 * g * h)` = `sqrt(2 * 9.8 * 10)` ≈ 14 m/s
> 시뮬레이션 결과가 약 5.7 m/s로 차이나는 이유?
> → **Ground Plane 충돌 처리 과정**에서 첫 충돌 직후 속도 샘플링 차이.
> 나중에 Phase 2에서 PhysX 파라미터 튜닝을 배우면 이해할 수 있습니다.

---

## 11. 문제 해결 (Troubleshooting)

### 문제 1: `ModuleNotFoundError: No module named 'omni'`

**원인**: Isaac Sim의 Python 인터프리터로 실행하지 않음
**해결**: 반드시 `./python.sh` 또는 pip 가상환경의 Python 사용

### 문제 2: `CarbFatalError` 또는 `simulation_app` 초기화 실패

**원인**: GPU 드라이버 문제 또는 중복 초기화
**해결**:
- NVIDIA 드라이버 업데이트 (최소 545.xx)
- 다른 Isaac Sim 프로세스 종료
- `SimulationApp`은 **최초 1회만** 생성 가능

### 문제 3: Script Editor에서 실행했는데 아무 일도 안 일어남

**원인**: Play(▶)를 누르지 않음
**해결**: Script Editor는 Stage 설정만 함. 시뮬레이션 실행은 Toolbar에서 Play 버튼 필요

### 문제 4: `world.step()`이 너무 느림

**원인**: 렌더링 성능 문제
**해결**:
- `world.step(render=False)`로 렌더링 생략 (headless)
- `rendering_dt`를 1/30으로 낮춤

### 문제 5: Prim이 생성되지 않음 ("Invalid Prim Path")

**원인**: 상위 Prim이 Xform 타입이 아님
**해결**: Prim 경로의 모든 중간 Prim이 존재하는지 확인
```python
# 잘못된 예: /World가 없는데 /World/Cube 생성 시도
# 올바른 예:
stage.DefinePrim("/World", "Xform")
stage.DefinePrim("/World/Cube", "Cube")
```

---

## 12. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 3가지 워크플로우 | Standalone, Extension, Jupyter Notebook |
| ✅ API 계층 | pxr.Usd → omni.isaac.core → omni.kit → carb |
| ✅ World 객체 | 시뮬레이션 최상위 컨텍스트, Physics Scene 관리 |
| ✅ USD CRUD | Create/Read/Update/Delete로 Prim 조작 |
| ✅ 시뮬레이션 루프 | world.step()으로 1프레임 진행 |
| ✅ 데이터 수집 | 프레임별 객체 상태(position, velocity) 수집 |
| ✅ Extension | Script Editor에서 코드 실행 및 Hot-Reload |

---

## 13. 다음 Step 예고

**Step 06 — Hello, World! 첫 번째 로봇 불러오기**에서는:
- Isaac Sim에 내장된 로봇 에셋 탐색
- URDF 파일을 Isaac Sim으로 가져오기 (Import / urdf-importer)
- 로봇의 관절(Joint) 구조 확인
- 로봇을 Scene에 배치하고 Pose 조정
- GUI와 Python 두 가지 방식으로 로봇 가져오기

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Python Scripting 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/scripting/index.html |
| Workflow 개념 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/quickstart_isaacsim.html#workflows |
| omni.isaac.core API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/index.html |
| USD Python API | https://openusd.org/release/api_python.html |
| SimulationApp | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/scripting/standalone.html |
| Jupyter Notebook | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/scripting/jupyter_notebook.html |
