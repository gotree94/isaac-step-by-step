# Step 03 — OmniGraph 기초

> **소요 시간**: 60분
> **난이도**: ★★☆☆☆ (초급~중급)
> **선수 조건**: Step 02 완료 (GUI 기본 사용법)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **OmniGraph**가 무엇이고 왜 사용하는지 설명한다
2. **Action Graph**의 기본 구조(Event → Node → Data Flow)를 이해한다
3. 그래프 편집기에서 **노드 추가 및 연결**을 할 수 있다
4. `On Playback Tick` + `Write Prim Property` 로 큐브를 실시간 회전시킨다
5. `On Impulse Event`로 단발성 동작(점프 등)을 트리거한다
6. **Keyboard Input** 노드로 키보드로 큐브를 제어한다
7. 그래프의 **Pipeline Stage**(Execution, On-Demand) 개념을 이해한다
8. USD에 포함된 그래프를 **저장/불러오기**한다

---

## 1. OmniGraph 개념

### 1.1 OmniGraph란?

**OmniGraph**는 NVIDIA Omniverse의 **비주얼 프로그래밍 프레임워크**입니다.
쉽게 말해 **코드 없이 노드를 연결하여 로직을 만드는 도구**입니다.

> **비유**: Unreal Engine의 Blueprint, Blender의 Geometry Nodes, Unity의 Visual Scripting과 유사합니다.

Isaac Sim에서 OmniGraph가 사용되는 곳:

| 용도 | 설명 |
|------|------|
| **로봇 제어** | Articulation Controller로 로봇 관절 명령 |
| **ROS2 Bridge** | ROS2 토픽 Publish/Subscribe |
| **센서** | 카메라, LiDAR 데이터 스트리밍 |
| **Replicator** | 데이터 생성 파이프라인 |
| **입력 장치** | 키보드/마우스/조이스틱 제어 |
| **커스텀 로직** | OnPlaybackTick + WriteProperty로 객체 애니메이션 |

### 1.2 Action Graph vs State Graph

OmniGraph에는 두 가지 주요 그래프 타입이 있습니다:

| 타입 | 설명 | 사용처 |
|------|------|--------|
| **Action Graph** | 이벤트 기반 실행. 트리거가 발생할 때 노드 실행 | 시뮬레이션 로직, ROS2, 센서 |
| **State Graph** | State 머신 기반. 상태 전이로 로직 구성 | 복잡한 의사결정, AI 행동 트리 |

> **이 과정에서는 Action Graph에 집중합니다.**

### 1.3 그래프 기본 구조

모든 Action Graph는 다음 3가지 요소로 구성됩니다:

```
[1. Event Node]  →  [2. Logic/Data Nodes]  →  [3. Action Node]
    (트리거)           (처리/변환)               (실행)
```

**실제 예: 큐브를 매 프레임마다 회전**

```
On Playback Tick → Write Prim Property → /World/Cube의 rotation 변경
   (매 프레임)        (속성 쓰기)           (결과)
```

---

## 2. Action Graph 편집기 사용법

### 2.1 Action Graph 열기

```bash
Window > Graph Editors > Action Graph
```

1. 상단 메뉴에서 **Window** 클릭
2. **Graph Editors** 선택
3. **Action Graph** 클릭

기본적으로 Content Browser 영역에 그래프 편집기 탭이 추가됩니다.

### 2.2 새 Action Graph 생성

Action Graph 창 가운데 **"New Action Graph"** 버튼을 클릭합니다.

그러면 다음이 자동 생성됩니다:
- Stage Window에 `/ActionGraph` 경로 추가
- 빈 그래프 편집기 열림
- **Toolbar**에 그래프 관련 버튼 표시

### 2.3 그래프 편집기 조작

| 조작 | 동작 |
|------|------|
| **가운데 버튼 드래그** | 캔버스 이동 (Pan) |
| **휠 스크롤** | 확대/축소 (Zoom) |
| **노드 선택 + Delete** | 노드 삭제 |
| **핀(Pin) → 드래그** | 연결선 생성 |
| **연결선 선택 + Delete** | 연결 제거 |

### 2.4 노드 추가

1. 그래프 편집기 빈 공간에서 **더블클릭** 또는 **Tab** 키
2. 검색창에 노드 이름 입력 (예: `Write Prim Property`)
3. 결과에서 노드 선택하여 추가

또는 상단 검색 바에 노드명을 입력하고 드래그할 수도 있습니다.

---

## 3. 첫 번째 Action Graph: 큐브 회전시키기

On Playback Tick + Write Prim Property를 사용하여 큐브를 매 프레임 회전시킵니다.

### 3.1 Stage 준비

1. **File > New**로 새 Scene
2. **Create > Physics > Ground Plane** 추가
3. **Create > Shape > Cube** 추가
4. Viewport에서 큐브가 보이는지 확인

### 3.2 Action Graph 생성

1. **Window > Graph Editors > Action Graph**
2. **New Action Graph** 클릭

### 3.3 노드 추가 (2개)

**노드 1: On Playback Tick**
- 그래프 편집기에서 **Tab** (또는 더블클릭)
- 검색어: `On Playback Tick`
- `omni.graph.action.OnPlaybackTick` 선택하여 추가

> **On Playback Tick**이 하는 일:
> 시뮬레이션이 Play 중일 때 **매 프레임마다** Tick 신호(Execution Pulse)를 출력합니다.
> 시뮬레이션이 멈추면(Stop) 출력이 중단됩니다.

**노드 2: Write Prim Property**
- **Tab** → 검색어: `Write Prim Property`
- `omni.isaac.core_nodes.IsaacWritePrimProperty` 선택하여 추가

> **Write Prim Property**가 하는 일:
> 입력받은 Prim 경로의 속성값을 설정합니다.
> 예: `/World/Cube`의 `xformOp:rotateZ` 값을 매 프레임 변경

### 3.4 노드 연결

On Playback Tick의 **Tick (execution)** 출력 핀을
Write Prim Property의 **Exec In (execution)** 입력 핀으로 드래그 연결합니다.

```
On Playback Tick                   Write Prim Property
┌─────────────────┐                ┌─────────────────────────┐
│  outputs:        │                │  inputs:                │
│  · tick (exec) ──┼────────────────┼→· execIn (exec)         │
│  · step (int)    │                │  · targetPrim (token)   │
│  · time (double) │                │  · propertyName (token) │
│  · delta (double)│                │  · value (double)       │
└─────────────────┘                └─────────────────────────┘
```

### 3.5 노드 속성 설정

각 노드를 선택하고 **Property Panel**에서 속성을 설정합니다.

**Write Prim Property 속성:**

| 속성 | 값 | 설명 |
|------|------|------|
| **targetPrim** | `/World/Cube` | 변경할 Prim 경로 |
| **propertyName** | `xformOp:rotateZ` | 변경할 속성명 (회전 Z) |
| **value** | `1.0` (초기값) | 매 틱마다 더해질 각도 |

> **propertyName의 다양한 값**:
> - `xformOp:translateX`, `xformOp:translateY`, `xformOp:translateZ` — 위치
> - `xformOp:rotateX`, `xformOp:rotateY`, `xformOp:rotateZ` — 회전
> - `xformOp:scaleX`, `xformOp:scaleY`, `xformOp:scaleZ` — 크기

### 3.6 시뮬레이션 실행

1. **Play (▶)** 버튼 클릭
2. 큐브가 Z축 기준으로 계속 회전하는 모습 확인
3. **Stop (■)** 클릭 → 회전 중지

> **동작 원리**:
> ```
> 매 프레임:
>   On Playback Tick이 Tick 신호를 보냄
>     → Write Prim Property 실행
>       → /World/Cube의 xformOp:rotateZ에 1.0을 더함
>         → 큐브가 1도씩 회전
> ```

> **⚠️ 주의**: Isaac Sim에서 Stop을 누르면 물리 시뮬레이션만 초기화되고,
> OmniGraph 액션(WritePrimProperty)은 계속 누적된 값을 유지합니다.
> 즉, 큐브의 회전 각도가 리셋되지 않습니다.
> 처음부터 다시 시작하려면 큐브를 삭제하고 새로 만드세요.

---

## 4. 회전 방식 변경 (누적 → 절대값)

기본 Write Prim Property는 **value 값을 매 틱 더합니다** (accumulate).
만약 절대값을 설정하고 싶다면 다른 방법이 필요합니다.

### 4.1 문제: 누적 회전

위 실습에서 `value=1.0`으로 설정하면:
- Frame 1: rotateZ = 1.0
- Frame 2: rotateZ = 2.0
- Frame 3: rotateZ = 3.0
- ... 계속 누적됨

`value`가 매 틱 rotateZ에 **추가(Add)** 되기 때문입니다.

### 4.2 절대값으로 설정하려면?

Write Prim Property를 사용하는 대신 **USD의 `Set` 오퍼레이션**을 사용해야 합니다.
이는 Phase 1 후반부 Python Scripting Step에서 다룹니다.

> **지금은** 누적 회전이 기본 동작임을 이해하고 넘어갑니다.

---

## 5. On Impulse Event: 단발성 동작

On Playback Tick은 매 프레임 실행되는 반면,
**On Impulse Event**는 **한 번만 실행**됩니다.

### 5.1 사용 사례

- 점프 (한 번만 힘을 가함)
- 문 열기 (한 번만 열림 명령)
- 로봇 그리퍼 잡기/놓기

### 5.2 실습: Impulse로 큐브 회전

1. 새 Action Graph 생성
2. 노드 추가:
   - `On Impulse Event` — 수동 트리거
   - `Write Prim Property` — 속성 변경
3. 연결: `On Impulse Event.tick → Write Prim Property.execIn`
4. Write Prim Property 설정:
   - `targetPrim`: `/World/Cube`
   - `propertyName`: `xformOp:rotateZ`
   - `value`: `45.0`
5. On Impulse Event 노드 선택 → **Property Panel**에서 `enableImpulse` 체크박스 클릭

> **enableImpulse**를 체크할 때마다 Write Prim Property가 정확히 한 번 실행되어
> 큐브가 45도 회전합니다.

---

## 6. Keyboard Input으로 큐브 제어

### 6.1 Keyboard 노드 추가

Isaac Sim은 키보드 입력을 받아 OmniGraph에 전달하는 노드를 제공합니다.

**필요 노드:**
- `On Playback Tick` — 매 프레임 실행
- `Keyboard Input` — 키보드 상태 읽기
- `Write Prim Property` — 큐브 속성 변경
- `Constant Double` — 속도값 상수

### 6.2 실습: 방향키로 큐브 이동

**Step 1: Action Graph 생성**
1. **Window > Graph Editors > Action Graph** → **New Action Graph**

**Step 2: 노드 3개 추가**
```
1. On Playback Tick
2. Keyboard Input       (omni.graph.action.KeyboardInput)
3. Write Prim Property  (omni.isaac.core_nodes.IsaacWritePrimProperty)
```

**Step 3: Keyboard Input 설정**

Keyboard Input 노드를 선택하고 **Property Panel**에서:

| 속성 | 값 | 설명 |
|------|------|------|
| **key1** (primary) | `RightArrow` | 감지할 키 |
| **key1** (useAsAxis) | `true` | 축처럼 사용 (누르면+1, 안누르면0) |
| **key2** (secondary) | `LeftArrow` | 반대 방향 키 |
| **key2** (useAsAxis) | `true` | 축처럼 사용 |
| **axisOutputMode** | `Key1MinusKey2` | Key1 누르면 +1, Key2 누르면 -1 |

**Step 4: Constant Double 추가**
1. 노드 검색: `Constant Double`
2. `omni.graph.nodes.ConstantDouble` 추가
3. Property Panel에서 **value** = `10.0` (이동 속도)

**Step 5: Multiply (곱셈) 노드 추가**
1. 검색: `Multiply` → `omni.graph.nodes.Multiply` 추가
2. Keyboard Input의 **axis (double)** → Multiply의 **inputA (double)** 연결
3. Constant Double의 **value (double)** → Multiply의 **inputB (double)** 연결

**Step 6: 최종 연결**

```
On Playback Tick                    Write Prim Property
┌───────────────────┐              ┌─────────────────────────┐
│ tick (exec) ───────┼──────────────┼→ execIn                 │
└───────────────────┘              │  targetPrim: /World/Cube │
                                   │  propertyName: translateX│
Keyboard Input                     │  value ─── (from Mul)   │
┌───────────────────┐              └──────────┬──────────────┘
│ axis (double) ─────┼──┐                      │
└───────────────────┘  │                      │
                       ▼                      │
              ┌──────────────────┐            │
              │   Multiply       │            │
              │  inputA (double)◄┼────────────┘
              │  inputB (double)◄┼── Constant Double (10.0)
              │  result (double)─┼──→ Write.value
              └──────────────────┘
```

**Step 7: 실행**
1. **Play (▶)**
2. **→ (우측 방향키)** 누르면 큐브가 X축 양의 방향으로 이동
3. **← (좌측 방향키)** 누르면 큐브가 X축 음의 방향으로 이동

### 6.3 응용: WASD + 상하 이동

같은 원리로 Y축(좌우), Z축(상하)도 추가하면 3D 자유 이동이 가능합니다.

**Keyboard Input 노드가 지원하는 키 목록:**

| 입력 방식 | 예시 |
|-----------|------|
| 단일 키 | `Space`, `W`, `A`, `S`, `D`, `Shift` |
| 방향키 | `UpArrow`, `DownArrow`, `LeftArrow`, `RightArrow` |
| 조합 (axis) | `W`(primary) + `S`(secondary) → `Key1MinusKey2` |
| 숫자 | `1` ~ `9`, `0` |
| 마우스 | `MouseLeft`, `MouseRight`, `MouseMiddle`, `MousePos` |

---

## 7. Pipeline Stage 이해

OmniGraph에는 **Pipeline Stage**라는 개념이 있습니다.
그래프가 **언제 실행되는지** 결정합니다.

### 7.1 Pipeline Stage 종류

| Stage | 설명 | 사용처 |
|-------|------|--------|
| **PIPELINE_STAGE_EXECUTION** | 매 프레임 자동 실행 (기본값) | 일반적인 시뮬레이션 로직 |
| **PIPELINE_STAGE_ON_DEMAND** | 명시적으로 트리거할 때만 실행 | ROS2 콜백, 이벤트 기반 |
| **PIPELINE_STAGE_SIMULATION** | physics step과 동기화 | 물리 시뮬레이션과 정밀 동기화 |

### 7.2 변경 방법

1. Stage Window에서 `/ActionGraph` 선택
2. **Property Panel** > **Raw USD Properties** 섹션
3. `pipelineStage` 속성 변경:
   - `PIPELINE_STAGE_EXECUTION` (기본)
   - `PIPELINE_STAGE_ON_DEMAND`
   - `PIPELINE_STAGE_SIMULATION`

> **팁**: `PIPELINE_STAGE_SIMULATION`은 물리 시뮬레이션과 동기화되어야 하는
> 로봇 제어(Articulation Controller 등)에 사용합니다. 지금은 기본값을 사용하세요.

---

## 8. 그래프 저장 및 불러오기

### 8.1 USD에 포함된 그래프 저장

Action Graph는 **Stage의 일부**로 USD 파일에 저장됩니다.

1. **File > Save As** (Ctrl+Shift+S)
2. 파일명: `my_omnigraph.usd`
3. 저장

### 8.2 불러오기

1. **File > Open** (Ctrl+O)
2. 저장한 USD 파일 선택
3. Stage Window에서 `/ActionGraph` 확인
4. **Window > Graph Editors > Action Graph** 열기
5. 저장된 그래프가 그대로 표시됨

### 8.3 그래프만 따로 저장

Action Graph를 별도 USD 파일로 저장할 수도 있습니다:

1. Stage Window에서 `/ActionGraph` 우클릭
2. **Export As...** 선택
3. 저장

다른 Scene에서 불러올 때:
1. **File > Reference** 또는 **File > Import**
2. 저장한 USD 파일 선택 → `/ActionGraph`가 추가됨

---

## 9. 유용한 노드 레퍼런스

### 9.1 Event Nodes (트리거)

| 노드명 | 네임스페이스 | 설명 |
|--------|-------------|------|
| **On Playback Tick** | `omni.graph.action.OnPlaybackTick` | 매 프레임 실행 |
| **On Impulse Event** | `omni.graph.action.OnImpulseEvent` | 수동 트리거 (1회) |
| **On Physics Step** | `omni.graph.action.OnPhysicsStep` | 물리 스텝마다 실행 |
| **On Key Pressed** | `omni.graph.action.OnKeyPressed` | 키 누를 때 1회 실행 |

### 9.2 Isaac Sim 전용 Nodes

| 노드명 | 네임스페이스 | 설명 |
|--------|-------------|------|
| **Write Prim Property** | `omni.isaac.core_nodes.IsaacWritePrimProperty` | Prim 속성 변경 |
| **Read Prim Property** | `omni.isaac.core_nodes.IsaacReadPrimProperty` | Prim 속성 읽기 |
| **Articulation Controller** | `omni.isaac.core_nodes.IsaacArticulationController` | 로봇 관절 제어 |
| **Differential Controller** | `omni.isaac.core_nodes.IsaacDifferentialController` | 2륜 로봇 제어 |
| **ROS2 Publisher** | `omni.isaac.ros2_bridge.ROS2Publish*` | ROS2 토픽 발행 |
| **ROS2 Subscriber** | `omni.isaac.ros2_bridge.ROS2Subscribe*` | ROS2 토픽 구독 |

### 9.3 Math Nodes

| 노드명 | 설명 |
|--------|------|
| **Multiply** | 곱셈 |
| **Add** | 덧셈 |
| **Subtract** | 뺄셈 |
| **Divide** | 나눗셈 |
| **Constant Double** | double 상수값 |
| **Constant Int** | int 상수값 |
| **Constant Token** | 문자열(token) 상수 |
| **Make Array** | 여러 값을 배열로 결합 |
| **Clamp** | 값을 최소/최대 범위로 제한 |
| **Conditional** | 조건 분기 |

---

## 10. 종합 실습: 큐브 3D 자유 제어

### 목표

방향키(상/하/좌/우) + Space(위) + Shift(아래)로 큐브를 3D 공간에서 자유롭게 이동합니다.

### Action Graph 구성

```
[Event]                          [Keyboard + Math]                [Action]
                                                       
On Playback Tick ─┬─→ [Keyboard X] → Multiply(10) ─→ WritePrim(target: Cube, prop: translateX)
                  │                                
                  ├─→ [Keyboard Y] → Multiply(10) ─→ WritePrim(target: Cube, prop: translateY)
                  │                                
                  └─→ [Keyboard Z] → Multiply(10) ─→ WritePrim(target: Cube, prop: translateZ)
```

### 상세 설정

**Action Graph 1: X축 이동 (좌/우)**
- Keyboard Input 1: key1=`RightArrow`, key2=`LeftArrow`, axisMode=`Key1MinusKey2`
- Multiply: 10.0
- Write Prim Property: targetPrim=/World/Cube, propertyName=xformOp:translateX

**Action Graph 2: Y축 이동 (앞/뒤)**
- Keyboard Input 2: key1=`UpArrow`, key2=`DownArrow`, axisMode=`Key1MinusKey2`
- Multiply: 10.0
- Write Prim Property: targetPrim=/World/Cube, propertyName=xformOp:translateY

**Action Graph 3: Z축 이동 (위/아래)**
- Keyboard Input 3: key1=`Space`, key2=`LeftShift`, axisMode=`Key1MinusKey2`
- Multiply: 10.0
- Write Prim Property: targetPrim=/World/Cube, propertyName=xformOp:translateZ

> **중요**: 하나의 Action Graph에서 여러 개의 독립 로직을 구성하려면
> On Playback Tick의 Tick 출력을 여러 갈래로 분기(여러 노드의 execIn에 연결)할 수 있습니다.

---

## 11. 문제 해결 (Troubleshooting)

### 문제 1: Play를 눌렀는데 큐브가 회전하지 않습니다.

**확인 사항**:
- [ ] Action Graph가 생성되었는가? (Stage에 `/ActionGraph` 존재)
- [ ] On Playback Tick 노드가 있는가?
- [ ] Tick → Write Prim Property의 execIn 연결이 되어 있는가?
- [ ] Write Prim Property의 `targetPrim`이 정확한가? (대소문자 구분)
- [ ] Write Prim Property의 `propertyName`이 정확한가? (`xformOp:rotateZ`)

### 문제 2: 키보드 입력이 작동하지 않습니다.

**확인 사항**:
- [ ] Keyboard Input 노드의 `key1`, `key2` 이름이 정확한가? (`RightArrow` vs `rightArrow`)
- [ ] `useAsAxis`가 true인가?
- [ ] `axisOutputMode`가 `Key1MinusKey2`인가?
- [ ] Play(▶) 상태인가? (키보드는 Play 중에만 작동)
- [ ] Viewport가 활성화되어 있는가? (Viewport 클릭하여 포커스)

### 문제 3: 큐브가 한 방향으로만 계속 이동합니다.

**원인**: Keyboard Input이 축 모드가 아니라 단순 키 프레스 모드
**해결**: `useAsAxis = true` 확인. false면 키를 누를 때마다 1회만 출력

### 문제 4: Action Graph가 보이지 않습니다.

**해결**:
1. **Window > Graph Editors > Action Graph** 다시 열기
2. Stage Window에서 `/ActionGraph`가 있는지 확인
3. 없으면 **New Action Graph** 클릭

### 문제 5: 이전에 저장한 그래프가 없어졌습니다.

**해결**:
1. File > Open으로 저장한 USD 파일 열기
2. Stage에 `/ActionGraph`가 포함되어 있어야 함
3. Window > Graph Editors > Action Graph에서 확인

---

## 12. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ OmniGraph 개념 | 비주얼 프로그래밍 프레임워크, 노드 연결로 로직 구성 |
| ✅ Action Graph 구조 | Event Node → Logic/Data Nodes → Action Node |
| ✅ On Playback Tick | 매 프레임 실행되는 기본 트리거 |
| ✅ Write Prim Property | USD Prim의 속성값 변경 (누적 방식) |
| ✅ On Impulse Event | 수동으로 1회 트리거 |
| ✅ Keyboard Input | 키보드로 실시간 객체 제어 (축 모드) |
| ✅ Pipeline Stage | Execution / On-Demand / Simulation 3가지 실행 모드 |
| ✅ 그래프 저장/불러오기 | USD에 포함 저장, 별도 Export/Import |

### 핵심 개념 도식

```
OmniGraph 메인 흐름:

  Event ──→ Node ──→ Node ──→ Action
  (틱)      (변환)    (계산)    (쓰기)

  Isaac Sim 시뮬레이션 루프와의 관계:

  ┌──────────┐    ┌────────────┐    ┌──────────┐
  │ Physics  │    │ OmniGraph  │    │ Render   │
  │ Step     │───→│ Tick       │───→│ Frame    │
  └──────────┘    └────────────┘    └──────────┘
                   ↓
              노드 실행
              (컨트롤러, 속성변경, ROS2 등)
```

---

## 13. 다음 Step 예고

**Step 04 — USD Stage 심화**에서는:
- USD의 계층 구조(Prim, Attribute, Relationship) 이해
- USD 포맷(.usda)을 텍스트 에디터로 직접 편집
- Prim Specifier(Def, Over, Class, Reference) 차이
- Reference와 Payload로 대규모 Scene 구성

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| OmniGraph 공식 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/index.html |
| OmniGraph 튜토리얼 | https://docs.isaacsim.omniverse.nvidia.com/latest/omnigraph/omnigraph_tutorial.html |
| OmniGraph 아키텍처 | https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/Architecture.html |
| 키보드 입력 노드 | https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/nodes/omni-graph-action/keyboardinput.html |
| Commonly Used Shortcuts | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/commonly_used_omnigraph_shortcuts.html |
| OmniGraph Python Scripting | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/omnigraph_python_scripting.html |
