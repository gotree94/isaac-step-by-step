# Step 02 — GUI 기본 사용법

> **소요 시간**: 45분
> **난이도**: ★☆☆☆☆ (초급)
> **선수 조건**: Step 01 완료 (Isaac Sim 설치)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim의 **7가지 주요 UI 패널**을 식별하고 설명한다
2. **Create 메뉴**로 객체(Cube/Sphere/Cylinder)를 Stage에 추가한다
3. **Transform Gizmo**(이동/회전/크기)로 객체를 조작한다
4. **Property Panel**에서 물리 속성(Rigid Body, Collision)을 추가한다
5. 시뮬레이션을 **Play/Stop/Pause**하고 결과를 확인한다
6. Scene을 **USD 파일로 저장하고 다시 불러온다**
7. 작업 영역 **Layout을 저장하고 복원**한다

---

## 1. Isaac Sim UI 개요

Isaac Sim을 설치한 후 터미널에서 실행합니다.

```bash
# pip 설치 방식
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
isaacsim
```

> **첫 실행 시**: 확장 프로그램 캐싱으로 5~10분 정도 소요될 수 있습니다.
> 빈 화면이 나타나도 기다리세요.

### 1.1 메인 UI 구성

Isaac Sim이 완전히 로드되면 다음과 같은 화면이 나타납니다:

```
┌─────────────────────────────────────────────────────────────────┐
│ [Menu Bar]  File  Edit  Create  Window  Tools  Utilities  Layout│
├───────────┬────────────────────────────────────┬─────────────────┤
│           │                                    │   Property      │
│  Content  │           Viewport                 │   Panel         │
│  Browser  │       (3D 뷰포트)                  │                 │
│           │                                    │   ──────────    │
│           │                                    │   Stage         │
│           │                                    │   Window        │
├───────────┴────────────────────────────────────┴─────────────────┤
│ [Toolbar]  🖱️선택  ↔이동  ↻회전  ⇔크기  🔗스냅  ▶Play  ■Stop  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 7가지 주요 UI 요소

| # | 요소 | 이름 | 설명 |
|---|------|------|------|
| ① | **Menu Bar** | 메뉴바 | 최상단 메뉴. Create(생성), Window(창), Tools(도구) 등 |
| ② | **Viewport** | 뷰포트 | 3D 장면을 보는 메인 캔버스. 마우스로 시점 조작 |
| ③ | **Toolbar** | 도구 모음 | 객체 선택/이동/회전/크기 조절, 시뮬레이션 Play/Stop |
| ④ | **Content Browser** | 콘텐츠 브라우저 | 로봇, 환경, 머티리얼 등 에셋 탐색기 |
| ⑤ | **Stage Window** | 스테이지 창 | 현재 Scene의 모든 객체(Prim)를 계층 구조로 표시 |
| ⑥ | **Property Panel** | 속성 패널 | 선택한 객체의 상세 속성 표시 및 편집 |
| ⑦ | **Animation Bar** | 애니메이션 바 | 시뮬레이션 타임라인, Play/Pause/Stop, 프레임 제어 |

---

## 2. Viewport 시점 제어

Viewport는 3D 공간을 보는 창입니다. 마우스로 시점을 자유롭게 제어할 수 있습니다.

| 조작 | 동작 | 설명 |
|------|------|------|
| **좌클릭 + 드래그** | 객체 선택 | Viewport에서 객체를 클릭하여 선택 |
| **우클릭 + 드래그** | 시점 회전 (Orbit) | 카메라가 중심점을 기준으로 회전 |
| **휠 스크롤** | 확대/축소 (Zoom) | 시점 전/후 이동 |
| **가운데버튼 + 드래그** | 시점 이동 (Pan) | 카메라가 상하좌우로 평행 이동 |
| **F 키** | 선택 객체에 포커스 | 선택한 객체가 Viewport 중앙에 오도록 확대 |

### 실습 2.1: Viewport 탐색

실행 직후 빈 Stage가 나타납니다. 다음을 연습하세요:

1. 우클릭 드래그 → 시점 회전
2. 휠 스크롤 → 확대/축소
3. 가운데 버튼 드래그 → 시점 이동

> **💡 팁**: 익숙해질 때까지 2~3분간 자유롭게 시점을 움직여보세요.

---

## 3. Stage에 객체 추가하기

### 3.1 Create 메뉴로 객체 생성

**Create > Shape > Cube**를 선택하여 큐브를 추가합니다.

```
Create 메뉴 주요 항목:
├── Shape          → Cube, Sphere, Cylinder, Cone, Torus 등 기본 도형
├── Physics        → Ground Plane, Physics Scene
├── Lights         → Distant Light, Sphere Light, Dome Light
├── Environment    → Simple Room, Flat Grid, Warehouse 등
├── Robots         → Franka, Jetbot, TurtleBot, Nova Carter 등
└── Camera         → Camera, Depth Camera
```

**단계별 실습:**

1. 상단 메뉴 **Create > Shape > Cube** 선택
2. Stage Window에 `/World/Cube`가 추가된 것을 확인
3. Viewport에 회색 큐브가 나타난 것을 확인

### 3.2 Stage Window 이해

Stage Window는 현재 Scene의 **모든 객체(Prim) 계층 구조**를 표시합니다.

```
Stage Window 예시:
/World                              ← Root
├── Ground Plane                    ← 바닥면
├── Cube                            ← 큐브
├── Sphere                          ← 구체
├── DistantLight                    ← 광원
└── PhysicsScene                    ← 물리 Scene
```

**주요 개념**: Prim (Primitive)은 USD의 기본 객체 단위입니다.
- `/World`: Root Prim (모든 객체의 부모)
- `/World/Cube`: 큐브 Prim
- `/World/Ground Plane`: 바닥면 Prim

> **Stage 조작 팁**:
> - 객체 클릭 → Viewport에서 선택됨
> - 드래그 → 계층 구조 변경 (부모-자식 관계)
> - 우클릭 → Delete, Rename, Copy 등 컨텍스트 메뉴
> - 눈 아이콘 👁 → Visibility 토글 (보이기/숨기기)

---

## 4. Transform Gizmo로 객체 조작

### 4.1 Toolbar의 Gizmo 도구

Toolbar의 Gizmo로 객체를 변환(Transform)할 수 있습니다.

| 아이콘 | 단축키 | 기능 | 설명 |
|--------|--------|------|------|
| 🖱️ | **Q** | Select | 객체 선택 모드 |
| ↔ | **W** | Move (Translate) | 객체 이동 |
| ↻ | **E** | Rotate | 객체 회전 |
| ⇔ | **R** | Scale | 객체 크기 조절 |

### 4.2 이동 (Move, W 키)

1. 큐브를 선택한 상태에서 **W** 키 누름
2. 화살표 색상별 축:
   - **빨간색** → X축
   - **초록색** → Y축
   - **파란색** → Z축
3. 단일 축 드래그: 해당 축 방향으로만 이동
4. 사각형 면 드래그: 두 축 평면에서 이동
5. 중앙 점 드래그: 자유 이동

### 4.3 회전 (Rotate, E 키)

1. **E** 키로 회전 모드 전환
2. 색상 링이 나타남 (RGB = XYZ)
3. 링을 드래그하여 해당 축 기준 회전
4. 회전 각도는 Property Panel에서 정밀 입력 가능

### 4.4 크기 조절 (Scale, R 키)

1. **R** 키로 스케일 모드 전환
2. 단일 축 드래그 → 해당 축만 늘리기/줄이기
3. 중앙 드래그 → 전체 균등 스케일

### 4.5 Property Panel로 정밀 제어

Toolbar 대신 Property Panel에 직접 숫자를 입력하여 정밀하게 제어할 수 있습니다.

큐브 선택 → Property Panel 하단에서:

```
Transform:
  Translate:  X [0.00]  Y [0.00]  Z [0.50]
  Rotate:     X [0.00]  Y [0.00]  Z [0.00]
  Scale:      X [1.00]  Y [1.00]  Z [1.00]
```

값을 직접 변경하여 정밀하게 위치를 조정하세요.
> 파란색 █ 사각형을 클릭하면 값을 기본값으로 리셋합니다.

### 실습 4.1: 큐브 조작 종합

1. **Create > Shape > Cube** 큐브 1개 생성
2. W 키 → 큐브를 Viewport 중앙으로 이동 (X:0, Y:0, Z:0)
3. E 키 → Y축 기준 45도 회전
4. R 키 → X축만 2배로 늘리기
5. Property Panel에서 Translate Z를 2.0으로 변경
6. Q 키 → 선택 해제

---

## 5. 조명과 환경 추가

### 5.1 조명 추가

조명이 없으면 Scene이 어둡게 보입니다.

```bash
# Create > Lights > Distant Light
# Create > Lights > Sphere Light
# Create > Lights > Dome Light
```

**조명 종류별 특징:**

| 조명 | 설명 | 용도 |
|------|------|------|
| **Distant Light** | 태양광 같은 평행광 | 전체 Scene 균일 조명 |
| **Sphere Light** | 구형에서 방사되는 광원 | 국부 조명 |
| **Dome Light** | 돔 전체에서 오는 환경광 | 자연스러운 환경 조명 |
| **Cylinder Light** | 원통형 광원 | 형광등 효과 |

### 5.2 환경 에셋 추가

```bash
# Create > Environment > Simple Room
# Create > Environment > Flat Grid
# Create > Isaac > Environments > ...
```

**환경 종류:**
| 환경 | 설명 |
|------|------|
| **Simple Room** | 방 안에 탁자가 있는 실내 환경 |
| **Flat Grid** | 평평한 바닥의 빈 공간 (시뮬레이션에 자주 사용) |
| **Warehouse** | 창고 환경 (Phase 3에서 사용) |

### 실습 5.1: Scene 구성하기

1. **Create > Environment > Simple Room** 선택
   - 방 안에 테이블이 있는 Scene이 로드됨
2. Stage Window에서 `/World`의 구조 확인
3. Viewport에서 우클릭 드래그로 방 내부를 탐색
4. **Create > Lights > Dome Light**로 환경 조명 추가

---

## 6. 물리 속성 추가 및 시뮬레이션

### 6.1 Physics Scene 확인

Isaac Sim은 기본적으로 `PhysicsScene`이 있어야 물리 시뮬레이션이 동작합니다.

```
Stage Window에서 확인:
/World
├── PhysicsScene      ← (필수) 물리 Scene
├── Ground Plane      ← (권장) 바닥면
└── ...
```

**PhysicsScene**이 없으면:
```bash
# Create > Physics > Physics Scene
```

### 6.2 Ground Plane 추가

객체가 바닥 아래로 떨어지지 않도록 Ground Plane이 필요합니다.
```bash
# Create > Physics > Ground Plane
```

### 6.3 Rigid Body 속성 추가

큐브가 중력에 의해 낙하하도록 물리 속성을 추가합니다.

**GUI 방식:**

1. Stage Window에서 `/World/Cube` 선택
2. **Property Panel**에서 **+Add** 버튼 클릭
3. **Physics > Rigid Body with Colliders Preset** 선택
4. Rigid Body 속성이 추가됨:
   ```
   Physics:
     Rigid Body Enabled: ✅
     Mass: 1.0
     Linear Damping: 0.0
     Angular Damping: 0.0
   Collision:
     Collision Enabled: ✅
     Approximation Shape: Convex Hull
   ```

### 6.4 시뮬레이션 실행

```bash
도구 모음 하단:
▶ Play    = 시뮬레이션 시작
⏸ Pause  = 시뮬레이션 일시 정지
■ Stop   = 시뮬레이션 중지 및 초기화
```

**실습:**
1. 큐브에 Rigid Body 속성 추가
2. Play 버튼(▶) 클릭
3. 큐브가 중력에 의해 낙하 → Ground Plane에 충돌
4. 다시 초기화하려면 Stop(■) 클릭

> **💡 Tip**: Isaac Sim은 Stop 시 물리 상태만 초기화됩니다.
> 객체의 Transform 위치는 유지됩니다 (즉, 처음 Translate Z=5에서 Play하면
> 큐브가 낙하하고, Stop 누르면 Z=0으로 리셋되지 않고 낙하한 위치에 남음).

### 6.4 물리 속성 상세 설명

| 속성 | 설명 |
|------|------|
| **Rigid Body** | 중력, 힘, 토크 등 물리 영향 받음 |
| **Collision** | 다른 객체와 충돌 감지 |
| **Mass** | 질량 (kg). 높을수록 관성 큼 |
| **Linear Damping** | 선형 속도 감쇠 (공기 저항 효과) |
| **Angular Damping** | 회전 속도 감쇠 |
| **Kinematic** | 켜면 물리 영향 안 받음, 직접 Transform 제어 |

### 실습 6.1: 물리 객체 비교

1. **Create > Shape > Cube** → Z=3.0에 배치 → Rigid Body 추가
2. **Create > Shape > Sphere** → Z=5.0에 배치 → Rigid Body 추가
3. **Create > Shape > Cylinder** → Z=7.0에 배치 → Rigid Body 추가
4. Play 클릭 → 각 객체가 중력에 의해 낙하
5. 모양에 따라 다른 낙하/충돌 양상 관찰
6. 각 객체의 **Mass**를 변경 (1, 10, 100) → Play → 질량에 따른 차이 관찰

---

## 7. Material 적용하기

Material을 적용하여 객체의 색상과 표면 질감을 변경할 수 있습니다.

### 7.1 Material 생성 및 적용

1. Stage Window에서 큐브 선택
2. **Property Panel** > **Appearance** 섹션 찾기 (없으면 +Add > Appearance)
3. **Material** 필드 옆의 화살표 클릭 → **Create Material** 선택
4. MDL(Material Definition Language) 선택:
   ```
   Material Presets:
   ├── OmniSurface (범용)
   ├── OmniGlass   (유리)
   ├── OmniMetal   (금속)
   └── OmniGlossy  (광택)
   ```
5. Material 생성 후 Property Panel에서 색상, 거칠기, 금속성 조정

| 속성 | 설명 |
|------|------|
| **diffuse_color** | 기본 색상 |
| **roughness** | 표면 거칠기 (0=매끈, 1=거침) |
| **metallic** | 금속성 (0=비금속, 1=금속) |
| **opacity** | 불투명도 (1=완전 불투명, 0=투명) |

### 실습 7.1: Material 놀이

1. 큐브 생성 → 빨간색 OmniSurface Material 적용 (roughness=0.3)
2. 구체 생성 → 파란색 금속 Material 적용 (metallic=1.0, roughness=0.1)
3. 원기둥 생성 → 녹색 유리 Material 적용 (opacity=0.5)
4. 조명 추가 (Dome Light + Distant Light)
5. Play → 아름다운 Scene 감상

---

## 8. USD 파일 저장 및 불러오기

### 8.1 Scene 저장

```bash
# File > Save As... (Ctrl+Shift+S)
```

1. **File > Save As** 선택
2. 저장 위치와 파일명 지정 (예: `my_first_scene.usd`)
3. **저장**

> **권장 사항**: 프로젝트 폴더 내에 `assets/scenes/` 디렉토리를 만들어 저장하세요.
> ```
> isaac-step-curriculum/
> └── assets/
>     └── scenes/
>         └── my_first_scene.usd
> ```

### 8.2 Scene 불러오기

```bash
# File > Open... (Ctrl+O)
또는
# Content Browser > 로컬 파일 시스템 탐색 > USD 더블클릭
```

### 8.3 USD 파일 포맷 이해

| 확장자 | 설명 | 특징 |
|--------|------|------|
| `.usd` | 기본 USD 포맷 | 범용, 모든 기능 지원 |
| `.usda` | ASCII 텍스트 포맷 | 텍스트 에디터로 편집 가능, Git diff 용이 |
| `.usdc` | 바이너리 포맷 | 빠른 로딩, 용량 작음 |

> **Git 관리 시**: `.usda` 포맷을 권장합니다.
> 텍스트 기반이라 변경 사항을 diff로 확인할 수 있습니다.

---

## 9. Layout 저장 및 복원

### 9.1 Layout이란?

Layout은 패널의 위치, 크기, 가시성 등 **작업 영역 구성**을 저장한 것입니다.
코딩 작업 시 Script Editor가 필요하고, 모델링 시 Property Panel이 중요하는 등
작업에 따라 최적의 Layout이 다릅니다.

### 9.2 Layout 저장

```bash
# Layout > Save Layout...
```

1. **Layout** 메뉴 열기
2. **Save Layout...** 선택
3. Layout 이름 입력 (예: `core-api-work`)
4. 저장

### 9.3 Layout 불러오기

```bash
# Layout > [저장된 Layout 이름]
```

저장된 Layout 목록에서 선택하여 즉시 전환할 수 있습니다.

### 9.4 기본 Layout

| Layout | 설명 | 사용처 |
|--------|------|--------|
| **Default** | 기본 구성 | 일반 탐색 |
| **Scripting** | Script Editor 중심 | Python 코딩 |
| **Rendering** | Viewport 최대화 | 렌더링 결과 확인 |
| **Animation** | 타임라인 중심 | 애니메이션 작업 |

---

## 10. 종합 실습: 첫 번째 Scene 만들기

지금까지 배운 모든 것을 활용하여 하나의 완성된 Scene을 만들어보세요.

### 목표

```
방 안에 탁자 위에 컬러 큐브 3개가 쌓여있고,
시뮬레이션을 실행하면 큐브가 중력에 의해 떨어지는 Scene
```

### 단계별 가이드

**1단계: 환경 구성**
- File > New (새 Scene)
- Create > Physics > Ground Plane
- Create > Physics > Physics Scene
- Create > Environment > Simple Room (Stage에 드래그)
- Create > Lights > Dome Light

**2단계: 객체 생성 및 배치**
- Create > Shape > Cube → Z=1.0에 배치 (빨간 Material)
- Create > Shape > Cube → Z=1.8에 배치 (파란 Material)  
- Create > Shape > Cube → Z=2.6에 배치 (초록 Material)

**3단계: 물리 속성 추가**
- 큐브 3개 모두 선택 (Ctrl+클릭)
- Property Panel > +Add > Physics > Rigid Body with Colliders Preset
- 각각 Mass를 1.0, 0.5, 2.0으로 설정

**4단계: 시뮬레이션**
- ▶ Play 클릭
- 큐브 3개가 낙하하는 모습 관찰
- ■ Stop으로 중지

**5단계: 저장**
- Layout > Save Layout... > `tower-scene`
- File > Save As... > `my_tower_scene.usd`

---

## 11. 키보드 단축키 요약

| 단축키 | 기능 |
|--------|------|
| **Q** | Select 모드 |
| **W** | Move (이동) 모드 |
| **E** | Rotate (회전) 모드 |
| **R** | Scale (크기) 모드 |
| **F** | 선택 객체에 포커스 |
| **Ctrl+Z** | 실행 취소 |
| **Ctrl+Shift+Z** | 다시 실행 |
| **Ctrl+S** | 저장 |
| **Ctrl+Shift+S** | 다른 이름으로 저장 |
| **Ctrl+O** | 열기 |
| **Ctrl+N** | 새 Scene |
| **Delete** | 선택 객체 삭제 |
| **Space** | Play/Pause 토글 |

---

## 12. 문제 해결 (Troubleshooting)

### 문제 1: Scene이 너무 어둡습니다.

**해결**: 조명 추가
```bash
# Create > Lights > Dome Light (환경광)
# Create > Lights > Distant Light (태양광)
```

### 문제 2: 큐브가 Play를 눌러도 떨어지지 않습니다.

**원인**: Rigid Body 속성이 없음
**해결**: 
1. 큐브 선택
2. Property Panel > +Add > Physics > Rigid Body with Colliders Preset

### 문제 3: 큐브가 바닥을 뚫고 계속 떨어집니다.

**원인**: Ground Plane이 없음
**해결**:
```bash
# Create > Physics > Ground Plane
```
또는 큐브의 Collision 속성 확인

### 문제 4: Viewport가 너무 느립니다.

**해결**:
1. **Window > Rendering Settings**에서 RTX Renderer → **Path Tracing** 비활성화
2. **Preferences > Rendering > Max Framerate**를 30 FPS로 제한
3. 불필요한 조명/섀도우 제거

### 문제 5: 객체가 선택되지 않습니다.

**원인**: 선택 모드가 아님
**해결**: Q 키를 눌러 Select 모드로 전환

### 문제 6: USD 저장 후 다시 열었을 때 Material이 사라졌습니다.

**원인**: Material이 Scene에 포함되지 않았을 수 있음
**해결**: File > Save As에서 **"Flatten"** 옵션 활성화 (모든 레퍼런스를 단일 파일에 포함)

---

## 13. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ UI 구성 | 7가지 주요 패널 식별 및 설명 |
| ✅ Viewport 제어 | 마우스 조작으로 3D 시점 자유롭게 변경 |
| ✅ 객체 생성 | Create 메뉴로 Cube/Sphere 등 추가 |
| ✅ Transform | W/E/R Gizmo로 이동/회전/크기 조절 |
| ✅ Property Panel | 속성 추가 및 숫자 입력 정밀 제어 |
| ✅ 물리 시뮬레이션 | Rigid Body + Play/Stop |
| ✅ Material | 색상/질감/투명도 적용 |
| ✅ USD 저장/불러오기 | .usd / .usda / .usdc 포맷 |
| ✅ Layout | 작업 영역 구성 저장/복원 |

---

## 14. 다음 Step 예고

**Step 03 — OmniGraph 기초**에서는:
- Action Graph의 개념과 노드 연결 방식 이해
- 큐브 낙하를 OmniGraph으로 구현
- Event-driven 시뮬레이션 흐름 제어
- 시작/조건/동작 노드의 관계 학습

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim UI Reference | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/gui/reference_user_interface.html |
| Basic Usage Tutorial | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/quickstart_isaacsim.html |
| Create Menu Reference | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/gui/menu_create.html |
| Selection Modes | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/gui/selection-modes.html |
| Keyboard Shortcuts | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/gui/reference_keyboard_shortcuts.html |
