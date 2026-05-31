# Step 04 — USD Stage 구조 이해

> **소요 시간**: 60분
> **난이도**: ★★☆☆☆ (초급~중급)
> **선수 조건**: Step 03 완료

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **USD(Universal Scene Description)** 의 기본 개념을 설명한다
2. **Prim**과 **Attribute**, **Relationship**의 관계를 이해한다
3. **Specifier**(Def, Over, Class)의 차이를 구분한다
4. `.usda` 포맷을 **텍스트 에디터로 직접 편집**한다
5. **Reference**를 사용하여 외부 USD 파일을 재사용한다
6. **Payload**로 대규모 Scene을 지연 로딩한다
7. Python API로 USD Stage를 **프로그래밍 방식**으로 조작한다

---

## 1. USD란?

### 1.1 Universal Scene Description

**USD(Universal Scene Description)**는 Pixar에서 개발한 3D Scene 데이터 포맷입니다.
NVIDIA Omniverse와 Isaac Sim의 **핵심 데이터 교환 포맷**입니다.

> **비유**: USD는 3D 세계의 JSON/XML과 같습니다.
> - 모든 객체(Prim), 속성(Attribute), 계층 구조(Parent-Child)를 텍스트로 표현
> - Git으로 버전 관리 가능
> - Non-destructive 합성(Composition) 지원

### 1.2 USD의 장점

| 특징 | 설명 | Isaac Sim에서의 의미 |
|------|------|---------------------|
| **비파괴 편집** | 원본을 수정하지 않고 덮어쓰기(Override) | 로봇 USD를 수정하지 않고 Scene에 배치 |
| **Reference** | 외부 USD를 참조로 가져오기 | 복잡한 로봇을 별도 파일로 분리 |
| **Payload** | 조건부 지연 로딩 | 대규모 Scene의 빠른 로딩 |
| **Layer** | 여러 레이어를 합성 | 여러 팀이 동시에 작업 가능 |
| **Variant** | 같은 Prim의 여러 변형 | 로봇의 여러 스킨/형상 |

### 1.3 USD 포맷 종류

| 확장자 | 포맷 | 특징 |
|--------|------|------|
| `.usda` | **ASCII** (텍스트) | 사람이 읽을 수 있음, Git diff 가능 |
| `.usdc` | **Binary** | 빠른 로딩, 작은 용량 |
| `.usd` | **Container** | `.usda` 또는 `.usdc`로 저장 가능 |

> **권장**: 개발 중에는 `.usda`를 사용하세요. Git diff로 변경 사항을 추적할 수 있습니다.
> 최종 배포 시에는 `.usdc`로 변환하여 로딩 속도를 높이세요.

---

## 2. USD 기본 구조

### 2.1 Prim, Attribute, Relationship

USD Scene(Stage)은 세 가지 기본 요소로 구성됩니다:

```
Stage (Scene)
├── Prim (객체 노드)
│   ├── Attribute (속성: 값)
│   └── Relationship (관계: 다른 Prim 참조)
└── ...
```

| 용어 | 설명 | 예시 |
|------|------|------|
| **Prim** (Primitive) | Scene의 객체 또는 노드 | `/World/Cube`, `/World/Robot` |
| **Attribute** | Prim의 속성 (타입+값) | `double xformOp:translateX = 5.0` |
| **Relationship** | Prim 간의 연결 관계 | `physics:joint0 = </World/Joint>` |

### 2.2 .usda 파일 직접 보기

Isaac Sim에서 간단한 Cube를 생성하고 `.usda`로 저장한 후 텍스트 에디터로 열어보겠습니다.

**Step 1**: Isaac Sim에서 Cube 생성 + 저장
1. **Create > Shape > Cube**
2. **File > Save As...** → `my_cube.usda` (포맷: ASCII)
3. 저장 위치: `~/isaac-step-curriculum/assets/scenes/`

**Step 2**: 텍스트 에디터로 열기

```bash
cat ~/isaac-step-curriculum/assets/scenes/my_cube.usda
```

다음과 같은 내용이 표시됩니다:

```usda
#usda 1.0                          # USD 버전 선언
(
    defaultPrim = "World"           # 기본 Prim
    metersPerUnit = 1               # 단위 (1m)
    upAxis = "Z"                    # 위쪽 방향 (Z Up)
)

def Xform "World"                   # def = 정의 (Specifier)
{
    def Cube "Cube"                 # World 아래 Cube Prim
    {
        double3 xformOp:translate = (0, 0, 2)  # 위치 속성
        float3 xformOp:rotateXYZ = (0, 0, 0)   # 회전 속성
        float3 xformOp:scale = (1, 1, 1)       # 스케일 속성
        uniform token[] xformOpOrder = [        # Transform 적용 순서
            "xformOp:translate",
            "xformOp:rotateXYZ", 
            "xformOp:scale"
        ]
    }
}
```

### 2.3 한 줄씩 해석

```usda
#usda 1.0
```
USD 버전 1.0을 사용합니다.

```usda
def Xform "World"
```
- `def`: **Specifier** (정의) — 새 Prim을 생성합니다.
- `Xform`: **Prim Type** — 변환(Transform)을 가질 수 있는 그룹.
- `"World"`: **Prim Name**.

```usda
{
    def Cube "Cube"
```
- World 아래에 `Cube` 타입의 Prim을 `"Cube"` 이름으로 생성.

```usda
    double3 xformOp:translate = (0, 0, 2)
```
- **Attribute**: `double3 xformOp:translate` = 3개의 double 값.
- `= (0, 0, 2)`: 초기값.

```usda
    uniform token[] xformOpOrder = [...]
```
- `uniform`: 모든 자식이 동일한 값을 공유.
- `token[]`: 문자열 배열.
- Transform 적용 순서 정의 (Translate → Rotate → Scale).

---

## 3. Specifier (Def / Over / Class / Reference)

USD에는 Prim을 선언하는 4가지 방식(Specifier)이 있습니다:

### 3.1 `def` — 정의 (Define)

새로운 Prim을 **생성**합니다.

```usda
def Sphere "MySphere"
{
    double radius = 0.5
}
```

### 3.2 `over` — 오버라이드 (Override)

기존 Prim의 속성만 **덮어씁니다**. Prim이 없으면 무시됩니다.

```usda
over "MySphere"
{
    double radius = 1.0  # 반지름을 1.0으로 변경
}
```

> **유용한 상황**: 
> - 참조된 USD 파일의 로봇 색상만 변경하고 싶을 때
> - Layer를 사용한 비파괴 편집

### 3.3 `class` — 클래스 (Class)

**인스턴스화되지 않는** 템플릿 Prim입니다. `class` 자체는 Scene에 나타나지 않지만, 상속의 기준이 됩니다.

```usda
class "MaterialBase"
{
    token inputs:diffuseColor = (1, 1, 1)
}

def Sphere "RedSphere" (
    inherits = </MaterialBase>
)
{
    token inputs:diffuseColor = (1, 0, 0)
}
```

### 3.4 `ref` (Reference) — 참조

별도 USD 파일의 Prim을 **참조**하여 현재 Stage에 포함합니다.
원본 파일은 수정되지 않습니다.

```usda
def Xform "MyRobot" (
    prepend references = @robot/robot.usda@</Robot>
)
{
    double3 xformOp:translate = (0, 0, 0.5)
}
```

> **Reference는 Isaac Sim에서 가장 중요한 기능 중 하나입니다.**
> 로봇, 환경, 머티리얼 등 대부분의 에셋이 Reference로 불러와집니다.

---

## 4. 실습: .usda를 텍스트 에디터로 직접 편집

### 4.1 직접 .usda 파일 만들기

텍스트 에디터로 새 파일을 만들겠습니다:

```bash
mkdir -p ~/isaac-step-curriculum/assets/scenes/
```

파일: `~/isaac-step-curriculum/assets/scenes/hello_usd.usda`

```usda
#usda 1.0
(
    defaultPrim = "HelloScene"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "HelloScene"
{
    def Sphere "MySphere"
    {
        double3 xformOp:translate = (0, 0, 1.0)
        float3 xformOp:rotateXYZ = (0, 0, 0)
        float3 xformOp:scale = (1, 1, 1)
        uniform token[] xformOpOrder = [
            "xformOp:translate",
            "xformOp:rotateXYZ", 
            "xformOp:scale"
        ]
    }

    def Cube "MyCube"
    {
        double3 xformOp:translate = (2, 0, 0.5)
        float3 xformOp:rotateXYZ = (0, 45, 0)
        float3 xformOp:scale = (1, 2, 1)
        uniform token[] xformOpOrder = [
            "xformOp:translate",
            "xformOp:rotateXYZ", 
            "xformOp:scale"
        ]
    }
}
```

### 4.2 Isaac Sim에서 불러오기

1. **File > Open...** (Ctrl+O)
2. 방금 만든 `hello_usd.usda` 선택
3. Viewport에 구체와 큐브가 나타나는지 확인

**결과 예상**:
```
  ←──── 2m ────→  
  ┌──────┐  ●
  │      │  (Sphere)
  │ Cube │  Z=0.5
  │(2x scale │
  │ Y축)  │  
  └──────┘  
```

> **💡 연습**: .usda 파일을 수정하고 Isaac Sim에서 **File > Reload** (Ctrl+R)로
> 즉시 변경 사항을 확인할 수 있습니다. Isaac Sim을 재시작할 필요가 없습니다!

### 4.3 Attribute 수정 연습

다음 변경을 `.usda`에 직접 적용하고 결과를 확인하세요:

1. 구체의 색상 추가 (아래 참고)
2. 큐브의 위치를 Z=2.0으로 변경
3. 새 Cylinder Prim 추가

**색상 추가 방법**:
```usda
def Sphere "MySphere"
{
    ...
    color3f[] primvars:displayColor = [(1, 0, 0)]  # 빨간색
}
```

---

## 5. Reference로 로봇 불러오기

### 5.1 Reference 개념

Reference는 **외부 USD 파일을 현재 Scene에 포함**시키는 USD의 합성(Composition) 기능입니다.

```
robot.usda
┌────────────────┐
│  def Robot     │     Scene.usda
│  ├─ chassis    │     ┌──────────────────────────────┐
│  ├─ arm        │     │  def Xform "MyRobot" (       │
│  └─ gripper    │     │    prepend references =       │
└────────────────┘     │      @robot.usda@</Robot>     │
                       │  ) {                          │
                       │    ...                        │
                       │  }                            │
                       └──────────────────────────────┘
```

**특징**:
- 원본 USD는 **읽기 전용**으로 유지됨
- **Over**로 원본 속성을 덮어쓸 수 있음
- 하나의 USD를 여러 번 Reference 가능 (다른 위치에 여러 로봇)

### 5.2 .usda로 Reference 직접 작성

파일: `~/isaac-step-curriculum/assets/scenes/ref_demo.usda`

```usda
#usda 1.0
(
    defaultPrim = "RefDemo"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "RefDemo"
{
    # Ground Plane (참고: Ground Plane도 USD)
    def Plane "Ground"
    {
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    # Cube를 Reference로 불러오기 (hello_usd.usda에서)
    def Xform "ReferencedCube" (
        prepend references = @./hello_usd.usda@</HelloScene/MyCube>
    )
    {
        double3 xformOp:translate = (-2, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    # Sphere를 Reference로 불러오기
    def Xform "ReferencedSphere" (
        prepend references = @./hello_usd.usda@</HelloScene/MySphere>
    )
    {
        double3 xformOp:translate = (2, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
```

### 5.3 Isaac Sim GUI로 Reference 추가

1. **Create > Reference** 선택 (또는 **File > Reference**)
2. 참조할 USD 파일 선택
3. Stage에 Reference Prim이 추가됨
4. [선택] 원본 Prim 경로 지정 (`</Robot>` 등)

**또는**:
1. **Content Browser**에서 USD 파일 찾기
2. Viewport로 드래그 앤 드롭
3. 자동으로 Reference가 생성됨

---

## 6. Payload로 대규모 Scene 최적화

### 6.1 Payload 개념

**Payload**는 Reference와 유사하지만, **"필요할 때만 로딩"** 하는 지연 로딩(Lazy Loading) 기능입니다.

```usda
def Xform "LargeWarehouse" (
    prepend payload = @warehouse_v3.usda@</Warehouse>
)
{
}
```

| | Reference | Payload |
|------|-----------|---------|
| 로딩 시점 | 파일 열 때 즉시 로딩 | 필요할 때까지 로딩 안 함 |
| 메모리 | 항상 메모리에 유지 | 언로드 가능 |
| 사용처 | 필수 요소 (로봇 등) | 선택적 요소 (배경, 먼 환경) |
| Stage Window 표시 | 항상 보임 | 아이콘 차이 (클릭 시 로딩) |

### 6.2 Payload 토글

Stage Window에서 Payload Prim 옆에는 특별한 아이콘이 표시됩니다.
- **▶ (로딩 안 됨)**: 클릭하여 로딩
- **▼ (로딩됨)**: 현재 메모리에 있음

---

## 7. Layer 개념

### 7.1 Layer란?

Layer는 USD Scene의 **겹쳐지는 편집 레이어**입니다.
Photoshop의 Layer처럼 여러 Layer를 합성하여 최종 Scene을 만듭니다.

```
Layer 0 (Root Layer / Session Layer)
  └─ 실시간 편집 내용 (Play 중 변경사항)
Layer 1 (Current Layer)
  └─ 현재 Stage에 로드된 USD 파일
Layer 2 (Reference Layer)
  └─ Reference로 불러온 USD 파일들
```

### 7.2 Layer 확인

**Window > Layer Editor**에서 현재 Layer 구조를 확인할 수 있습니다.

**실습**:
1. Isaac Sim에서 USD 파일 열기
2. **Window > Layer Editor** 열기
3. 세션 Layer와 서브 Layer 확인
4. Layer별 편집 내용 확인

---

## 8. Python으로 USD Stage 조작

### 8.1 기본 API

```python
from pxr import Gf, Sdf, Usd, UsdGeom

# 현재 Stage 가져오기
stage = omni.usd.get_context().get_stage()

# Prim 찾기
prim = stage.GetPrimAtPath("/World/Cube")

# Attribute 읽기
translate_attr = prim.GetAttribute("xformOp:translate")
value = translate_attr.Get()  # Gf.Vec3d 반환
print(f"Position: {value}")

# Attribute 쓰기
translate_attr.Set(Gf.Vec3d(5.0, 0.0, 2.0))

# 새 Prim 생성
new_prim = stage.DefinePrim("/World/MyNewCube", "Cube")
```

### 8.2 Reference 추가 (Python)

```python
from pxr import Sdf

# Reference 추가
cube_prim = stage.GetPrimAtPath("/World/Cube")
ref_path = Sdf.Path("./hello_usd.usda</HelloScene/MyCube>")
cube_prim.GetReferences().AddReference(ref_path)
```

### 8.3 Payload 추가 (Python)

```python
payload = Sdf.Payload(
    Sdf.AssetPath("./warehouse_v3.usda"),
    Sdf.Path("/Warehouse")
)
prim.SetPayload(payload)
```

---

## 9. 종합 실습: USD로 Scene 설계하기

### 목표

텍스트 에디터로 `.usda`를 직접 작성하고 Isaac Sim에서 불러옵니다.

### 요구사항

```
/World
├── Ground (Plane)
├── Table (Cube): Z=1.0, 크기 (1.5, 0.8, 1.0), 갈색
├── RedBall (Sphere): 테이블 위 Z=1.6, 빨간색, 반지름 0.3
├── GreenBall (Sphere): Z=1.6, 초록색, 반지름 0.3  
├── Light (DistantLight): 강도 500
└── Label (Text): "My USD Scene" (Z=0.1)
```

### 정답 예시

파일: `~/isaac-step-curriculum/assets/scenes/my_usd_scene.usda`

```usda
#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    # 바닥
    def Plane "Ground"
    {
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    # 테이블
    def Cube "Table"
    {
        double3 xformOp:translate = (0, 0, 1.0)
        float3 xformOp:scale = (1.5, 0.8, 1.0)
        uniform token[] xformOpOrder = [
            "xformOp:translate",
            "xformOp:scale"
        ]
        color3f[] primvars:displayColor = [(0.55, 0.27, 0.07)]
    }

    # 빨간 공
    def Sphere "RedBall"
    {
        double3 xformOp:translate = (-0.3, 0, 1.6)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        color3f[] primvars:displayColor = [(1, 0, 0)]
    }

    # 초록 공
    def Sphere "GreenBall"
    {
        double3 xformOp:translate = (0.3, 0, 1.6)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        color3f[] primvars:displayColor = [(0, 1, 0)]
    }

    # 조명
    def DistantLight "Light"
    {
        float intensity = 500
        double3 xformOp:translate = (5, 5, 10)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
```

---

## 10. 문제 해결 (Troubleshooting)

### 문제 1: USD 파일을 열었는데 아무것도 보이지 않습니다.

**원인**: `defaultPrim`이 없거나 잘못된 경로
**해결**: `.usda` 파일 상단에 `defaultPrim = "World"`가 있는지 확인

### 문제 2: Reference가 로드되지 않습니다.

**확인 사항**:
- Reference 경로가 올바른가? (`@./path/to/file.usda@</Prim>`)
- 상대 경로는 USD 파일 위치 기준
- 절대 경로는 `@/absolute/path/file.usda@` 형식

### 문제 3: .usda 파일을 수정했는데 Isaac Sim에 반영되지 않습니다.

**해결**:
- **File > Reload** (Ctrl+R)로 현재 Scene 다시 로드
- 또는 USD 파일을 닫았다가 다시 열기

### 문제 4: Prim의 Transform이 이상하게 적용됩니다.

**원인**: `xformOpOrder`가 올바르지 않음
**해결**: Transform 순서는 항상 Translate → Rotate → Scale 순서로 정의
```usda
uniform token[] xformOpOrder = [
    "xformOp:translate",
    "xformOp:rotateXYZ", 
    "xformOp:scale"
]
```

### 문제 5: Git에서 .usda 파일의 변경 사항을 보니 너무 큽니다.

**해결**: 
- 불필요한 속성 제거 (예: 기본값은 생략 가능)
- `.usda`는 텍스트라 크기가 클 수 있음 → `.usdc`로 변환하여 관리
- 필요시 `git-lfs` 사용

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ USD 기본 개념 | Pixar 개발, Omniverse 핵심 포맷 |
| ✅ Prim/Attribute/Relationship | Scene의 3가지 기본 요소 |
| ✅ Specifier | def(정의), over(덮어쓰기), class(템플릿) |
| ✅ .usda 텍스트 편집 | 직접 작성하고 Isaac Sim에서 열기 |
| ✅ USD 포맷 | .usda(텍스트), .usdc(바이너리), .usd(컨테이너) |
| ✅ Reference | 외부 USD를 재사용 (읽기 전용) |
| ✅ Payload | 조건부 지연 로딩으로 성능 최적화 |
| ✅ Layer | 여러 편집 레이어 합성 |

### USD Data Flow

```
작성 단계:
  .usda (텍스트, 개발용)
    │
    ▼
  .usdc (바이너리, 배포용)
    │
    ▼
  Isaac Sim Stage (메모리)
    │
    ├── Reference (다른 USD 파일)
    ├── Payload (지연 로딩)
    ├── Layer (편집 레이어)
    └── Override (속성 덮어쓰기)
```

---

## 12. 다음 Step 예고

**Step 05 — Python Scripting 기초**에서는:
- Isaac Sim의 3가지 개발 워크플로우 (Extension vs Standalone)
- omni.isaac.core API 시작하기
- World, Stage, Prim 조작 기본
- USD Python API (pxr.Usd) vs Core API 관계
- 첫 번째 Extension 패키지 구조

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| USD 공식 문서 | https://openusd.org/release/index.html |
| USD 개념 가이드 | https://openusd.org/release/intro.html |
| Isaac Sim USD 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/usd_basics/index.html |
| USD Reference | https://docs.omniverse.nvidia.com/usd/latest/index.html |
| Stage Window | https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_stage.html |
| Composition (Ref/Payload) | https://openusd.org/release/glossary.html#composition |
