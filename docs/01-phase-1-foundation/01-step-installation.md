# Step 01 — Isaac Sim 5.1 설치하기

> **소요 시간**: 30분 ~ 2시간 (인터넷 속도에 따라 다름)
> **난이도**: ★☆☆☆☆ (초급)
> **선수 조건**: NVIDIA GPU (RTX 4080 이상 권장), Python 기본 지식

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. 시스템이 Isaac Sim 5.1을 실행할 수 있는지 확인한다
2. Isaac Sim 5.1을 **3가지 방법** 중 하나로 설치한다
3. 설치를 검증하고 최초 실행한다
4. 설치 과정에서 발생하는 일반적인 문제를 해결한다

---

## 1. 개요

Isaac Sim 5.1은 NVIDIA의 로봇 시뮬레이션 플랫폼으로, 다음과 같은 **5가지 설치 방법**을 제공합니다.

| 방법 | 난이도 | 용도 | 권장 대상 |
|------|--------|------|-----------|
| **Quick Install** (ZIP) | 쉬움 | 평가/데모 | Isaac Sim을 처음 접하는 사용자 |
| **pip 설치** | 보통 | 개발/확장 | Python 개발자, 기존 환경과 통합 필요 |
| **GitHub 빌드** | 어려움 | 소스 수정/커스텀 | 고급 사용자, 최신 기능 필요 |
| **Container (Docker)** | 보통 | 서버/클라우드 | Headless 환경, CI/CD |
| **Cloud — NVIDIA Brev** | 보통 | 클라우드 GPU | 로컬 GPU 부족 / 팀 협업 |

> **💡 이 커리큘럼에서는 `pip 설치`를 권장합니다.**
> 이유: Extension 개발, 패키지 관리, ROS2 연동이 가장 자유롭습니다.

---

## 2. 사전 요구사항 확인

### 2.1 하드웨어 요구사항

Isaac Sim 5.1 공식 요구사항:

| 구성 요소 | 최소 사양 | 권장 사양 | 이상적 |
|---------|----------|-----------|--------|
| **GPU** | GeForce RTX 4080 | GeForce RTX 5080 | RTX PRO 6000 Blackwell |
| **VRAM** | 16GB | 16GB | 48GB |
| **CPU** | Intel i7 7세대 / AMD R5 | Intel i7 9세대 / AMD R7 | Intel i9 / AMD R9 |
| **코어** | 4코어 | 8코어 | 16코어 |
| **RAM** | 32GB | 64GB | 64GB |
| **저장소** | 50GB SSD | 500GB SSD | 1TB NVMe SSD |
| **OS** | Ubuntu 22.04/24.04 또는 Windows 10/11 | 동일 | 동일 |
| **네트워크** | 인터넷 연결 필수 (에셋 다운로드) | — | — |

> **⚠️ 중요**: Isaac Sim은 **RT Core가 있는 GPU만 지원**합니다.
> - 지원: GeForce RTX 시리즈, RTX Ada, RTX PRO
> - **미지원**: A100, H100 (RT Core 없음), Jetson Orin Nano
> - **Jetson Orin Nano**는 Isaac Sim을 직접 실행할 수 없습니다. ROS2 Agent 노드로만 사용 가능합니다.

> **⚠️ VRAM 주의**: 16GB 미만 GPU는 복잡한 씬(16MP 이상 프레임)에서 실행이 어려울 수 있습니다.

### 2.2 GPU 확인 (본인 환경)

본인(사용자) 환경:
```
노트북: ASUS ROG Strix SCAR 16 (2025)
CPU: Intel Core Ultra 9
GPU: GeForce RTX 5090 Laptop (24GB VRAM)
RAM: 64GB
저장소: 4TB SSD
OS: Ubuntu 22.04 LTS
```

→ **이상적(Ideal)** 사양을 모두 충족합니다.

### 2.3 NVIDIA 드라이버 확인

터미널에서 다음 명령으로 GPU 드라이버를 확인하세요.

```bash
# GPU 상태 확인
nvidia-smi
```

출력 예시:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 580.95.05    Driver Version: 580.95.05    CUDA Version: 13.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
|===============================+======================+======================|
|   0  NVIDIA GeForce RTX 5090  | Off  | 00000000:01:00.0  On |  N/A |
|-------------------------------+----------------------+----------------------+
```

> **드라이버 버전**: Isaac Sim 5.1은 Linux에서 드라이버 버전 **580.65.06 이상**을 권장합니다.
> 이전 버전의 드라이버를 사용 중이라면 NVIDIA 공식 사이트에서 업데이트하세요.

### 2.4 Python 버전 확인

Isaac Sim 5.1은 **Python 3.11**이 필요합니다.

```bash
# Python 버전 확인
python3 --version
# 또는
python --version
```

Python 3.11이 설치되어 있지 않다면:

```bash
# Ubuntu 22.04에서 Python 3.11 설치
sudo apt update
sudo apt install -y python3.11 python3.11-venv python3.11-dev

# 또는 deadsnakes PPA 사용
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.11 python3.11-venv python3.11-dev
```

### 2.5 GLIBC 버전 확인 (Linux)

pip 설치를 위해 GLIBC 2.35+가 필요합니다.

```bash
ldd --version | head -n1
# 출력: ldd (Ubuntu GLIBC 2.35-0ubuntu3.8) 2.35  ← 2.35 이상이어야 함
```

Ubuntu 22.04는 기본적으로 GLIBC 2.35를 제공하므로 문제가 없습니다.

---

## 3. 설치 방법 A: Quick Install (ZIP) — 가장 쉬운 방법

> 평가/데모 목적에 적합합니다. 개발 환경 구축에는 pip 설치를 권장합니다.

### 3.1 다운로드

NVIDIA 공식 다운로드 링크에서 ZIP 파일을 받습니다.

| OS | 다운로드 링크 |
|----|-------------|
| Windows | https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-windows-x86_64.zip |
| Linux (x86_64) | https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip |
| Linux (aarch64) | https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-linux-aarch64.zip |

### 3.2 압축 풀기

```bash
# Linux
sudo mkdir -p /isaac-sim
cd /isaac-sim
sudo unzip ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

> **Windows**: ZIP 파일을 `C:\isaac-sim`에 압축 해제합니다.

### 3.3 사후 설정 및 실행

```bash
# Linux: 사후 설정 스크립트 실행
cd /isaac-sim
./post_install.sh

# App Selector 실행
./isaac-sim.selector.sh
```

```bat
:: Windows: App Selector 직접 실행
C:\isaac-sim\isaac-sim.selector.bat
```

### 3.4 설치 검증

Isaac Sim Application Selector 창에서 **Start**를 클릭합니다.
개발 환경이 열리면 다음을 확인합니다:

1. 상단 메뉴바가 보이는가?
2. Stage, Viewport, Property 패널이 보이는가?
3. **Create > Environment > Simple Room**이 실행되는가?

> **처음 실행 시 5~10분 정도 소요**될 수 있습니다. 빈 화면이 오래 보여도 기다리세요.

---

## 4. 설치 방법 B: pip 설치 (권장)

> 이 커리큘럼 전체에서 사용할 설치 방법입니다.
> Python 가상 환경을 사용하여 시스템과 격리된 상태로 설치합니다.

### 4.1 가상 환경 생성 및 활성화

```bash
# 프로젝트 디렉토리 생성
mkdir -p ~/isaac-step-curriculum
cd ~/isaac-step-curriculum

# Python 3.11 가상 환경 생성
python3.11 -m venv env_isaacsim

# 가상 환경 활성화
source env_isaacsim/bin/activate

# pip 업그레이드
pip install --upgrade pip
```

> **Windows**: (관리자 PowerShell)
> ```powershell
> mkdir C:\isaac-step-curriculum -Force
> cd C:\isaac-step-curriculum
> python3.11 -m venv env_isaacsim
> .\env_isaacsim\Scripts\Activate.ps1
> pip install --upgrade pip
> ```

> **Conda 사용 시**:
> ```bash
> conda create -n isaacsim python=3.11
> conda activate isaacsim
> pip install --upgrade pip
> ```

### 4.2 Isaac Sim pip 패키지 설치

**전체 설치 (권장, ~25GB)**:
```bash
pip install isaacsim[all,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com
```

**ROS2 번들만 설치**:
```bash
pip install isaacsim[ros2,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com
```

**최소 설치 (Core만)**:
```bash
pip install isaacsim==5.1.0 isaacsim-kernel==5.1.0 isaacsim-app==5.1.0 isaacsim-core==5.1.0 isaacsim-extscache-kit==5.1.0 --extra-index-url https://pypi.nvidia.com
```

> **설치 시간**: 인터넷 속도에 따라 10~60분 소요됩니다.
> **설치 용량**: 약 15~25GB (선택한 번들에 따라 다름)

### 4.3 EULA 동의

첫 실행 시 NVIDIA Omniverse EULA 동의가 필요합니다.
환경 변수로 미리 동의할 수 있습니다:

```bash
# EULA 자동 동의 (환경 변수 설정)
export OMNIVERSE_ACCEPT_EULA=YES
```

이 환경 변수를 `~/.bashrc`에 추가하면 영구 적용됩니다:
```bash
echo 'export OMNIVERSE_ACCEPT_EULA=YES' >> ~/.bashrc
```

### 4.4 설치 검증 — GUI 실행

```bash
# 가상 환경 활성화 상태에서
isaacsim
```

```python
# 또는 Python 코드로 검증
python -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
print('Isaac Sim 5.1 installed successfully!')
app.close()
"
```

> **Headless 검증**은 GUI 없이 설치가 제대로 되었는지만 확인합니다.
> 실제 GUI 사용을 위해서는 `isaacsim` 명령어를 직접 실행하세요.

### 4.5 설치 위치 확인

```bash
pip show isaacsim
# 설치 경로 확인 가능
```

---

## 5. 설치 방법 C: GitHub 빌드 (고급)

> 최신 개발 버전이 필요하거나 소스 수정이 필요할 때 사용합니다.

### 5.1 저장소 클론

```bash
git clone https://github.com/isaac-sim/IsaacSim.git isaacsim
cd isaacsim
git lfs install
git lfs pull
```

### 5.2 빌드

```bash
# Linux
./build.sh

# Windows
build.bat
```

### 5.3 실행

```bash
# Linux
cd _build/linux-x86_64/release
./isaac-sim.sh

# Windows
cd _build/windows-x86_64/release
isaac-sim.bat
```

---

## 6. 설치 방법 D: Container (Docker)

> 서버 환경, Headless 모드, CI/CD 파이프라인에 적합합니다.

### 6.1 Docker 이미지 Pull

```bash
# NGC에서 Isaac Sim Container 가져오기
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### 6.2 Container 실행

```bash
# Headless 모드 실행
docker run --rm --gpus all -it \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  bash -c "cd /isaac-sim && ./isaac-sim.sh --headless"
```

---

## 7. 설치 방법 E: Cloud — NVIDIA Brev (클라우드 배포)

> 로컬 GPU가 없거나 더 강력한 컴퓨팅이 필요할 때 사용합니다.
> **NVIDIA Brev**는 NVIDIA가 인수한 GPU 클라우드 플랫폼으로, Isaac Sim에 최적화된 환경을 제공합니다.

### 7.1 Brev 개요

NVIDIA Brev (구 Brev.dev)는 GPU 기반 클라우드 개발 플랫폼입니다.
NVIDIA에 인수된 후 Isaac Sim과의 공식 통합을 지원하며, WebRTC 스트리밍을 통해
클라우드 GPU에서 실행되는 Isaac Sim GUI를 로컬 브라우저로 바로 사용할 수 있습니다.

**Brev를 선택해야 하는 경우**:
- 로컬 GPU가 RTX 4080 미만이거나 RT Core가 없음
- Warehouse / Humanoid 등 대규모 시뮬레이션에 48GB VRAM 이상 필요
- 팀 협업을 위해 공유 가능한 클라우드 개발 환경이 필요
- 로컬 저장소가 부족함 (Isaac Sim + 에셋으로 100GB+ 필요)

### 7.2 GPU 인스턴스 종류 및 비용

Brev에서 Isaac Sim을 실행할 수 있는 주요 GPU 인스턴스:

| GPU | VRAM | Isaac Sim 용도 | 예상 시간당 비용 (2026) | 추천 |
|-----|------|----------------|------------------------|:----:|
| **L40S** | 48GB | Warehouse / Humanoid / Multi-Robot | ~$1.50–$2.50 (spot) | ⭐ |
| **A6000 (RTX A6000)** | 48GB | AI Worker / Deep Learning / Large-Scale | ~$2.00–$3.00 (spot) | |
| **RTX 4090** | 24GB | 교육용 / 단일 로봇 / 기본 ROS2 | ~$0.80–$1.50 (spot) | |
| **L40** | 48GB | 고급 시각화 / 다중 Viewport | ~$2.50–$3.50 (on-demand) | |
| **B200 (Blackwell)** | 180GB | 초대규모 환경 / Digital Twin | ~$2.25–$3.50 (spot) | |

> **💡 L40S 권장 이유**: 48GB VRAM으로 Warehouse(Step 21), Humanoid(Step 20), AI Worker(Step 22)까지
> 모두 커버 가능하고, Brev에서 가장 널리 사용되는 Isaac Sim 호환 GPU입니다.

**비용 절감 팁**:
- **Spot 인스턴스** 사용 시 50~70% 할인 (단, 최대 6시간 후 재시작 가능)
- **On-demand**는 Spot보다 비싸지만 중단 없이 장기 실행 가능
- **GPU Storage**는 100GB 기본 제공, 초과 시 GB당 추가 요금
- **Idle Timeout**을 30분으로 설정하면 미사용 시 자동 종료되어 비용 절감

### 7.3 Brev 계정 및 환경 설정

**Step 1: Brev 가입**

```bash
# 1. Brev 웹사이트 접속
#    https://www.brev.dev

# 2. GitHub / Google / Email 계정으로 회원가입

# 3. Brev CLI 설치 (선택사항, 웹 UI로도 가능)
pip install brev
```

**Step 2: Isaac Sim 인스턴스 생성 (웹 UI)**

1. [Brev Console](https://console.brev.dev) → **Create Instance**
2. **Name**: `isaac-sim-curriculum` (원하는 이름)
3. **GPU Type**: `L40S` 선택 (또는 필요에 따라 다른 GPU)
4. **Disk Size**: 최소 **100GB** (Isaac Sim + 에셋 = ~50GB, 여유분 포함)
5. **Region**: 가장 가까운 리전 선택 (도쿄/싱가포르/오레곤 등)
6. **Instance Type**: `Spot` (비용 절감) 또는 `On-Demand` (안정성)
7. **Advanced Options**:
   - **Startup Script**: 아래 스크립트 추가 (Isaac Sim 자동 설치)

**Step 3: Startup Script (인스턴스 시작 시 자동 실행)**

```bash
#!/bin/bash
# Isaac Sim 5.1 자동 설치 스크립트 (Brev Startup Script)

# EULA 자동 동의
export OMNIVERSE_ACCEPT_EULA=YES

# Python 3.11 가상 환경 생성
python3.11 -m venv /workspace/env_isaacsim

# Isaac Sim pip 설치
source /workspace/env_isaacsim/bin/activate
pip install --upgrade pip
pip install isaacsim[all,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com

# 설치 확인
python -c "from isaacsim import SimulationApp; app=SimulationApp(headless=True); print('Isaac Sim installed!'); app.close()" >> /workspace/install.log 2>&1

echo "Startup script completed!" >> /workspace/install.log
```

> **설치 시간**: L40S 기준 약 15~30분 소요 (인터넷 속도에 따라 다름)

**Step 4: SSH 접속 (CLI 사용 시)**

```bash
# Brev CLI로 접속
brev open isaac-sim-curriculum

# 또는 SSH 키를 등록했다면 직접 SSH 접속
ssh -L 8011:localhost:8011 brev@<instance-ip>
```

**Step 5: 포트 포워딩**

GUI 접속을 위해 로컬 포트를 Brev 인스턴스로 포워딩:

| 포트 | 용도 | 설명 |
|------|------|------|
| **8011** | WebRTC 스트리밍 | Isaac Sim GUI를 브라우저로 스트리밍 |
| **47911** | Isaac Sim 서버 | Python 코드에서 원격 연결 |
| **5900** | VNC (대체) | VNC 클라이언트로 GUI 접속 (WebRTC가 안 될 때) |

### 7.4 Isaac Sim GUI 접속 (WebRTC 스트리밍)

Brev의 가장 큰 장점은 **WebRTC 스트리밍**을 통해 클라우드 GPU의 Isaac Sim GUI를
로컬 브라우저에서 바로 사용할 수 있다는 점입니다. X11 포워딩보다 훨씬 빠릅니다.

**WebRTC 접속 방법**:

```bash
# 1. Brev 인스턴스에 접속 후 Isaac Sim 실행 (WebRTC 활성화)
cd /workspace/env_isaacsim
source bin/activate

# GUI 모드 실행 (WebRTC 자동 활성화)
isaacsim --enable-rtx-webrtc --webrtc-server-port 8011
```

```python
# 또는 Python 코드로 실행
python -c "
from isaacsim import SimulationApp

config = {
    'headless': False,
    'enable_rtx_webrtc': True,
    'webrtc_server_port': 8011
}
app = SimulationApp(config)
print('WebRTC streaming on port 8011')
# ... 시뮬레이션 코드 ...
app.close()
"
```

2. 로컬 브라우저에서 `http://localhost:8011` 접속
3. Isaac Sim GUI가 브라우저에 표시됨 (WebRTC 기반, 저지연)

> **WebRTC 요구사항**:
> - 최신 Chrome / Edge 브라우저 권장
> - 로컬 인터넷 속도 10Mbps 이상
> - WebRTC는 **localhost** 또는 **HTTPS** 도메인에서만 동작
> - 방화벽에서 UDP 8011 포트가 열려 있어야 함

### 7.5 ROS2 Bridge와 Brev

Brev 클라우드에서 실행되는 Isaac Sim과 로컬 ROS2 노드를 연결하려면
**SSH 터널링**으로 ROS2 통신을 포워딩해야 합니다.

```bash
# 로컬 머신에서 SSH 터널 생성 (ROS_DOMAIN_ID 일치 필수)
export ROS_DOMAIN_ID=42

# SSH 터널링으로 ROS2 통신 포워딩
ssh -L 1188:localhost:1188 \
    -L 1189:localhost:1189 \
    -L 1190:localhost:1190 \
    brev@<instance-ip>
```

> **⚠️ 클라우드 ROS2 주의사항**:
> - ROS2는 기본적으로 UDP Multicast를 사용하므로, SSH 터널링만으로는 완전한 통신이 어려움
> - 대안 1: **ROS2_WIFI** 환경 변수 설정으로 TCP로 강제 전환
> - 대안 2: **ROS2 Bridge**(Isaac Sim 내장)를 통해 WebSocket으로 통신
> - 대안 3: **Zenoh Bridge**(eclipse-zenoh/zenoh-plugin-ros2dds)로 WAN 환경에서 ROS2 통신
> - **권장**: 로컬에서 ROS2 노드를 실행하고, Isaac Sim만 Brev에서 실행하는 하이브리드 방식

### 7.6 Local vs Cloud 결정 가이드

| 상황 | 로컬 설치 | Brev Cloud |
|------|:---------:|:----------:|
| RTX 4090/5080 보유 && 교육용 | ⭐ 권장 | 선택 |
| RTX 5090 보유 && Phase 1~2 학습 | ⭐ 권장 | 선택 |
| VRAM 24GB 미만 || 복잡한 씬 | - | ⭐ 권장 |
| Warehouse (Step 21) / Humanoid (Step 20) | - | ⭐ 권장 (L40S) |
| 팀 협업 / 공유 환경 필요 | - | ⭐ 권장 |
| 장기 실행 (24h+) | ⭐ (전기료만) | 고비용 주의 |
| 첫 설치/평가 목적 | ⭐ (Quick Install) | - |

### 7.7 Brev 주의사항

1. **비용 관리**: 매시간 과금되므로 사용 후 반드시 **Stop Instance**
2. **데이터 유지**: 중지된 인스턴스의 디스크는 유지되나, 장기 미사용 시 삭제될 수 있음
3. **Storage 백업**: 중요한 프로젝트는 GitHub Push 또는 S3에 백업
4. **Region 선택**: 도쿄/싱가포르 리전이 아시아에서 가장 빠름 (한국 기준)
5. **조직 계정**: Enterprise 플랜은 팀 단위 결제 및 권한 관리 가능

### 7.8 Brev 설치 확인

```bash
# Brev 인스턴스에서 Isaac Sim 설치 확인
source /workspace/env_isaacsim/bin/activate
python -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
print('Isaac Sim 5.1 on Brev Cloud - Installed Successfully!')

import omni.usd
stage = omni.usd.get_context().get_stage()
print(f'Stage: {stage}')

# GPU 정보
import subprocess
result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader'],
                       capture_output=True, text=True)
print(f'GPU: {result.stdout.strip()}')

app.close()
"
```

> **예상 출력**:
> ```
> Isaac Sim 5.1 on Brev Cloud - Installed Successfully!
> Stage: <Usd.Stage>
> GPU: NVIDIA L40S, 48000 MiB
> ```

---

## 8. 설치 검증 스크립트

`code/phase-1/step01_verify_installation.py` 파일을 생성하여 설치 검증에 사용하세요.

```python
#!/usr/bin/env python3
"""
step01_verify_installation.py
Isaac Sim 5.1 설치 검증 스크립트
사용법: python step01_verify_installation.py
"""

import sys
import platform

def check_python_version():
    """Python 버전 확인 (3.11 필요)"""
    version = sys.version_info
    print(f"[1/4] Python 버전: {sys.version}")
    
    if version.major == 3 and version.minor == 11:
        print("  ✅ Python 3.11 - 적합")
        return True
    else:
        print(f"  ❌ Python 3.11 필요 (현재: {version.major}.{version.minor})")
        return False

def check_isaacsim_import():
    """Isaac Sim import 확인"""
    print("\n[2/4] Isaac Sim 패키지 확인...")
    try:
        from isaacsim import SimulationApp
        print("  ✅ isaacsim 패키지 import 성공")
        return True
    except ImportError as e:
        print(f"  ❌ Import 실패: {e}")
        print("  💡 pip install isaacsim 실행했는지 확인하세요")
        return False

def check_headless_creation():
    """Headless SimulationApp 생성 확인"""
    print("\n[3/4] SimulationApp 생성 (headless)...")
    try:
        from isaacsim import SimulationApp
        
        # Headless 앱 생성
        app = SimulationApp({'headless': True})
        print("  ✅ SimulationApp 생성 성공")
        
        # 기본 Stage 정보 출력
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if stage:
            print(f"  ✅ Stage 로드 완료: {stage}")
        else:
            print("  ⚠️  Stage 정보 없음")
        
        # 정리
        app.close()
        print("  ✅ SimulationApp 종료 완료")
        return True
        
    except Exception as e:
        print(f"  ❌ SimulationApp 생성 실패: {e}")
        return False

def check_gpu_info():
    """GPU 정보 확인"""
    print("\n[4/4] GPU 정보 확인...")
    try:
        import subprocess
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,memory.total,driver_version',
             '--format=csv,noheader'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            print(f"  ✅ GPU 감지됨:")
            for line in result.stdout.strip().split('\n'):
                print(f"     {line}")
            return True
        else:
            print("  ❌ nvidia-smi 실행 실패")
            return False
    except FileNotFoundError:
        print("  ❌ nvidia-smi를 찾을 수 없음 (NVIDIA 드라이버 미설치)")
        return False
    except Exception as e:
        print(f"  ⚠️  GPU 정보 확인 중 오류: {e}")
        return False

def main():
    print("=" * 60)
    print("Isaac Sim 5.1 설치 검증")
    print("=" * 60)
    print(f"시스템: {platform.system()} {platform.release()}")
    print(f"머신: {platform.machine()}")
    print()
    
    results = []
    results.append(check_python_version())
    results.append(check_isaacsim_import())
    results.append(check_headless_creation())
    results.append(check_gpu_info())
    
    print("\n" + "=" * 60)
    print("검증 결과 요약")
    print("=" * 60)
    all_pass = all(results)
    for i, r in enumerate(results):
        status = "✅ PASS" if r else "❌ FAIL"
        print(f"  Test {i+1}: {status}")
    
    if all_pass:
        print("\n🎉 모든 검증 통과! Isaac Sim 5.1 설치 완료!")
    else:
        print("\n⚠️  일부 검증이 실패했습니다. Troubleshooting 섹션을 확인하세요.")
    
    return 0 if all_pass else 1

if __name__ == "__main__":
    sys.exit(main())
```

### 스크립트 실행

```bash
# 가상 환경 활성화 상태에서
cd code/phase-1
python step01_verify_installation.py
```

---

## 9. 최초 실행 및 화면 구성 확인

### 9.1 GUI 최초 실행

```bash
# 가상 환경 활성화 상태에서
isaacsim
```

### 9.2 Isaac Sim UI 구성

최초 실행 시 다음과 같은 화면이 나타납니다:

```
┌──────────────────────────────────────────────────────────┐
│ Menu Bar  [File] [Edit] [Create] [Window] [Tools] ...   │
├──────────┬───────────────────────────────┬───────────────┤
│          │                               │   Property    │
│          │          Viewport             │   Panel       │
│ Content  │      (3D 화면)                │               │
│ Browser  │                               │   ────────    │
│          │                               │   Stage       │
│          │                               │   Window      │
├──────────┴───────────────────────────────┴───────────────┤
│            Toolbar  [Select] [Move] [Rotate] [Scale]     │
└──────────────────────────────────────────────────────────┘
```

| 구성 요소 | 설명 |
|-----------|------|
| **Menu Bar** | 최상단 메뉴. File, Edit, Create, Window, Tools 등 |
| **Viewport** | 3D 장면을 보는 메인 화면 |
| **Stage Window** | 현재 Scene의 모든 Prim 계층 구조 표시 |
| **Property Panel** | 선택한 Prim의 속성 표시/편집 |
| **Content Browser** | 에셋 브라우저. 로봇/환경/머티리얼 |
| **Toolbar** | 객체 조작 도구 (선택/이동/회전/크기) |

### 9.3 Quick Test — 로봇 추가해보기

설치 확인을 위해 간단히 로봇을 추가해봅니다:

1. Isaac Sim이 실행된 상태에서 상단 메뉴 **Create > Environment > Simple Room** 선택
2. **Create > Robots > Franka Emika Panda Arm** 선택
3. 화면 왼쪽 아래의 **Play** 버튼(▶) 클릭
4. Franka 로봇팔이 중력에 의해 아래로 떨어지는지 확인

---

## 10. 문제 해결 (Troubleshooting)

### 문제 1: `pip install` 중 `No matching distribution found for isaacsim`

**원인**: Python 버전 또는 GLIBC 버전 불일치

**해결**:
```bash
# Python 3.11 확인
python3 --version  # 3.11.x여야 함

# GLIBC 버전 확인 (Linux)
ldd --version | head -n1  # 2.35+ 필요

# --extra-index-url이 누락된 경우
pip install isaacsim==5.1.0 --extra-index-url https://pypi.nvidia.com
```

### 문제 2: `Isaac Sim` 실행 시 빈 화면/검은 화면

**원인**: GPU 드라이버 문제, 또는 첫 실행 시 캐싱 시간 소요

**해결**:
- 최대 **10분**까지 기다려 보세요 (첫 실행은 확장 프로그램 캐싱으로 오래 걸림)
- `nvidia-smi`로 GPU가 인식되는지 확인
- NVIDIA 드라이버를 최신 Studio 드라이버로 업데이트
- Wayland 대신 Xorg 사용 (Linux)

### 문제 3: `Failed to initialize graphics`

**원인**: GPU 호환성 문제

**해결**:
```bash
# GPU가 RT Core를 지원하는지 확인
nvidia-smi --query-gpu=name --format=csv,noheader

# Compatibility Checker 실행 (Isaac Sim 설치 폴더 내)
./isaac-sim.compatibility_check.sh  # Linux
isaac-sim.compatibility_check.bat   # Windows
```

### 문제 4: Windows에서 `enable long path` 오류

**해결**:
```powershell
# Windows에서 긴 경로 지원 활성화 (관리자 PowerShell)
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
    -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
```

### 문제 5: Docker 실행 시 GPU 접근 불가

**해결**:
```bash
# nvidia-container-toolkit 설치
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker

# --gpus all 플래그 확인
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 시스템 요구사항 확인 | GPU, RAM, Python 버전 |
| ✅ Quick Install 방법 | ZIP 다운로드 → 압축 풀기 → 실행 |
| ✅ pip 설치 방법 (권장) | Python venv → pip install → `isaacsim` |
| ✅ GitHub 빌드 방법 | 소스 클론 → 빌드 → 실행 |
| ✅ Container 방법 | Docker Pull → 실행 |
| ✅ Cloud (Brev) 방법 | 클라우드 GPU 인스턴스 → WebRTC 스트리밍 |
| ✅ 설치 검증 | 검증 스크립트로 4단계 확인 |
| ✅ 최초 실행 | GUI 확인, 로봇 추가 테스트 |
| ✅ 문제 해결 | 일반적인 오류 5가지 해결 방법 |

---

## 12. 다음 Step 예고

**Step 02 — GUI 기본 사용법**에서는:
- Isaac Sim의 UI 구성 요소를 상세히 탐색합니다
- Stage에 객체를 추가/조작합니다
- Material을 적용하고 Scene을 저장/불러옵니다
- 물리 시뮬레이션을 실행합니다

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim 5.1 공식 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ |
| Quick Install 가이드 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html |
| System Requirements | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html |
| Python pip 설치 가이드 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html |
| Isaac Sim GitHub | https://github.com/isaac-sim/IsaacSim |
| NVIDIA Driver 다운로드 | https://www.nvidia.com/Download/index.aspx |
| NVIDIA Brev Cloud Setup | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_advanced_cloud_setup_brev.html |
| Brev Console | https://console.brev.dev |
| Brev 공식 사이트 | https://www.brev.dev |
