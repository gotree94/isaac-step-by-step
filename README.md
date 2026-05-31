# Isaac-Step-Curriculum

> **Isaac Sim 5.1을 Step-by-Step으로 마스터하는 교육 커리큘럼**

## 개요

NVIDIA Isaac Sim 5.1 기반 로봇 시뮬레이션 교육 과정입니다. <br>
TurtleBot3 예제를 중심으로 설치부터 ROS2 통합, 고급 로봇 제어까지 <br>
**따라하기 방식**의 완전한 학습 경로를 제공합니다.

## 대상

- 로봇 시뮬레이션을 처음 시작하는 개발자
- ROS2를 Isaac Sim과 연동하려는 엔지니어
- AI 기반 로봇 제어를 학습하려는 연구자

## 학습 환경

| 항목 | 사양 |
|------|------|
| **Isaac Sim** | 5.1.0 |
| **OS (주)** | Ubuntu 22.04 LTS |
| **OS (보조)** | Windows 10/11 (WSL2) |
| **GPU (권장)** | RTX 4080 이상 (16GB VRAM) |
| **GPU (최소)** | RTX 3070 (8GB VRAM) |
| **RAM** | 32GB 이상 (64GB 권장) |
| **ROS2** | Humble (Ubuntu 22.04) / Jazzy (Ubuntu 24.04) |
| **Python** | 3.11 |
| **Jetson Orin Nano** | ROS2 Agent 노드 전용 (Isaac Sim 미지원) |

> **⚠️ 중요**: Isaac Sim은 RT Core가 있는 GPU만 지원합니다.
> RTX 계열 GPU가 필요하며, Jetson Orin Nano(8GB)는 Isaac Sim을 직접 실행할 수 없습니다.
> Orin Nano는 ROS2 브릿지를 통해 시뮬레이션 데이터를 송수신하는 **원격 Agent 노드**로 활용합니다.

## 클라우드 배포 (NVIDIA Brev)

로컬 GPU가 부족하다면 **NVIDIA Brev** 클라우드에서 Isaac Sim을 실행할 수 있습니다.

| 항목 | 내용 |
|------|------|
| **플랫폼** | NVIDIA Brev (brev.dev, NVIDIA 인수) |
| **권장 GPU** | L40S (48GB VRAM, ~$1.50–$2.50/hr spot) |
| **GUI 접속** | WebRTC 스트리밍 (브라우저 기반) |
| **적합한 경우** | GPU VRAM 24GB 미만 / Warehouse 시뮬레이션 / 팀 협업 |

> 자세한 설정 방법은 [Step 01 설치 문서](docs/01-phase-1-foundation/01-step-installation.md)의
> **"7. 설치 방법 E: Cloud — NVIDIA Brev"** 섹션을 참고하세요.

## 커리큘럼 구조

```
Phase 1: 기초 (Foundation)      — 10 Steps
Phase 2: ROS2 통합              — 8 Steps
Phase 3: 고급 (Advanced)        — 8 Steps
Final Projects                  — 8개 실전 프로젝트
```

## 프로젝트 구조

```
isaac-step-curriculum/
├── README.md                          # 이 파일
├── docs/                              # 모든 문서 (Markdown)
│   ├── 00-curriculum-overview.md      # 커리큘럼 전체 계획
│   ├── 01-phase-1-foundation/         # Phase 1 모듈
│   │   ├── 01-step-installation.md
│   │   ├── 02-step-gui-basics.md
│   │   └── ...
│   ├── 02-phase-2-ros2/              # Phase 2 모듈
│   │   ├── 11-step-ros2-install.md
│   │   └── ...
│   └── 03-phase-3-advanced/          # Phase 3 모듈
│       ├── 19-step-extension-dev.md
│       └── ...
├── code/                              # 모든 예제 코드 스크립트
│   ├── phase-1/
│   ├── phase-2/
│   └── phase-3/
├── assets/                            # 이미지, 다이어그램 등
│   └── images/
├── final-projects/                    # 최종 프로젝트 문서
│   ├── 01-ffw-bg2.md
│   └── ...
└── .references/                       # 참고 자료
```

## 라이선스

본 커리큘럼은 학습 목적으로 제공됩니다.
NVIDIA Isaac Sim은 NVIDIA의 라이선스 정책을 따릅니다.
