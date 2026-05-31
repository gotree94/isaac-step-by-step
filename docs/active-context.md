# Active Context — Isaac Step Curriculum

> **최종 갱신**: 2026-05-31
> **상태**: ✅ 26 Step 완료 + 8개 Final Projects

---

## 프로젝트 개요

**Isaac-Step-Curriculum**: NVIDIA Isaac Sim 5.1.0 교육 커리큘럼 (Foundation → ROS2 → Advanced)

## 디렉토리 구조

```
c:\isaac-step-curriculum\
├── README.md                         ✅
├── active-context.md                 ✅ (현재 파일)
│
├── docs/
│   ├── 00-curriculum-overview.md     ✅
│   ├── 01-phase-1-foundation/        ✅ 10개 문서
│   ├── 02-phase-2-ros2/             ✅ 8개 문서
│   └── 03-phase-3-advanced/          ✅ 8개 문서
│
├── code/
│   ├── phase-1/                      ✅ 10개 스크립트
│   ├── phase-2/                      ✅ 8개 스크립트
│   └── phase-3/                      ✅ 8개 스크립트
│
├── final-projects/                   ✅ 8개 프로젝트
│
├── assets/                           ✅ (준비)
├── scripts/                          ✅ (준비)
└── config/                           ✅ (준비)
```

## 완료된 작업

### Phase 1 — Foundation (Steps 01-10) ✅
| Step | 제목 | 문서 | 코드 |
|------|------|------|------|
| 01 | Install & Setup | ✅ | ✅ |
| 02 | Interface Overview | ✅ | ✅ |
| 03 | USD Basics | ✅ | ✅ |
| 04 | Robot Loading | ✅ | ✅ |
| 05 | Sensors | ✅ | ✅ |
| 06 | Physics | ✅ | ✅ |
| 07 | Articulations | ✅ | ✅ |
| 08 | Action Graph | ✅ | ✅ |
| 09 | OmniGraph | ✅ | ✅ |
| 10 | Scene Composition | ✅ | ✅ |

### Phase 2 — ROS2 Integration (Steps 11-18) ✅
| Step | 제목 | 문서 | 코드 |
|------|------|------|------|
| 11 | ROS2 Bridge | ✅ | ✅ |
| 12 | ROS2 Teleop | ✅ | ✅ |
| 13 | ROS2 SLAM | ✅ | ✅ |
| 14 | ROS2 Nav2 | ✅ | ✅ |
| 15 | ROS2 MoveIt | ✅ | ✅ |
| 16 | ROS2 Multi-Robot | ✅ | ✅ |
| 17 | Synthetic Data | ✅ | ✅ |
| 18 | Performance | ✅ | ✅ |

### Phase 3 — Advanced Robotics (Steps 19-26) ✅
| Step | 제목 | 문서 | 코드 |
|------|------|------|------|
| 19 | Digital Twin | ✅ | ✅ |
| 20 | Humanoid Robot | ✅ | ✅ |
| 21 | Warehouse Automation | ✅ | ✅ |
| 22 | AI Worker | ✅ | ✅ |
| 23 | Deep Learning | ✅ | ✅ |
| 24 | ROS2 Advanced | ✅ | ✅ |
| 25 | Large-Scale | ✅ | ✅ |
| 26 | Final Integration | ✅ | ✅ |

### Final Projects ✅
| # | 프로젝트 | 난이도 | 시간 |
|---|----------|--------|------|
| 1 | 휴머노이드 창고 작업자 | ★★★★ | 8h |
| 2 | AI Worker Cobot Cell | ★★★★ | 8h |
| 3 | 자율 주행 Fleet | ★★★★★ | 12h |
| 4 | Digital Twin Factory | ★★★★★ | 12h |
| 5 | Deep RL Robot 학습 | ★★★★★ | 16h |
| 6 | Human-Robot 협업 | ★★★★★ | 10h |
| 7 | 대규모 물류 Warehouse | ★★★★★ | 16h |
| 8 | 통합 Smart Factory | ★★★★★ | 24h |

## 핵심 설정

### Isaac Sim 5.1.0
- **설치 경로**: `~/isaac-sim` (로컬) / `/workspace/env_isaacsim` (Brev Cloud)
- **실행**: `./python.sh script.py` (로컬) / `source env_isaacsim/bin/activate && isaacsim` (Brev)
- **Physics DT**: 1/60.0 (기본)
- **Renderer**: RayTracedLighting
- **Cloud GPU**: L40S 48GB 권장 (Brev)

### Cloud — NVIDIA Brev
- **플랫폼**: NVIDIA Brev (brev.dev, NVIDIA 인수)
- **권장 GPU**: L40S (48GB VRAM, ~$1.50–$2.50/hr spot)
- **GUI 접속**: WebRTC 스트리밍 (localhost:8011)
- **설치 방식**: pip install isaacsim (Startup Script 자동화)
- **용도**: 로컬 GPU 부족 시 / Warehouse / Humanoid / 팀 협업
- **비용 절감**: Spot 인스턴스 + Idle Timeout 30분 설정

### ROS2 Humble
- **ROS_DOMAIN_ID**: 0 (기본)
- **Bridge**: Internal (Action Graph)
- **Multi-Robot Namespace**: /tb1, /tb2, ...
- **Fast DDS**: 필요 시 fastdds.xml 설정

### 주요 USD 경로
- TurtleBot3: `/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd`
- Franka: `/Isaac/Robots/Franka/franka_alt_fingers.usd`
- GR1: `/Isaac/Robots/GR1/gr1.usd`
- H1: `/Isaac/Robots/H1/h1.usd`

## 주요 결정 사항

1. **설치**: pip 설치 권장 (ROS2 연동 자유도)
2. **API**: omni.isaac.core + pxr.Usd 혼용
3. **ROS2**: Internal Bridge (별도 설치 불필요)
4. **OS**: Ubuntu 22.04 + ROS2 Humble 기준
5. **교육 경로**: TurtleBot3 → Franka → GR1/H1

## 검증 필요 사항

- [ ] Phase 2 전체 코드 ROS2 환경 검증
- [ ] Phase 3 고급 스크립트 실 실행 검증
- [ ] Step 12 Teleop 오타 검토 (`ROS2SubscribeJointTrajectory`)
- [ ] Final Project Scenario 상세 설계
- [ ] Makefile/Launch Scripts 최종 생성
- [ ] 오타 및 문법 검토 (전체 문서)

## 학습 경로

| 경로 | Steps | 목표 |
|------|-------|------|
| **ROS2 Robotics** | 11→12→13→14→15→16 | ROS2 기반 로봇 제어 |
| **AI Robotics** | 17→22→23→24 | AI/ML 기반 로봇 |
| **Industrial** | 19→20→21→25 | 산업용 시뮬레이션 |
| **Full Stack** | 01→26 (전체) | Isaac Sim 전문가 |

## 사용 기술

- **Simulator**: NVIDIA Isaac Sim 5.1.0
- **Physics**: PhysX 5 (GPU)
- **ROS2**: Humble Hawksbill
- **AI/ML**: PyTorch, TensorRT, Ultralytics YOLO
- **USD**: Pixar USD 23.11
- **Language**: Python 3.10+
- **GPU**: NVIDIA RTX (Ampere+)

---

*이 문서는 Isaac Step Curriculum의 최종 Context를 기록합니다.*
