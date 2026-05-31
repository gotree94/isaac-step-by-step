# Isaac-Step-Curriculum — 전체 커리큘럼 계획

> **버전**: 1.0 | **Isaac Sim**: 5.1.0 | **최종 업데이트**: 2026-05-31

---

## 📋 목차

1. [커리큘럼 설계 철학](#1-커리큘럼-설계-철학)
2. [전체 로드맵](#2-전체-로드맵)
3. [Phase 1: 기초 (Foundation)](#3-phase-1-기초-foundation)
4. [Phase 2: ROS2 통합](#4-phase-2-ros2-통합)
5. [Phase 3: 고급 (Advanced)](#5-phase-3-고급-advanced)
6. [최종 프로젝트](#6-최종-프로젝트)
7. [학습 가이드](#7-학습-가이드)
8. [참고 자료](#8-참고-자료)

---

## 1. 커리큘럼 설계 철학

### 1.1 배경

이 커리큘럼은 기존 IsaacSimKR 유튜브 채널의 튜토리얼을 기반으로,
다음 문제점을 해결하기 위해 설계되었습니다.

**기존 문제점 (IsaacSimKR v4 기준):**

| 문제 | 설명 | 해결 방안 |
|------|------|-----------|
| ❌ **버전 불일치** | Isaac Sim 4.x 기준, 5.1과 API 차이 | 전체 5.1 Experimental API 기준으로 재작성 |
| ❌ **번호 체계 혼란** | 02→05→06→04→08→11→10... 순서 불규칙 | 일관된 Step 번호 체계 |
| ❌ **비공개 콘텐츠** | 중간 단계 비공개로 연결성 부족 | 모든 Step 완전 공개 |
| ❌ **전방향 참조** | "이전 영상 보세요" 반복 → 학습자 혼란 | 각 Step에 필요한 컨텍스트 직접 제공 |
| ❌ **코드 부재** | 설명만 있고 완전한 코드 스크립트 부족 | Extension + Standalone Python 코드 모두 제공 |

### 1.2 설계 원칙

1. **Step-by-Step**: 각 Step은 독립적으로 완료 가능하되, 선수 관계를 명시
2. **따라하기 방식**: GUI 조작 → Python 코드 순으로 이중 학습 경로
3. **코드 우선**: 모든 예제에 완전한 실행 가능 코드 스크립트 제공
4. **중복 개념 재설명**: 필요한 컨텍스트는 매 Step에서 간략히 재설명
5. **실전 지향**: 최종 프로젝트에서 실제 로봇 시스템으로 확장

---

## 2. 전체 로드맵

```
Phase 1: 기초 (Foundation)
├── Step 01: Isaac Sim 5.1 설치
├── Step 02: GUI 기본 사용법
├── Step 03: OmniGraph 기초
├── Step 04: USD와 Stage 개념
├── Step 05: Python 스크립트 기초
├── Step 06: Core API #1 — Hello World
├── Step 07: Core API #2 — Hello Robot
├── Step 08: Core API #3 — Adding a Controller
├── Step 09: Core API #4 — Adding a Manipulator
└── Step 10: Core API #5 — Multiple Robots
                        │
                        ▼
Phase 2: ROS2 통합 (ROS2 Integration)
├── Step 11: ROS2 Humble/Jazzy 설치
├── Step 12: URDF Import — TurtleBot3
├── Step 13: Driving TurtleBot via ROS2
├── Step 14: ROS2 Camera + LiDAR
├── Step 15: ROS2 TF + Odometry
├── Step 16: ROS2 Navigation (Nav2)
├── Step 17: ROS2 Joint Control (Franka)
└── Step 18: Multiple Robot ROS2 Navigation
                        │
                        ▼
Phase 3: 고급 (Advanced)
├── Step 19: Extension 개발
├── Step 20: VSCode Debugging + Jupyter
├── Step 21: Synthetic Data (Replicator)
├── Step 22: Isaac Lab 기초 — RL
├── Step 23: Policy Deployment
├── Step 24: MoveIt 2 통합
├── Step 25: Cloud Deployment
└── Step 26: Digital Twin (Warehouse/Cortex)
                        │
                        ▼
Final Projects (실전)
├── AI Worker — FFW-BG2
├── AI Worker — FFW-SG2
├── AI Worker — FFW-LG2
├── OMX-AI (Follower/Leader)
├── RM-X52-TNM
├── HX5-D20-MLT
├── HX5-D20-MRT
└── AI Sapiens 통합
```

---

## 3. Phase 1: 기초 (Foundation)

> **목표**: Isaac Sim 5.1의 기본 개념, GUI 조작, Python 스크립팅을 익힌다.
> **소요**: 약 20시간 (Step당 1-2시간)
> **선수 조건**: Python 기본 문법, 기본적인 물리/로봇 공학 개념

### Step 01 — Isaac Sim 5.1 설치

| 항목 | 내용 |
|------|------|
| **학습 목표** | Isaac Sim 5.1을 노트북/데스크탑에 설치하고 최초 실행 |
| **핵심 내용** | pip 설치 / GitHub 빌드 / Container / App Selector |
| **실습** | 설치 후 GUI 실행 확인, Compatibility Checker 실행 |
| **코드** | 설치 검증 스크립트 (`verify_installation.py`) |
| **산출물** | `docs/01-phase-1-foundation/01-step-installation.md` |
| | `code/phase-1/step01_verify_installation.py` |

### Step 02 — GUI 기본 사용법

| 항목 | 내용 |
|------|------|
| **학습 목표** | Isaac Sim의 GUI 인터페이스 이해 및 기본 조작 |
| **핵심 내용** | Stage, Viewport, Property Panel, Menu Bar, Toolbar |
| **실습** | Stage에 Object 생성/조작, Material 적용, Save/Load |
| **코드** | 없음 (GUI 조작) |
| **산출물** | `docs/01-phase-1-foundation/02-step-gui-basics.md` |

### Step 03 — OmniGraph 기초

| 항목 | 내용 |
|------|------|
| **학습 목표** | Action Graph 개념 이해 및 노드 연결 방법 습득 |
| **핵심 내용** | OmniGraph 구조, 노드 타입, Execution/Data Flow, Physics |
| **실습** | 큐브 낙하 Action Graph 만들기 |
| **코드** | 없음 (GUI 기반 OmniGraph) |
| **산출물** | `docs/01-phase-1-foundation/03-step-omnigraph.md` |

### Step 04 — USD와 Stage 개념

| 항목 | 내용 |
|------|------|
| **학습 목표** | USD(Universal Scene Description)의 기본 개념 이해 |
| **핵심 내용** | Prim, Transform, Reference, Layer, Composition |
| **실습** | USD 파일 구조 분석, Stage Window 탐색 |
| **코드** | USD API 기본 사용법 |
| **산출물** | `docs/01-phase-1-foundation/04-step-usd-stage.md` |

### Step 05 — Python 스크립트 기초

| 항목 | 내용 |
|------|------|
| **학습 목표** | Isaac Sim의 Python 스크립팅 이해 (Extension vs Standalone) |
| **핵심 내용** | SimulationApp, Extension 구조, `python.sh`/`python.bat` |
| **실습** | `SimulationApp` 생성 → Physics Step → 종료 |
| **코드** | `step05_simulation_app_basics.py` |
| **산출물** | `docs/01-phase-1-foundation/05-step-python-scripting.md` |
| | `code/phase-1/step05_simulation_app_basics.py` |

### Step 06 — Core API #1: Hello World

| 항목 | 내용 |
|------|------|
| **학습 목표** | Core Experimental API를 사용해 큐브 추가 + 물리 시뮬레이션 |
| **핵심 내용** | `add_reference_to_stage`, RigidBody, Physics Step Loop |
| **실습** | 큐브가 중력에 낙하하는 시뮬레이션 |
| **코드** | `step06_hello_world.py` (Standalone + Extension 2버전) |
| **산출물** | `docs/01-phase-1-foundation/06-step-core-hello-world.md` |
| | `code/phase-1/step06_hello_world.py` |

### Step 07 — Core API #2: Hello Robot

| 항목 | 내용 |
|------|------|
| **학습 목표** | 로봇을 Stage에 추가하고 Wheel 제어 |
| **핵심 내용** | Jetbot USD 로드, `Articulation`, Wheel 속도 제어 |
| **실습** | Jetbot 추가 → Wheel 회전 → 이동 |
| **코드** | `step07_hello_robot.py` |
| **산출물** | `docs/01-phase-1-foundation/07-step-core-hello-robot.md` |
| | `code/phase-1/step07_hello_robot.py` |

### Step 08 — Core API #3: Adding a Controller

| 항목 | 내용 |
|------|------|
| **학습 목표** | Differential Controller를 사용한 Wheeled Robot 제어 |
| **핵심 내용** | `DifferentialController`, `ArticulationController`, 속도 명령 |
| **실습** | 로봇에 Controller 추가 → `cmd_vel` → 실제 이동 |
| **코드** | `step08_adding_controller.py` |
| **산출물** | `docs/01-phase-1-foundation/08-step-core-controller.md` |
| | `code/phase-1/step08_adding_controller.py` |

### Step 09 — Core API #4: Adding a Manipulator

| 항목 | 내용 |
|------|------|
| **학습 목표** | Franka 로봇팔을 추가하고 관절 제어 |
| **핵심 내용** | Manipulator USD 로드, `Articulation` 관절 제어, FK |
| **실습** | Franka 로드 → 관절 위치 명령 → 특정 포즈 |
| **코드** | `step09_adding_manipulator.py` |
| **산출물** | `docs/01-phase-1-foundation/09-step-core-manipulator.md` |
| | `code/phase-1/step09_adding_manipulator.py` |

### Step 10 — Core API #5: Multiple Robots

| 항목 | 내용 |
|------|------|
| **학습 목표** | 동일 Stage에 여러 로봇을 추가하고 개별 제어 |
| **핵심 내용** | Robot Cloning, 개별 Articulation 제어, Namespace |
| **실습** | Jetbot 3대 추가 → 각각 다른 속도로 주행 |
| **코드** | `step10_multiple_robots.py` |
| **산출물** | `docs/01-phase-1-foundation/10-step-core-multiple-robots.md` |
| | `code/phase-1/step10_multiple_robots.py` |

---

## 4. Phase 2: ROS2 통합

> **목표**: Isaac Sim과 ROS2를 연결하여 TurtleBot3를 완전히 제어한다.
> **소요**: 약 24시간 (Step당 3시간)
> **선수 조건**: Phase 1 완료, ROS2 기본 개념 (노드/토픽/서비스)

### Step 11 — ROS2 Humble/Jazzy 설치

| 항목 | 내용 |
|------|------|
| **학습 목표** | ROS2를 설치하고 Isaac Sim과 연동 환경 구성 |
| **핵심 내용** | ROS2 설치 (Ubuntu), WSL2 (Windows), Bridge, Docker |
| **실습** | ROS2 노드 ↔ Isaac Sim 통신 확인 (Talker/Listener) |
| **코드** | `step11_ros2_bridge_test.py` |
| **산출물** | `docs/02-phase-2-ros2/11-step-ros2-install.md` |

### Step 12 — URDF Import: TurtleBot3

| 항목 | 내용 |
|------|------|
| **학습 목표** | TurtleBot3 URDF를 Isaac Sim으로 가져오기 |
| **핵심 내용** | URDF Import, Moveable Base, Wheel Velocity Drive, Joint 튜닝 |
| **실습** | URDF Import → Wheel 튜닝 → Play → 낙하 확인 |
| **코드** | URDF Import (GUI) + Python Import 스크립트 |
| **산출물** | `docs/02-phase-2-ros2/12-step-urdf-turtlebot.md` |

### Step 13 — Driving TurtleBot via ROS2

| 항목 | 내용 |
|------|------|
| **학습 목표** | ROS2 `/cmd_vel` 메시지로 TurtleBot3 구동 |
| **핵심 내용** | ROS2 Bridge, Differential Controller, Action Graph, Teleop |
| **실습** | Action Graph 구성 → `/cmd_vel` Subscribe → 키보드 주행 |
| **코드** | `step13_turtlebot_drive.py` |
| **산출물** | `docs/02-phase-2-ros2/13-step-turtlebot-drive.md` |
| | `code/phase-2/step13_turtlebot_drive.py` |

### Step 14 — ROS2 Camera + LiDAR

| 항목 | 내용 |
|------|------|
| **학습 목표** | TurtleBot3에 카메라/라이다 센서 추가 및 ROS2 토픽 발행 |
| **핵심 내용** | Camera, RTX Lidar, PhysX Lidar, ROS2 Image/LaserScan |
| **실습** | 카메라 추가 → `/camera/image_raw` → LiDAR 추가 → `/scan` |
| **코드** | `step14_turtlebot_sensors.py` |
| **산출물** | `docs/02-phase-2-ros2/14-step-ros2-camera-lidar.md` |
| | `code/phase-2/step14_turtlebot_sensors.py` |

### Step 15 — ROS2 TF + Odometry

| 항목 | 내용 |
|------|------|
| **학습 목표** | Transform Tree와 Odometry 설정 |
| **핵심 내용** | TF Tree 구조, Wheel Odometry, `/odom`, `/tf` |
| **실습** | TurtleBot3 주행 + RViz2에서 TF/Odometry 시각화 |
| **코드** | `step15_turtlebot_tf_odom.py` |
| **산출물** | `docs/02-phase-2-ros2/15-step-ros2-tf-odom.md` |
| | `code/phase-2/step15_turtlebot_tf_odom.py` |

### Step 16 — ROS2 Navigation (Nav2)

| 항목 | 내용 |
|------|------|
| **학습 목표** | Nav2를 사용한 TurtleBot3 자율 주행 |
| **핵심 내용** | Occupancy Map, Nav2 Stack, AMCL, Path Planning |
| **실습** | 맵 생성 → Nav2 실행 → 목표 지점까지 자율 주행 |
| **코드** | `step16_turtlebot_nav2.py` |
| **산출물** | `docs/02-phase-2-ros2/16-step-ros2-navigation.md` |
| | `code/phase-2/step16_turtlebot_nav2.py` |

### Step 17 — ROS2 Joint Control (Franka)

| 항목 | 내용 |
|------|------|
| **학습 목표** | Franka 로봇팔 관절을 ROS2로 제어 |
| **핵심 내용** | Joint Trajectory Controller, ROS2 Action, FollowJointTrajectory |
| **실습** | Franka 로드 → Joint 목표 전송 → 특정 포즈 |
| **코드** | `step17_franka_joint_control.py` |
| **산출물** | `docs/02-phase-2-ros2/17-step-ros2-joint-control.md` |
| | `code/phase-2/step17_franka_joint_control.py` |

### Step 18 — Multiple Robot ROS2 Navigation

| 항목 | 내용 |
|------|------|
| **학습 목표** | 다중 TurtleBot3 각각의 네비게이션 |
| **핵심 내용** | Namespace, Multi-Robot Nav2, 충돌 회피 |
| **실습** | TurtleBot3 2대 → 각각 목표 지점 → 동시 주행 |
| **코드** | `step18_multi_robot_nav.py` |
| **산출물** | `docs/02-phase-2-ros2/18-step-multi-robot-nav.md` |
| | `code/phase-2/step18_multi_robot_nav.py` |

---

## 5. Phase 3: 고급 (Advanced)

> **목표**: Extension 개발, AI 학습, 클라우드 배포 등 실전 기술 습득
> **소요**: 약 32시간 (Step당 4시간)
> **선수 조건**: Phase 1, 2 완료

### Step 19 — Extension 개발

| 항목 | 내용 |
|------|------|
| **학습 목표** | 나만의 Isaac Sim Extension 만들기 |
| **핵심 내용** | Extension Template, extension.toml, UI Builder |
| **실습** | Custom Extension 생성 → Toolbar 버튼 → 기능 구현 |
| **코드** | `my_extension/` (전체 Extension 패키지) |
| **산출물** | `docs/03-phase-3-advanced/19-step-extension-dev.md` |

### Step 20 — VSCode Debugging + Jupyter

| 항목 | 내용 |
|------|------|
| **학습 목표** | VSCode 원격 디버깅 및 Jupyter Notebook 활용 |
| **핵심 내용** | VSCode Attach Debugger, Breakpoint, Jupyter Kernel |
| **실습** | VSCode에서 Isaac Sim Python 디버깅, Notebook 단계별 실행 |
| **산출물** | `docs/03-phase-3-advanced/20-step-debugging-jupyter.md` |

### Step 21 — Synthetic Data (Replicator)

| 항목 | 내용 |
|------|------|
| **학습 목표** | Replicator를 사용한 합성 데이터 생성 |
| **핵심 내용** | Replicator API, Randomization, SDG Pipeline |
| **실습** | TurtleBot3 + 배경 Randomization → 데이터셋 생성 |
| **코드** | `step21_replicator_sdg.py` |
| **산출물** | `docs/03-phase-3-advanced/21-step-replicator-sdg.md` |

### Step 22 — Isaac Lab 기초 — RL

| 항목 | 내용 |
|------|------|
| **학습 목표** | Isaac Lab에서 강화학습 환경 구축 및 학습 |
| **핵심 내용** | RL Environment, Reward Function, PPO Training |
| **실습** | Cartpole 환경 → PPO 학습 → Policy 저장 |
| **코드** | `step22_isaaclab_rl.py` |
| **산출물** | `docs/03-phase-3-advanced/22-step-isaaclab-rl.md` |

### Step 23 — Policy Deployment

| 항목 | 내용 |
|------|------|
| **학습 목표** | 학습된 RL Policy를 Isaac Sim에 배포 |
| **핵심 내용** | Policy Inference, ROS2 Bridge, Real Robot 연동 |
| **실습** | 학습된 Policy 로드 → Sim Inference → ROS2 Publish |
| **코드** | `step23_policy_deployment.py` |
| **산출물** | `docs/03-phase-3-advanced/23-step-policy-deployment.md` |

### Step 24 — MoveIt 2 통합

| 항목 | 내용 |
|------|------|
| **학습 목표** | MoveIt 2로 매니퓰레이터 모션 플래닝 |
| **핵심 내용** | MoveIt 2 Setup, Motion Planning, Collision Avoidance |
| **실습** | Franka + MoveIt 2 → Pick & Place |
| **산출물** | `docs/03-phase-3-advanced/24-step-moveit2.md` |

### Step 25 — Cloud Deployment

| 항목 | 내용 |
|------|------|
| **학습 목표** | 클라우드에서 Isaac Sim Headless 실행 |
| **핵심 내용** | AWS EC2 / Azure VM / GCP, WebRTC Streaming |
| **실습** | 클라우드 VM → Isaac Sim 설치 → Headless 실행 → WebRTC 접속 |
| **산출물** | `docs/03-phase-3-advanced/25-step-cloud-deployment.md` |

### Step 26 — Digital Twin (Warehouse/Cortex)

| 항목 | 내용 |
|------|------|
| **학습 목표** | Warehouse 물류 시뮬레이션 및 Cortex 다중 로봇 제어 |
| **핵심 내용** | Warehouse Creator, Cortex, cuOpt, Conveyor Belt |
| **실습** | Warehouse 환경 생성 → 다중 로봇 물류 시뮬레이션 |
| **산출물** | `docs/03-phase-3-advanced/26-step-digital-twin.md` |

---

## 6. 최종 프로젝트

Phase 1-3을 완료한 후 진행하는 **실전 종합 프로젝트**입니다.

### 프로젝트 목록

| # | 프로젝트 | 유형 | 관련 기술 | 난이도 |
|---|----------|------|-----------|--------|
| P1 | **FFW-BG2** | AI Worker (소형) | URDF, Navigation, Manipulation | ★★★ |
| P2 | **FFW-SG2** | AI Worker (중형) | Multi-Robot, Fleet Management | ★★★☆ |
| P3 | **FFW-LG2** | AI Worker (대형) | Heavy Lifting, Path Planning | ★★★★ |
| P4 | **OMX-AI (Follower/Leader)** | 자율 주행 | Follow-Me, Multi-Agent, Nav2 | ★★★★ |
| P5 | **RM-X52-TNM** | 매니퓰레이터 | MoveIt 2, Pick & Place, Vision | ★★★★ |
| P6 | **HX5-D20-MLT** | 휴머노이드 (이동) | Isaac Lab, RL Locomotion | ★★★★★ |
| P7 | **HX5-D20-MRT** | 휴머노이드 (조작) | RL + Manipulation, Dual Arm | ★★★★★ |
| P8 | **AI Sapiens** | 통합 플랫폼 | 모든 기술 융합 | ★★★★★ |

> **참고**: 각 프로젝트는 상세 로봇 스펙(URDF/MJCF, 센서 구성, 제어 방식)이 확정되는 대로
> 구체적인 요구사항과 구현 계획을 추가합니다.

---

## 7. 학습 가이드

### 7.1 학습 순서

```
권장 경로:
  Step 01 → 02 → 03 → ... → Step 10
       → Step 11 → 12 → ... → Step 18
       → Step 19 → 20 → ... → Step 26
       → Final Projects

선택 경로 (ROS2가 필요없는 경우):
  Step 01 → 02 → ... → Step 10
       → Step 19 → 20 → 21 → 22 → 23
       → Final Projects
```

### 7.2 Step 구성 형식

각 Step 문서는 다음 형식을 따릅니다:

```markdown
# Step N — 제목

## 학습 목표
## 선수 조건
## 개요 (이론)
## 실습 (단계별 따라하기)
### 1단계: ...
### 2단계: ...
### ...
## 코드 스크립트 설명
## 실행 확인
## 문제 해결 (Troubleshooting)
## 정리
## 다음 Step 예고
```

### 7.3 학습 팁

- **처음부터 완벽할 필요 없음**: 실행이 안 되면 Troubleshooting 섹션 확인
- **코드는 직접 타이핑**: 복붙보다 직접 입력하며 익숙해지기
- **GUI와 코드 병행**: GUI로 개념 이해 → 코드로 자동화 순서
- **질문은 GitHub Issues**: 각 Step의 Issues 탭에서 질문/토론

---

## 8. 참고 자료

### 공식 문서

| 자료 | 링크 |
|------|------|
| Isaac Sim 5.1 문서 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ |
| Isaac Sim 5.1 NVIDIA Brev Cloud Setup | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_advanced_cloud_setup_brev.html |
| Isaac Sim GitHub | https://github.com/isaac-sim/IsaacSim |
| Isaac Lab | https://isaac-sim.github.io/IsaacLab/ |
| ROS2 Humble | https://docs.ros.org/en/humble/ |
| TurtleBot3 e-Manual | https://emanual.robotis.com/docs/en/platform/turtlebot3/overview |

### 커뮤니티

| 자료 | 링크 |
|------|------|
| IsaacSimKR 유튜브 | https://www.youtube.com/@IsaacSimKR |
| IsaacSimKR 네이버 카페 | https://cafe.naver.com/isaacsimkr |
| NVIDIA Developer Forums | https://forums.developer.nvidia.com/c/omniverse/isaac-sim/ |

---

> **다음**: Step 01 — Isaac Sim 5.1 설치하기
> `docs/01-phase-1-foundation/01-step-installation.md` 로 이동
