# Step 11 — ROS2 Bridge 설치 및 설정

> **소요 시간**: 60분
> **난이도**: ★★★☆☆ (중급)
> **선수 조건**: Step 01 (Isaac Sim 설치), ROS2 기본 개념

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim과 ROS2 간의 **Bridge 아키텍처**를 이해한다
2. **ROS2 Humble/Jazzy**를 설치하고 환경을 설정한다
3. Isaac Sim의 **ROS2 Bridge를 활성화**한다 (3가지 방법)
4. **Internal/External** ROS2 라이브러리 옵션을 이해한다
5. **ROS_DOMAIN_ID**와 **FastDDS** 환경을 설정한다
6. Isaac Sim ↔ ROS2 간 **기본 Pub/Sub 통신을 테스트**한다
7. **ROS2 Clock** 발행을 확인한다

---

## 1. ROS2 Bridge 아키텍처

### 1.1 Bridge란?

Isaac Sim ROS2 Bridge는 **Isaac Sim 내부의 시뮬레이션 데이터를 ROS2 메시지로 변환**하여 외부 ROS2 노드와 통신할 수 있게 해주는 Extension입니다.

```
┌──────────────────────┐        DDS (FastDDS/Cyclone)        ┌──────────────────────┐
│     Isaac Sim        │ ◄──────────────────────────────►    │   ROS2 Node          │
│                      │     ROS_DOMAIN_ID 공유              │                      │
│  ┌────────────────┐  │                                     │  ros2 topic list     │
│  │ ROS2 Bridge    │  │                                     │  ros2 topic echo     │
│  │ Extension      │──┼─────────────────────────────────────┼──►                   │
│  │                │  │                                     │  ros2 control        │
│  │  OmniGraph     │  │                                     │  rviz2               │
│  │  ROS2 Nodes    │──┼─────────────────────────────────────┼──►                   │
│  └────────────────┘  │                                     │                      │
│                      │                                     │  ┌────────────────┐  │
│  ┌────────────────┐  │                                     │  │ ROS2 Node      │  │
│  │ Sim Data       │  │                                     │  │ (Python/C++)   │  │
│  │ (Physics/RGB/  │  │                                     │  └────────────────┘  │
│  │  Depth/LiDAR)  │  │                                     │                      │
│  └────────────────┘  │                                     │                      │
└──────────────────────┘                                     └──────────────────────┘
```

### 1.2 지원 ROS2 배포판

| 플랫폼 | ROS2 배포판 | 비고 |
|--------|------------|------|
| Ubuntu 22.04 | **Humble** (권장) | Python 3.10 (외부), Isaac Sim은 3.11 사용 |
| Ubuntu 22.04 | Jazzy | 소스 빌드 필요 |
| Ubuntu 24.04 | **Jazzy** (권장) | 기본 설치 |
| Windows 10/11 | Humble (WSL2) | WSL2를 통해 설치 |
| pip 설치 환경 | Humble / Jazzy | Internal 라이브러리 자동 사용 |

### 1.3 Internal vs External ROS2 라이브러리

| 옵션 | 설명 | 사용 시기 |
|------|------|-----------|
| **Internal** | Isaac Sim에 내장된 ROS2 라이브러리 사용 | 시스템에 ROS2가 설치되지 않은 경우 |
| **External** | 시스템에 설치된 ROS2 라이브러리 사용 | 기존 ROS2 환경과 통합 시 |

**Internal 라이브러리 활성화 방법:**
```bash
# App Selector에서 "Use internal ROS2 libraries" 체크
# 또는 환경 변수 설정
export ISAAC_SIM_USE_INTERNAL_ROS2=true
```

**External 라이브러리 활성화 방법:**
```bash
# ROS2 환경을 먼저 source한 후 Isaac Sim 실행
source /opt/ros/humble/setup.bash
isaacsim

# 또는 App Selector에서 "Use internal ROS2 libraries" 체크 해제
```

---

## 2. ROS2 설치

### 2.1 Ubuntu 22.04 + ROS2 Humble

```bash
# 1. Locale 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. ROS2 GPG 키 추가
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. ROS2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. ROS2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# 5. 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. 확인
ros2 --version
```

### 2.2 Ubuntu 24.04 + ROS2 Jazzy

```bash
# 위와 동일한 과정, 패키지명만 ros-jazzy-desktop으로 변경
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 2.3 pip 설치 환경 (Internal 라이브러리)

pip로 설치한 Isaac Sim은 **ROS2 라이브러리를 내장**하고 있습니다.
별도 ROS2 설치 없이 Bridge를 사용할 수 있습니다.

```bash
# ROS2 설치 없이 바로 Bridge 활성화 가능
# 단, 외부 ROS2 노드와 통신하려면 외부 시스템에 ROS2가 필요
```

---

## 3. ROS2 Bridge 활성화

### 3.1 방법 1: App Selector (권장)

```bash
cd ~/isaacsim  # 설치 경로
./isaac-sim.selector.sh
```

App Selector 창에서:
1. **ROS2 bridge extension** → 사용할 브리지 선택 (권장: `isaacsim.ros2.bridge`)
2. **Use internal ROS2 libraries** → 체크 (권장)
3. OK 클릭 → 자동으로 Bridge가 활성화된 상태로 Isaac Sim 실행

### 3.2 방법 2: 명령줄 인자

```bash
# Internal ROS2 라이브러리 사용
isaacsim --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge

# 또는 pip 설치 환경
source ~/isaacsim_env/bin/activate
isaacsim --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge
```

### 3.3 방법 3: Extension Manager (런타임)

Isaac Sim 실행 후:
1. **Window > Extensions** (Ctrl+Shift+X)
2. 검색: `ROS2 Bridge`
3. **omni.isaac.ros2_bridge** 찾아서 **Enable** 토글

> **⚠️ 주의**: Extension Manager에서 활성화하는 방법은 App Selector보다
> 일부 노드가 늦게 로드될 수 있습니다. App Selector 방식을 권장합니다.

---

## 4. ROS2 환경 설정

### 4.1 ROS_DOMAIN_ID

같은 네트워크에서 여러 ROS2 시스템이 충돌하지 않도록 **Domain ID**를 설정합니다.

```bash
# Isaac Sim 실행 전 (또는 동일 터미널에서)
export ROS_DOMAIN_ID=0

# ROS2 노드에서도 동일한 Domain ID 사용
export ROS_DOMAIN_ID=0
```

> **기본값**: `ROS_DOMAIN_ID=0` (범위: 0~101)
> - 여러 팀이 같은 네트워크를 사용하면 각자 다른 ID 할당
> - Isaac Sim과 ROS2 노드는 반드시 **동일한 Domain ID** 사용

### 4.2 FastDDS 설정

Isaac Sim은 기본적으로 **FastDDS**를 DDS 미들웨어로 사용합니다.

```bash
# FastDDS 환경 변수 (선택 사항)
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profile.xml

# 특정 네트워크 인터페이스 바인딩 (멀티 NIC 환경)
export ROS_LOCALHOST_ONLY=1  # localhost로 제한 (보안/안정성)
```

### 4.3 Cyclone DDS 사용 (선택 사항)

```bash
# Isaac Sim에서 Cyclone DDS 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS2 노드도 동일하게 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 5. 기본 Pub/Sub 테스트

### 5.1 시나리오: Isaac Sim이 Clock 발행 → ROS2 노드가 수신

**터미널 1: Isaac Sim 실행 (Bridge 활성화)**

```bash
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./isaac-sim.selector.sh
# App Selector에서 ROS2 Bridge + Internal 라이브러리 선택
```

**Isaac Sim에서 Clock 확인:**

1. Isaac Sim이 실행되면 자동으로 `/clock` 토픽 발행 시작
2. 시뮬레이션 Play(▶) 시 Clock 메시지 발생

**터미널 2: ROS2 토픽 확인**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# 토픽 목록 확인
ros2 topic list

# 예상 출력:
# /clock
# /parameter_events
# /rosout

# /clock 토픽 내용 확인
ros2 topic echo /clock
# 예상 출력:
# ---
# clock:
#   sec: 0
#   nanosec: 100000000
# ---
```

### 5.2 Clock 메시지 구조

```
$ ros2 topic info /clock
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Subscriber count: 0

$ ros2 interface show rosgraph_msgs/msg/Clock
# rostime
builtin_interfaces/Time clock
        int32 sec
        uint32 nanosec
```

### 5.3 직접 토픽 발행 테스트 (Python)

**Isaac Sim Script Editor에서 실행:**

```python
import rclpy
from std_msgs.msg import String

# rclpy 초기화 (이미 Bridge에서 초기화됨)
if not rclpy.ok():
    rclpy.init()

# 노드 생성
node = rclpy.create_node('isaac_test_publisher')
publisher = node.create_publisher(String, '/isaac_test', 10)

# 메시지 발행
msg = String()
msg.data = "Hello from Isaac Sim!"
publisher.publish(msg)
print(f"Published: {msg.data}")

node.destroy_node()
```

**터미널 2에서 수신 확인:**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /isaac_test
```

---

## 6. ROS2 OmniGraph 노드

### 6.1 주요 ROS2 노드 목록

Bridge가 활성화되면 OmniGraph에서 다음 ROS2 노드를 사용할 수 있습니다:

| 노드 이름 | 기능 |
|-----------|------|
| **ROS2 Context** | ROS2 컨텍스트 설정 (Domain ID 등) |
| **Publish Clock** | `/clock` 토픽 발행 |
| **Publish TF** | TF 트리 발행 |
| **Publish JointState** | Joint State 발행 |
| **Subscribe JointState** | Joint State 구독 |
| **Camera Helper** | RGB/Depth 이미지 발행 |
| **Lidar Helper** | LaserScan 발행 |
| **IMU Helper** | IMU 데이터 발행 |
| **Odometry Helper** | Odometry 발행 |

### 6.2 기본 Clock 발행 Graph

Isaac Sim 실행 시 **자동 생성**되는 Graph:

```
Action Graph: /ROS2_Clock
├── On Playback Tick → Publish Clock (topic: /clock)
└── ROS2 Context (domain_id: 0)
```

### 6.3 Python으로 ROS2 Graph 생성

```python
import omni.graph.core as og
from pxr import Sdf

graph_config = {
    "graph_path": "/ActionGraph/ROSTest",
    "evaluator_name": "execution",
}

og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:domain_id", 0),
            ("PublishClock.inputs:topicName", "/sim_clock"),
        ],
    },
)
print("ROS2 Clock Publisher Graph created!")
```

---

## 7. 실습: Isaac Sim ↔ ROS2 통신 확인

### 7.1 전체 절차

```bash
# Step 1: ROS2 터미널 준비
# 터미널 1:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list  # Isaac Sim 실행 후 확인

# Step 2: Isaac Sim 실행 (다른 터미널)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./isaac-sim.selector.sh  # ROS2 Bridge 활성화

# Step 3: Isaac Sim에서 Play (▶)
# /clock 토픽이 활성화되는지 확인

# Step 4: ROS2 subscriber로 데이터 확인
# 터미널 1:
ros2 topic hz /clock  # 발행 빈도 확인

# Step 5: ROS2 publisher로 Isaac Sim 제어
# 터미널 1:
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
# → TurtleBot3가 제어되는지 확인 (Step 12에서 상세 학습)
```

### 7.2 확인 명령어 요약

```bash
# 토픽 목록
ros2 topic list

# 토픽 정보 (타입, Publisher/Subscriber 수)
ros2 topic info /clock

# 토픽 내용 출력
ros2 topic echo /clock

# 토픽 발행 빈도
ros2 topic hz /clock

# 노드 목록
ros2 node list

# 노드 정보
ros2 node info /isaac_sim

# 서비스 목록
ros2 service list
```

---

## 8. 문제 해결 (Troubleshooting)

### 문제 1: ROS2 토픽이 보이지 않습니다.

**확인 사항**:
```bash
# 1. ROS_DOMAIN_ID가 일치하는가?
echo $ROS_DOMAIN_ID  # Isaac Sim 터미널과 ROS2 터미널에서 동일한 값?

# 2. Bridge가 활성화되어 있는가?
# Isaac Sim > Window > Extensions > omni.isaac.ros2_bridge 검색

# 3. 네트워크 방화벽 확인
sudo ufw status
# DDS는 UDP 멀티캐스트 사용 (포트 7400~7500)
```

### 문제 2: ImportError: No module named 'rclpy'

**원인**: Isaac Sim 내부 Python에서 rclpy를 찾을 수 없음
**해결**:
```bash
# Internal 라이브러리 사용 확인
# App Selector에서 "Use internal ROS2 libraries" 체크

# 또는 pip 설치 환경의 경우 ROS2 패키지 추가 설치
pip install "isaacsim[all,extscache,ros2]"
```

### 문제 3: FastDDS 통신 오류

**증상**: `Failed to find publisher` 또는 Participant 오류
**해결**:
```bash
# 1. localhost로 제한 (멀티 캐스트 문제 회피)
export ROS_LOCALHOST_ONLY=1

# 2. Cyclone DDS로 변경
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. 네트워크 인터페이스 지정
export FASTRTPS_DEFAULT_PROFILES_FILE=<profile_xml>
```

### 문제 4: /clock 토픽이 발행되지 않습니다.

**원인**: Isaac Sim이 Play 상태가 아님
**해결**: Viewport Toolbar에서 **Play (▶)** 버튼 클릭

### 문제 5: Windows + WSL2에서 통신이 안 됩니다.

**해결**: 
```bash
# WSL2에서 Windows 호스트의 Isaac Sim과 통신
# 1. WSL2 IP 확인
ip addr show eth0

# 2. Windows 방화벽에서 UDP 포트 허용
# 3. 동일한 ROS_DOMAIN_ID 사용

# 또는 Docker 네트워크 브리지 설정 참고
```

---

## 9. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Bridge 아키텍처 | Isaac Sim ↔ DDS ↔ ROS2 Node 구조 |
| ✅ ROS2 설치 | Humble (22.04) / Jazzy (24.04) |
| ✅ Bridge 활성화 | App Selector / CLI / Extension Manager |
| ✅ Internal/External | 내장 라이브러리 vs 시스템 ROS2 |
| ✅ ROS_DOMAIN_ID | 0~101, Isaac Sim과 노드가 동일해야 함 |
| ✅ 기본 Pub/Test | /clock 토픽 확인, 직접 발행 테스트 |
| ✅ ROS2 OmniGraph | PublishClock, Context 등 노드 |

### 데이터 흐름 요약

```
Isaac Sim (Play 상태)
    │
    ├── OmniGraph: PublishClock
    │       ↓
    │   /clock (rosgraph_msgs/Clock) ──→ ros2 topic echo /clock
    │
    ├── OmniGraph: PublishTF
    │       ↓
    │   /tf (tf2_msgs/TFMessage) ──→ rviz2에서 로봇 표시
    │
    ├── OmniGraph: PublishJointState
    │       ↓
    │   /joint_states (sensor_msgs/JointState)
    │
    └── ROS2 Subscribe (예: /cmd_vel)
                ↑
        ros2 topic pub /cmd_vel ... ──→ TurtleBot3 제어
```

---

## 10. 다음 Step 예고

**Step 12 — ROS2 TurtleBot3 Teleop**에서는:
- ROS2 `/cmd_vel` 토픽으로 TurtleBot3 제어
- keyboard_teleop.py 실행
- Isaac Sim + ROS2 완전한 양방향 통신
- rviz2에서 TurtleBot3 시각화

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| ROS2 Installation (Isaac Sim) | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html |
| ROS2 Humble 설치 | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html |
| ROS2 Jazzy 설치 | https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html |
| ROS2 Bridge Tutorial | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html |
| ROS2 Bridge Python | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_python.html |
| DDS 설정 | https://docs.ros.org/en/humble/Tutorials/Advanced/FastDDS-Configuration.html |
