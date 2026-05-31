# Step 12 — ROS2 TurtleBot3 Teleop

> **소요 시간**: 60분
> **난이도**: ★★★☆☆ (중급)
> **선수 조건**: Step 11 완료 (ROS2 Bridge 설치)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **ROS2 `/cmd_vel`** 토픽을 Isaac Sim에서 구독한다
2. TurtleBot3의 **Odometry**를 ROS2 토픽으로 발행한다
3. **keyboard_teleop**으로 TurtleBot3를 실시간 제어한다
4. **rviz2**에서 TurtleBot3의 상태를 시각화한다
5. **TF 트리**를 발행하여 좌표계 변환을 확인한다
6. ROS2 **Joint State**를 발행한다
7. 완전한 **Isaac Sim ↔ ROS2 양방향 통신**을 구현한다

---

## 1. 시스템 아키텍처

이 Step에서 구축할 전체 시스템:

```
┌──────────────────────┐                    ┌──────────────────────────┐
│     Isaac Sim        │                    │     ROS2 Ecosystem       │
│                      │                    │                          │
│  ┌────────────────┐  │   /cmd_vel         │  ┌──────────────────┐   │
│  │ Subscriber     │◄─┼────────────────────┼──│ teleop_keyboard  │   │
│  │ (ROS2 Subscribe│  │                    │  └──────────────────┘   │
│  │  Twist)        │  │                    │                          │
│  └────────┬───────┘  │                    │  ┌──────────────────┐   │
│           │          │                    │  │     rviz2        │   │
│  ┌────────▼───────┐  │   /odom           │  └──────────────────┘   │
│  │ Publisher      │──┼────────────────────┼─►                       │
│  │ (Odometry)     │  │   /tf             │  ┌──────────────────┐   │
│  └────────────────┘  │────────────────────┼─►│  ros2 topic echo │   │
│                      │   /joint_states   │  └──────────────────┘   │
│                      │────────────────────┼─►                       │
│                      │   /scan           │                          │
│                      │────────────────────┼─►                       │
└──────────────────────┘                    └──────────────────────────┘
```

### 1.1 사용할 OmniGraph 노드

| ROS2 노드 | 기능 | 메시지 타입 |
|-----------|------|------------|
| **Subscribe Twist** | `/cmd_vel` 구독 → 속도 명령 | `geometry_msgs/Twist` |
| **Publish Odometry** | Odometry 발행 | `nav_msgs/Odometry` |
| **Publish TF** | TF 트리 발행 | `tf2_msgs/TFMessage` |
| **Publish JointState** | 관절 상태 발행 | `sensor_msgs/JointState` |
| **Publish LaserScan** | LiDAR 스캔 발행 (선택) | `sensor_msgs/LaserScan` |

### 1.2 좌표계 (Frame)

TurtleBot3의 주요 TF 좌표계:

| Frame | 설명 |
|-------|------|
| `odom` | 월드 고정 좌표계 (Odometry 원점) |
| `base_footprint` | 바닥 투영 좌표계 |
| `base_link` | 로봇 베이스 좌표계 |
| `base_scan` | LiDAR 센서 좌표계 |
| `wheel_left_link` | 좌측 바퀴 |
| `wheel_right_link` | 우측 바퀴 |

---

## 2. ROS2 Subscriber: /cmd_vel

### 2.1 OmniGraph로 /cmd_vel 구독

**Graph 구성:**

```
On Playback Tick
    │
    ├──→ Subscribe Twist (topic: /cmd_vel)
    │         → Differential Controller → Articulation Controller → 로봇
    │
    └──→ Publish Odometry (topic: /odom)
    │         → Publish TF
    │
    └──→ Publish JointState (topic: /joint_states)
```

**GUI 설정:**
1. **Window > Graph Editors > Action Graph** → **New Action Graph**
2. 다음 노드 추가:
   - `On Playback Tick`
   - `ROS2 Context` (domain_id=0)
   - `Subscribe Twist` (`omni.isaac.ros2_bridge.ROS2SubscribeTwist`)
   - `Differential Controller` (`omni.isaac.core_nodes.IsaacDifferentialController`)
   - `Articulation Controller` (`omni.isaac.core_nodes.IsaacArticulationController`)

3. **Differential Controller 설정:**
   - `wheelDistance`: 0.141
   - `wheelRadius`: 0.033
   - `maxLinearSpeed`: 0.5
   - `maxAngularSpeed`: 2.0

4. **Articulation Controller 설정:**
   - `robotPath`: /World/TurtleBot3
   - `jointNames`: `[left_wheel_joint, right_wheel_joint]`
   - `speed`: 1.0

**연결:**

```
Subscribe Twist.outputs:linearX  →  Differential Controller.inputs:linearVelocity
Subscribe Twist.outputs:angularZ →  Differential Controller.inputs:angularVelocity

Differential Controller.outputs:velocityCommand 
  → Articulation Controller.inputs:velocityCommand

On Playback Tick.outputs:tick 
  → Subscribe Twist.inputs:execIn
  → Differential Controller.inputs:execIn
  → Articulation Controller.inputs:execIn
```

### 2.2 Python으로 /cmd_vel 구독 Graph 생성

```python
import omni.graph.core as og
from pxr import Sdf

graph_config = {
    "graph_path": "/ActionGraph/ROS2_Teleop",
    "evaluator_name": "execution",
}

og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
            ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
            ("OnTick.outputs:tick", "ArticCtrl.inputs:execIn"),
            ("Context.outputs:context", "SubTwist.inputs:context"),
            ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
            ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),
            ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:domain_id", 0),
            ("SubTwist.inputs:topicName", "/cmd_vel"),
            ("DiffCtrl.inputs:wheelDistance", 0.141),
            ("DiffCtrl.inputs:wheelRadius", 0.033),
            ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
            ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
            ("ArticCtrl.inputs:robotPath", Sdf.Path("/World/TurtleBot3")),
            ("ArticCtrl.inputs:jointNames", 
             ["left_wheel_joint", "right_wheel_joint"]),
        ],
    },
)
```

---

## 3. ROS2 Publisher: Odometry

### 3.1 Odometry 발행 Graph

```python
og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "ReadOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
            ("Context.outputs:context", "PubOdom.inputs:context"),
            ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
            ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ReadOdom.inputs:chassisPrim", 
             Sdf.Path("/World/TurtleBot3/base_link")),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:frameId", "odom"),
            ("PubOdom.inputs:childFrameId", "base_footprint"),
        ],
    },
)
```

### 3.2 발행되는 Odometry 메시지

```bash
$ ros2 topic echo /odom
---
header:
  stamp:
    sec: 2
    nanosec: 500000000
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: 0.153
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.076
      w: 0.997
twist:
  twist:
    linear:
      x: 0.15
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
```

---

## 4. TF 브로드캐스터

### 4.1 TF Graph

```python
# Publish TF 노드 추가
og.Controller.edit(
    config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
            ("Context.outputs:context", "PubTF.inputs:context"),
        ],
    },
)
```

### 4.2 rviz2에서 확인

```bash
# 터미널에서 rviz2 실행
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2
```

rviz2에서:
1. **Displays > Add** → **By topic** → `/odom` 선택 (Odometry 표시)
2. **Displays > Add** → **TF** 선택 (좌표계 표시)
3. **Fixed Frame**: `odom`으로 설정
4. Isaac Sim에서 Play(▶) → 로봇 움직임 확인

---

## 5. Keyboard Teleop

### 5.1 ROS2 teleop_twist_keyboard

```bash
# 터미널 1: Isaac Sim 실행 (ROS2 Bridge + Teleop Graph)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./isaac-sim.selector.sh  # ROS2 Bridge 활성화

# Isaac Sim에서 Play (▶) 후

# 터미널 2: keyboard teleop 실행
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# teleop_twist_keyboard 설치
sudo apt install -y ros-humble-teleop-twist-keyboard
# 또는
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
pip install teleop-twist-keyboard

# 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**조작 키:**
```
   U    I    O
   J    K    L
```

| 키 | 기능 |
|----|------|
| **I** | 전진 |
| **,** | 후진 |
| **J** | 좌회전 |
| **L** | 우회전 |
| **K** | 정지 |
| **U** | 좌측 전진 |
| **O** | 우측 전진 |
| **Ctrl+C** | 종료 |

### 5.2 Python Keyboard Teleop

```python
#!/usr/bin/env python3
"""
간단한 ROS2 Keyboard Teleop Node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        twist = Twist()
        try:
            print("Keyboard Teleop (WASD):")
            print("  W=전진  S=후진  A=좌회전  D=우회전")
            print("  Q=정지  Ctrl+C=종료")
            
            while True:
                key = self.get_key()
                
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                if key == 'w':
                    twist.linear.x = 0.2
                elif key == 's':
                    twist.linear.x = -0.2
                elif key == 'a':
                    twist.angular.z = 0.5
                elif key == 'd':
                    twist.angular.z = -0.5
                elif key == 'q':
                    pass  # 정지
                elif key == '\x03':  # Ctrl+C
                    break
                
                self.publisher.publish(twist)
                print(f"Published: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
        
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main():
    rclpy.init()
    teleop = TeleopKeyboard()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. 실습: 완전한 Teleop 시스템

### 6.1 전체 절차

```bash
# === 터미널 1: Isaac Sim ===
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./isaac-sim.selector.sh  # ROS2 Bridge 활성화

# Isaac Sim 내:
# 1. File > New
# 2. Create > Physics > Ground Plane
# 3. Content Browser > TurtleBot3 > Viewport로 드래그
# 4. Window > Graph Editors > Action Graph
# 5. New Action Graph → 위 Omnigraph 구성
# 6. Play (▶)

# === 터미널 2: teleop ===
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# === 터미널 3: rviz2 ===
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2  # Fixed Frame: odom

# === 터미널 4: 모니터링 ===
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /odom
```

### 6.2 확인 사항

| 확인 항목 | 명령어 | 기대 결과 |
|-----------|--------|-----------|
| 토픽 목록 | `ros2 topic list` | `/cmd_vel`, `/odom`, `/tf`, `/joint_states` |
| 속도 명령 | `ros2 topic pub /cmd_vel ...` | TurtleBot3 움직임 |
| Odometry | `ros2 topic echo /odom` | 위치/속도 데이터 |
| TF | `ros2 run tf2_tools view_frames` | TF 트리 PDF 생성 |
| rviz2 | `rviz2` | 로봇 모델 + Odometry 시각화 |

---

## 7. 문제 해결 (Troubleshooting)

### 문제 1: /cmd_vel을 발행해도 로봇이 움직이지 않습니다.

**확인 사항**:
- [ ] ROS2 Bridge가 활성화되었는가?
- [ ] `ROS_DOMAIN_ID`가 일치하는가?
- [ ] Isaac Sim에서 Play(▶) 상태인가?
- [ ] OmniGraph가 올바르게 연결되었는가?
- [ ] `/World/TurtleBot3` 경로가 올바른가?

### 문제 2: /odom 토픽이 발행되지 않습니다.

**확인 사항**:
- [ ] `ReadOdometry` 노드의 `chassisPrim` 경로가 올바른가?
- [ ] `PublishOdometry`가 `OnPlaybackTick`에 연결되었는가?
- [ ] ROS2 Context가 연결되었는가?

### 문제 3: rviz2에 로봇 모델이 표시되지 않습니다.

**해결**:
```bash
# 로봇 모델 로드 (rviz2에서)
# Displays > Add > RobotModel
# Robot Description: turtlebot3_waffle.urdf

# 또는 URDF를 직접 로드
export TURTLEBOT3_MODEL=waffle
ros2 run robot_state_publisher robot_state_publisher \
  $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf
```

### 문제 4: /tf에 base_link → odom 변환이 없습니다.

**해결**: `PublishTransformTree` 노드가 Graph에 포함되었는지 확인
```python
# PublishTF 노드 누락 시 추가
og.Controller.edit(
    config, {
        og.Controller.Keys.CREATE_NODES: [
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
    }
)
```

### 문제 5: Keyboard Teleop이 너무 느리게 반응합니다.

**해결**:
```bash
# teleop_twist_keyboard 반복율 조정
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p repeat_rate:=20.0
```

---

## 8. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ ROS2 Subscriber | `/cmd_vel` Twist 구독 → Differential Controller |
| ✅ ROS2 Publisher | `/odom` Odometry 발행 |
| ✅ TF 브로드캐스터 | `odom` → `base_footprint` TF 발행 |
| ✅ Keyboard Teleop | `teleop_twist_keyboard`로 실시간 제어 |
| ✅ rviz2 시각화 | 로봇 모델 + Odometry + TF 표시 |
| ✅ 양방향 통신 | Isaac Sim ← ROS2 (완전한 통신 사이클) |

### 데이터 흐름 완성

```
사용자 (키보드)
    │
    ▼
teleop_twist_keyboard
    │
    ▼  /cmd_vel (geometry_msgs/Twist)
Isaac Sim ROS2 Subscriber
    │
    ▼
Differential Controller → Articulation Controller
    │
    ▼
TurtleBot3 움직임
    │
    ├── Odometry (/odom)
    ├── TF (/tf)
    └── Joint State (/joint_states)
          │
          ▼
    rviz2 시각화 / ros2 topic echo
```

---

## 9. 다음 Step 예고

**Step 13 — ROS2 SLAM in Isaac Sim**에서는:
- Isaac Sim에서 LiDAR 스캔 데이터를 ROS2로 발행
- Google Cartographer 또는 SLAM Toolbox로 SLAM 실행
- TurtleBot3를 원격 조종하면서 Map 생성
- 생성된 Map 저장 및 활용

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| ROS2 TurtleBot3 Teleop | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_turtlebot.html |
| ROS2 Subscribe Twist | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_python.html |
| Differential Controller | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/commonly_used_omnigraph_shortcuts.html |
| teleop_twist_keyboard | https://index.ros.org/p/teleop_twist_keyboard/ |
| rviz2 사용법 | https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz2-Overview.html |
| ROS2 TF2 | https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2.html |
