# Step 16 — Multi-Robot ROS2 System

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 12~14 완료 (Teleop + SLAM + Nav2)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **다중 TurtleBot3**를 Isaac Sim 한 Scene에서 생성한다
2. **ROS2 Namespace**를 사용하여 각 로봇의 토픽을 분리한다
3. **Multi-Robot SLAM**을 실행하여 각 로봇이 개별 Map을 생성한다
4. **Multi-Robot Navigation**에서 충돌 회피(Collision Avoidance)를 구현한다
5. **중앙 관리자(Task Dispatcher)** 가 여러 로봇에 작업을 할당한다
6. **Inter-Robot 통신**을 통해 로봇 간 정보를 공유한다
7. **멀티 에이전트 시스템**의 기초를 이해한다

---

## 1. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────┐
│                 Isaac Sim                           │
│                                                     │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐   │
│  │ TurtleBot1 │  │ TurtleBot2 │  │ TurtleBot3 │   │
│  │ /tb1       │  │ /tb2       │  │ /tb3       │   │
│  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘   │
│        │               │               │          │
└────────┼───────────────┼───────────────┼──────────┘
         │               │               │
    ┌────▼────┐     ┌────▼────┐     ┌────▼────┐
    │ Namesp. │     │ Namesp. │     │ Namesp. │
    │ /tb1    │     │ /tb2    │     │ /tb3    │
    └────┬────┘     └────┬────┘     └────┬────┘
         │               │               │
         └───────────────┼───────────────┘
                         │
               ┌─────────▼─────────┐
               │  ROS2 Middleware  │
               │                   │
               │  /cmd_vel         │
               │  /odom            │
               │  /scan            │
               │  /tf              │
               └─────────┬─────────┘
                         │
               ┌─────────▼─────────┐
               │  Task Dispatcher  │
               │  (중앙 관리자)    │
               └───────────────────┘
```

### 1.1 Namespace 구조

각 로봇은 고유한 Namespace를 사용하여 토픽이 충돌하지 않도록 합니다:

| 로봇 | Namespace | /cmd_vel | /odom | /scan |
|------|-----------|----------|-------|-------|
| TurtleBot1 | `/tb1` | `/tb1/cmd_vel` | `/tb1/odom` | `/tb1/scan` |
| TurtleBot2 | `/tb2` | `/tb2/cmd_vel` | `/tb2/odom` | `/tb2/scan` |
| TurtleBot3 | `/tb3` | `/tb3/cmd_vel` | `/tb3/odom` | `/tb3/scan` |

### 1.2 TF 트리 (Multi-Robot)

```
/tb1/odom ──→ /tb1/base_footprint ──→ /tb1/base_link ──→ /tb1/base_scan
/tb2/odom ──→ /tb2/base_footprint ──→ /tb2/base_link ──→ /tb2/base_scan
/tb3/odom ──→ /tb3/base_footprint ──→ /tb3/base_link ──→ /tb3/base_scan
```

---

## 2. 다중 TurtleBot3 생성

### 2.1 Isaac Sim에서 다중 로봇 로딩

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import numpy as np

ROBOT_CONFIGS = [
    {
        "name": "TurtleBot1",
        "prim_path": "/World/Robots/TurtleBot1",
        "position": np.array([-1.5, -1.5, 0.1]),
        "namespace": "/tb1",
    },
    {
        "name": "TurtleBot2",
        "prim_path": "/World/Robots/TurtleBot2",
        "position": np.array([1.5, -1.5, 0.1]),
        "namespace": "/tb2",
    },
    {
        "name": "TurtleBot3",
        "prim_path": "/World/Robots/TurtleBot3",
        "position": np.array([0.0, 1.5, 0.1]),
        "namespace": "/tb3",
    },
]

robots = []
for cfg in ROBOT_CONFIGS:
    if not is_prim_path_valid(cfg["prim_path"]):
        add_reference_to_stage(
            "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
            cfg["prim_path"],
        )
    
    robot = Robot(
        prim_path=cfg["prim_path"],
        name=cfg["name"],
        position=cfg["position"],
    )
    world.scene.add(robot)
    robots.append((robot, cfg))
    
    print(f"  + {cfg['name']} loaded at {cfg['position']}")
```

---

## 3. Namespaced OmniGraph 구조

### 3.1 각 로봇별 Graph 생성

```python
import omni.graph.core as og
from pxr import Sdf
import omni.usd

def create_robot_graph(robot_path, namespace, ns_index):
    """각 로봇에 Namespace가 적용된 OmniGraph 생성"""
    
    graph_path = f"/ActionGraph/{namespace.strip('/')}_Bridge"
    config = {
        "graph_path": graph_path,
        "evaluator_name": "execution",
    }
    
    # 기존 Graph 제거
    stage = omni.usd.get_context().get_stage()
    if og.Controller.graph_exists(graph_path):
        stage.RemovePrim(Sdf.Path(graph_path))
    
    host_name = namespace.strip("/")
    
    og.Controller.edit(
        config,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                
                # Subscriber: /{ns}/cmd_vel
                ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
                ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
                
                # Publisher: /{ns}/odom
                ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
                ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                
                # Publisher: /{ns}/scan
                ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
                ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                
                # Publisher: /{ns}/tf
                ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                
                # Publisher: /{ns}/joint_states
                ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
                ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadScan.inputs:execIn"),
                ("OnTick.outputs:tick", "PubScan.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadJoint.inputs:execIn"),
                ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
                
                ("Context.outputs:context", "SubTwist.inputs:context"),
                ("Context.outputs:context", "PubOdom.inputs:context"),
                ("Context.outputs:context", "PubScan.inputs:context"),
                ("Context.outputs:context", "PubTF.inputs:context"),
                ("Context.outputs:context", "PubJoint.inputs:context"),
                
                ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
                ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),
                ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),
                
                ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
                ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
                ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                
                ("ReadScan.outputs:rangeData", "PubScan.inputs:rangeData"),
                
                ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
                ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
                ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("Context.inputs:namespace", namespace),
                
                ("SubTwist.inputs:topicName", f"{namespace}/cmd_vel"),
                
                ("DiffCtrl.inputs:wheelDistance", 0.141),
                ("DiffCtrl.inputs:wheelRadius", 0.033),
                ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
                ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
                
                ("ArticCtrl.inputs:robotPath", Sdf.Path(robot_path)),
                ("ArticCtrl.inputs:jointNames",
                 ["left_wheel_joint", "right_wheel_joint"]),
                
                ("ReadOdom.inputs:chassisPrim",
                 Sdf.Path(f"{robot_path}/base_link")),
                ("PubOdom.inputs:topicName", f"{namespace}/odom"),
                ("PubOdom.inputs:frameId", f"{namespace}/odom"),
                ("PubOdom.inputs:childFrameId", f"{namespace}/base_footprint"),
                
                ("ReadScan.inputs:laserPrim",
                 Sdf.Path(f"{robot_path}/base_scan/Lidar")),
                ("PubScan.inputs:topicName", f"{namespace}/scan"),
                ("PubScan.inputs:frameId", f"{namespace}/base_scan"),
                ("PubScan.inputs:rangeMin", 0.1),
                ("PubScan.inputs:rangeMax", 3.5),
                ("PubScan.inputs:rangeThreshold", 3500.0),
                
                ("ReadJoint.inputs:robotPrim", Sdf.Path(robot_path)),
                ("PubJoint.inputs:topicName", f"{namespace}/joint_states"),
            ],
        },
    )
    
    print(f"  + Graph created: {graph_path}")
    return graph_path
```

### 3.2 동시 충돌 방지

```bash
# 각 로봇의 Domain ID를 다르게 설정 (선택)
# ROS2 Namespace를 사용하면 Domain ID가 같아도 충돌 없음

# 각 SLAM Toolbox를 별도 실행
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args -r __ns:=/tb1
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args -r __ns:=/tb2
```

---

## 4. Multi-Robot Navigation

### 4.1 개별 Nav2 인스턴스 실행

각 로봇마다 Namespace가 적용된 Nav2를 실행합니다.

```bash
# TurtleBot1 Nav2
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/isaac-step-curriculum/config/nav2_params_tb1.yaml \
  namespace:=/tb1 \
  use_sim_time:=True

# TurtleBot2 Nav2
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/isaac-step-curriculum/config/nav2_params_tb2.yaml \
  namespace:=/tb2 \
  use_sim_time:=True
```

**Nav2 파라미터 (namespace 적용)** (`nav2_params_tb1.yaml`):

```yaml
# TurtleBot1 Namespace 파라미터
amcl:
  ros__parameters:
    use_sim_time: True
    base_frame_id: "/tb1/base_footprint"
    global_frame_id: "/tb1/map"
    odom_frame_id: "/tb1/odom"
    ...

global_costmap:
  global_costmap:
    ros__parameters:
      robot_base_frame: /tb1/base_footprint
      ...
      scan:
        topic: /tb1/scan
        sensor_frame: /tb1/base_scan

local_costmap:
  local_costmap:
    ros__parameters:
      robot_base_frame: /tb1/base_footprint
      ...
      scan:
        topic: /tb1/scan
        sensor_frame: /tb1/base_scan
```

### 4.2 충돌 회피 (Collision Avoidance)

다중 로봇 환경에서는 각 로봇이 서로를 장애물로 인식해야 합니다.

**방법 1: Local Costmap 장애물 감지**
- 각 로봇의 LiDAR가 다른 로봇을 장애물로 감지
- Local Costmap Obstacle Layer가 실시간 장애물 회피

**방법 2: Inter-Robot 통신**
```python
# ROS2 토픽으로 다른 로봇 위치 공유
class RobotProximityBroadcast(Node):
    """각 로봇의 위치를 브로드캐스트"""
    
    def __init__(self, namespace):
        super().__init__(f'{namespace.strip("/")}_proximity')
        self.ns = namespace
        
        # 내 위치 발행
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'{namespace}/pose_shared',
            10,
        )
        
        # 다른 로봇 위치 수신
        self.subs = []
        for other_ns in ['/tb1', '/tb2', '/tb3']:
            if other_ns != namespace:
                sub = self.create_subscription(
                    PoseStamped,
                    f'{other_ns}/pose_shared',
                    lambda msg, ns=other_ns: self.on_robot_pose(msg, ns),
                    10,
                )
                self.subs.append(sub)
        
        self.other_robots = {}  # ns → pose
    
    def on_robot_pose(self, msg, ns):
        self.other_robots[ns] = msg.pose

# 속도 감소 (다른 로봇이 가까우면)
def adjust_speed_based_on_proximity(self):
    for ns, pose in self.other_robots.items():
        dx = pose.position.x - self.current_x
        dy = pose.position.y - self.current_y
        dist = (dx**2 + dy**2)**0.5
        
        if dist < 0.5:  # 50cm 이내
            self.request_speed_reduction(0.5)
            logger.warn(f"{ns} is too close ({dist:.2f}m)! Slowing down.")
        elif dist < 1.0:  # 1m 이내
            self.request_speed_reduction(0.8)
```

---

## 5. Task Dispatcher (중앙 관리자)

### 5.1 Goal Pose 디스패처

```python
#!/usr/bin/env python3
"""
Multi-Robot Task Dispatcher
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
import random

ROBOTS = {
    '/tb1': BasicNavigator(namespace='/tb1'),
    '/tb2': BasicNavigator(namespace='/tb2'),
    '/tb3': BasicNavigator(namespace='/tb3'),
}

TASKS = [
    # (robot_ns, target_x, target_y, description)
    ('/tb1', 1.0, 1.0, 'Zone A patrol'),
    ('/tb2', -1.0, 1.5, 'Zone B patrol'),
    ('/tb3', -0.5, -1.5, 'Zone C patrol'),
    ('/tb1', -1.5, -0.5, 'Zone D patrol'),
    ('/tb2', 1.5, -1.0, 'Zone E patrol'),
    ('/tb3', 1.0, -0.5, 'Zone F patrol'),
]

class TaskDispatcher(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        self.task_queue = list(TASKS)
        self.active_tasks = {}
        
        # 각 네비게이터 초기화
        self.navigators = {}
        for ns, navigator in ROBOTS.items():
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = f'{ns}/map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.w = 1.0
            
            navigator.setInitialPose(initial_pose)
        
        self.timer = self.create_timer(1.0, self.dispatch_loop)
        self.get_logger().info('Task Dispatcher initialized')
    
    def dispatch_loop(self):
        """주기적으로 태스크 할당"""
        if not self.task_queue:
            return
        
        for robot_ns, nav in self.navigators.items():
            if not nav.isTaskComplete():
                continue  # 아직 태스크 진행 중
            
            if not self.task_queue:
                continue
            
            # 다음 태스크 할당
            ns, x, y, desc = self.task_queue.pop(0)
            
            goal = PoseStamped()
            goal.header.frame_id = f'{ns}/map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0
            
            nav.goToPose(goal)
            
            self.get_logger().info(
                f'[{robot_ns}] Assigned: {desc} ({x:.1f}, {y:.1f})')
            
            # 태스크 완료 후 재할당 (순환)
            self.task_queue.append((ns, x, y, desc))
            break  # 한 번에 한 태스크만 할당


def main():
    rclpy.init()
    
    # 모든 네비게이터가 활성화될 때까지 대기
    for ns, nav in ROBOTS.items():
        nav.waitUntilNav2Active()
        print(f'  + {ns} navigator ready')
    
    dispatcher = TaskDispatcher()
    rclpy.spin(dispatcher)
    
    for nav in ROBOTS.values():
        nav.lifecycleShutdown()
    
    dispatcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2 고급 디스패처: 동적 우선순위

```python
class DynamicTaskDispatcher(Node):
    """로봇 상태에 따라 동적 할당"""
    
    def __init__(self):
        super().__init__('dynamic_dispatcher')
        
        # 각 로봇의 상태 수신
        self.robot_status = {}
        for ns in ['/tb1', '/tb2', '/tb3']:
            self.create_subscription(
                String, f'{ns}/status',
                lambda msg, robot=ns: self.on_status(msg, robot),
                10,
            )
            self.robot_status[ns] = {
                'battery': 100.0,
                'task_completed': 0,
                'position': (0.0, 0.0),
            }
        
        self.tasks = self._generate_tasks()
        self.timer = self.create_timer(2.0, self.dispatch)
    
    def _generate_tasks(self):
        """작업 목록 (위치 + 중요도)"""
        return [
            {'x': 2.0, 'y': 2.0, 'priority': 3, 'desc': 'Critical Zone'},
            {'x': -2.0, 'y': 2.0, 'priority': 2, 'desc': 'Warehouse A'},
            {'x': 2.0, 'y': -2.0, 'priority': 1, 'desc': 'Warehouse B'},
            {'x': -2.0, 'y': -2.0, 'priority': 2, 'desc': 'Loading Dock'},
            {'x': 0.0, 'y': 0.0, 'priority': 1, 'desc': 'Charging Station'},
        ]
    
    def dispatch(self):
        """가장 적합한 로봇에 태스크 할당"""
        available_tasks = [t for t in self.tasks 
                          if t not in self.active_tasks]
        available_robots = [ns for ns, status in self.robot_status.items()
                           if not self._is_busy(ns)
                           and status['battery'] > 20.0]
        
        if not available_tasks or not available_robots:
            return
        
        # 높은 우선순위 태스크부터
        task = max(available_tasks, key=lambda t: t['priority'])
        
        # 가장 가까운 로봇 할당
        robot = min(available_robots,
                   key=lambda ns: self._distance_to_task(ns, task))
        
        self._assign_task(robot, task)
    
    def _distance_to_task(self, ns, task):
        pos = self.robot_status[ns]['position']
        return ((pos[0] - task['x'])**2 + 
                (pos[1] - task['y'])**2)**0.5
    
    def _assign_task(self, robot_ns, task):
        """Goal Pose 전송"""
        goal = PoseStamped()
        goal.header.frame_id = f'{robot_ns}/map'
        goal.pose.position.x = task['x']
        goal.pose.position.y = task['y']
        goal.pose.orientation.w = 1.0
        
        self.goal_pubs[robot_ns].publish(goal)
        self.active_tasks[task] = robot_ns
        self.get_logger().info(
            f'→ {robot_ns} dispatched to {task["desc"]}')
```

---

## 6. 멀티 에이전트 통신

### 6.1 로봇 상태 공유 시스템

```python
class RobotStatusBroadcaster(Node):
    """각 로봇의 상태를 주기적으로 발행"""
    
    def __init__(self, namespace, robot_obj):
        super().__init__(f'{namespace.strip("/")}_status')
        self.ns = namespace
        self.robot = robot_obj
        
        self.status_pub = self.create_publisher(
            String, f'{namespace}/status', 10)
        self.timer = self.create_timer(1.0, self.broadcast_status)
    
    def broadcast_status(self):
        """배터리, 위치, 태스크 완료 수 발행"""
        pos = self.robot.get_world_pose()
        status = String()
        status.data = json.dumps({
            'namespace': self.ns,
            'timestamp': self.get_clock().now().nanoseconds,
            'position': {
                'x': float(pos[0][0]),
                'y': float(pos[0][1]),
                'z': float(pos[0][2]),
            },
            'battery': self.battery_level,
            'task_completed': self.task_count,
        })
        self.status_pub.publish(status)
```

### 6.2 중앙 로그 수집

```bash
# 모든 로봇의 상태 확인
ros2 topic echo /tb1/status
ros2 topic echo /tb2/status
ros2 topic echo /tb3/status

# 특정 네임스페이스 토픽 리스트
ros2 topic list | grep /tb1
ros2 topic list | grep /tb2
```

---

## 7. 전역 충돌 회피 전략

### 7.1 속도 기반 충돌 회피

```python
import math

class CollisionAvoidance(Node):
    """다중 로봇 간 충돌 회피 관리자"""
    
    ROBOT_RADIUS = 0.2  # TurtleBot3 반경
    SAFE_DISTANCE = 0.6  # 안전 거리
    
    def __init__(self):
        super().__init__('collision_avoider')
        
        self.robot_poses = {}
        self.robot_vels = {}
        
        for ns in ['/tb1', '/tb2', '/tb3']:
            self.create_subscription(
                Odometry, f'{ns}/odom',
                lambda msg, robot=ns: self.on_odom(msg, robot),
                10,
            )
        
        # 속도 재조정 Publisher
        self.vel_pubs = {
            ns: self.create_publisher(
                Twist, f'{ns}/cmd_vel_override', 10)
            for ns in ['/tb1', '/tb2', '/tb3']
        }
        
        self.timer = self.create_timer(0.1, self.check_proximity)
    
    def on_odom(self, msg, robot_ns):
        """Odometry로 로봇 위치/속도 추적"""
        self.robot_poses[robot_ns] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.robot_vels[robot_ns] = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )
    
    def check_proximity(self):
        """로봇 간 거리 확인 → 필요시 속도 조정"""
        for r1 in self.robot_poses:
            for r2 in self.robot_poses:
                if r1 >= r2:
                    continue
                
                p1 = self.robot_poses[r1]
                p2 = self.robot_poses[r2]
                
                dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                min_dist = 2 * self.ROBOT_RADIUS + self.SAFE_DISTANCE
                
                if dist < min_dist:
                    # 충돌 위험! 속도 감소 명령
                    urgency = 1.0 - (dist / min_dist)
                    self.apply_emergency_brake(r1, urgency)
                    self.apply_emergency_brake(r2, urgency)
                    
                    self.get_logger().warn(
                        f'⚠ COLLISION RISK: {r1} ↔ {r2} '
                        f'(dist={dist:.2f}m)')
    
    def apply_emergency_brake(self, robot_ns, urgency):
        """비상 제동"""
        twist = Twist()
        if urgency > 0.7:
            twist.linear.x = 0.0  # 정지
        else:
            twist.linear.x = max(0.0, 0.2 - urgency * 0.3)
        
        self.vel_pubs[robot_ns].publish(twist)
```

---

## 8. 실행 절차

### 8.1 전체 실행

```bash
# ════════════════════════════════════════════════════
# Multi-Robot System — 6 Terminal Setup
# ════════════════════════════════════════════════════

# 터미널 1: Isaac Sim (3x TurtleBot3 + Namespaced Bridges)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step16_ros2_multi_robot.py

# 터미널 2-4: 각 로봇의 Teleop (별도 터미널)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/tb1/cmd_vel &

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/tb2/cmd_vel &

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/tb3/cmd_vel &

# 터미널 5: 모니터링
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /tb1/odom --once | head
ros2 topic echo /tb2/odom --once | head
ros2 topic echo /tb3/odom --once | head

# 터미널 6: rviz2 (3개 Namespace 표시)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d ~/isaac-step-curriculum/config/multi_robot.rviz
```

### 8.2 rviz2 멀티 로봇 설정

rviz2에서 각 로봇의 Namespace를 개별 표시:

```bash
# 각 로봇별 Display 추가:
# 1. Displays > Add > By topic > /tb1/odom → Odometry
# 2. Displays > Add > By topic > /tb1/scan → LaserScan
# 3. Displays > Add > TF (Prefix: tb1_)

# 또는 Config 파일 미리 작성
```

**multi_robot.rviz** 설정:

```
Panels:
  - Name: tb1_odom
    Topic: /tb1/odom
    Color: 255, 0, 0
  - Name: tb2_odom
    Topic: /tb2/odom  
    Color: 0, 255, 0
  - Name: tb3_odom
    Topic: /tb3/odom
    Color: 0, 0, 255
  - Name: Global LaserScan
    Topic: /tb1/scan
  - Name: Global LaserScan
    Topic: /tb2/scan
  - Name: Global LaserScan
    Topic: /tb3/scan
```

---

## 9. Multi-Robot Synthetic Data 수집

```python
class MultiRobotDataCollector(Node):
    """다중 로봇 환경에서 Synthetic Data 수집"""
    
    def __init__(self):
        super().__init__('multi_robot_collector')
        
        # 각 로봇의 데이터 수집
        self.scan_subs = []
        self.rgb_subs = []
        
        for ns in ['/tb1', '/tb2', '/tb3']:
            scan_sub = self.create_subscription(
                LaserScan, f'{ns}/scan',
                lambda msg, robot=ns: self.on_scan(msg, robot),
                10,
            )
            self.scan_subs.append(scan_sub)
        
        self.scan_data = {f'/tb{i}': [] for i in range(1, 4)}
        self.save_timer = self.create_timer(5.0, self.save_batch)
    
    def on_scan(self, msg, robot_ns):
        self.scan_data[robot_ns].append({
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'timestamp': self.get_clock().now().nanoseconds,
        })
    
    def save_batch(self):
        for ns, data in self.scan_data.items():
            if len(data) >= 10:
                filename = f'scandata_{ns.strip("/")}_{int(time.time())}.npy'
                np.save(filename, data[:10])
                self.scan_data[ns] = self.scan_data[ns][10:]
                self.get_logger().info(
                    f'Saved 10 scans from {ns} to {filename}')
```

---

## 10. 문제 해결 (Troubleshooting)

### 문제 1: Namespace가 적용되지 않습니다.

**확인:**
```bash
# Context 노드에 namespace가 설정되었는지 확인
# OmniGraph Editor에서 Context.inputs 설정 확인
```

**해결:** Context 노드의 `inputs:namespace`를 명시적으로 설정

### 문제 2: TF 트리가 충돌합니다.

**원인:** 모든 로봇이 같은 TF frame (`odom`, `base_footprint`)을 사용
**해결:** 각 Graph의 PublishTF에 `frameId`를 Namespace 포함으로 설정

### 문제 3: LiDAR가 다른 로봇을 감지하지 못합니다.

**원인:** LiDAR 충돌 속성이 꺼져 있음
**해결:**
```python
# USD에서 LiDAR 충돌 활성화
lidar_prim.GetAttribute("drawSensors").Set(True)
lidar_prim.GetAttribute("range").Set(3.5)
```

### 문제 4: 다중 Nav2가 같은 Map을 사용합니다.

**해결:** 각 Nav2 인스턴스에 개별 Map 파일 지정
```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=./maps/tb1_map.yaml -r __ns:=/tb1
```

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 다중 로봇 로딩 | 3x TurtleBot3 동시 로딩 |
| ✅ ROS2 Namespace | 토픽/서비스/TF 분리 |
| ✅ Multi-Robot Bridge | 각 로봇별 OmniGraph |
| ✅ 충돌 회피 | Inter-Robot 거리 기반 속도 제어 |
| ✅ Task Dispatcher | Goal Pose 자동 할당 |
| ✅ 상태 공유 | 각 로봇 위치/배터리/태스크 상태 |
| ✅ 다중 데이터 수집 | Synthetic Data 배치 수집 |

### Namespace 설계 원칙

```
/<robot_name>/cmd_vel       — 속도 명령 (입력)
/<robot_name>/odom          — Odometry (출력)
/<robot_name>/scan          — LiDAR 스캔 (출력)
/<robot_name>/odom → ...    — TF (접두사)
/<robot_name>/map           — Occupancy Grid (Nav2)
```

---

## 12. 다음 Step 예고

**Step 17 — Synthetic Data Generation**에서는:
- Isaac Sim의 Replicator로 학습 데이터 생성
- Bounding Box, Segmentation, Depth, Pose 데이터
- Randomization (자세, 조명, 텍스처, 카메라)
- 데이터셋 포맷 (COCO, KITTI, NuScenes)
- ROS2 Bridge를 통한 실시간 데이터 스트리밍

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| ROS2 Namespace | https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-Nodes.html |
| Nav2 Multi-Robot | https://docs.nav2.org/tutorials/docs/navigation2_with_multiple_robots.html |
| Isaac Sim Multi-Robot | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/tutorial_robotics_multi_robot.html |
| Multi-Agent Systems | https://arxiv.org/abs/2103.00201 |
| Collision Avoidance | https://www.cmu.edu/news/stories/archives/2020/january/multi-robot-collision-avoidance.html |
