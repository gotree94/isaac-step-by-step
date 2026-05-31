# Step 15 — ROS2 MoveIt2 + Franka Panda

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 08 완료 (Franka Control), Step 11 완료 (ROS2 Bridge)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **MoveIt2**를 설치하고 Isaac Sim의 Franka Panda 로봇팔과 연동한다
2. **Move Group**을 설정하고 **Planning Scene**을 구성한다
3. **Python MoveIt2 API**로 모션 계획을 생성하고 실행한다
4. **Pick & Place** 동작을 ROS2 + MoveIt2로 구현한다
5. **RViz2 MoveIt Plugin**으로 모션 계획을 시각화한다
6. **Cartesian Path**와 **Collision Avoidance**를 이해한다
7. **Joint State Publishers** — Isaac Sim이 발행하는 Franka 관절 상태를 MoveIt2와 동기화한다

---

## 1. 시스템 아키텍처

```
┌────────────────────┐          ┌──────────────────────────────┐
│    Isaac Sim       │          │      ROS2 + MoveIt2          │
│                    │          │                              │
│  ┌──────────────┐  │ /joint_states │  ┌───────────────────┐  │
│  │ Franka Panda │  │───────────►│  │  Robot State        │  │
│  │ (Articulation)│  │           │  │  Publisher          │  │
│  └──────┬───────┘  │           │  └─────────┬───────────┘  │
│         │           │           │            │               │
│         │           │ /tf       │  ┌─────────▼───────────┐  │
│         │           │───────────►│  │  MoveGroup          │  │
│         │           │           │  │  (Planning Scene)   │  │
│         │           │           │  └─────────┬───────────┘  │
│         │           │           │            │               │
│  ┌──────▼───────┐  │ /plan     │  ┌─────────▼───────────┐  │
│  │ MoveIt2      │  │◄──────────│  │  Motion Planner     │  │
│  │ Controller   │  │           │  │  (OMPL / Pilz / STOMP)│  │
│  └──────────────┘  │           │  └─────────┬───────────┘  │
│                    │           │            │               │
│                    │           │  ┌─────────▼───────────┐  │
│                    │           │  │  Trajectory         │  │
│                    │           │  │  Execution          │  │
│                    │           │  └─────────────────────┘  │
└────────────────────┘          └──────────────────────────────┘
```

### 1.1 MoveIt2 구성 요소

| 구성 요소 | 역할 |
|-----------|------|
| **MoveGroup Node** | 모션 계획의 메인 인터페이스 |
| **Planning Scene** | 환경 + 로봇 상태 관리 |
| **Motion Planner** | 경로 계획 알고리즘 (OMPL, Pilz, STOMP) |
| **Kinematics Plugin** | IK/FK 해석기 |
| **Collision Object** | 장애물 정보 |
| **Trajectory Execution** | 계획된 궤적을 Isaac Sim에 전송 |

### 1.2 통신 프로토콜

| 토픽/서비스 | 타입 | 방향 | 역할 |
|-------------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | Isaac Sim → MoveIt2 | Franka 관절 상태 |
| `/tf` | `tf2_msgs/TFMessage` | Isaac Sim → MoveIt2 | 좌표계 |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | MoveIt2 → rviz2 | 계획 경로 시각화 |
| `/planning_scene` | `moveit_msgs/PlanningScene` | MoveIt2 → rviz2 | Scene 정보 |
| `/compute_cartesian_path` | 서비스 | 내부 | Cartesian 경로 |
| `/plan_kinematic_path` | 서비스 | 내부 | 모션 계획 |

---

## 2. MoveIt2 설치

### 2.1 Ubuntu (ROS2 Humble)

```bash
# MoveIt2 패키지 설치
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-core \
  ros-humble-moveit-common \
  ros-humble-moveit-msgs \
  ros-humble-moveit-ros \
  ros-humble-moveit-planners \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-resources \
  ros-humble-moveit-resources-franka-description \
  ros-humble-moveit-resources-panda-description \
  ros-humble-graphics-utils \
  ros-humble-rviz2 \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

### 2.2 Franka SRDF/URDF 설정

MoveIt2가 Franka Panda 로봇을 인식하려면 SRDF(Setup Assistant에서 생성)가 필요합니다.

**프랑카 Panda SRDF** (`~/isaac-step-curriculum/config/franka_panda.srdf`):

```xml
<?xml version="1.0" ?>
<robot name="franka_panda">
  <!-- SRDF: Semantic Robot Description Format -->
  <!-- MoveIt2 Setup Assistant에서 생성된 파일 -->
  
  <!-- Group: panda_arm (7-DOF manipulator) -->
  <group name="panda_arm">
    <joint name="joint1"/>
    <joint name="joint2"/>
    <joint name="joint3"/>
    <joint name="joint4"/>
    <joint name="joint5"/>
    <joint name="joint6"/>
    <joint name="joint7"/>
    <chain base_link="panda_link0" tip_link="panda_link8"/>
  </group>
  
  <!-- Group: panda_hand (gripper) -->
  <group name="panda_hand">
    <joint name="panda_finger_joint1"/>
    <link name="panda_leftfinger"/>
    <link name="panda_rightfinger"/>
  </group>
  
  <!-- Group: panda_arm_hand (arm + gripper) -->
  <group name="panda_arm_hand">
    <group name="panda_arm"/>
    <group name="panda_hand"/>
  </group>
  
  <!-- Default pose: Home -->
  <group_state name="home" group="panda_arm">
    <joint name="joint1" value="0.0"/>
    <joint name="joint2" value="-0.78"/>
    <joint name="joint3" value="0.0"/>
    <joint name="joint4" value="-2.18"/>
    <joint name="joint5" value="0.0"/>
    <joint name="joint6" value="1.57"/>
    <joint name="joint7" value="0.78"/>
  </group_state>
  
  <group_state name="ready" group="panda_arm">
    <joint name="joint1" value="0.0"/>
    <joint name="joint2" value="-0.3"/>
    <joint name="joint3" value="0.0"/>
    <joint name="joint4" value="-1.5"/>
    <joint name="joint5" value="0.0"/>
    <joint name="joint6" value="1.2"/>
    <joint name="joint7" value="0.5"/>
  </group_state>
  
  <group_state name="extended" group="panda_arm">
    <joint name="joint1" value="0.0"/>
    <joint name="joint2" value="0.0"/>
    <joint name="joint3" value="0.0"/>
    <joint name="joint4" value="0.0"/>
    <joint name="joint5" value="0.0"/>
    <joint name="joint6" value="0.0"/>
    <joint name="joint7" value="0.0"/>
  </group_state>
  
  <!-- End Effector -->
  <end_effector name="end_effector" parent_group="panda_arm" 
                parent_link="panda_link8" group="panda_hand"/>
  
  <!-- Virtual Joint -->
  <virtual_joint name="world_joint" type="fixed" parent_frame="world" 
                 child_link="panda_link0"/>
  
  <!-- Disable Collisions (adjacent links) -->
  <disable_collisions link1="panda_link1" link2="panda_link2" reason="Adjacent"/>
  <disable_collisions link1="panda_link2" link2="panda_link3" reason="Adjacent"/>
  <disable_collisions link1="panda_link3" link2="panda_link4" reason="Adjacent"/>
  <disable_collisions link1="panda_link4" link2="panda_link5" reason="Adjacent"/>
  <disable_collisions link1="panda_link5" link2="panda_link6" reason="Adjacent"/>
  <disable_collisions link1="panda_link6" link2="panda_link7" reason="Adjacent"/>
  <disable_collisions link1="panda_link7" link2="panda_link8" reason="Adjacent"/>
  <disable_collisions link1="panda_finger_joint1" link2="panda_link8" reason="Adjacent"/>
</robot>
```

### 2.3 MoveIt2 Config 패키지

`~/isaac-step-curriculum/config/franka_moveit_config/kinematics.yaml`:

```yaml
panda_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005

panda_hand:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

`~/isaac-step-curriculum/config/franka_moveit_config/joint_limits.yaml`:

```yaml
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint2:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint3:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint4:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint5:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint6:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  joint7:
    has_velocity_limits: true
    max_velocity: 2.175
    has_acceleration_limits: true
    max_acceleration: 3.0
  panda_finger_joint1:
    has_velocity_limits: true
    max_velocity: 0.5
    has_acceleration_limits: true
    max_acceleration: 1.0
```

---

## 3. Isaac Sim에서 Franka + MoveIt2 연동

### 3.1 OmniGraph: Franka Joint State 발행

```python
import omni.graph.core as og
from pxr import Sdf

def setup_franka_moveit_graph(robot_path="/World/Franka"):
    """Franka Panda용 MoveIt2 지원 Graph 생성"""
    
    graph_config = {
        "graph_path": "/ActionGraph/Franka_MoveIt2",
        "evaluator_name": "execution",
    }
    
    og.Controller.edit(
        graph_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                
                # Subscriber: JointTrajectory from MoveIt2
                ("SubTraj", "omni.isaac.ros2_bridge.ROS2SubscribeJointTrajectory"),
                ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
                
                # Publisher: Joint State to MoveIt2
                ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
                ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                
                # Publisher: TF
                ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTraj.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadJoint.inputs:execIn"),
                ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                
                ("Context.outputs:context", "SubTraj.inputs:context"),
                ("Context.outputs:context", "PubJoint.inputs:context"),
                ("Context.outputs:context", "PubTF.inputs:context"),
                
                ("SubTraj.outputs:jointNames", "ArticCtrl.inputs:jointNames"),
                ("SubTraj.outputs:positionCommand", "ArticCtrl.inputs:positionCommand"),
                
                ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
                ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
                ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
                ("ReadJoint.outputs:jointEfforts", "PubJoint.inputs:jointEfforts"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                
                ("SubTraj.inputs:topicName", "/joint_trajectory"),
                ("ArticCtrl.inputs:robotPath", Sdf.Path(robot_path)),
                ("ArticCtrl.inputs:jointNames", [
                    "joint1", "joint2", "joint3", "joint4",
                    "joint5", "joint6", "joint7",
                ]),
                
                ("ReadJoint.inputs:robotPrim", Sdf.Path(robot_path)),
                ("PubJoint.inputs:topicName", "/joint_states"),
                
                # Gripper (선택)
                # ("ArticCtrl.inputs:jointNames", ["joint1", ..., "panda_finger_joint1", "panda_finger_joint2"])
            ],
        },
    )
```

### 3.2 Franka Panda 로딩 및 위치 설정

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import numpy as np

# Franka Panda 로딩
FRANKA_PATH = "/World/Franka"
add_reference_to_stage(
    "/Isaac/Robots/Franka/franka_alt_fingers.usd",
    FRANKA_PATH,
)

franka = Robot(
    prim_path=FRANKA_PATH,
    name="Franka",
    position=np.array([0.0, 0.0, 0.0]),
)
```

---

## 4. Python MoveIt2 API

### 4.1 MoveGroup Commander 예제

```python
#!/usr/bin/env python3
"""
MoveIt2 Python Commander — Franka Panda 모션 계획
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveGroupInterface
from moveit.core import RobotState
import numpy as np
import time
import sys

class FrankaMoveItCommander(Node):
    def __init__(self):
        super().__init__('franka_moveit_commander')
        
        # MoveGroup Interface 초기화
        self.move_group = MoveGroupInterface(
            node=self,
            group_name='panda_arm',
            robot_description='robot_description',
            robot_description_semantic='robot_description_semantic',
            ns='',
        )
        
        # Planning Parameters
        self.move_group.set_planner_id('RRTConnectkConfigDefault')
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # Goal Tolerance
        self.move_group.set_goal_tolerance(0.01)  # 1cm
        self.move_group.set_goal_orientation_tolerance(0.05)  # ~3°
        
        self.get_logger().info('MoveIt2 Commander initialized')
    
    def move_to_joint_state(self, joint_positions):
        """Joint Space Motion"""
        self.get_logger().info(f'Moving to joint positions: {joint_positions}')
        
        self.move_group.set_joint_value_target(joint_positions)
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        
        if success:
            self.get_logger().info('Joint goal reached!')
        else:
            self.get_logger().error('Joint goal FAILED!')
        
        return success
    
    def move_to_pose(self, pose):
        """Task Space (Cartesian) Motion"""
        self.get_logger().info(f'Moving to pose: {pose}')
        
        self.move_group.set_pose_target(pose)
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            self.get_logger().info('Pose goal reached!')
        else:
            self.get_logger().error('Pose goal FAILED!')
        
        return success
    
    def plan_and_execute_cartesian(self, waypoints):
        """Cartesian Path (straight line)"""
        self.get_logger().info(f'Planning cartesian path with {len(waypoints)} waypoints')
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step (step size)
            0.0,   # jump_threshold
        )
        
        self.get_logger().info(f'Cartesian path computed: {fraction:.1%} coverage')
        
        if fraction > 0.8:
            self.move_group.execute(plan, wait=True)
            self.get_logger().info('Cartesian path executed!')
        else:
            self.get_logger().error(f'Cartesian path only {fraction:.1%} achievable')
        
        return plan, fraction
    
    def current_pose(self):
        """현재 End-Effector Pose 반환"""
        return self.move_group.get_current_pose()
    
    def plan_to_pose(self, target_pose):
        """Plan만 하고 실행하지 않음 (시각화용)"""
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        self.move_group.clear_pose_targets()
        return plan


def main():
    rclpy.init()
    
    commander = FrankaMoveItCommander()
    
    # ── Step 1: Home Pose ──
    print("\n=== Step 1: Move to HOME ===")
    home_joints = [0.0, -0.785, 0.0, -2.18, 0.0, 1.57, 0.785]
    commander.move_to_joint_state(home_joints)
    
    # ── Step 2: Ready Pose ──
    print("\n=== Step 2: Move to READY ===")
    ready_joints = [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.5]
    commander.move_to_joint_state(ready_joints)
    
    # ── Step 3: Cartesian Pose ──
    print("\n=== Step 3: Move to PICK position ===")
    pick_pose = Pose()
    pick_pose.position.x = 0.5
    pick_pose.position.y = 0.0
    pick_pose.position.z = 0.3
    pick_pose.orientation.w = 1.0
    commander.move_to_pose(pick_pose)
    
    # ── Step 4: Cartesian Straight Line ──
    print("\n=== Step 4: Cartesian Path (approach) ===")
    current = commander.current_pose()
    
    waypoints = []
    # 현재 위치에서 10cm 아래로
    waypoint = Pose()
    waypoint.position.x = current.pose.position.x
    waypoint.position.y = current.pose.position.y
    waypoint.position.z = current.pose.position.z - 0.10
    waypoint.orientation = current.pose.orientation
    waypoints.append(waypoint)
    
    # 좌측으로 15cm
    waypoint = Pose()
    waypoint.position.x = current.pose.position.x
    waypoint.position.y = current.pose.position.y + 0.15
    waypoint.position.z = current.pose.position.z - 0.10
    waypoint.orientation = current.pose.orientation
    waypoints.append(waypoint)
    
    commander.plan_and_execute_cartesian(waypoints)
    
    # ── Step 5: Back to Home ──
    print("\n=== Step 5: Return to HOME ===")
    commander.move_to_joint_state(home_joints)
    
    print("\nAll tasks completed!")
    
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 Pick & Place 완전 구현

```python
#!/usr/bin/env python3
"""
Pick & Place with MoveIt2 + Franka Panda
"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveGroupInterface
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
        self.arm_group = MoveGroupInterface(
            node=self, group_name='panda_arm')
        self.gripper_group = MoveGroupInterface(
            node=self, group_name='panda_hand')
        
        self.arm_group.set_planner_id('RRTConnectkConfigDefault')
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        
        self.gripper_group.set_max_velocity_scaling_factor(0.5)
        
        self.get_logger().info('Pick & Place ready')
    
    def gripper_open(self):
        """Gripper Open"""
        self.gripper_group.set_joint_value_target([0.04, 0.04])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        self.get_logger().info('Gripper OPEN')
    
    def gripper_close(self):
        """Gripper Close"""
        self.gripper_group.set_joint_value_target([0.0, 0.0])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        self.get_logger().info('Gripper CLOSE')
    
    def pick(self, position, orientation=None):
        """물체 집기"""
        if orientation is None:
            orientation = (0.0, 0.0, 0.0, 1.0)  # w=1
        
        self.get_logger().info(f'Picking at: {position}')
        
        # Step 1: Approach (위에서 15cm)
        approach_pose = Pose()
        approach_pose.position.x = position[0]
        approach_pose.position.y = position[1]
        approach_pose.position.z = position[2] + 0.15
        approach_pose.orientation.x = orientation[0]
        approach_pose.orientation.y = orientation[1]
        approach_pose.orientation.z = orientation[2]
        approach_pose.orientation.w = orientation[3]
        
        self.arm_group.set_pose_target(approach_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        # Step 2: Open gripper
        self.gripper_open()
        
        # Step 3: Descend to object
        grasp_pose = Pose()
        grasp_pose.position.x = position[0]
        grasp_pose.position.y = position[1]
        grasp_pose.position.z = position[2]
        grasp_pose.orientation.x = orientation[0]
        grasp_pose.orientation.y = orientation[1]
        grasp_pose.orientation.z = orientation[2]
        grasp_pose.orientation.w = orientation[3]
        
        waypoints = [grasp_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        
        # Step 4: Close gripper
        self.gripper_close()
        time.sleep(0.5)
        
        # Step 5: Lift
        lift_pose = Pose()
        lift_pose.position.x = position[0]
        lift_pose.position.y = position[1]
        lift_pose.position.z = position[2] + 0.20
        lift_pose.orientation.x = orientation[0]
        lift_pose.orientation.y = orientation[1]
        lift_pose.orientation.z = orientation[2]
        lift_pose.orientation.w = orientation[3]
        
        waypoints = [lift_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        
        self.get_logger().info('Pick completed!')
        return True
    
    def place(self, position, orientation=None):
        """물체 놓기"""
        if orientation is None:
            orientation = (0.0, 0.0, 0.0, 1.0)
        
        self.get_logger().info(f'Placing at: {position}')
        
        # Step 1: Move above place position
        approach_pose = Pose()
        approach_pose.position.x = position[0]
        approach_pose.position.y = position[1]
        approach_pose.position.z = position[2] + 0.20
        approach_pose.orientation.x = orientation[0]
        approach_pose.orientation.y = orientation[1]
        approach_pose.orientation.z = orientation[2]
        approach_pose.orientation.w = orientation[3]
        
        self.arm_group.set_pose_target(approach_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        # Step 2: Descend
        place_pose = Pose()
        place_pose.position.x = position[0]
        place_pose.position.y = position[1]
        place_pose.position.z = position[2] + 0.02
        place_pose.orientation.x = orientation[0]
        place_pose.orientation.y = orientation[1]
        place_pose.orientation.z = orientation[2]
        place_pose.orientation.w = orientation[3]
        
        waypoints = [place_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        
        # Step 3: Open gripper
        self.gripper_open()
        time.sleep(0.5)
        
        # Step 4: Retreat
        retreat_pose = Pose()
        retreat_pose.position.x = position[0]
        retreat_pose.position.y = position[1]
        retreat_pose.position.z = position[2] + 0.30
        retreat_pose.orientation.x = orientation[0]
        retreat_pose.orientation.y = orientation[1]
        retreat_pose.orientation.z = orientation[2]
        retreat_pose.orientation.w = orientation[3]
        
        waypoints = [retreat_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self.arm_group.execute(plan, wait=True)
        
        self.get_logger().info('Place completed!')
        return True
    
    def home(self):
        """Home position으로 복귀"""
        home_joints = [0.0, -0.785, 0.0, -2.18, 0.0, 1.57, 0.785]
        self.arm_group.set_joint_value_target(home_joints)
        self.arm_group.go(wait=True)
        self.arm_group.stop()


def main():
    rclpy.init()
    
    pnp = PickAndPlace()
    
    pnp.home()
    
    # Pick from (0.4, 0.2, 0.02)
    pnp.pick(position=(0.4, 0.2, 0.02))
    
    # Place to (0.0, -0.3, 0.02)
    pnp.place(position=(0.0, -0.3, 0.02))
    
    pnp.home()
    
    pnp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5. Collision Objects (Planning Scene)

### 5.1 장애물 추가

```python
from moveit.core.planning_scene import PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Planning Scene에 장애물 추가
def add_collision_object(scene, name, shape, pose):
    """Collision Object 추가"""
    
    co = CollisionObject()
    co.id = name
    co.header.frame_id = 'panda_link0'
    
    if shape == 'box':
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.3, 0.3, 0.5]
        co.primitives.append(primitive)
    elif shape == 'cylinder':
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.4, 0.1]  # height, radius
        co.primitives.append(primitive)
    
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    
    scene.apply_collision_object(co)

# 사용 예
scene = PlanningScene()
table_pose = Pose()
table_pose.position.x = 0.5
table_pose.position.y = 0.0
table_pose.position.z = -0.25

add_collision_object(scene, 'table', 'box', table_pose)
```

### 5.2 Attach Object (물체 집기)

```python
# Gripper에 물체 부착
def attach_object(scene, object_id, touch_links=None):
    """Gripper에 물체 부착"""
    if touch_links is None:
        touch_links = ['panda_leftfinger', 'panda_rightfinger']
    
    co = CollisionObject()
    co.id = object_id
    co.header.frame_id = 'panda_link8'
    
    # 물체 제거 (world에서)
    co.operation = CollisionObject.REMOVE
    
    # Gripper에 부착
    scene.attach_object(
        object_id,
        'panda_link8',
        touch_links=touch_links,
    )

def detach_object(scene, object_id):
    """Gripper에서 물체 분리"""
    scene.detach_object(object_id)
```

---

## 6. rviz2 + MoveIt2 시각화

### 6.1 rviz2 실행

```bash
# 터미널 1: Isaac Sim
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step15_ros2_moveit.py

# 터미널 2: MoveIt2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Franka Panda MoveIt2 설정 로드
ros2 launch moveit2_tutorials demo.launch.py \
  robot_description:=~/isaac-step-curriculum/config/franka_panda.urdf \
  robot_description_semantic:=~/isaac-step-curriculum/config/franka_panda.srdf \
  kinematics_yaml:=~/isaac-step-curriculum/config/franka_moveit_config/kinematics.yaml \
  joint_limits_yaml:=~/isaac-step-curriculum/config/franka_moveit_config/joint_limits.yaml

# 터미널 3: rviz2 + MoveIt Plugin
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d $(ros2 pkg prefix moveit2_tutorials)/share/moveit2_tutorials/config/demo.rviz
```

### 6.2 rviz2 MoveIt Plugin 설정

rviz2에서:

1. **Panels > Add > Motion Planning** (MoveIt 패널)
2. **Displays > Add > RobotModel** (`robot_description` 토픽)
3. **Displays > Add > Trajectory** (Plan trajectory 표시)
4. **Displays > Add > PlanningScene** (장애물 표시)

**Motion Planning Panel:**
- **Planning Group**: `panda_arm`
- **Planner**: `RRTConnectkConfigDefault`
- **Goal State**: `home`, `ready`
- **Query**: 목표 Joint/Pose 설정 후 **Plan & Execute**

---

## 7. MoveIt2와 Isaac Sim 동기화

### 7.1 실행 순서

```bash
# ════════════════════════════════════════════════════════
# Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim (Franka + Joint State 발행 + Trajectory 수신)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step15_ros2_moveit.py

# 터미널 2: MoveIt2 + Robot State Publisher
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Robot State Publisher (URDF 로드)
ros2 run robot_state_publisher robot_state_publisher \
  ~/isaac-step-curriculum/config/franka_panda.urdf

# 터미널 3: MoveIt2 Launch
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch moveit2_tutorials demo.launch.py \
  robot_description:=~/isaac-step-curriculum/config/franka_panda.urdf \
  robot_description_semantic:=~/isaac-step-curriculum/config/franka_panda.srdf \
  kinematics_yaml:=~/isaac-step-curriculum/config/franka_moveit_config/kinematics.yaml \
  use_sim_time:=True

# 터미널 4: Python Commander
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
export ROS_DOMAIN_ID=0
python ~/isaac-step-curriculum/code/phase-2/step15_moveit_commander.py
```

### 7.2 동기화 확인

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames
evince frames.pdf &
# expected: world → panda_link0 → panda_link1 → ... → panda_link8

# Joint State 확인
ros2 topic echo /joint_states --once

# MoveIt2 계획 확인
ros2 topic echo /display_planned_path --once | head
```

---

## 8. 문제 해결 (Troubleshooting)

### 문제 1: MoveIt2가 /joint_states를 받지 못합니다.

**확인:**
```bash
# Isaac Sim이 발행하는지 확인
ros2 topic list | grep joint_states
ros2 topic info /joint_states
ros2 topic echo /joint_states --once | head
```

**해결:** `ROS_DOMAIN_ID`가 일치하는지 확인

### 문제 2: MoveIt2가 계획한 궤적이 Isaac Sim에서 실행되지 않습니다.

**해결:**
- `ROS2SubscribeJointTrajectory` 노드가 올바르게 연결되었는지 확인
- Topic 이름이 `/joint_trajectory`로 일치하는지 확인

### 문제 3: rviz2에 Franka 로봇이 표시되지 않습니다.

**해결:**
```bash
# Robot State Publisher가 URDF를 발행하는지 확인
ros2 topic echo /robot_description --once | head
```

### 문제 4: IK/FK가 실패합니다.

**해결:**
```yaml
kinematics_solver_timeout: 0.05  # 증가
kinematics_solver_search_resolution: 0.001  # 감소
```

### 문제 5: Motion Planning이 너무 느립니다.

**해결:**
```python
move_group.set_planning_time(2.0)  # 감소
move_group.set_num_planning_attempts(5)  # 감소
```

---

## 9. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ MoveIt2 설치 | ROS2 Humble MoveIt2 패키지 |
| ✅ Franka SRDF | Semantic Robot Description |
| ✅ Joint State | Isaac Sim → MoveIt2 |
| ✅ Trajectory | MoveIt2 → Isaac Sim |
| ✅ Python Commander | move_group.go(), set_pose_target() |
| ✅ Pick & Place | Approach → Grasp → Lift → Place |
| ✅ Cartesian Path | 직선 경로 계획 |
| ✅ Collision Objects | Planning Scene 장애물 |

### 아키텍처

```
Isaac Sim                    MoveIt2 / ROS2
    │                            │
    │ ── /joint_states ──────►   │
    │                            │ plan() → trajectory
    │ ◄── /joint_trajectory ──   │
    │                            │
    │ ── /tf ───────────────►    │
    │                            │
    │                            └──► rviz2
    │                                 │
    │                                 └── Motion Planning Panel
    │                                      │
    │                              Goal Pose / Joint State
```

---

## 10. 다음 Step 예고

**Step 16 — Multi-Robot ROS2 System**에서는:
- 다중 TurtleBot3를 ROS2로 동시 제어
- Namespace를 사용한 로봇 분리
- Multi-Robot SLAM / Navigation
- 중앙 집중식 Task 할당
- Inter-Robot Collision Avoidance

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| MoveIt2 공식 문서 | https://moveit.picknik.ai/ |
| MoveIt2 Tutorials (Humble) | https://docs.ros.org/en/humble/Tutorials/Intermediate/MoveIt2.html |
| Move Group Interface API | https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html |
| Isaac Sim + MoveIt2 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_moveit.html |
| Franka Panda URDF | https://github.com/ros-planning/moveit_resources |
| Pilz Planner | https://moveit.picknik.ai/main/doc/examples/industrial/industrial.html |
| OMPL Planner | https://ompl.kavrakilab.org/ |
