# Step 14 — ROS2 Navigation2 (Nav2) in Isaac Sim

> **소요 시간**: 120분
> **난이도**: ★★★★☆ (고급)
> **선수 조건**: Step 13 완료 (ROS2 SLAM, Map 생성)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Nav2**(Navigation2)를 설치하고 Isaac Sim과 연동한다
2. 생성된 **Occupancy Grid Map**을 기반으로 자율 주행을 실행한다
3. **Global Planner**(NavFn / Smac Hybrid-A*)와 **Local Planner**(Regulated Pure Pursuit / DWB)를 이해한다
4. rviz2에서 **Goal Pose**를 지정하여 TurtleBot3를 자율 주행시킨다
5. **Behavior Tree**를 통해 다양한 Navigation 동작을 제어한다
6. **Recovery Behaviors**(Spin, Backup, Wait)를 이해하고 테스트한다
7. Nav2 파라미터를 튜닝하여 주행 품질을 최적화한다

---

## 1. 시스템 아키텍처

```
┌──────────────────────┐          ┌──────────────────────────────┐
│    Isaac Sim         │          │      ROS2 Navigation2        │
│                      │   /scan  │                              │
│  ┌────────────────┐  │─────────►│  ┌────────────────────────┐  │
│  │ LiDAR          │  │          │  │  Global Costmap        │  │
│  └────────────────┘  │          │  │  (Static Map Layer)    │  │
│                      │   /odom  │  └──────────┬─────────────┘  │
│  ┌────────────────┐  │─────────►│             │                 │
│  │ Odometry + TF  │  │          │  ┌──────────▼─────────────┐  │
│  └────────────────┘  │          │  │  Global Planner        │  │
│                      │   /tf    │  │  (NavFn / Smac)        │  │
│  ┌────────────────┐  │─────────►│  └──────────┬─────────────┘  │
│  │ TurtleBot3     │  │          │             │                 │
│  │ (Physics)      │  │          │  ┌──────────▼─────────────┐  │
│  └───────┬────────┘  │          │  │  Local Costmap         │  │
│          │           │          │  │  (Obstacle Layer)      │  │
│          │           │          │  └──────────┬─────────────┘  │
│          │ /cmd_vel  │          │             │                 │
│          │◄──────────┼──────────│  ┌──────────▼─────────────┐  │
│          │           │          │  │  Local Planner         │  │
│          │           │          │  │  (Regulated PP / DWB)  │  │
│          │           │          │  └────────────────────────┘  │
│          │           │          │                              │
│          │           │          │  ┌────────────────────────┐  │
│          │           │          │  │  Behavior Tree         │  │
│          │           │          │  │  (NavigateToPose)      │  │
│          │           │          │  └────────────────────────┘  │
└──────────────────────┘          └──────────────┬───────────────┘
                                                  │
                                                  │ /map
                                                  ▼
                                          ┌────────────────┐
                                          │  rviz2         │
                                          │  (Goal Pose)   │
                                          └────────────────┘
```

### 1.1 Nav2 주요 구성 요소

| 구성 요소 | 역할 | 알고리즘 예 |
|-----------|------|------------|
| **Global Costmap** | 전체 Map 기반 비용 맵 | Static Map Layer |
| **Local Costmap** | 로봇 주변 동적 비용 맵 | Obstacle Layer |
| **Global Planner** | 전역 경로 계획 | NavFn, Smac Hybrid-A* |
| **Local Planner** | 지역 경로 추종 | Regulated Pure Pursuit, DWB |
| **Behavior Tree** | Navigation 동작 제어 | NavigateToPose |
| **Recovery** | 장애물 회복 동작 | Spin, Backup, Wait |
| **Map Server** | Map 로드/제공 | nav2_map_server |

### 1.2 필요 토픽 흐름

| 토픽 | 방향 | 역할 |
|------|------|------|
| `/scan` | Isaac Sim → Nav2 | LiDAR (Costmap 업데이트) |
| `/odom` | Isaac Sim → Nav2 | Odometry (Localization) |
| `/tf` | Isaac Sim → Nav2 | TF 트리 (좌표계) |
| `/cmd_vel` | Nav2 → Isaac Sim | 속도 명령 (Twist) |
| `/map` | Map Server → Nav2 | Occupancy Grid |
| `initialpose` | rviz2 → Nav2 | 초기 위치 설정 |
| `goal_pose` | rviz2 → Nav2 | 목표 위치 설정 |

---

## 2. Nav2 설치

### 2.1 Ubuntu (ROS2 Humble)

```bash
# Nav2 패키지 설치
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-behavior-tree \
  ros-humble-nav2-msgs \
  ros-humble-nav2-util \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-core \
  ros-humble-nav2-common \
  ros-humble-nav2-lifecycle-manager

# TurtleBot3 관련
sudo apt install -y \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2  # Nav2 설정 파일 포함
```

### 2.2 Conda 환경에 Nav2 패키지 설치

```bash
# Conda/Pip 환경
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
pip install nav2-simple-commander
```

---

## 3. Nav2 설정 파일

### 3.1 Nav2 파라미터 (nav2_params.yaml)

`~/isaac-step-curriculum/config/nav2_params.yaml`:

```yaml
# ═══════════════════════════════════════════════════════
# Nav2 Parameters for Isaac Sim + TurtleBot3
# ═══════════════════════════════════════════════════════

amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 3.5
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

# ── Global Costmap ──
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      robot_base_frame: base_footprint
      footprint: "[ [0.18, 0.14], [0.18, -0.14], [-0.18, -0.14], [-0.18, 0.14] ]"
      footprint_padding: 0.03
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      observation_sources: scan
      scan:
        topic: /scan
        sensor_frame: base_scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
      track_unknown_space: true
      transform_tolerance: 0.2
      width: 10
      height: 10
      origin_x: -5.0
      origin_y: -5.0
      resolution: 0.05

# ── Local Costmap ──
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      robot_base_frame: base_footprint
      footprint: "[ [0.18, 0.14], [0.18, -0.14], [-0.18, -0.14], [-0.18, 0.14] ]"
      footprint_padding: 0.03
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          sensor_frame: base_scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      observation_sources: scan
      track_unknown_space: true
      transform_tolerance: 0.2
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05

# ── Global Planner ──
planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

# ── Local Planner (Regulated Pure Pursuit) ──
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.2
      max_linear_accel: 1.0
      max_linear_decel: 0.5
      lookahead_dist: 0.4
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.7
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 0.6
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.01
      regulated_linear_scaling_min_speed: 0.0
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 1.0
      max_linear_vel: 0.3
      max_angular_vel: 1.0

# ── Behavior Tree ──
bt_navigator:
  ros__parameters:
    use_sim_time: True
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node

# ── Recovery Behaviors ──
recoveries_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
      sim_granularity: 0.017
      min_rotational_vel: 0.2
      max_rotational_vel: 0.6
    backup:
      plugin: "nav2_recoveries/BackUp"
      sim_granularity: 0.025
      min_linear_vel: -0.1
      max_linear_vel: -0.2
    wait:
      plugin: "nav2_recoveries/Wait"

# ── Map Server ──
map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "~/maps/my_isaac_map.yaml"
```

### 3.2 Behavior Tree XML

`~/isaac-step-curriculum/config/navigate_w_replanning_and_recovery.xml`:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="NavigateToPose">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <Sequence>
          <ComputePathToPose
            goal="{goal}"
            path="{path}"
            planner_id="GridBased"/>
          <GoalReached/>
        </Sequence>
      </RateController>
      <FollowPath
        path="{path}"
        controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

---

## 4. Nav2 실행

### 4.1 전체 실행 순서

```bash
# ════════════════════════════════════════════════════════════
# Nav2 with Isaac Sim — 5 Terminal Setup
# ════════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim (LiDAR + Odometry + TF + cmd_vel)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step14_ros2_nav2.py

# 터미널 2: Map Server + Nav2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# Map Server 실행
ros2 run nav2_map_server map_server \
  ~/isaac-step-curriculum/config/nav2_params.yaml &

# Lifecycle Manager (Map Server 활성화)
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args -p node_names:="['map_server']" \
  -p autostart:="True" &

# Nav2 전체 실행 (Bringup)
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/isaac-step-curriculum/config/nav2_params.yaml \
  use_sim_time:=True

# ════════════════════════════════════════════════════════════
# 터미널 3: rviz2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run nav2_bringup rviz_launch.py

# 또는 수동 실행:
rviz2 -d ~/isaac-step-curriculum/config/nav2_default_view.rviz

# ════════════════════════════════════════════════════════════
# 터미널 4: rqt_graph (선택)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rqt_graph

# ════════════════════════════════════════════════════════════
# 터미널 5: 모니터링
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /plan
ros2 topic echo /cmd_vel
```

### 4.2 Nav2 Bringup 실행 방법

**방법 A: 모든 Nav2 노드를 한 번에 실행**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/isaac-step-curriculum/config/nav2_params.yaml \
  use_sim_time:=True
```

**방법 B: 개별 노드 실행**
```bash
# 하나씩 실행 (디버깅용)
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=~/maps/my_isaac_map.yaml

# AMCL
ros2 run nav2_amcl amcl --ros-args \
  --params-file ~/isaac-step-curriculum/config/nav2_params.yaml

# Planner Server
ros2 run nav2_planner planner_server --ros-args \
  --params-file ~/isaac-step-curriculum/config/nav2_params.yaml

# Controller Server
ros2 run nav2_controller controller_server --ros-args \
  --params-file ~/isaac-step-curriculum/config/nav2_params.yaml

# BT Navigator
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file ~/isaac-step-curriculum/config/nav2_params.yaml

# Lifecycle Manager (모든 노드 활성화)
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args -p node_names:="['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']" \
  -p autostart:="True"
```

---

## 5. rviz2에서 Navigation

### 5.1 Nav2 Toolbar 사용

rviz2 실행 후:

1. **Panels > Add > Navigation 2** (Nav2 패널 표시)
2. **2D Pose Estimate** 버튼 클릭 → TurtleBot3 위치 클릭 + 방향 드래그
3. **Nav2 Goal** 버튼 클릭 → 목표 위치 클릭 + 방향 드래그
4. 자율 주행 시작!

### 5.2 초기 위치 설정

```bash
# 명령행으로 초기 위치 설정
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
{
  header: { frame_id: 'map' },
  pose: {
    pose: {
      position: { x: -2.0, y: -2.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    },
    covariance: [
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0.068
    ]
  }
}"
```

### 5.3 Goal Pose 전송

```bash
# 명령행으로 Goal 전송
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
{
  header: { frame_id: 'map' },
  pose: {
    position: { x: 2.0, y: 2.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
  }
}"

# 또는 Nav2 Action 사용
ros2 action send_goal /navigate_to_pose nav2_msgs/NavigateToPose "
{
  pose: {
    header: { frame_id: 'map' },
    pose: {
      position: { x: 2.0, y: 2.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
    }
  }
}"
```

---

## 6. Python Nav2 Simple Commander

### 6.1 Simple Commander 예제

```python
#!/usr/bin/env python3
"""
Nav2 Simple Commander — Python에서 Goal Pose 전송
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 초기 위치 설정
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -2.0
    initial_pose.pose.orientation.w = 1.0
    
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to be active
    navigator.waitUntilNav2Active()
    
    print("Nav2 is ready! Sending goal poses...")
    
    # Goal Pose 1
    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = 2.0
    goal_pose_1.pose.position.y = 2.0
    goal_pose_1.pose.orientation.w = 0.707
    goal_pose_1.pose.orientation.z = 0.707
    
    print(f"Navigating to: ({goal_pose_1.pose.position.x}, {goal_pose_1.pose.position.y})")
    navigator.goToPose(goal_pose_1)
    
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and feedback.navigation_duration:
            print(f"  Distance remaining: {feedback.distance_remaining:.2f}m, "
                  f"Duration: {feedback.navigation_duration.sec}s")
        time.sleep(0.5)
    
    result = navigator.getResult()
    print(f"Navigation result: {result}")
    
    # 선택: 연속 Goal Poses
    waypoints = [
        (2.0, 1.0, 0.0, 1.0),
        (1.0, -1.0, 0.0, 1.0),
        (-1.0, -1.0, 0.0, 1.0),
        (-2.0, 2.0, 0.707, 0.707),
    ]
    
    for x, y, z, w in waypoints:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            time.sleep(0.5)
        print(f"  Reached waypoint: ({x}, {y})")
    
    print("All waypoints completed!")
    
    # 지도 상의 좌표 리스트로 경로 따라가기
    path_poses = []
    for x, y in [(2.0, 0.0), (2.0, -1.0), (1.0, -2.0), (-1.0, -2.0), (-2.0, -1.0), (-2.0, 0.0), (-2.0, 1.0), (-1.0, 2.0), (1.0, 2.0), (2.0, 1.0)]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path_poses.append(pose)
    
    navigator.followWaypoints(path_poses)
    
    while not navigator.isTaskComplete():
        time.sleep(0.5)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.2 실습: rviz2에서 Goal Pose 테스트

```bash
# 1. Isaac Sim 실행 (LiDAR + Odometry + TF 발행)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step14_ros2_nav2.py

# 2. Nav2 실행
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=~/isaac-step-curriculum/config/nav2_params.yaml \
  use_sim_time:=True

# 3. rviz2 실행
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d ~/isaac-step-curriculum/config/nav2_default_view.rviz

# 4. Python Simple Commander 실행
source ~/isaac-step-curriculum/env_isaacsim/bin/activate
export ROS_DOMAIN_ID=0
python ~/isaac-step-curriculum/code/phase-2/step14_nav2_commander.py
```

---

## 7. Costmap 이해

### 7.1 Costmap 레이어

```
┌─────────────────────────────────────┐
│          Global Costmap             │
│  ┌───────────────────────────────┐  │
│  │   Static Layer (Map)          │  │  ← 저장된 Occupancy Grid
│  └───────────────────────────────┘  │
│  ┌───────────────────────────────┐  │
│  │   Inflation Layer             │  │  ← 장애물 팽창
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│          Local Costmap              │
│  ┌───────────────────────────────┐  │
│  │   Obstacle Layer (LiDAR)      │  │  ← 실시간 장애물
│  └───────────────────────────────┘  │
│  ┌───────────────────────────────┐  │
│  │   Inflation Layer             │  │  ← 장애물 팽창
│  └───────────────────────────────┘  │
│  ← Rolling Window (3m x 3m)        │
└─────────────────────────────────────┘
```

### 7.2 Costmap 값의 의미

| 값 | 의미 | 색상 |
|----|------|------|
| 0-50 | 자유 공간 (Free) | 검정 |
| 50-97 | 위험 공간 (Cautious) | 회색 |
| 98 | Inscribed (로봇과 충돌) | 빨강 |
| 99 | Lethal (죽음) | 빨강 |
| 100 | Unknown | 파랑 |

---

## 8. Planner 튜닝

### 8.1 Global Planner 옵션

```yaml
# NavFn Planner (기본)
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.5
  use_astar: false
  allow_unknown: true

# Smac Hybrid-A* (더 나은 경로)
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner"
  tolerance: 0.5
  downsample_costmap: false
  allow_unknown: true
  max_iterations: 1000000
  max_on_approach_iterations: 1000
  cost_travel_multiplier: 2.0
  smooth: true
  minimum_turning_radius: 0.2
  use_rotate_to_heading: true
```

### 8.2 Local Planner 옵션

```yaml
# Regulated Pure Pursuit (간단, 안정적)
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.2
  lookahead_dist: 0.5

# DWB (Dynamic Window Approach, 더 정교)
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  min_vel_x: 0.0
  max_vel_x: 0.3
  min_vel_y: 0.0
  max_vel_y: 0.0
  max_vel_theta: 1.0
  min_speed_xy: 0.0
  max_speed_xy: 0.3
  min_speed_theta: 0.0
  acc_lim_x: 0.5
  acc_lim_y: 0.0
  acc_lim_theta: 0.5
  xy_goal_tolerance: 0.1
  yaw_goal_tolerance: 0.1
  transform_tolerance: 0.2
```

### 8.3 튜닝 체크리스트

| 문제 | 파라미터 | 조정 |
|------|---------|------|
| 경로가 벽에 너무 가까움 | `inflation_radius` | 증가 (0.55 → 0.7) |
| 로봇이 흔들림 | `lookahead_dist` | 증가 (0.4 → 0.6) |
| 너무 느림 | `desired_linear_vel` | 증가 (0.2 → 0.3) |
| 회전이 너무 느림 | `max_angular_vel` | 증가 (1.0 → 1.5) |
| 회전이 너무 급함 | `rotate_to_heading_min_angle` | 증가 (0.785 → 1.0) |
| 장애물에 너무 민감 | `cost_scaling_factor` | 감소 (3.0 → 2.0) |

---

## 9. Behavior Tree 상세

### 9.1 NavigateToPose BT 구조

```
NavigateToPose
  └── PipelineSequence (NavigateWithReplanning)
        ├── RateController (1Hz)
        │     └── Sequence
        │           ├── ComputePathToPose (Global Planner)
        │           └── GoalReached?
        └── FollowPath (Local Planner + Controller)

Recovery Subtree (실패 시):
  ├── ClearCostmap
  ├── Spin (360° 회전)
  ├── ClearCostmap
  ├── Backup (후진)
  ├── ClearCostmap
  └── Wait (대기)
```

### 9.2 사용자 정의 BT 생성

`~/isaac-step-curriculum/config/navigate_slow_and_safe.xml`:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="NavigateToPose">
    <PipelineSequence name="NavigateSafe">
      <RateController hz="2.0">
        <Sequence>
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <GoalReached/>
        </Sequence>
      </RateController>
      <Sequence>
        <SpeedController speed="0.15">
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </SpeedController>
      </Sequence>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

---

## 10. 문제 해결 (Troubleshooting)

### 문제 1: Nav2가 /cmd_vel을 발행하지 않습니다.

**확인:**
```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# AMCL이 위치를 추정하는지 확인
ros2 topic echo /amcl_pose --once

# Costmap이 로드되었는지 확인
ros2 topic echo /global_costmap/costmap --once
```

**해결:**
```bash
# 2D Pose Estimate로 초기 위치 설정
# 또는 명령행으로 설정
```

### 문제 2: 로봇이 제자리에서 회전만 합니다.

**원인:**
- Global Planner가 경로를 찾지 못함
- Map과 실제 환경이 일치하지 않음
- Goal Pose가 도달 불가능한 위치

**해결:**
```bash
# 계획된 경로 확인
ros2 topic echo /plan --once | head

# 경로가 비어있지 않은지 확인
# rviz2에서 /plan 토픽 표시
```

### 문제 3: 로봇이 장애물에 너무 가까이 갑니다.

**해결:** Inflation Radius 증가
```yaml
inflation_layer:
  inflation_radius: 0.7  # 0.55 → 0.7
```

### 문제 4: Nav2 노드가 활성화되지 않습니다.

**확인:**
```bash
# Lifecycle 상태 확인
ros2 lifecycle list /map_server
ros2 lifecycle list /amcl
ros2 lifecycle list /planner_server
ros2 lifecycle list /controller_server

# 모든 노드 수동 활성화
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args -p node_names:="['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']" \
  -p autostart:="True"
```

### 문제 5: Map Server가 Map을 로드하지 못합니다.

**해결:**
```bash
# Map 파일 확인
file ~/maps/my_isaac_map.pgm
cat ~/maps/my_isaac_map.yaml

# YAML의 image 경로가 절대 경로인지 확인
# 절대 경로 사용 권장
```

---

## 11. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Nav2 설치 | navigation2, nav2-bringup 설치 |
| ✅ Costmap | Global + Local Costmap 설정 |
| ✅ Global Planner | NavFn / Smac 경로 계획 |
| ✅ Local Planner | Regulated Pure Pursuit / DWB |
| ✅ Behavior Tree | NavigateToPose 동작 제어 |
| ✅ Recovery | Spin / Backup / Wait 복구 |
| ✅ Simple Commander | Python으로 Goal Pose 전송 |
| ✅ 파라미터 튜닝 | 속도, 안전 거리, 회전 최적화 |

### Nav2 파이프라인 완성

```
사용자 (rviz2 Goal Pose)
    │
    ▼ NavigateToPose Action
Behavior Tree
    │
    ├── ComputePathToPose (Global Planner)
    │       │
    │       ▼
    │   전역 경로 (NavFn / Smac)
    │
    └── FollowPath (Local Planner)
            │
            ▼
        속도 명령 (Regulated Pure Pursuit)
            │
            ▼
    Isaac Sim → TurtleBot3 Wheel
            │
            ▼
        /odom + /scan (Feedback)
```

---

## 12. 다음 Step 예고

**Step 15 — ROS2 MoveIt2 + Franka**에서는:
- MoveIt2를 설치하고 Isaac Sim의 Franka Panda 로봇팔과 연동
- Move Group을 설정하고 Planning Scene 구성
- Python MoveIt2 API로 Pick & Place 구현
- RViz2 MoveIt Plugin으로 모션 계획 시각화
- Carthesian Path와 Collision Avoidance

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Nav2 공식 문서 | https://docs.nav2.org/ |
| Nav2 튜토리얼 | https://docs.nav2.org/tutorials/ |
| Nav2 파라미터 | https://docs.nav2.org/configuration/ |
| Behavior Tree | https://docs.nav2.org/behavior_trees/ |
| Simple Commander | https://docs.nav2.org/commander_api/ |
| Isaac Sim Nav2 튜토리얼 | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_nav2.html |
