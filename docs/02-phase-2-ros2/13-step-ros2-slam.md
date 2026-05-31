# Step 13 — ROS2 SLAM in Isaac Sim

> **소요 시간**: 90분
> **난이도**: ★★★★☆ (고급)
> **선수 조건**: Step 12 완료 (ROS2 Teleop)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim에서 **LiDAR 스캔 데이터** (`/scan`)를 ROS2로 발행한다
2. **SLAM Toolbox** 또는 **Cartographer**를 실행하여 실시간 Mapping한다
3. TurtleBot3를 원격 조종하면서 **실내 지도(Map)**를 생성한다
4. 생성된 **Occupancy Grid Map**을 저장하고 시각화한다
5. Mapping된 환경에서 로봇의 **자세 추정(AMCL 없이 SLAM 활용)**을 이해한다

---

## 1. 시스템 아키텍처

```
┌─────────────────────┐          ┌───────────────────────────────┐
│    Isaac Sim        │          │        ROS2 Ecosystem         │
│                     │   /scan  │                               │
│  ┌───────────────┐  │─────────►│  ┌─────────────────────────┐  │
│  │ LiDAR         │  │          │  │    SLAM Toolbox          │  │
│  │ (LaserScan)   │  │          │  │   (slam_mapping)         │  │
│  └───────┬───────┘  │          │  └──────────┬──────────────┘  │
│          │          │          │             │                  │
│  ┌───────▼───────┐  │  /odom   │  ┌──────────▼──────────────┐  │
│  │ Odometry      │  │─────────►│  │    OccupancyGrid         │  │
│  │ + TF          │  │          │  │    (/map)                │  │
│  └───────────────┘  │          │  └─────────────────────────┘  │
│                     │          │                               │
│  ┌───────────────┐  │  /tf     │  ┌─────────────────────────┐  │
│  │ TF Tree       │  │─────────►│  │    rviz2                │  │
│  └───────────────┘  │          │  │    (지도 시각화)         │  │
│                     │          │  └─────────────────────────┘  │
│  ┌───────────────┐  │  /cmd_vel│                               │
│  │ TurtleBot3    │  │◄─────────│  teleop_keyboard              │
│  │ (Controller)  │  │          │                               │
│  └───────────────┘  │          └───────────────────────────────┘
└─────────────────────┘
```

### 1.1 필요 토픽

| 토픽 | 타입 | 역할 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 360° 스캔 (SLAM 입력) |
| `/odom` | `nav_msgs/Odometry` | Odometry (SLAM 예측) |
| `/tf` | `tf2_msgs/TFMessage` | odom→base_link 변환 |
| `/map` | `nav_msgs/OccupancyGrid` | 생성된 Occupancy Grid (출력) |
| `/cmd_vel` | `geometry_msgs/Twist` | 로봇 제어 (Mapping 중 이동) |

### 1.2 좌표계 구조

```
map (Fixed Frame)
  └── odom (SLAM이 추정한 위치)
        └── base_footprint
              └── base_link
                    ├── base_scan (LiDAR)
                    ├── wheel_left_link
                    └── wheel_right_link
```

> **참고**: SLAM 실행 전에는 `odom`이 기준 좌표계, SLAM 실행 후에는 `map`이 글로벌 기준 좌표계가 됩니다.

---

## 2. Isaac Sim에서 LiDAR 발행 설정

### 2.1 OmniGraph: LiDAR Scan 발행

**필요 노드:**
- `On Playback Tick`
- `ROS2 Context`
- `Isaac Read LaserScan` (omni.isaac.core_nodes)
- `ROS2 Publish LaserScan` (omni.isaac.ros2_bridge)

```python
# LiDAR Scan 발행 Graph
og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
            ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "ReadScan.inputs:execIn"),
            ("OnTick.outputs:tick", "PubScan.inputs:execIn"),
            ("Context.outputs:context", "PubScan.inputs:context"),
            ("ReadScan.outputs:rangeData", "PubScan.inputs:rangeData"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ReadScan.inputs:laserPrim",
             Sdf.Path("/World/TurtleBot3/base_scan/Lidar")),
            ("PubScan.inputs:topicName", "/scan"),
            ("PubScan.inputs:frameId", "base_scan"),
            ("PubScan.inputs:rangeMin", 0.1),
            ("PubScan.inputs:rangeMax", 3.5),
            ("PubScan.inputs:rangeThreshold", 3500.0),
        ],
    },
)
```

### 2.2 LiDAR 속성 설정 (USD)

TurtleBot3의 LiDAR 센서 속성을 확인/설정:

```python
import omni.usd
from pxr import Usd, UsdGeom, Sdf

stage = omni.usd.get_context().get_stage()
lidar_prim = stage.GetPrimAtPath("/World/TurtleBot3/base_scan/Lidar")

if lidar_prim:
    # LiDAR 속성 설정
    lidar_prim.GetAttribute("range").Set(3.5)
    lidar_prim.GetAttribute("horizontalFov").Set(360.0)
    lidar_prim.GetAttribute("rotationRate").Set(10.0)
    
    # LaserScan 발행 설정 확인
    lidar_prim.CreateAttribute("drawSensors", Sdf.ValueTypeNames.Bool).Set(True)
    print("  + LiDAR configured: range=3.5m, FOV=360°, rate=10Hz")
else:
    print("  ⚠ LiDAR prim not found at /World/TurtleBot3/base_scan/Lidar")
```

---

## 3. SLAM Toolbox 설치 및 실행

### 3.1 SLAM Toolbox 설치

```bash
# SLAM Toolbox 설치 (ROS2 Humble)
sudo apt install -y ros-humble-slam-toolbox

# 또는 소스 빌드
cd ~/ros2_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble
cd ~/ros2_ws
colcon build --packages-select slam_toolbox
source install/setup.bash
```

### 3.2 SLAM Toolbox 실행 (Online Async)

```bash
# 터미널 1: Isaac Sim
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./isaac-sim.selector.sh  # ROS2 Bridge 활성화 + LiDAR Scan Graph

# Isaac Sim에서 Play(▶)

# 터미널 2: SLAM Toolbox (Online Async)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

ros2 run slam_toolbox async_slam_toolbox_node

# 터미널 3: Keyboard Teleop
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 터미널 4: rviz2 (지도 확인)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d ~/isaac-step-curriculum/config/slam.rviz
```

### 3.3 SLAM Toolbox 설정 파일

`~/isaac-step-curriculum/config/slam_toolbox_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_loss_function: None

    # Input topics
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping  # mapping / localization

    # Map parameters
    resolution: 0.05
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # General
    enable_interactive_mode: true
    map_update_interval: 1.0  # seconds
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.

    # Ceres
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_big: 0.45
    loop_match_minimum_response_ratio: 0.35
    correlation_accumulation_threshold: 0.15
    correlation_accumulation_cloud_size: 500
```

### 3.4 SLAM 실행 순서

```bash
# === Phase 1: SLAM 시작 ===
# 1. Isaac Sim Play
# 2. SLAM Toolbox 실행
# 3. rviz2에서 지도 확인 (빈 공간)

# === Phase 2: Mapping ===
# 4. Keyboard Teleop으로 천천히 이동
#    - 직진: I
#    - 회전: J/L
#    - 모든 공간을 2회 이상 커버
#    - 급회전 금지, 천천히 주행

# === Phase 3: 지도 저장 ===
# 5. rviz2에서 지도 저장
#    또는 명령행
```

---

## 4. Cartographer (대안)

### 4.1 Cartographer 설치

```bash
# Cartographer 설치 (ROS2 Humble)
sudo apt install -y ros-humble-cartographer-ros

# 또는 소스 빌드
sudo apt install -y python3-wstool python3-rosdep ninja-build stow
cd ~/ros2_ws/src
git clone https://github.com/ros2/cartographer.git -b ros2
git clone https://github.com/ros2/cartographer_ros.git -b ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cartographer cartographer_ros
source install/setup.bash
```

### 4.2 Cartographer 실행

```bash
# Cartographer Online
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch cartographer_ros cartographer_ros.launch.py \
  configuration_directory:=~/isaac-step-curriculum/config \
  configuration_basename:=cartographer_isaac.lua

# rviz2에서 Cartographer 결과 확인
rviz2 -d $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/configuration_files/demo.rviz
```

### 4.3 Cartographer 설정 (cartographer_isaac.lua)

```lua
-- Cartographer Lua 설정 파일
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_scan",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003

MAP_BUILDER.use_trajectory_builder_2d = true

return options
```

---

## 5. 지도 저장 및 활용

### 5.1 Occupancy Grid 저장 (map_server)

```bash
# map_saver_cli로 저장
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# 방법 1: map_saver_cli
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_isaac_map

# 방법 2: map_saver (구버전)
ros2 run nav2_map_server map_saver -f ~/maps/my_isaac_map
```

**출력 파일:**
- `~/maps/my_isaac_map.pgm` — Occupancy Grid 이미지
- `~/maps/my_isaac_map.yaml` — Map 메타데이터

#### my_isaac_map.yaml

```yaml
image: my_isaac_map.pgm
mode: trinary
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 5.2 rviz2에서 지도 표시

```
[rviz2]
Displays > Add > Map
  Topic: /map
  Color Scheme: map
  Alpha: 0.7

Displays > Add > LaserScan
  Topic: /scan
  Size (m): 0.03
  Color: 255, 255, 255

Displays > Add > RobotModel
  Description Topic: robot_description
```

### 5.3 생성된 지도 시각화

```python
import matplotlib.pyplot as plt
import cv2
import numpy as np
import yaml

def visualize_map(map_path="~/maps/my_isaac_map.yaml"):
    """저장된 Occupancy Grid Map 시각화"""
    
    # YAML 메타데이터 로드
    with open(map_path, 'r') as f:
        metadata = yaml.safe_load(f)
    
    # PGM 이미지 로드
    pgm_path = metadata['image']
    if not pgm_path.startswith('/'):
        import os
        pgm_path = os.path.join(os.path.dirname(map_path), pgm_path)
    
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    
    print(f"Map size: {img.shape[1]} x {img.shape[0]} pixels")
    print(f"Resolution: {metadata['resolution']} m/pixel")
    print(f"Origin: {metadata['origin']}")
    
    # 실제 크기 계산
    width_m = img.shape[1] * metadata['resolution']
    height_m = img.shape[0] * metadata['resolution']
    print(f"Physical size: {width_m:.2f} x {height_m:.2f} meters")
    
    # 시각화
    plt.figure(figsize=(10, 10))
    plt.imshow(img, cmap='gray', 
               extent=[0, width_m, 0, height_m])
    plt.title('SLAM Map (Occupancy Grid)')
    plt.xlabel('X [meters]')
    plt.ylabel('Y [meters]')
    plt.grid(alpha=0.3)
    plt.colorbar(label='Occupancy')
    plt.show()

# 실행
visualize_map()
```

---

## 6. 전체 통합 파이프라인

### 6.1 Python 통합 스크립트 (Isaac Sim 전용)

```python
"""
Isaac Sim에서 LiDAR + Odometry + TF를 발행하는
완전한 SLAM 지원 스크립트
"""
import omni.graph.core as og
from pxr import Sdf
import carb
import numpy as np

def setup_slam_pipeline(robot_path="/World/TurtleBot3"):
    """SLAM에 필요한 모든 ROS2 Publisher/Subscriber Graph 생성"""
    
    graph_config = {
        "graph_path": "/ActionGraph/SLAM_Pipeline",
        "evaluator_name": "execution",
    }
    
    og.Controller.edit(
        graph_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                # ── Core ──
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                
                # ── Subscriber: /cmd_vel ──
                ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
                ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
                
                # ── Publisher: Odometry ──
                ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
                ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                
                # ── Publisher: LaserScan ──
                ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
                ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                
                # ── Publisher: TF ──
                ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                
                # ── Publisher: Joint State ──
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
                
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                
                ("DiffCtrl.inputs:wheelDistance", 0.141),
                ("DiffCtrl.inputs:wheelRadius", 0.033),
                ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
                ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
                
                ("ArticCtrl.inputs:robotPath", Sdf.Path(robot_path)),
                ("ArticCtrl.inputs:jointNames",
                 ["left_wheel_joint", "right_wheel_joint"]),
                
                ("ReadOdom.inputs:chassisPrim",
                 Sdf.Path(f"{robot_path}/base_link")),
                ("PubOdom.inputs:topicName", "/odom"),
                ("PubOdom.inputs:frameId", "odom"),
                ("PubOdom.inputs:childFrameId", "base_footprint"),
                
                ("ReadScan.inputs:laserPrim",
                 Sdf.Path(f"{robot_path}/base_scan/Lidar")),
                ("PubScan.inputs:topicName", "/scan"),
                ("PubScan.inputs:frameId", "base_scan"),
                ("PubScan.inputs:rangeMin", 0.1),
                ("PubScan.inputs:rangeMax", 3.5),
                ("PubScan.inputs:rangeThreshold", 3500.0),
                
                ("ReadJoint.inputs:robotPrim", Sdf.Path(robot_path)),
                ("PubJoint.inputs:topicName", "/joint_states"),
            ],
        },
    )
    
    print("+ SLAM Pipeline Graph created!")
    print(f"  - Robot: {robot_path}")
    print("  - Subscriber: /cmd_vel")
    print("  - Publishers: /odom, /scan, /tf, /joint_states")

# 사용 예
# setup_slam_pipeline("/World/TurtleBot3")
```

### 6.2 환경 설정 (장애물 배치)

Mapping을 위한 환경을 Isaac Sim에 생성:

```python
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from pxr import UsdGeom, Gf

# 맵핑용 장애물 배치 (Visual)
obstacles = [
    (1.5, 0.0, 0.5),    # 박스 1
    (0.0, 1.5, 0.5),    # 박스 2
    (-1.5, 0.0, 0.5),   # 박스 3
    (0.0, -1.5, 0.5),   # 박스 4
    (2.0, 1.5, 0.3),    # 박스 5
    (-2.0, -1.5, 0.3),  # 박스 6
    (2.5, -1.0, 0.4),   # 박스 7
    (-2.5, 2.0, 0.4),   # 박스 8
]

# 벽 (벽면)
walls = [
    # 긴 벽 1
    {"pos": (3.5, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    # 긴 벽 2
    {"pos": (-3.5, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    # 긴 벽 3
    {"pos": (0.0, 3.5, 1.0), "scale": (6.0, 0.1, 2.0)},
    # 긴 벽 4
    {"pos": (0.0, -3.5, 1.0), "scale": (6.0, 0.1, 2.0)},
]

# 장애물 생성
for i, (x, y, h) in enumerate(obstacles):
    VisualCuboid(
        prim_path=f"/World/Obstacles/Box_{i:02d}",
        name=f"box_{i}",
        position=np.array([x, y, h / 2]),
        scale=np.array([0.4, 0.4, h]),
        color=np.array([0.2, 0.6, 0.8]),
    )

# 벽 생성
for i, wall in enumerate(walls):
    VisualCuboid(
        prim_path=f"/World/Walls/Wall_{i:02d}",
        name=f"wall_{i}",
        position=np.array(wall["pos"]),
        scale=np.array(wall["scale"]),
        color=np.array([0.5, 0.3, 0.1]),
    )
```

---

## 7. 문제 해결 (Troubleshooting)

### 문제 1: /scan 토픽이 발행되지 않습니다.

**확인 사항:**
- [ ] LiDAR prim 경로가 올바른가? (`/World/TurtleBot3/base_scan/Lidar`)
- [ ] LiDAR에 `range`, `horizontalFov` 속성이 설정되었는가?
- [ ] `IsaacReadLaserScan` → `ROS2PublishLaserScan` 연결이 올바른가?
- [ ] Isaac Sim에서 Play(▶) 상태인가?

**LiDAR 경로 확인:**
```python
stage = omni.usd.get_context().get_stage()
lidar_prim = stage.GetPrimAtPath("/World/TurtleBot3/base_scan/Lidar")
print("LiDAR prim valid:", bool(lidar_prim))
```

### 문제 2: SLAM Toolbox가 Scan 데이터를 수신하지 않습니다.

**확인:**
```bash
# Scan 토픽 발행 확인
ros2 topic echo /scan --once

# 발행 주기 확인
ros2 topic hz /scan

# TF 트리 확인
ros2 run tf2_tools view_frames
evince frames.pdf &
```

### 문제 3: 맵이 제대로 생성되지 않습니다.

**원인:**
- Mapping 속도가 너무 빠름 (느리게 이동)
- 급회전 (SLAM이 예측 불가)
- 루프 클로징 실패 (같은 장소를 다시 지나가야 함)
- LiDAR range가 너무 짧음 (`maxRange=3.5`가 충분한가?)

**개선:**
```bash
# SLAM Toolbox 파라미터 조정
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args -p minimum_travel_distance:=0.3 \
             -p minimum_travel_heading:=0.3
```

### 문제 4: rviz2가 /map 토픽을 표시하지 않습니다.

**해결:**
```bash
# Fixed Frame을 map으로 설정
# Displays > Global Options > Fixed Frame: map
# 또는 odom으로 테스트 (SLAM이 map을 발행하는지 확인)

# 수동으로 /map 발행 확인
ros2 topic echo /map --once | head -20
```

### 문제 5: rviz2가 너무 느립니다.

**해결:**
```bash
# Decay Time 증가 (점들이 오래 유지됨)
rviz2 -d ~/isaac-step-curriculum/config/slam_rviz_low.rviz

# 또는 Decay Time: 5초로 설정
```

---

## 8. 실습: SLAM Mapping 실행

### 8.1 전체 실행 시나리오

```bash
# ══════════════════════════════════════════════════
# Isaac Sim + SLAM Toolbox + Teleop Mapping
# ══════════════════════════════════════════════════

# 터미널 1: Isaac Sim + SLAM 지원 스크립트
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step13_ros2_slam.py
# (Play는 자동으로 실행됨)

# 터미널 2: SLAM Toolbox
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run slam_toolbox async_slam_toolbox_node

# 터미널 3: rviz2 시각화
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 &
# Displays > Add > Map (topic: /map)
# Displays > Add > LaserScan (topic: /scan)
# Global Options > Fixed Frame: map

# 터미널 4: Keyboard Teleop
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ══════════════════════════════════════════════════
# Mapping 진행
# ══════════════════════════════════════════════════
# 1. TurtleBot3를 천천히 조종 (직진-회전-직진)
# 2. 모든 공간을 골고루 스캔
# 3. 루프 클로징을 위해 같은 장소로 복귀
# 4. rviz2에서 맵이 완성되는 과정 관찰

# ══════════════════════════════════════════════════
# 지도 저장
# ══════════════════════════════════════════════════
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/slam_result_map
```

### 8.2 Mapping 체크리스트

| 단계 | 체크 | 내용 |
|------|------|------|
| 1. 초기화 | □ | Isaac Sim Play 확인 |
| 2. 센서 | □ | `ros2 topic echo /scan` 데이터 확인 |
| 3. SLAM | □ | `async_slam_toolbox_node` 실행 확인 |
| 4. 이동 | □ | Teleop으로 천천히 이동 (0.2 m/s) |
| 5. 맵 | □ | rviz2에서 /map 확인 |
| 6. 루프 | □ | 루프 클로징 성공 확인 |
| 7. 저장 | □ | `map_saver_cli`로 저장 |

---

## 9. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ LiDAR 데이터 발행 | Isaac Sim에서 `/scan` 토픽 발행 |
| ✅ SLAM Toolbox | Online Async SLAM 실행 |
| ✅ Cartographer (대안) | Google Cartographer 연동 |
| ✅ Occupancy Grid | 실시간 Map 생성 및 시각화 |
| ✅ 지도 저장 | map_saver_cli로 .pgm + .yaml 저장 |
| ✅ 환경 생성 | 장애물 + 벽 배치로 Mapping 환경 구축 |

### 아키텍처 요약

```
                   ┌──────────────┐
                   │  Isaac Sim   │
                   │  TurtleBot3  │
                   └──────┬───────┘
                          │
                ┌─────────┼────────────┐
                │         │            │
           /scan     /odom          /tf
                │         │            │
                ▼         ▼            ▼
          ┌─────────────────────────────────┐
          │         SLAM Toolbox            │
          │  (Scan Matching + Loop Closing) │
          └──────────────┬──────────────────┘
                         │
                    /map (OccupancyGrid)
                         │
                         ▼
                    rviz2 + 저장
```

---

## 10. 다음 Step 예고

**Step 14 — ROS2 Navigation2 (Nav2)**에서는:
- 생성된 Map을 바탕으로 자율 주행
- Nav2 Action Server (`NavigateToPose`) 사용
- Global/Local Path Planner 설정
- Recovery Behaviors (복구 동작) 이해
- rviz2에서 Goal Pose를 지정하여 자율 주행

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| SLAM Toolbox 문서 | https://github.com/SteveMacenski/slam_toolbox |
| Cartographer ROS2 | https://google-cartographer-ros.readthedocs.io/ |
| Isaac Sim LiDAR | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_lidar.html |
| Occupancy Grid | https://docs.ros.org/en/humble/Tutorials/Intermediate/Navigation/Tutorial-1.html |
| map_saver_cli | https://docs.nav2.org/configuration/packages/configuring-map-server.html |
