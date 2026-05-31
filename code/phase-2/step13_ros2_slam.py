"""
Step 13 — ROS2 SLAM in Isaac Sim: LiDAR + Odometry 발행
=========================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step13_ros2_slam.py

사전 준비:
    1. Isaac Sim 5.1.0 (ROS2 Bridge 활성화)
    2. 4개의 터미널 (Isaac Sim / SLAM Toolbox / teleop / rviz2)
    3. SLAM Toolbox 설치: sudo apt install ros-humble-slam-toolbox

목표:
    1. LiDAR (/scan) + Odometry (/odom) + TF 발행
    2. SLAM Toolbox 또는 Cartographer 연동
    3. Mapping 환경 (장애물 + 벽) 생성
    4. 실시간 Occupancy Grid 생성 확인
"""

# ── 1. SimulationApp 초기화 ──
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 13 — ROS2 SLAM in Isaac Sim")
print("=" * 60)

# ── 2. Core API 임포트 ──
import numpy as np
from pxr import Sdf, UsdGeom, Gf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. TurtleBot3 로딩 ──
print("\n[1/6] Loading TurtleBot3...")

ROBOT_PATH = "/World/TurtleBot3"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage(
        "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
        ROBOT_PATH,
    )

robot = Robot(prim_path=ROBOT_PATH, name="TurtleBot3", position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)

# LiDAR 확인
stage = omni.usd.get_context().get_stage()
lidar_prim = stage.GetPrimAtPath("/World/TurtleBot3/base_scan/Lidar")
if lidar_prim:
    print("  + LiDAR found at /World/TurtleBot3/base_scan/Lidar")
else:
    print("  ⚠ LiDAR prim not found - check USD structure")

print("  + TurtleBot3 loaded")

# ── 5. Mapping 환경 생성 (장애물 + 벽) ──
print("\n[2/6] Creating mapping environment...")

# 장애물 배치
obstacle_positions = [
    (1.5, 0.0, 0.5),
    (0.0, 1.5, 0.5),
    (-1.5, 0.0, 0.5),
    (0.0, -1.5, 0.5),
    (2.0, 1.8, 0.3),
    (-2.0, -1.8, 0.3),
    (2.5, -1.0, 0.4),
    (-2.5, 2.0, 0.4),
    (0.8, 2.5, 0.6),
    (-0.8, -2.5, 0.6),
    (2.8, 0.5, 0.35),
    (-2.8, -0.5, 0.35),
]

for i, (x, y, h) in enumerate(obstacle_positions):
    VisualCuboid(
        prim_path=f"/World/Obstacles/Box_{i:02d}",
        name=f"obstacle_{i}",
        position=np.array([x, y, h / 2]),
        scale=np.array([0.3, 0.3, h]),
        color=np.array([0.2, 0.6, 0.8]),
    )

# 벽 배치 (7x7 미터 공간)
wall_configs = [
    {"pos": (3.5, 0.0, 1.0), "scale": (0.1, 7.0, 2.0)},   # 북쪽 벽
    {"pos": (-3.5, 0.0, 1.0), "scale": (0.1, 7.0, 2.0)},  # 남쪽 벽
    {"pos": (0.0, 3.5, 1.0), "scale": (7.0, 0.1, 2.0)},   # 동쪽 벽
    {"pos": (0.0, -3.5, 1.0), "scale": (7.0, 0.1, 2.0)},  # 서쪽 벽
    # 내부 벽 (L자형)
    {"pos": (1.0, 0.0, 0.75), "scale": (0.8, 0.1, 1.5)},
    {"pos": (0.0, -1.5, 0.5), "scale": (0.1, 1.5, 1.0)},
]

for i, wall in enumerate(wall_configs):
    VisualCuboid(
        prim_path=f"/World/Walls/Wall_{i:02d}",
        name=f"wall_{i}",
        position=np.array(wall["pos"]),
        scale=np.array(wall["scale"]),
        color=np.array([0.5, 0.3, 0.1]),
    )

print(f"  + {len(obstacle_positions)} obstacles created")
print(f"  + {len(wall_configs)} walls created")
print("  + Environment: 7m x 7m with internal structures")

# ── 6. SLAM Pipeline OmniGraph 생성 ──
print("\n[3/6] Creating SLAM Pipeline OmniGraph...")

graph_config = {
    "graph_path": "/ActionGraph/SLAM_Pipeline",
    "evaluator_name": "execution",
}

# 기존 Graph 제거
if og.Controller.graph_exists("/ActionGraph/SLAM_Pipeline"):
    stage.RemovePrim(Sdf.Path("/ActionGraph/SLAM_Pipeline"))

(
    graph_handle,
    graph_nodes,
    _,
    _,
) = og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            # Core
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),

            # Subscriber: /cmd_vel
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),

            # Publisher: Odometry
            ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),

            # Publisher: LaserScan
            ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
            ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),

            # Publisher: TF
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

            # Publisher: JointState
            ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
            ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            # Tick → 모든 Exec
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

            # Context
            ("Context.outputs:context", "SubTwist.inputs:context"),
            ("Context.outputs:context", "PubOdom.inputs:context"),
            ("Context.outputs:context", "PubScan.inputs:context"),
            ("Context.outputs:context", "PubTF.inputs:context"),
            ("Context.outputs:context", "PubJoint.inputs:context"),

            # /cmd_vel → Differential Drive
            ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
            ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),

            # Differential → Articulation
            ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),

            # Odometry
            ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
            ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),

            # LaserScan
            ("ReadScan.outputs:rangeData", "PubScan.inputs:rangeData"),

            # JointState
            ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
            ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
            ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Context
            ("Context.inputs:domain_id", 0),

            # /cmd_vel Subscriber
            ("SubTwist.inputs:topicName", "/cmd_vel"),

            # Differential Controller
            ("DiffCtrl.inputs:wheelDistance", 0.141),
            ("DiffCtrl.inputs:wheelRadius", 0.033),
            ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
            ("DiffCtrl.inputs:maxAngularSpeed", 2.0),

            # Articulation Controller
            ("ArticCtrl.inputs:robotPath", Sdf.Path(ROBOT_PATH)),
            ("ArticCtrl.inputs:jointNames",
             ["left_wheel_joint", "right_wheel_joint"]),

            # Odometry Publisher
            ("ReadOdom.inputs:chassisPrim",
             Sdf.Path(f"{ROBOT_PATH}/base_link")),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:frameId", "odom"),
            ("PubOdom.inputs:childFrameId", "base_footprint"),

            # LaserScan Publisher
            ("ReadScan.inputs:laserPrim",
             Sdf.Path(f"{ROBOT_PATH}/base_scan/Lidar")),
            ("PubScan.inputs:topicName", "/scan"),
            ("PubScan.inputs:frameId", "base_scan"),
            ("PubScan.inputs:rangeMin", 0.1),
            ("PubScan.inputs:rangeMax", 3.5),
            ("PubScan.inputs:rangeThreshold", 3500.0),

            # JointState Publisher
            ("ReadJoint.inputs:robotPrim", Sdf.Path(ROBOT_PATH)),
            ("PubJoint.inputs:topicName", "/joint_states"),
        ],
    },
)
print("  + SLAM Pipeline Graph created!")
print("    - Subscribes: /cmd_vel")
print("    - Publishes: /odom, /scan, /tf, /joint_states")

# ── 7. LiDAR 속성 확인 ──
print("\n[4/6] Verifying LiDAR attributes...")

if lidar_prim:
    # 기존 속성 확인
    for attr_name in ["range", "horizontalFov", "rotationRate", 
                       "drawSensors", "sensorRange"]:
        attr = lidar_prim.GetAttribute(attr_name)
        if attr:
            print(f"  + {attr_name} = {attr.Get()}")

    # 필요한 속성 설정
    lidar_prim.GetAttribute("range").Set(3.5)
    lidar_prim.GetAttribute("horizontalFov").Set(360.0)
    lidar_prim.GetAttribute("rotationRate").Set(10.0)
    lidar_prim.CreateAttribute("drawSensors", Sdf.ValueTypeNames.Bool).Set(True)
    print("  + LiDAR configured: range=3.5m, FOV=360°, rate=10Hz")

# ── 8. 시뮬레이션 루프 ──
print("\n[5/6] Running simulation...")
print()
print("  Open 3 NEW terminals and run:")
print()
print("  # Terminal 1 — SLAM Toolbox:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  ros2 run slam_toolbox async_slam_toolbox_node")
print()
print("  # Terminal 2 — Keyboard Teleop:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print()
print("  # Terminal 3 — rviz2:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  rviz2")
print("  # → Add /map (topic), /scan (LaserScan)")
print("  # → Fixed Frame: map")
print()
print("  [TIP] Move slowly (0.15 m/s) with the teleop keyboard.")
print("  [TIP] Cover all areas twice for loop closure.")
print("  [TIP] After mapping: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map")
print()

# 시뮬레이션 루프 (60초)
for i in range(1200):
    world.step(render=True)

    if i % 100 == 0:
        print(f"  Frame {i:4d}: Publishing /scan + /odom + /tf...")

    if i == 50:
        print()
        print("  ====== BEGIN MAPPING =====")
        print("  1. Press 'I' in teleop terminal to move forward")
        print("  2. Explore the 7x7m environment slowly")
        print("  3. Watch rviz2 for real-time map building")
        print()

# ── 9. 요약 ──
print("\n[6/6] Step 13 Summary")
print("-" * 60)
print()
print("  Published topics:")
print("    - /scan      (sensor_msgs/LaserScan)  ← LiDAR")
print("    - /odom      (nav_msgs/Odometry)      ← Odometry")
print("    - /tf        (tf2_msgs/TFMessage)      ← Transforms")
print("    - /joint_states (sensor_msgs/JointState)")
print()
print("  Subscribed topics:")
print("    - /cmd_vel   (geometry_msgs/Twist)    ← Teleop/Nav2")
print()
print("  External tools needed:")
print("    - slam_toolbox (async_slam_toolbox_node)")
print("    - teleop_twist_keyboard")
print("    - rviz2")
print("    - nav2_map_server (map_saver_cli)")
print()
print("  Key concepts:")
print("  - SLAM: Simultaneous Localization and Mapping")
print("  - Occupancy Grid: /map (nav_msgs/OccupancyGrid)")
print("  - Loop Closure: Detecting revisited locations")
print("  - Scan Matching: Aligning laser scans to build map")
print("  - TF: map ← odom ← base_footprint ← base_scan")
print("=" * 60)

simulation_app.close()
