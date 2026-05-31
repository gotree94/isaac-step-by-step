"""
Step 14 — ROS2 Navigation2 (Nav2) in Isaac Sim
================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step14_ros2_nav2.py

사전 준비:
    1. Isaac Sim 5.1.0 (ROS2 Bridge 활성화)
    2. Nav2 설치: sudo apt install ros-humble-navigation2
    3. Step 13에서 생성한 Map 필요 (또는 이 스크립트가 자동 생성)
    4. 3-4개 터미널 (Isaac Sim / Nav2 / rviz2 / simple_commander)

목표:
    1. Isaac Sim에서 LiDAR + Odometry + TF 발행
    2. Nav2 (Global/Local Planner + Costmap) 연동
    3. rviz2에서 2D Pose Estimate + Nav2 Goal 테스트
    4. Python Simple Commander로 자율 주행
    5. Recovery Behaviors 테스트
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
print("Step 14 — ROS2 Navigation2 (Nav2)")
print("=" * 60)

# ── 2. Core API 임포트 ──
import numpy as np
from pxr import Sdf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. TurtleBot3 로딩 ──
print("\n[1/5] Loading TurtleBot3...")

ROBOT_PATH = "/World/TurtleBot3"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage(
        "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
        ROBOT_PATH,
    )

robot = Robot(prim_path=ROBOT_PATH, name="TurtleBot3", position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)
print("  + TurtleBot3 loaded")

# ── 5. Mapping 환경 생성 (Nav2 테스트용) ──
print("\n[2/5] Creating navigation test environment...")

# 장애물
obstacle_positions = [
    (1.2, 0.8, 0.5),
    (-1.0, 1.5, 0.5),
    (0.5, -1.2, 0.5),
    (-0.8, -1.0, 0.5),
    (1.8, -0.5, 0.3),
    (-1.5, 0.0, 0.4),
]

for i, (x, y, h) in enumerate(obstacle_positions):
    VisualCuboid(
        prim_path=f"/World/Obstacles/Box_{i:02d}",
        name=f"obs_{i}",
        position=np.array([x, y, h / 2]),
        scale=np.array([0.3, 0.3, h]),
        color=np.array([0.2, 0.6, 0.8]),
    )

# 벽
wall_configs = [
    {"pos": (2.8, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    {"pos": (-2.8, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    {"pos": (0.0, 2.8, 1.0), "scale": (6.0, 0.1, 2.0)},
    {"pos": (0.0, -2.8, 1.0), "scale": (6.0, 0.1, 2.0)},
]

for i, wall in enumerate(wall_configs):
    VisualCuboid(
        prim_path=f"/World/Walls/Wall_{i:02d}",
        name=f"wall_{i}",
        position=np.array(wall["pos"]),
        scale=np.array(wall["scale"]),
        color=np.array([0.5, 0.3, 0.1]),
    )

print(f"  + {len(obstacle_positions)} obstacles + {len(wall_configs)} walls created")

# ── 6. Nav2 지원 OmniGraph 생성 ──
print("\n[3/5] Creating Nav2 Support Graph...")

graph_config = {
    "graph_path": "/ActionGraph/Nav2_Bridge",
    "evaluator_name": "execution",
}

# 기존 Graph 제거
stage = omni.usd.get_context().get_stage()
if og.Controller.graph_exists("/ActionGraph/Nav2_Bridge"):
    stage.RemovePrim(Sdf.Path("/ActionGraph/Nav2_Bridge"))

(
    graph_handle,
    graph_nodes,
    _,
    _,
) = og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),

            # /cmd_vel → TurtleBot3
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),

            # Odometry
            ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),

            # LaserScan
            ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
            ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),

            # TF + JointState
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
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
            ("ArticCtrl.inputs:robotPath", Sdf.Path(ROBOT_PATH)),
            ("ArticCtrl.inputs:jointNames",
             ["left_wheel_joint", "right_wheel_joint"]),
            ("ReadOdom.inputs:chassisPrim",
             Sdf.Path(f"{ROBOT_PATH}/base_link")),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:frameId", "odom"),
            ("PubOdom.inputs:childFrameId", "base_footprint"),
            ("ReadScan.inputs:laserPrim",
             Sdf.Path(f"{ROBOT_PATH}/base_scan/Lidar")),
            ("PubScan.inputs:topicName", "/scan"),
            ("PubScan.inputs:frameId", "base_scan"),
            ("PubScan.inputs:rangeMin", 0.1),
            ("PubScan.inputs:rangeMax", 3.5),
            ("PubScan.inputs:rangeThreshold", 3500.0),
            ("ReadJoint.inputs:robotPrim", Sdf.Path(ROBOT_PATH)),
            ("PubJoint.inputs:topicName", "/joint_states"),
        ],
    },
)

# LiDAR 속성 확인
lidar_prim = stage.GetPrimAtPath("/World/TurtleBot3/base_scan/Lidar")
if lidar_prim:
    lidar_prim.GetAttribute("range").Set(3.5)
    lidar_prim.GetAttribute("horizontalFov").Set(360.0)
    lidar_prim.GetAttribute("rotationRate").Set(10.0)
    lidar_prim.CreateAttribute("drawSensors", Sdf.ValueTypeNames.Bool).Set(True)

print("  + Nav2 Bridge Graph created!")
print("    - Subscribes: /cmd_vel (from Nav2 Controller)")
print("    - Publishes: /odom, /scan, /tf, /joint_states")

# ── 7. 시뮬레이션 루프 ──
print("\n[4/5] Running simulation for Nav2...")
print()
print("  === Nav2 Setup Instructions ===")
print()
print("  Terminal 1 (this window): Isaac Sim running")
print()
print("  Terminal 2 — Map Server + AMCL:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 run nav2_map_server map_server \\")
print("      ~/isaac-step-curriculum/config/nav2_params.yaml &")
print("    ros2 run nav2_lifecycle_manager lifecycle_manager \\")
print("      --ros-args -p node_names:=\"['map_server']\" \\")
print("      -p autostart:=\"True\"")
print("    # (or use Step 13's saved map)")
print()
print("  Terminal 3 — Nav2 Bringup:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 launch nav2_bringup navigation_launch.py \\")
print("      params_file:=~/isaac-step-curriculum/config/nav2_params.yaml \\")
print("      use_sim_time:=True")
print()
print("  Terminal 4 — rviz2:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    rviz2")
print("    # → Set 2D Pose Estimate (initial pose)")
print("    # → Set Nav2 Goal (goal pose)")
print()
print("  Terminal 5 — Python Commander (optional):")
print("    source ~/isaac-step-curriculum/env_isaacsim/bin/activate")
print("    export ROS_DOMAIN_ID=0")
print("    python ~/isaac-step-curriculum/code/phase-2/step14_nav2_commander.py")
print()

for i in range(1200):
    world.step(render=True)

    if i % 100 == 0:
        print(f"  Frame {i:4d}: Nav2 bridge active — waiting for /cmd_vel from Nav2...")

    if i == 50:
        print()
        print("  Set 2D Pose Estimate in rviz2, then send a Nav2 Goal!")
        print()

# ── 8. 요약 ──
print("\n[5/5] Step 14 Summary")
print("-" * 60)
print()
print("  Data flow for Nav2:")
print()
print("  rviz2 (Goal Pose)")
print("    │")
print("    ▼")
print("  Nav2 Behavior Tree")
print("    │")
print("    ├── Global Planner (NavFn)")
print("    │     └── Global Costmap (Static + Inflation)")
print("    │")
print("    └── Local Planner (Regulated Pure Pursuit)")
print("          └── Local Costmap (Voxel + Inflation)")
print("                │")
print("                ▼ /cmd_vel")
print("          Isaac Sim → TurtleBot3")
print("                │")
print("                ├── /odom")
print("                ├── /scan")
print("                └── /tf")
print()
print("  Key concepts:")
print("  - Costmap layers: Static, Obstacle, Inflation")
print("  - Global planner: computes path across full map")
print("  - Local planner: follows path with obstacle avoidance")
print("  - Behavior Tree: controls navigation lifecycle")
print("  - Recovery: spin/backup/wait when stuck")
print("  - Lifecycle: map_server → AMCL → planner → controller")
print("=" * 60)

simulation_app.close()
