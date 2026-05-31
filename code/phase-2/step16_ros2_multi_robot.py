"""
Step 16 — Multi-Robot ROS2 System
===================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step16_ros2_multi_robot.py

사전 준비:
    1. Isaac Sim 5.1.0 (ROS2 Bridge 활성화)
    2. 5-6개 터미널 (Isaac Sim / 3x Teleop / 모니터링 / rviz2)

목표:
    1. 3대의 TurtleBot3를 동시에 로딩
    2. 각 로봇에 Namespace (/tb1, /tb2, /tb3) 적용
    3. Namespaced OmniGraph Bridge 생성
    4. 충돌 회피 환경 (벽 + 장애물)
    5. 각 로봇별 독립 cmd_vel 수신
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
print("Step 16 — Multi-Robot ROS2 System")
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

# ── 4. 다중 TurtleBot3 로딩 ──
print("\n[1/6] Loading 3x TurtleBot3...")

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
    robots.append(robot)
    print(f"  + {cfg['name']} loaded at {cfg['position']}")

# ── 5. 환경 생성 (4x4m 공간 + 장애물) ──
print("\n[2/6] Creating multi-robot environment...")

# 외부 벽
wall_configs = [
    {"pos": (2.8, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    {"pos": (-2.8, 0.0, 1.0), "scale": (0.1, 6.0, 2.0)},
    {"pos": (0.0, 2.8, 1.0), "scale": (6.0, 0.1, 2.0)},
    {"pos": (0.0, -2.8, 1.0), "scale": (6.0, 0.1, 2.0)},
]

for i, wall in enumerate(wall_configs):
    VisualCuboid(
        prim_path=f"/World/Environment/Wall_{i:02d}",
        name=f"wall_{i}",
        position=np.array(wall["pos"]),
        scale=np.array(wall["scale"]),
        color=np.array([0.5, 0.3, 0.1]),
    )

# 내부 장애물
obstacle_positions = [
    (0.0, 0.0, 0.5),
    (1.8, 1.0, 0.3),
    (-1.8, 1.0, 0.4),
    (1.0, -1.5, 0.35),
    (-1.0, -1.5, 0.45),
]

for i, (x, y, h) in enumerate(obstacle_positions):
    VisualCuboid(
        prim_path=f"/World/Environment/Obstacle_{i:02d}",
        name=f"obs_{i}",
        position=np.array([x, y, h / 2]),
        scale=np.array([0.3, 0.3, h]),
        color=np.array([0.2, 0.6, 0.8]),
    )

# 로봇별 개별 LiDAR 속성 확인
for cfg in ROBOT_CONFIGS:
    stage = omni.usd.get_context().get_stage()
    lidar_path = f"{cfg['prim_path']}/base_scan/Lidar"
    lidar_prim = stage.GetPrimAtPath(lidar_path)
    if lidar_prim:
        lidar_prim.GetAttribute("range").Set(3.5)
        lidar_prim.GetAttribute("horizontalFov").Set(360.0)
        lidar_prim.GetAttribute("rotationRate").Set(10.0)
        lidar_prim.CreateAttribute("drawSensors", Sdf.ValueTypeNames.Bool).Set(True)

print(f"  + {len(wall_configs)} walls + {len(obstacle_positions)} obstacles created")

# ── 6. Namespaced OmniGraph 생성 ──
print("\n[3/6] Creating Namespaced Bridges for each robot...")
print("  (Each bridge publishes/subscribes under separate namespaces)")

stage = omni.usd.get_context().get_stage()

for idx, cfg in enumerate(ROBOT_CONFIGS):
    ns = cfg["namespace"]
    ns_clean = ns.strip("/")
    graph_path = f"/ActionGraph/{ns_clean}_Bridge"
    
    # 기존 Graph 제거
    if og.Controller.graph_exists(graph_path):
        stage.RemovePrim(Sdf.Path(graph_path))
    
    graph_config = {
        "graph_path": graph_path,
        "evaluator_name": "execution",
    }
    
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
                
                ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
                ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
                
                ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
                ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                
                ("ReadScan", "omni.isaac.core_nodes.IsaacReadLaserScan"),
                ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                
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
                ("Context.inputs:namespace", ns),
                
                ("SubTwist.inputs:topicName", f"{ns}/cmd_vel"),
                
                ("DiffCtrl.inputs:wheelDistance", 0.141),
                ("DiffCtrl.inputs:wheelRadius", 0.033),
                ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
                ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
                
                ("ArticCtrl.inputs:robotPath", Sdf.Path(cfg["prim_path"])),
                ("ArticCtrl.inputs:jointNames",
                 ["left_wheel_joint", "right_wheel_joint"]),
                
                ("ReadOdom.inputs:chassisPrim",
                 Sdf.Path(f'{cfg["prim_path"]}/base_link')),
                ("PubOdom.inputs:topicName", f"{ns}/odom"),
                ("PubOdom.inputs:frameId", f"{ns}/odom"),
                ("PubOdom.inputs:childFrameId", f"{ns}/base_footprint"),
                
                ("ReadScan.inputs:laserPrim",
                 Sdf.Path(f'{cfg["prim_path"]}/base_scan/Lidar')),
                ("PubScan.inputs:topicName", f"{ns}/scan"),
                ("PubScan.inputs:frameId", f"{ns}/base_scan"),
                ("PubScan.inputs:rangeMin", 0.1),
                ("PubScan.inputs:rangeMax", 3.5),
                ("PubScan.inputs:rangeThreshold", 3500.0),
                
                ("ReadJoint.inputs:robotPrim", Sdf.Path(cfg["prim_path"])),
                ("PubJoint.inputs:topicName", f"{ns}/joint_states"),
            ],
        },
    )
    
    print(f"  + Graph created: {graph_path}")
    print(f"    - cmd_vel: {ns}/cmd_vel")
    print(f"    - odom:    {ns}/odom")
    print(f"    - scan:    {ns}/scan")

# ── 7. 시뮬레이션 루프 ──
print("\n[4/6] Running simulation with 3 robots...")
print()
print("  === Multi-Robot Instructions ===")
print()
print("  Terminal 2 — Teleop for /tb1:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\")
print("      --ros-args -r /cmd_vel:=/tb1/cmd_vel")
print()
print("  Terminal 3 — Teleop for /tb2:")
print("    (same as above but /tb2/cmd_vel)")
print()
print("  Terminal 4 — Teleop for /tb3:")
print("    (same as above but /tb3/cmd_vel)")
print()
print("  Terminal 5 — Monitor:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 topic list | grep /tb")
print()

for i in range(1800):
    world.step(render=True)

    if i % 200 == 0:
        # 각 로봇 위치 출력
        for robot, cfg in zip(robots, ROBOT_CONFIGS):
            pos, orient = robot.get_world_pose()
            print(f"  Frame {i:4d}: {cfg['namespace']} position=({pos[0]:.2f}, {pos[1]:.2f})")

    if i == 50:
        print()
        print("  [TIP] Control each robot from a separate teleop terminal!")
        print("  [TIP] Watch them avoid each other (each LiDAR sees other robots)")
        print()

# ── 8. 확인 ──
print("\n[5/6] Verifying graphs...")

for cfg in ROBOT_CONFIGS:
    graph_path = f"/ActionGraph/{cfg['namespace'].strip('/')}_Bridge"
    if og.Controller.graph_exists(graph_path):
        g = og.Graph(graph_path)
        nodes = g.get_nodes()
        print(f"  + {graph_path}: {len(nodes)} nodes active")
    else:
        print(f"  ⚠ {graph_path}: NOT FOUND")

# ── 9. 요약 ──
print("\n[6/6] Step 16 Summary")
print("-" * 60)
print()
print("  Robots and their namespaces:")
print()
for cfg in ROBOT_CONFIGS:
    print(f"  {cfg['name']:15s} {cfg['namespace']:5s}  →  {cfg['prim_path']}")
print()
print("  Topic mapping:")
print("    /tb1/cmd_vel  →  TurtleBot1 wheel controller")
print("    /tb2/cmd_vel  →  TurtleBot2 wheel controller")
print("    /tb3/cmd_vel  →  TurtleBot3 wheel controller")
print("    /tb1/odom     →  TurtleBot1 odometry")
print("    /tb2/odom     →  TurtleBot2 odometry")
print("    /tb3/odom     →  TurtleBot3 odometry")
print()
print("  Key concepts:")
print("  - ROS2 Namespace: separates topics per robot")
print("  - Context.inputs:namespace: applies prefix to all bridge topics")
print("  - Each robot gets its own OmniGraph Bridge")
print("  - TF frames are prefixed (/tb1/odom, /tb1/base_footprint, ...)")
print("  - LiDAR naturally detects other robots as obstacles")
print("=" * 60)

simulation_app.close()
