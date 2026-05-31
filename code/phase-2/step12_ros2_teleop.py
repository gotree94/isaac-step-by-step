"""
Step 12 — ROS2 TurtleBot3 Teleop: 양방향 ROS2 통신
==================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step12_ros2_teleop.py

사전 준비:
    1. ROS2 Bridge Extension 활성화 (App Selector)
    2. 다른 터미널: source /opt/ros/humble/setup.bash
    3. ROS2 환경에서 teleop 실행 준비

목표:
    1. Isaac Sim → ROS2: Odometry + TF + JointState 발행
    2. ROS2 → Isaac Sim: /cmd_vel 구독 → TurtleBot3 제어
    3. teleop_twist_keyboard로 실시간 원격 조종
    4. rviz2에서 로봇 상태 시각화
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
print("Step 12 — ROS2 TurtleBot3 Teleop")
print("=" * 60)

# ── 2. Core API 임포트 ──
import numpy as np
from pxr import Sdf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

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
print("  + TurtleBot3 loaded at", ROBOT_PATH)

# ── 5. ROS2 Teleop OmniGraph 생성 ──
print("\n[2/5] Creating ROS2 Teleop Action Graph...")

# 기존 Graph 제거
if og.Controller.graph_exists("/ActionGraph/ROS2_Teleop"):
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(Sdf.Path("/ActionGraph/ROS2_Teleop"))

graph_config = {
    "graph_path": "/ActionGraph/ROS2_Teleop",
    "evaluator_name": "execution",
}

(
    graph_handle,
    graph_nodes,
    graph_connections,
    graph_attrs,
) = og.Controller.edit(
    graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            # ── Core ──
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),

            # ── Subscriber: /cmd_vel → Differential Drive ──
            ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffCtrl", "omni.isaac.core_nodes.IsaacDifferentialController"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),

            # ── Publisher: Odometry ──
            ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),

            # ── Publisher: TF ──
            ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

            # ── Publisher: Joint State ──
            ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
            ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            # ── Tick 연결 ──
            ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
            ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
            ("OnTick.outputs:tick", "ArticCtrl.inputs:execIn"),
            ("OnTick.outputs:tick", "ReadOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
            ("OnTick.outputs:tick", "ReadJoint.inputs:execIn"),
            ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),

            # ── Context ──
            ("Context.outputs:context", "SubTwist.inputs:context"),
            ("Context.outputs:context", "PubOdom.inputs:context"),
            ("Context.outputs:context", "PubTF.inputs:context"),
            ("Context.outputs:context", "PubJoint.inputs:context"),

            # ── Subscriber → Differential Controller ──
            ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
            ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),

            # ── Differential Controller → Articulation Controller ──
            ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),

            # ── Odometry 읽기 → 발행 ──
            ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
            ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),

            # ── Joint State 읽기 → 발행 ──
            ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
            ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
            ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # ── Context ──
            ("Context.inputs:domain_id", 0),

            # ── /cmd_vel Subscriber ──
            ("SubTwist.inputs:topicName", "/cmd_vel"),

            # ── Differential Controller (TurtleBot3 물리 파라미터) ──
            ("DiffCtrl.inputs:wheelDistance", 0.141),
            ("DiffCtrl.inputs:wheelRadius", 0.033),
            ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
            ("DiffCtrl.inputs:maxAngularSpeed", 2.0),

            # ── Articulation Controller ──
            ("ArticCtrl.inputs:robotPath", Sdf.Path(ROBOT_PATH)),
            ("ArticCtrl.inputs:jointNames",
             ["left_wheel_joint", "right_wheel_joint"]),

            # ── Odometry Publisher ──
            ("ReadOdom.inputs:chassisPrim",
             Sdf.Path("/World/TurtleBot3/base_link")),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:frameId", "odom"),
            ("PubOdom.inputs:childFrameId", "base_footprint"),

            # ── Joint State Publisher ──
            ("ReadJoint.inputs:robotPrim", Sdf.Path(ROBOT_PATH)),
            ("PubJoint.inputs:topicName", "/joint_states"),
        ],
    },
)
print("  + ROS2 Teleop Graph created!")
print("    - Subscribes to: /cmd_vel")
print("    - Publishes: /odom, /tf, /joint_states")

# ── 6. 시뮬레이션 루프 ──
print("\n[3/5] Running simulation...")
print()
print("  Open 3 NEW terminals and run:")
print()
print("  # Terminal 1 — Teleop:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print()
print("  # Terminal 2 — rviz2:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  rviz2")
print("  # → Add /odom topic, set Fixed Frame=odom")
print()
print("  # Terminal 3 — Monitoring:")
print("  source /opt/ros/humble/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  ros2 topic list")
print("  ros2 topic echo /odom")
print()

for i in range(600):
    world.step(render=True)

    if i % 50 == 0:
        # ROS2에서 들어오는 cmd_vel 확인
        print(f"  Frame {i:3d}: "
              f"Waiting for /cmd_vel from teleop_twist_keyboard...")

    if i == 100:
        print()
        print("  [TIP] Press 'I' in the teleop terminal to move forward!")
        print("  [TIP] Check rviz2 for live visualization!")
        print()

    # 30초 후 자동 종료 (Ctrl+C로 수동 종료도 가능)
    if i >= 599:
        print()
        print("  [Timeout] 30 seconds elapsed.")
        print("  Use Ctrl+C to exit, or restart for more testing.")

# ── 7. Graph 노드 확인 ──
print("\n[4/5] Verifying ROS2 Graph nodes...")

if og.Controller.graph_exists("/ActionGraph/ROS2_Teleop"):
    print("  + Action Graph exists")

    g = og.Graph("/ActionGraph/ROS2_Teleop")
    nodes = g.get_nodes()
    print(f"  + Nodes in graph: {len(nodes)}")
    for node in nodes:
        node_type = node.get_node_type_name()
        print(f"    - {node.get_display_name()} ({node_type})")

# ── 8. 요약 ──
print("\n[5/5] Step 12 Summary")
print("-" * 60)
print("  Communciation flow:")
print("    ROS2 Teleop  ── /cmd_vel ──>  Isaac Sim SubTwist")
print("                                      │")
print("                                      ▼")
print("                             Differential Controller")
print("                                      │")
print("                                      ▼")
print("                             Articulation Controller")
print("                                      │")
print("                                      ▼")
print("                             TurtleBot3 Wheel Motion")
print("                                      │")
print("            ┌─────────────────────────┼──────────────────┐")
print("            ▼                         ▼                  ▼")
print("       /odom (Odom)              /tf (TF)        /joint_states")
print("            │                         │                  │")
print("            └─────────> ROS2 Ecosystem <─────────────────┘")
print("                              │")
print("                         rviz2 / echo")

print("\n  Key concepts:")
print("  - Subscribe: OmniGraph ROS2SubscribeTwist reads /cmd_vel")
print("  - Publish: OmniGraph ROS2PublishOdometry emits /odom")
print("  - TF: ROS2PublishTransformTree broadcasts transforms")
print("  - Joint State: ROS2PublishJointState emits joint data")
print("  - Domain ID: MUST match across all nodes (default: 0)")
print("=" * 60)

simulation_app.close()
