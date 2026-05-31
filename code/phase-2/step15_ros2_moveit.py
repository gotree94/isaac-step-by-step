"""
Step 15 — ROS2 MoveIt2 + Franka Panda
=======================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step15_ros2_moveit.py

사전 준비:
    1. Isaac Sim 5.1.0 (ROS2 Bridge 활성화)
    2. MoveIt2 설치: sudo apt install ros-humble-moveit
    3. Franka Panda URDF/SRDF 준비
    4. 3-4개 터미널 (Isaac Sim / MoveIt2 / rviz2 / Python Commander)

목표:
    1. Isaac Sim에서 Franka Panda + Joint State 발행
    2. ROS2 JointTrajectory Subscribe → Franka 제어
    3. MoveIt2 (MoveGroup) 연동 기반 제공
    4. rviz2 MoveIt Plugin과 Joint State 동기화
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
print("Step 15 — ROS2 MoveIt2 + Franka Panda")
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
from omni.isaac.core.articulations import ArticulationView

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. Franka Panda 로딩 ──
print("\n[1/5] Loading Franka Panda...")

FRANKA_PATH = "/World/Franka"
if not is_prim_path_valid(FRANKA_PATH):
    add_reference_to_stage(
        "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        FRANKA_PATH,
    )

franka = Robot(
    prim_path=FRANKA_PATH,
    name="Franka",
    position=np.array([0.0, 0.0, 0.0]),
)
world.scene.add(franka)

# Franka 관절 이름 확인
dof_names = franka.dof_names
print(f"  + Franka loaded: {len(dof_names)} DOF")
for i, name in enumerate(dof_names):
    print(f"    - {i}: {name}")

# Franka를 Home Pose로 초기화
home_joints = [0.0, -0.785, 0.0, -2.18, 0.0, 1.57, 0.785, 0.02, 0.02]
franka.set_joint_positions(np.array(home_joints[:len(dof_names)]))
world.step(render=True)
print("  + Franka set to HOME position")

# ── 5. 테이블 및 작업 공간 생성 ──
print("\n[2/5] Creating workbench and objects...")

from omni.isaac.core.objects import VisualCuboid, VisualSphere, DynamicCuboid

# 테이블
table = VisualCuboid(
    prim_path="/World/Workbench/Table",
    name="table",
    position=np.array([0.5, 0.0, -0.15]),
    scale=np.array([0.8, 0.6, 0.05]),
    color=np.array([0.4, 0.3, 0.2]),
)

# Pick 대상 물체
target = VisualCuboid(
    prim_path="/World/Objects/Target",
    name="target",
    position=np.array([0.5, 0.2, 0.03]),
    scale=np.array([0.04, 0.04, 0.06]),
    color=np.array([0.8, 0.2, 0.1]),
)

# Place 위치 표시
place_marker = VisualCuboid(
    prim_path="/World/Objects/PlaceMarker",
    name="place_marker",
    position=np.array([0.0, -0.3, 0.005]),
    scale=np.array([0.06, 0.06, 0.01]),
    color=np.array([0.2, 0.8, 0.2]),
)

print("  + Workbench, target, and place marker created")

# ── 6. MoveIt2 Bridge OmniGraph 생성 ──
print("\n[3/5] Creating MoveIt2 Bridge OmniGraph...")

graph_config = {
    "graph_path": "/ActionGraph/Franka_MoveIt2",
    "evaluator_name": "execution",
}

# 기존 Graph 제거
stage = omni.usd.get_context().get_stage()
if og.Controller.graph_exists("/ActionGraph/Franka_MoveIt2"):
    stage.RemovePrim(Sdf.Path("/ActionGraph/Franka_MoveIt2"))

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

            # Subscriber: JointTrajectory (from MoveIt2)
            ("SubTraj", "omni.isaac.ros2_bridge.ROS2SubscribeJointTrajectory"),
            ("ArticCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),

            # Publisher: JointState (to MoveIt2)
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

            # Trajectory → Articulation Controller
            ("SubTraj.outputs:jointNames", "ArticCtrl.inputs:jointNames"),
            ("SubTraj.outputs:positionCommand", "ArticCtrl.inputs:positionCommand"),

            # Joint State Read → Publish
            ("ReadJoint.outputs:jointNames", "PubJoint.inputs:jointNames"),
            ("ReadJoint.outputs:jointPositions", "PubJoint.inputs:jointPositions"),
            ("ReadJoint.outputs:jointVelocities", "PubJoint.inputs:jointVelocities"),
            ("ReadJoint.outputs:jointEfforts", "PubJoint.inputs:jointEfforts"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:domain_id", 0),

            ("SubTraj.inputs:topicName", "/joint_trajectory"),

            ("ArticCtrl.inputs:robotPath", Sdf.Path(FRANKA_PATH)),
            ("ArticCtrl.inputs:jointNames", [
                "joint1", "joint2", "joint3", "joint4",
                "joint5", "joint6", "joint7",
                "panda_finger_joint1", "panda_finger_joint2",
            ]),

            ("ReadJoint.inputs:robotPrim", Sdf.Path(FRANKA_PATH)),
            ("PubJoint.inputs:topicName", "/joint_states"),
        ],
    },
)
print("  + MoveIt2 Bridge Graph created!")
print("    - Subscribes: /joint_trajectory (from MoveIt2)")
print("    - Publishes: /joint_states (to MoveIt2 / RobotStatePublisher)")

# ── 7. 시뮬레이션 루프 ──
print("\n[4/5] Running simulation...")
print()
print("  === MoveIt2 + Franka Setup ===")
print()
print("  Terminal 1 (this window): Isaac Sim running")
print()
print("  Terminal 2 — Robot State Publisher:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 run robot_state_publisher robot_state_publisher \\")
print("      ~/isaac-step-curriculum/config/franka_panda.urdf")
print()
print("  Terminal 3 — MoveIt2 Launch:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 launch moveit2_tutorials demo.launch.py \\")
print("      robot_description:=~/isaac-step-curriculum/config/franka_panda.urdf \\")
print("      robot_description_semantic:=~/isaac-step-curriculum/config/franka_panda.srdf \\")
print("      kinematics_yaml:=~/isaac-step-curriculum/config/franka_moveit_config/kinematics.yaml")
print()
print("  Terminal 4 — Python Commander:")
print("    source ~/isaac-step-curriculum/env_isaacsim/bin/activate")
print("    export ROS_DOMAIN_ID=0")
print("    python ~/isaac-step-curriculum/code/phase-2/step15_moveit_commander.py")
print()

for i in range(1200):
    world.step(render=True)

    if i % 100 == 0:
        # 현재 Franka 관절 상태 출력
        joint_pos = franka.get_joint_positions()[:7]
        jp_str = ", ".join([f"{j:.2f}" for j in joint_pos])
        print(f"  Frame {i:4d}: Franka joints=[{jp_str}] — waiting for MoveIt2 trajectory...")

    if i == 50:
        print()
        print("  [TIP] In rviz2 Motion Planning panel:")
        print("    - Set Planning Group: panda_arm")
        print("    - Set Goal State: ready, then Plan & Execute")
        print("    - Watch Franka move in Isaac Sim!")
        print()

# ── 8. Graph 노드 확인 ──
print("\n[5/5] Verifying MoveIt2 Bridge nodes...")

if og.Controller.graph_exists("/ActionGraph/Franka_MoveIt2"):
    g = og.Graph("/ActionGraph/Franka_MoveIt2")
    nodes = g.get_nodes()
    print(f"  + Graph nodes ({len(nodes)}):")
    for node in nodes:
        print(f"    - {node.get_display_name()} ({node.get_node_type_name()})")

print()
print("  Key concepts:")
print("  - MoveIt2 plans motions in Joint Space or Cartesian Space")
print("  - /joint_states: Isaac Sim → MoveIt2 (current state)")
print("  - /joint_trajectory: MoveIt2 → Isaac Sim (commanded trajectory)")
print("  - MoveGroup: Main API for motion planning & execution")
print("  - Planning Scene: Robot + environment + collision objects")
print("  - SRDF: Semantic description of robot groups & states")
print("=" * 60)

simulation_app.close()
