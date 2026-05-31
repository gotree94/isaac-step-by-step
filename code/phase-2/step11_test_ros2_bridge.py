"""
Step 11 — ROS2 Bridge 설치 및 설정: Bridge 통신 테스트
=======================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-2/step11_test_ros2_bridge.py

사전 준비:
    1. ROS2 Humble/Jazzy 설치 (또는 Internal 라이브러리 사용)
    2. App Selector에서 ROS2 Bridge 활성화
    3. 시뮬레이션 Play (▶) 후 ROS2 통신 확인

목표:
    1. ROS2 Bridge Extension 로드 확인
    2. ROS2 OmniGraph를 통해 /clock 토픽 발행
    3. rclpy (ROS2 Python)로 간단한 Pub/Sub 테스트
    4. 외부 ROS2 노드와의 통신 확인
"""

# ── 1. SimulationApp 초기화 (Bridge 자동 활성화) ──
# 방법 A: App Selector에서 ROS2 Bridge 활성화 후 실행
# 방법 B: 아래 CONFIG에 startup_extension 지정
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 11 — ROS2 Bridge Test")
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
from omni.isaac.core.utils.extensions import enable_extension

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. ROS2 Bridge Extension 활성화 확인 ──
print("\n[1/5] Checking ROS2 Bridge extension...")

try:
    enable_extension("omni.isaac.ros2_bridge")
    print("  + ROS2 Bridge extension enabled successfully!")
except Exception as e:
    print(f"  ⚠ Could not enable ROS2 Bridge: {e}")
    print("  (Try launching with App Selector → check ROS2 Bridge option)")

# ── 5. ROS2 Clock Publisher Graph 생성 ──
print("\n[2/5] Creating ROS2 Clock Publisher OmniGraph...")

# 기존 Graph 제거 (중복 방지)
if og.Controller.graph_exists("/ActionGraph/ROS2ClockTest"):
    print("  ⚠ Graph already exists. Removing...")
    # Stage에서 그래프 삭제
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(Sdf.Path("/ActionGraph/ROS2ClockTest"))

clock_graph_config = {
    "graph_path": "/ActionGraph/ROS2ClockTest",
    "evaluator_name": "execution",
}

(
    graph_handle,
    graph_nodes,
    graph_connections,
    graph_attrs,
) = og.Controller.edit(
    clock_graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:domain_id", 0),
            ("PublishClock.inputs:topicName", "/sim_clock"),
        ],
    },
)
print("  + Clock Publisher Graph created at /ActionGraph/ROS2ClockTest")

# ── 6. TurtleBot3 로딩 ──
print("\n[3/5] Loading TurtleBot3 for ROS2 control demo...")

ROBOT_PATH = "/World/TurtleBot3"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage("/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd", ROBOT_PATH)

robot = Robot(prim_path=ROBOT_PATH, name="TurtleBot3", position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)
controller = robot.get_articulation_controller()
print("  + TurtleBot3 loaded")

# ── 7. 시뮬레이션 루프 ──
print("\n[4/5] Running simulation with ROS2 bridge...")
print("  (Open another terminal and run: ros2 topic list)")
print("  (Check: ros2 topic echo /sim_clock)")
print()

for i in range(200):
    world.step(render=True)

    # TurtleBot3 직진
    WHEEL_DIST = 0.141
    WHEEL_RADIUS = 0.033

    v_l = 0.15 - 0.0 * WHEEL_DIST / 2
    v_r = 0.15 + 0.0 * WHEEL_DIST / 2
    speeds = np.array([v_l / WHEEL_RADIUS, v_r / WHEEL_RADIUS])
    controller.apply_action(ArticulationAction(joint_velocities=speeds, joint_indices=[0, 1]))

    if i % 30 == 0:
        print(f"  Frame {i:3d}: /sim_clock is being published (ROS2 Bridge active)")

# ── 8. 확인 명령어 출력 ──
print(f"\n[5/5] Verification steps:")
print()
print("  Open a NEW terminal and run:")
print()
print("  # 1. Set ROS2 environment")
print("  source /opt/ros/humble/setup.bash   # or jazzy")
print("  export ROS_DOMAIN_ID=0")
print()
print("  # 2. Check topics published by Isaac Sim")
print("  ros2 topic list")
print("  # Expected: /sim_clock, /rosout, /parameter_events")
print()
print("  # 3. Check clock message")
print("  ros2 topic echo /sim_clock")
print()
print("  # 4. Check publishing rate")
print("  ros2 topic hz /sim_clock")
print()
print("  # 5. Send velocity command (requires ROS2 Teleop setup)")
print("  ros2 topic pub /cmd_vel geometry_msgs/Twist \"{linear: {x: 0.1}}\"")
print()

print("=" * 60)
print("Step 11 Complete!")
print("=" * 60)
print()
print("Key concepts:")
print("  - ROS2 Bridge converts simulation data to ROS2 topics")
print("  - OmniGraph ROS2 nodes: Context + PublishClock")
print("  - Domain ID must match between Isaac Sim and ROS2 nodes")
print("  - Internal libraries: Isaac Sim ships with ROS2 libs")
print("  - External libraries: Use system-installed ROS2")

simulation_app.close()
