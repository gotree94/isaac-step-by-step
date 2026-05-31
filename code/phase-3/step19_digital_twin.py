"""
Step 19 — Digital Twin with Isaac Sim
========================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step19_digital_twin.py

사전 준비:
    1. Isaac Sim 5.1.0 (ROS2 Bridge 활성화)
    2. 추가 Python 패키지: pip install paho-mqtt flask flask-socketio h5py

목표:
    1. Factory Digital Twin Scene 생성
    2. 다중 AMR 로봇 배치
    3. Sync Engine (ROS2 → USD 실시간 업데이트)
    4. What-If 시나리오 엔진
    5. Digital Twin Dashboard 구축
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
print("Step 19 — Digital Twin with Isaac Sim")
print("=" * 60)

# ── 2. Core API 임포트 ──
import time
import numpy as np
from pxr import Sdf, UsdGeom, Gf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid, VisualCylinder
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. Factory Scene 생성 ──
print("\n[1/6] Creating Factory Digital Twin scene...")

# 바닥
VisualCuboid(
    prim_path="/World/Factory/Floor",
    name="factory_floor",
    position=np.array([0.0, 0.0, -0.01]),
    scale=np.array([10.0, 8.0, 0.02]),
    color=np.array([0.3, 0.3, 0.3]),
)

# 벽
walls = [
    ("Wall_N", (5.0, 0.0, 1.5), (0.1, 8.0, 3.0)),
    ("Wall_S", (-5.0, 0.0, 1.5), (0.1, 8.0, 3.0)),
    ("Wall_E", (0.0, 4.0, 1.5), (10.0, 0.1, 3.0)),
    ("Wall_W", (0.0, -4.0, 1.5), (10.0, 0.1, 3.0)),
]
for name, pos, scale in walls:
    VisualCuboid(
        prim_path=f"/World/Factory/{name}", name=name,
        position=np.array(pos), scale=np.array(scale),
        color=np.array([0.5, 0.5, 0.5]),
    )

# 설비
equipment = [
    ("Machine_1", (-2.0, 1.5, 0.6), (0.8, 0.6, 1.2), (0.2, 0.4, 0.8)),
    ("Machine_2", (2.0, -1.5, 0.6), (0.8, 0.6, 1.2), (0.2, 0.4, 0.8)),
    ("Conveyor", (0.0, 0.0, 0.15), (3.0, 0.4, 0.3), (0.3, 0.3, 0.3)),
    ("Rack_1", (3.5, 0.0, 1.0), (0.5, 2.0, 2.0), (0.4, 0.3, 0.2)),
    ("Rack_2", (-3.5, 0.0, 1.0), (0.5, 2.0, 2.0), (0.4, 0.3, 0.2)),
    ("Table_1", (-1.0, -2.0, 0.3), (0.6, 0.4, 0.6), (0.6, 0.4, 0.2)),
    ("Table_2", (1.0, 2.0, 0.3), (0.6, 0.4, 0.6), (0.6, 0.4, 0.2)),
]
for name, pos, scl, col in equipment:
    VisualCuboid(
        prim_path=f"/World/Factory/{name}", name=name,
        position=np.array(pos), scale=np.array(scl),
        color=np.array(col),
    )
print(f"  + Factory scene: 1 floor, 4 walls, {len(equipment)} equipment")

# ── 5. AMR 로봇 배치 ──
print("\n[2/6] Deploying AMR robots...")

AMR_CONFIGS = [
    {"name": "AMR_1", "pos": np.array([-3.0, -2.0, 0.1]),
     "path": "/World/Robots/AMR_1"},
    {"name": "AMR_2", "pos": np.array([3.0, 2.0, 0.1]),
     "path": "/World/Robots/AMR_2"},
    {"name": "AMR_3", "pos": np.array([-1.0, 3.0, 0.1]),
     "path": "/World/Robots/AMR_3"},
    {"name": "AMR_4", "pos": np.array([1.0, -3.0, 0.1]),
     "path": "/World/Robots/AMR_4"},
]

amrs = []
for cfg in AMR_CONFIGS:
    if not is_prim_path_valid(cfg["path"]):
        add_reference_to_stage(
            "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
            cfg["path"],
        )
    robot = Robot(prim_path=cfg["path"], name=cfg["name"], position=cfg["pos"])
    world.scene.add(robot)
    amrs.append(robot)
    print(f"  + {cfg['name']} deployed at {cfg['pos']}")

# ── 6. Sync Engine (OmniGraph + ROS2 Bridge) ──
print("\n[3/6] Setting up Sync Engine bridges...")

stage = omni.usd.get_context().get_stage()

for idx, cfg in enumerate(AMR_CONFIGS):
    ns = f"/amr{idx+1}"
    graph_path = f"/ActionGraph/DT_Sync_{idx+1}"
    
    if og.Controller.graph_exists(graph_path):
        stage.RemovePrim(Sdf.Path(graph_path))
    
    graph_config = {
        "graph_path": graph_path,
        "evaluator_name": "execution",
    }
    
    og.Controller.edit(
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
                ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                ("Context.outputs:context", "SubTwist.inputs:context"),
                ("Context.outputs:context", "PubOdom.inputs:context"),
                ("Context.outputs:context", "PubTF.inputs:context"),
                ("SubTwist.outputs:linearX", "DiffCtrl.inputs:linearVelocity"),
                ("SubTwist.outputs:angularZ", "DiffCtrl.inputs:angularVelocity"),
                ("DiffCtrl.outputs:velocityCommand", "ArticCtrl.inputs:velocityCommand"),
                ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
                ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
                ("ReadOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                ("ReadOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("Context.inputs:namespace", ns),
                ("SubTwist.inputs:topicName", f"{ns}/cmd_vel"),
                ("DiffCtrl.inputs:wheelDistance", 0.141),
                ("DiffCtrl.inputs:wheelRadius", 0.033),
                ("DiffCtrl.inputs:maxLinearSpeed", 0.5),
                ("DiffCtrl.inputs:maxAngularSpeed", 2.0),
                ("ArticCtrl.inputs:robotPath", Sdf.Path(cfg["path"])),
                ("ArticCtrl.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
                ("ReadOdom.inputs:chassisPrim", Sdf.Path(f"{cfg['path']}/base_link")),
                ("PubOdom.inputs:topicName", f"{ns}/odom"),
                ("PubOdom.inputs:frameId", f"{ns}/odom"),
                ("PubOdom.inputs:childFrameId", f"{ns}/base_footprint"),
            ],
        },
    )
    print(f"  + Sync bridge: {ns} ↔ {cfg['name']}")

# ── 7. What-If Engine ──
print("\n[4/6] Setting up What-If scenario engine...")

class WhatIfEngine:
    def __init__(self, world, robots):
        self.world = world
        self.robots = robots
        self.scenarios = {}
        self.current_scenario = None

    def register_scenario(self, name, setup_fn, duration=5.0):
        self.scenarios[name] = {'setup': setup_fn, 'duration': duration}
        print(f"  + Scenario: {name} ({duration}s)")

    def run_scenario(self, name):
        if name not in self.scenarios:
            return
        s = self.scenarios[name]
        self.current_scenario = name
        s['setup']()
        steps = int(s['duration'] / self.world.get_physics_dt())
        for step in range(steps):
            self.world.step(render=True)
        return True

engine = WhatIfEngine(world, amrs)

# 시나리오: 모든 AMR 직진
def scenario_all_forward():
    for robot in amrs:
        robot.apply_action(ArticulationAction(
            joint_velocities=np.array([10.0, 10.0]), joint_indices=[0, 1]))

# 시나리오: AMR 교차
def scenario_cross():
    for i, robot in enumerate(amrs):
        v = 5.0 + i * 2.0
        robot.apply_action(ArticulationAction(
            joint_velocities=np.array([v, v]), joint_indices=[0, 1]))

engine.register_scenario("All Forward", scenario_all_forward, 3.0)
engine.register_scenario("Cross Pattern", scenario_cross, 3.0)

# ── 8. 시뮬레이션 루프 ──
print("\n[5/6] Running Digital Twin simulation...")
print()
print("  === Digital Twin Dashboard ===")
print()
print("  Terminal 2 — Monitor AMR status:")
print("    source /opt/ros/humble/setup.bash")
print("    export ROS_DOMAIN_ID=0")
print("    ros2 topic list | grep /amr")
print("    ros2 topic echo /amr1/odom --once")
print()
print("  Terminal 3 — Control AMR_1:")
print("    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\")
print("      --ros-args -r /cmd_vel:=/amr1/cmd_vel")
print()

for i in range(900):
    world.step(render=True)

    if i == 200:
        print("  [What-If] Running scenario: All Forward")
        engine.run_scenario("All Forward")

    if i == 500:
        print("  [What-If] Running scenario: Cross Pattern")
        engine.run_scenario("Cross Pattern")

    if i % 200 == 0:
        positions = []
        for robot in amrs:
            pos, _ = robot.get_world_pose()
            positions.append(f"({pos[0]:.1f}, {pos[1]:.1f})")
        print(f"  [DT] Frame {i:4d}: {', '.join(positions)}")

# ── 9. 요약 ──
print("\n[6/6] Step 19 Summary")
print("=" * 60)
print()
print("  Digital Twin Components:")
print()
print("  Scene:")
print("    - Factory 10m x 8m with walls, machines, racks")
print("    - 4 AMR robots (TurtleBot3-based)")
print()
print("  Sync:")
print("    - 4 Namespaced ROS2 bridges (/amr1-4)")
print("    - cmd_vel → wheels, odom → ROS2")
print()
print("  What-If Engine:")
print("    - 2 scenarios registered & executed")
print("    - Scenario: All Forward, Cross Pattern")
print()
print("  Dashboard:")
print("    - CLI-based real-time position tracking")
print("    - ROS2 topic monitoring")
print()
print("  Key concepts:")
print("  - Digital Twin: virtual replica of physical system")
print("  - Sync Engine: bi-directional state synchronization")
print("  - What-If: parameter changes → simulation → analysis")
print("  - ROI: reduce downtime, optimize operations, train AI")
print("=" * 60)

simulation_app.close()
