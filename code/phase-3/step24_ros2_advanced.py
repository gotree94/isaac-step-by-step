"""
Step 24 — ROS2 Advanced in Isaac Sim
======================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step24_ros2_advanced.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. ROS2 Humble (별도 설치)
    3. ros2_control (선택)

목표:
    1. Lifecycle Node Simulation (Configure → Activate → Deactivate)
    2. Node Composition Demo (Multi-Node Single Process)
    3. ROS2 Action Server (Pick-and-Place)
    4. ros2_control Hardware Interface 시뮬레이션
    5. Dynamic Parameters
    6. Real Robot Bridge Simulation
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
print("Step 24 — ROS2 Advanced")
print("=" * 60)

# ── 2. Core API 임포트 ──
import time
import math
import numpy as np
from pxr import Sdf, Gf
import omni.graph.core as og
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── 4. Franka Panda 로딩 ──
print("\n[1/8] Loading Franka Panda for ROS2 Control...")

franka_path = "/World/Robots/Franka"
if not is_prim_path_valid(franka_path):
    add_reference_to_stage(
        "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        franka_path,
    )

franka = Robot(
    prim_path=franka_path,
    name="Franka_ROS2",
    position=np.array([0.0, 0.0, 0.0]),
)
world.scene.add(franka)

print(f"  + Franka Panda: {franka.num_dof} DOF")
for i, name in enumerate(franka.dof_names):
    print(f"    {i}: {name}")

# ── 5. Lifecycle Node Simulation ──
print("\n[2/8] Simulating Lifecycle Node States...")

class LifecycleSimulator:
    """Isaac Sim Lifecycle 상태 시뮬레이션"""
    
    STATES = {
        'unconfigured': 0,
        'inactive': 1,
        'active': 2,
        'finalized': 3,
    }
    
    def __init__(self, robot):
        self.robot = robot
        self.state = 'unconfigured'
        self.home_pose = np.zeros(robot.num_dof)
        self.control_freq = 0
        self.is_bridge_active = False
    
    def configure(self):
        """Configure: 리소스 할당, 파라미터 로드"""
        print("\n  ⚙ Lifecycle: Configuring...")
        self.state = 'inactive'
        self.control_freq = 100  # Hz
        self.home_pose = np.array(
            [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785, 0.04, 0.04])
        self.robot.set_joint_positions(self.home_pose)
        print(f"  ✓ Configured: frequency={self.control_freq}Hz")
        return True
    
    def activate(self):
        """Activate: ROS2 Bridge 시작, Timer 시작"""
        print("\n  ⚙ Lifecycle: Activating...")
        self.state = 'active'
        self.is_bridge_active = True
        self._start_ros2_bridge()
        print(f"  ✓ Activated: ROS2 Bridge running")
        return True
    
    def deactivate(self):
        """Deactivate: ROS2 Bridge 중지"""
        print("\n  ⚙ Lifecycle: Deactivating...")
        self.state = 'inactive'
        self.is_bridge_active = False
        self._stop_ros2_bridge()
        print(f"  ✓ Deactivated")
        return True
    
    def cleanup(self):
        """Cleanup: 리소스 해제"""
        print("\n  ⚙ Lifecycle: Cleaning up...")
        self.state = 'unconfigured'
        self.control_freq = 0
        print(f"  ✓ Cleaned up")
        return True
    
    def shutdown(self):
        """Shutdown: 종료"""
        print("\n  ⚙ Lifecycle: Shutting down...")
        self.state = 'finalized'
        print(f"  ✓ Shutdown")
        return True
    
    def _start_ros2_bridge(self):
        """ROS2 Bridge 시작 (ActionGraph)"""
        graph_path = "/ActionGraph/ROS2_Advanced"
        
        if og.Controller.graph_exists(graph_path):
            return
        
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ReadJoint", "omni.isaac.core_nodes.IsaacReadJointState"),
                    ("PubJoint", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "ReadJoint.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                    ("Context.outputs:context", "PubJoint.inputs:context"),
                    ("Context.outputs:context", "PubTF.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Context.inputs:domain_id", 0),
                    ("Context.inputs:namespace", "/franka"),
                    ("ReadJoint.inputs:robotPrim", Sdf.Path(franka_path)),
                    ("PubJoint.inputs:topicName", "/franka/joint_states"),
                ],
            },
        )
        print("  + ROS2 Bridge started")
    
    def _stop_ros2_bridge(self):
        """ROS2 Bridge 중지"""
        graph_path = "/ActionGraph/ROS2_Advanced"
        if og.Controller.graph_exists(graph_path):
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(Sdf.Path(graph_path))
            print("  + ROS2 Bridge stopped")

lifecycle = LifecycleSimulator(franka)

# Lifecycle 상태 전이 실행
print("\n  Lifecycle State Machine:")
print("  Unconfigured → Configure → Activate → Deactivate → Cleanup → Shutdown")

lifecycle.configure()
lifecycle.activate()

# ── 6. Dynamic Parameters ──
print("\n[3/8] Dynamic Parameters Simulation...")

class ParameterServer:
    """Isaac Sim Dynamic Parameter Server"""
    
    def __init__(self):
        self.params = {
            'control_rate': 100,
            'joint_stiffness': 100.0,
            'joint_damping': 10.0,
            'max_velocity': 1.0,
            'max_effort': 50.0,
            'safety_enabled': True,
            'safety_stop_distance': 0.25,
        }
        self.callbacks = {}
    
    def set(self, name, value):
        if name in self.params:
            old = self.params[name]
            self.params[name] = value
            print(f"  + Parameter '{name}': {old} → {value}")
            
            if name in self.callbacks:
                self.callbacks[name](name, value)
            return True
        return False
    
    def get(self, name):
        return self.params.get(name)
    
    def list_params(self):
        for name, value in self.params.items():
            print(f"    {name}: {value}")
    
    def on_change(self, name, callback):
        self.callbacks[name] = callback

params = ParameterServer()

print("  Initial parameters:")
params.list_params()

# 파라미터 동적 변경
print("\n  Dynamic parameter changes:")
params.set('control_rate', 50)
params.set('safety_stop_distance', 0.5)
params.set('max_velocity', 0.5)

# ── 7. ROS2 Action 시뮬레이션 ──
print("\n[4/8] Simulating ROS2 Action (Pick-and-Place)...")

class PickAndPlaceAction:
    """Pick-and-Place Action Server Simulation"""
    
    GOAL_ACCEPTED = 1
    GOAL_REJECTED = 2
    EXECUTING = 3
    SUCCEEDED = 4
    CANCELLED = 5
    
    def __init__(self, robot):
        self.robot = robot
        self.status = None
        self.goal = None
        self.progress = 0.0
        self.phase = 'idle'
    
    def handle_goal(self, object_id=0, pick_x=0.5, pick_y=0.0):
        """Goal 수신 및 처리"""
        print(f"\n  Action: Pick object #{object_id}")
        print(f"  Pick position: ({pick_x:.2f}, {pick_y:.2f}, 0.3)")
        
        self.goal = {
            'object_id': object_id,
            'pick': (pick_x, pick_y, 0.3),
            'place': (-0.3, 0.0, 0.3),
        }
        
        self.status = self.GOAL_ACCEPTED
        print("  Goal ACCEPTED")
        return True
    
    def execute(self):
        """Action 실행 (simulated)"""
        self.status = self.EXECUTING
        
        # Phase 1: Move to Pick
        self.phase = 'moving_to_pick'
        print("\n  Phase 1: Moving to pick position...")
        world.step(render=True)
        world.step(render=True)
        
        # Phase 2: Grasp
        self.phase = 'grasping'
        print("  Phase 2: Grasping object...")
        time.sleep(0.1)
        
        # Phase 3: Move to Place
        self.phase = 'moving_to_place'
        print("  Phase 3: Moving to place position...")
        world.step(render=True)
        world.step(render=True)
        
        # Phase 4: Release
        self.phase = 'placing'
        print("  Phase 4: Placing object...")
        time.sleep(0.1)
        
        self.status = self.SUCCEEDED
        self.phase = 'completed'
        print("\n  Action SUCCEEDED")
        
    def cancel(self):
        self.status = self.CANCELLED
        print("  Action CANCELLED")
    
    def get_feedback(self):
        """Action Feedback"""
        return {
            'phase': self.phase,
            'progress': self.progress,
            'status': self.status,
        }

action = PickAndPlaceAction(franka)

# Action 실행 시뮬레이션
action.handle_goal(object_id=42)
action.execute()

# ── 8. ros2_control 시뮬레이션 ──
print("\n[5/8] Simulating ros2_control Hardware Interface...")

class JointTrajectoryController:
    """Joint Trajectory Controller (ros2_control 호환)"""
    
    def __init__(self, robot):
        self.robot = robot
        self.dof_count = robot.num_dof
        self.trajectory = []
        self.trajectory_index = 0
        self.is_running = False
    
    def load_trajectory(self, target_positions, duration=2.0):
        """Trajectory 로드"""
        current = self.robot.get_joint_positions()
        steps = int(duration * 60)  # 60fps
        
        self.trajectory = []
        for i in range(steps):
            alpha = i / steps
            interpolated = current + (np.array(target_positions) - current) * alpha
            self.trajectory.append(interpolated)
        
        self.trajectory_index = 0
        self.is_running = True
        print(f"  + Trajectory loaded: {steps} steps, {duration}s")
    
    def update(self):
        """Trajectory 실행"""
        if not self.is_running or self.trajectory_index >= len(self.trajectory):
            self.is_running = False
            return
        
        target = self.trajectory[self.trajectory_index]
        self.robot.set_joint_positions(target)
        self.trajectory_index += 1

controller = JointTrajectoryController(franka)

# Joint trajectory 실행
target_pose = [0.5, -0.8, 0.0, -2.0, 0.0, 2.5, 0.785, 0.04, 0.04]
print(f"\n  Trajectory target: {target_pose[:7]}")
controller.load_trajectory(target_pose)

# ── 9. Real Robot Bridge 시뮬레이션 ──
print("\n[6/8] Simulating Real Robot Bridge...")

class DigitalTwinSimulator:
    """Real Robot ↔ Isaac Sim Bridge"""
    
    def __init__(self, robot):
        self.robot = robot
        self.latency = 0.0  # ms
        self.sync_enabled = True
        self.real_joint_states = None
        self.sync_count = 0
    
    def receive_real_robot_data(self, positions):
        """실제 로봇 데이터 수신 (시뮬레이션)"""
        self.real_joint_states = np.array(positions)
        self.sync_count += 1
        
        if self.sync_enabled:
            self.robot.set_joint_positions(self.real_joint_states)
    
    def send_to_real_robot(self):
        """Isaac Sim → Real Robot"""
        sim_pose = self.robot.get_joint_positions()
        self.latency = np.random.uniform(0.5, 2.0)  # simulated ms
        return sim_pose, self.latency
    
    def check_sync_status(self):
        """동기화 상태 확인"""
        if self.real_joint_states is not None:
            sim = self.robot.get_joint_positions()
            diff = np.max(np.abs(sim[:7] - self.real_joint_states[:7]))
            return diff < 0.01  # 0.01 rad tolerance
        return False

bridge = DigitalTwinSimulator(franka)

# Real Robot → Sim 동기화
real_robot_data = [0.1, -0.4, 0.05, -2.1, 0.02, 2.1, 0.7, 0.04, 0.04]
bridge.receive_real_robot_data(real_robot_data)

sim_data, latency = bridge.send_to_real_robot()
print(f"  + Real → Sim sync: {len(real_robot_data)} joints")
print(f"  + Sim → Real feedback latency: {latency:.2f}ms")
print(f"  + Sync status: {'✓ OK' if bridge.check_sync_status() else '⚠ Mismatch'}")

# ── 10. Composition 시뮬레이션 ──
print("\n[7/8] Simulating Node Composition...")

class ComponentContainer:
    """Component Container (단일 프로세스 다중 노드)"""
    
    def __init__(self, name="isaac_container"):
        self.name = name
        self.components = {}
        print(f"  + Component Container '{name}' created")
    
    def load_component(self, name, component_type):
        """컴포넌트 로드"""
        component = component_type(self)
        self.components[name] = component
        print(f"  + Loaded: {name} ({component_type.__name__})")
        return component
    
    def unload_component(self, name):
        """컴포넌트 언로드"""
        if name in self.components:
            del self.components[name]
            print(f"  + Unloaded: {name}")
    
    def list_components(self):
        for name, comp in self.components.items():
            print(f"    {name}: {comp.get_info()}")

class CameraComponent:
    def __init__(self, container):
        self.container = container
        self.resolution = (640, 480)
        self.fps = 30
        self.is_active = True
    
    def get_info(self):
        return f"Camera ({self.resolution[0]}x{self.resolution[1]} @{self.fps}fps)"

class ControlComponent:
    def __init__(self, container):
        self.container = container
        self.control_mode = "position"
        self.update_rate = 100
    
    def get_info(self):
        return f"Control ({self.control_mode}, {self.update_rate}Hz)"

class PerceptionComponent:
    def __init__(self, container):
        self.container = container
        self.model = "yolov8n"
        self.confidence = 0.5
    
    def get_info(self):
        return f"Perception ({self.model}, conf={self.confidence})"

container = ComponentContainer()
container.load_component("camera", CameraComponent)
container.load_component("control", ControlComponent)
container.load_component("perception", PerceptionComponent)

print("\n  Loaded components:")
container.list_components()

# ── 11. Security (SROS2 시뮬레이션) ──
print("\n[8/8] Simulating SROS2 Security...")

class SecurityEnclave:
    """SROS2 Security Enclave Simulation"""
    
    PERMISSIONS = {
        '/isaac/perception': {
            'subscribe': ['parameter_events'],
            'publish': ['camera/image'],
        },
        '/isaac/control': {
            'subscribe': ['joint_trajectory'],
            'publish': ['joint_states'],
        },
        '/isaac/navigation': {
            'subscribe': ['goal_pose'],
            'publish': ['odometry'],
        },
    }
    
    def __init__(self):
        self.enclaves = {}
        self.keystore_path = "~/sros2_keystore"
    
    def create_enclave(self, name, enclave_path):
        """보안 Enclave 생성"""
        self.enclaves[name] = {
            'path': enclave_path,
            'permissions': self.PERMISSIONS.get(enclave_path, {}),
            'is_secure': True,
        }
        print(f"\n  Security Enclave for {name}:")
        print(f"    Path: {enclave_path}")
        print(f"    Permissions: {self.PERMISSIONS.get(enclave_path, {})}")
        return True
    
    def verify_access(self, enclave_name, operation, topic):
        """접근 권한 확인"""
        if enclave_name in self.enclaves:
            perms = self.enclaves[enclave_name]['permissions']
            allowed = perms.get(operation, [])
            if topic in allowed:
                return True
            print(f"  ⚠ Access denied: {enclave_name} {operation} {topic}")
        return False

security = SecurityEnclave()
security.create_enclave("perception_node", "/isaac/perception")
security.create_enclave("control_node", "/isaac/control")
security.create_enclave("navigation_node", "/isaac/navigation")

# ── 12. 통합 시뮬레이션 루프 ──
print("\n" + "="*60)
print("  Running Integrated ROS2 Advanced Simulation...")
print("="*60)
print()

for i in range(600):
    world.step(render=True)
    
    # Trajectory controller update
    controller.update()
    
    # Logging
    if i % 200 == 0:
        print(f"  [{i//10:3d}s] Lifecycle: {lifecycle.state}, "
              f"Bridge: {'ON' if lifecycle.is_bridge_active else 'OFF'}, "
              f"Trajectory: {i}/600")
        
        if lifecycle.state == 'active':
            pos = franka.get_joint_positions()
            print(f"           Joint position: [{pos[0]:.2f}, {pos[1]:.2f}, "
                  f"{pos[2]:.2f}, {pos[3]:.2f}...]")

# Lifecycle cleanup
lifecycle.deactivate()
lifecycle.cleanup()
lifecycle.shutdown()

# ── 13. 요약 ──
print("\n" + "="*60)
print("  Step 24 — Summary")
print("="*60)
print()
print("  ROS2 Advanced:")
print()
print("  ✅ Lifecycle Node")
print(f"     {lifecycle.state} (final state)")
print("     4 transitions: Configure → Activate → Deactivate → Cleanup")
print()
print("  ✅ Dynamic Parameters")
print(f"     {len(params.params)} parameters")
print(f"     3 dynamic changes applied")
print()
print("  ✅ ROS2 Action")
print(f"     Goal accepted, executed, succeeded")
print("     Phases: move_to_pick → grasp → move_to_place → place")
print()
print("  ✅ ros2_control Joint Trajectory")
print(f"     600-step trajectory executed")
print()
print("  ✅ Node Composition")
print(f"     {len(container.components)} components in container")
print()
print("  ✅ Real Robot Bridge")
print(f"     {bridge.sync_count} sync operations")
print(f"     Latency: {latency:.2f}ms")
print()
print("  ✅ SROS2 Security")
print(f"     {len(security.enclaves)} secure enclaves")
print()
print("  ✅ Key Concepts:")
print("     - Managed (Lifecycle) Nodes")
print("     - Component Composition")
print("     - Action Servers/Clients")
print("     - ros2_control Hardware Interface")
print("     - Dynamic Reconfigure")
print("     - DDS Security (SROS2)")
print("     - Digital Twin (Real ↔ Sim)")
print("="*60)

simulation_app.close()
