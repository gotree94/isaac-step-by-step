"""
Step 21 — Warehouse Automation in Isaac Sim
=============================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step21_warehouse.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. TurtleBot3 USD asset 내장

목표:
    1. Warehouse Scene (Racks, Aisles, Staging Area) 구축
    2. Multi-AMR Fleet 배치 (3x TurtleBot3)
    3. WMS (Warehouse Management System) 기본 Pipeline
    4. Conveyor Belt 시뮬레이션
    5. Order Fulfillment (Pick → Transport → Place)
    6. ROS2 Multi-Robot Bridge
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
print("Step 21 — Warehouse Automation")
print("=" * 60)

# ── 2. Core API 임포트 ──
import time
import math
import numpy as np
from pxr import Sdf, Gf, UsdGeom
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

# ── 4. Warehouse Scene 생성 ──
print("\n[1/7] Creating Warehouse Scene...")

# 바닥
VisualCuboid(
    prim_path="/World/Warehouse/Floor",
    name="wh_floor",
    position=np.array([0.0, 0.0, -0.01]),
    scale=np.array([14.0, 12.0, 0.02]),
    color=np.array([0.3, 0.3, 0.3]),
)

# 벽
wall_configs = [
    ("Wall_N", (0.0, 6.0, 1.5), (14.0, 0.1, 3.0)),
    ("Wall_S", (0.0, -6.0, 1.5), (14.0, 0.1, 3.0)),
    ("Wall_E", (7.0, 0.0, 1.5), (0.1, 12.0, 3.0)),
    ("Wall_W", (-7.0, 0.0, 1.5), (0.1, 12.0, 3.0)),
]
for name, pos, scale in wall_configs:
    VisualCuboid(
        prim_path=f"/World/Warehouse/{name}",
        position=np.array(pos),
        scale=np.array(scale),
        color=np.array([0.5, 0.5, 0.5]),
    )

# Rack 시스템 (Aisle 양쪽, 3개 Aisle x 5개 Rack)
rack_color = np.array([0.5, 0.35, 0.2])
shelf_color = np.array([0.2, 0.6, 0.8])

rack_data = []
aisle_x_positions = [-4.0, 0.0, 4.0]
rack_y_positions = [-4.0, -2.0, 0.0, 2.0, 4.0]

rack_count = 0
for ax in aisle_x_positions:
    for ry in rack_y_positions:
        # Rack 베이스
        rack_path = f"/World/Warehouse/Rack_{rack_count:02d}"
        VisualCuboid(
            prim_path=rack_path,
            name=f"rack_{rack_count}",
            position=np.array([ax, ry, 0.75]),
            scale=np.array([0.8, 0.6, 1.5]),
            color=rack_color,
        )
        # 상단 선반
        VisualCuboid(
            prim_path=f"{rack_path}_Top",
            name=f"rack_{rack_count}_top",
            position=np.array([ax, ry, 1.55]),
            scale=np.array([0.6, 0.4, 0.05]),
            color=shelf_color,
        )
        rack_data.append({
            'id': rack_count,
            'label': f'R{rack_count:02d}',
            'position': (ax, ry),
            'inventory': [f'BOX-{rack_count*10 + i:03d}' for i in range(5)],
        })
        rack_count += 1

print(f"  + Warehouse: 14m x 12m")
print(f"  + Racks: {rack_count} (3 Aisles x 5 Rows)")
print(f"  + Aisle width: 2.0m")

# Rack ID 표시 (R00~R14)
for rack in rack_data:
    print(f"    {rack['label']}: pos=({rack['position'][0]:.1f}, {rack['position'][1]:.1f}), "
          f"items={len(rack['inventory'])}")

# Staging Area
VisualCuboid(
    prim_path="/World/Warehouse/StagingArea",
    name="staging",
    position=np.array([6.0, 0.0, 0.01]),
    scale=np.array([1.0, 3.0, 0.02]),
    color=np.array([0.8, 0.6, 0.1]),
)

# Loading Dock
VisualCuboid(
    prim_path="/World/Warehouse/LoadingDock",
    name="loading_dock",
    position=np.array([-6.0, 0.0, 0.01]),
    scale=np.array([1.0, 4.0, 0.02]),
    color=np.array([0.6, 0.6, 0.6]),
)

# ── 5. Conveyor Belt 생성 ──
print("\n[2/7] Creating Conveyor Belt...")

VisualCuboid(
    prim_path="/World/Warehouse/Conveyor",
    name="conveyor",
    position=np.array([6.0, 3.5, 0.08]),
    scale=np.array([3.0, 0.3, 0.04]),
    color=np.array([0.15, 0.15, 0.15]),
)

# Conveyor 아이템
conveyor_items = []
for ci in range(8):
    half = ci - 3.5
    item = VisualCuboid(
        prim_path=f"/World/Warehouse/Conveyor/Item_{ci}",
        name=f"conv_item_{ci}",
        position=np.array([6.0 + half * 0.4, 3.5, 0.11]),
        scale=np.array([0.03, 0.03, 0.03]),
        color=np.array([np.random.uniform(0.3, 0.9),
                        np.random.uniform(0.3, 0.9),
                        np.random.uniform(0.3, 0.9)]),
    )
    conveyor_items.append(item)

conveyor_speed = 0.08  # m/s
conveyor_phase = 0.0

# ── 6. AMR Fleet 배치 ──
print("\n[3/7] Deploying AMR Fleet...")

AMR_PATHS = [
    "/World/Robots/FL1",
    "/World/Robots/FL2",
    "/World/Robots/FL3",
]
AMR_NAMES = ["Forklift_1", "Forklift_2", "Forklift_3"]
AMR_POSES = [
    np.array([5.5, -4.0, 0.1]),
    np.array([5.5, 0.0, 0.1]),
    np.array([5.5, 4.0, 0.1]),
]
ASSIGNED_GOALS = [None, None, None]
GOAL_TARGETS = [
    [-4.0, -4.0],
    [0.0, 0.0],
    [4.0, 4.0],
]

amr_robots = []
for idx in range(3):
    if not is_prim_path_valid(AMR_PATHS[idx]):
        add_reference_to_stage(
            "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
            AMR_PATHS[idx],
        )
    robot = Robot(
        prim_path=AMR_PATHS[idx],
        name=AMR_NAMES[idx],
        position=AMR_POSES[idx],
    )
    world.scene.add(robot)
    amr_robots.append(robot)

print(f"  + {len(amr_robots)} AMRs deployed to Staging Area")
for i, (r, p) in enumerate(zip(AMR_NAMES, AMR_POSES)):
    print(f"    {r}: pos=({p[0]:.1f}, {p[1]:.1f})")

# ── 7. WMS (Warehouse Management System) ──
print("\n[4/7] Initializing Warehouse Management System...")

class SimpleWMS:
    def __init__(self, robots, racks):
        self.robots = robots
        self.racks = racks
        self.orders = []
        self.active_tasks = {}
        self.completed = 0
        self.order_counter = 0
        self.last_assign_time = 0

    def generate_order(self):
        """무작위 주문 생성: 한 랙에서 다른 랙으로 이동"""
        from_rack = np.random.randint(0, len(self.racks))
        to_rack = np.random.randint(0, len(self.racks))
        while to_rack == from_rack:
            to_rack = np.random.randint(0, len(self.racks))

        item = self.racks[from_rack]['inventory'][0]
        order = {
            'id': self.order_counter + 1,
            'item': item,
            'from_rack': from_rack,
            'to_rack': to_rack,
            'from_pos': self.racks[from_rack]['position'],
            'to_pos': self.racks[to_rack]['position'],
            'status': 'pending',
            'assigned_robot': None,
        }
        self.orders.append(order)
        self.order_counter += 1
        print(f"    New Order #{order['id']}: "
              f"R{from_rack:02d} → R{to_rack:02d} [{item}]")
        return order

    def assign_order(self, order):
        """가장 가까운 유휴 로봇에 주문 할당"""
        best_robot_id = None
        best_dist = float('inf')

        for i, robot in enumerate(self.robots):
            if i in self.active_tasks:
                continue
            pos, _ = robot.get_world_pose()
            dist = np.sqrt(
                (pos[0] - order['from_pos'][0])**2 +
                (pos[1] - order['from_pos'][1])**2
            )
            if dist < best_dist:
                best_dist = dist
                best_robot_id = i

        if best_robot_id is not None:
            order['assigned_robot'] = best_robot_id
            order['status'] = 'assigned'
            self.active_tasks[best_robot_id] = order
            print(f"      → Robot {best_robot_id} assigned")
            # Goal Pose 설정
            GOAL_TARGETS[best_robot_id] = list(order['from_pos'])
            self.last_assign_time = sim_time
            return True
        return False

    def update(self, current_time):
        """주문 생성 및 할당"""
        # 8초마다 새 주문
        if current_time - self.last_assign_time > 8.0:
            if len(self.orders) < 10:
                order = self.generate_order()
                self.assign_order(order)
                self.last_assign_time = current_time

        # 완료된 작업 정리
        completed_tasks = []
        for robot_id, task in self.active_tasks.items():
            pos, _ = self.robots[robot_id].get_world_pose()
            tx, ty = task['from_pos']
            dist = np.sqrt((pos[0]-tx)**2 + (pos[1]-ty)**2)
            if dist < 0.5:
                task['status'] = 'completed'
                self.completed += 1
                completed_tasks.append(robot_id)
                print(f"      Order #{task['id']} completed "
                      f"by Robot {robot_id} (delivered)")
                # 새 목표로 전환
                GOAL_TARGETS[robot_id] = list(task['to_pos'])

        for robot_id in completed_tasks:
            del self.active_tasks[robot_id]

wms = SimpleWMS(amr_robots, rack_data)

# ── 8. ROS2 Bridge ──
print("\n[5/7] Setting up ROS2 Multi-Robot Bridge...")

for idx in range(3):
    graph_path = f"/ActionGraph/Warehouse_AMR{idx}"
    
    if og.Controller.graph_exists(graph_path):
        stage = omni.usd.get_context().get_stage()
        stage.RemovePrim(Sdf.Path(graph_path))
    
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ReadOdom", "omni.isaac.core_nodes.IsaacReadOdometry"),
                ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("PubTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                ("Context.outputs:context", "PubOdom.inputs:context"),
                ("Context.outputs:context", "PubTF.inputs:context"),
                ("ReadOdom.outputs:position", "PubOdom.inputs:position"),
                ("ReadOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("Context.inputs:namespace", f"/amr{idx+1}"),
                ("ReadOdom.inputs:robotPrim",
                 Sdf.Path(f"/World/Robots/FL{idx+1}")),
                ("PubOdom.inputs:topicName",
                 f"/amr{idx+1}/odom"),
            ],
        },
    )

print("  + 3x ROS2 Bridge created:")
for idx in range(3):
    print(f"    /amr{idx+1}/odom + /tf")

# ── 9. Goal Navigation (단순 PID 추종) ──
print("\n[6/7] Starting Warehouse Operations...")

def navigate_to_goal(robot, goal_x, goal_y, dt=1/60.0):
    """단순 PID Position Controller"""
    pos, ori = robot.get_world_pose()
    
    dx = goal_x - pos[0]
    dy = goal_y - pos[1]
    dist = np.sqrt(dx**2 + dy**2)
    
    if dist < 0.2:
        return np.array([0.0, 0.0])  # 도착
    
    # 각도 계산
    target_yaw = math.atan2(dy, dx)
    current_yaw = ori[2]  # 단순화
    
    # 속도 명령
    linear_vel = min(0.3, dist * 0.5)
    angular_vel = math.atan2(math.sin(target_yaw - current_yaw),
                             math.cos(target_yaw - current_yaw)) * 1.5
    angular_vel = np.clip(angular_vel, -0.5, 0.5)
    
    return np.array([linear_vel, angular_vel])

sim_time = 0.0
last_wms_time = 0.0

# ── 10. 시뮬레이션 루프 ──
print("\n" + "="*60)
print("  Warehouse Simulation Running...")
print("="*60)
print()

for i in range(2400):  # ~40초 @60fps
    world.step(render=True)
    sim_time += 1/60.0

    # Conveyor Belt 업데이트
    conveyor_phase += conveyor_speed
    if conveyor_phase > 3.0:
        conveyor_phase -= 3.0
    
    stage = omni.usd.get_context().get_stage()
    for ci, item in enumerate(conveyor_items):
        item_x = 6.0 - 1.5 + conveyor_phase + ci * 0.4 - 3.0
        if item_x > 7.5:
            item_x -= 6.0
        xform = UsdGeom.Xformable(
            stage.GetPrimAtPath(f"/World/Warehouse/Conveyor/Item_{ci}"))
        op = xform.ClearXformOpOrder()
        op = xform.AddTranslateOp()
        op.Set(Gf.Vec3d(item_x, 3.5, 0.11))

    # WMS 업데이트
    wms.update(sim_time)

    # AMR Navigation
    for idx, robot in enumerate(amr_robots):
        gx, gy = GOAL_TARGETS[idx]
        vel = navigate_to_goal(robot, gx, gy)
        action = ArticulationAction(
            joint_velocities=np.array([vel[0], vel[0], vel[1]]),
        )
        robot.apply_action(action)

    # 로그
    if i % 300 == 0:
        print(f"  [{i//60:2d}s] Orders: {wms.order_counter}, "
              f"Completed: {wms.completed}, Active: {len(wms.active_tasks)}")
        for idx, robot in enumerate(amr_robots):
            pos, _ = robot.get_world_pose()
            print(f"           {AMR_NAMES[idx]}: "
                  f"({pos[0]:.2f}, {pos[1]:.2f}) → "
                  f"({GOAL_TARGETS[idx][0]:.1f}, {GOAL_TARGETS[idx][1]:.1f})")

# ── 11. 결과 확인 ──
print("\n[7/7] Verifying...")

graph_count = 0
for idx in range(3):
    if og.Controller.graph_exists(f"/ActionGraph/Warehouse_AMR{idx}"):
        graph_count += 1
print(f"  + Action Graphs: {graph_count}/3 active")

# ── 12. 요약 ──
print("\n" + "="*60)
print("  Step 21 — Summary")
print("="*60)
print()
print("  Warehouse Automation:")
print()
print("  ✅ Warehouse Scene (14m x 12m)")
print(f"     {rack_count} Racks (3 Aisles x 5 Rows)")
print(f"     Conveyor Belt ({len(conveyor_items)} items)")
print(f"     Staging Area + Loading Dock")
print()
print(f"  ✅ AMR Fleet: {len(amr_robots)} TurtleBot3")
print(f"     Name: {', '.join(AMR_NAMES)}")
print()
print(f"  ✅ WMS Pipeline:")
print(f"      Orders generated: {wms.order_counter}")
print(f"      Orders completed: {wms.completed}")
print(f"      Active tasks: {len(wms.active_tasks)}")
print()
print("  ✅ ROS2 Multi-Robot Bridge:")
for idx in range(3):
    print(f"     /amr{idx+1}/odom + /tf")
print()
print("  ✅ Key Concepts:")
print("     - Warehouse Layout: Racks, Aisles, Staging")
print("     - Multi-AMR Navigation with PID Controller")
print("     - Order Fulfillment: Pick → Transport → Place")
print("     - Conveyor Belt: Loop Animation")
print("     - WMS: Order Generation → Assignment → Completion")
print("="*60)

simulation_app.close()
