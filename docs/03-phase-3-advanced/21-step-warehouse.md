# Step 21 — Warehouse Automation

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 16 (Multi-Robot), Step 14 (Nav2)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **창고 시뮬레이션 환경**을 Isaac Sim에 구축한다
2. **다중 AMR**을 창고에서 동시 운용한다
3. **Order Fulfillment Pipeline**을 구현한다 (Pick → Transport → Place)
4. **Rack-to-Rack** 자율 주행을 Nav2로 실행한다
5. **Shelving System**과 **Conveyor Belt**를 시뮬레이션한다
6. **Warehouse Management System (WMS)** 의 기본 개념을 구현한다
7. **ROS2 Multi-Robot Warehouse System**을 구축한다

---

## 1. Warehouse 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                    Isaac Sim Warehouse                    │
│                                                          │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐        │
│  │ Rack 1 │  │ Rack 2 │  │ Rack 3 │  │ Rack 4 │ ...    │
│  └───┬────┘  └───┬────┘  └───┬────┘  └───┬────┘        │
│      │           │           │           │              │
│  ┌───▼───────────▼───────────▼───────────▼──────────┐   │
│  │               Aisle (Navigation Corridor)          │   │
│  └───────────────────────┬───────────────────────────┘   │
│                          │                              │
│  ┌───────────────────────▼───────────────────────────┐   │
│  │              Staging Area                          │   │
│  │  ┌────────┐  ┌────────┐  ┌────────────────────┐  │   │
│  │  │ AMR 1  │  │ AMR 2  │  │  Conveyor Belt     │  │   │
│  │  └────────┘  └────────┘  └────────────────────┘  │   │
│  └───────────────────────────────────────────────────┘   │
│                                                          │
│                    ROS2 Bridge                           │
│  /amr1/cmd_vel  /amr2/cmd_vel  /wms/orders              │
└──────────────────────────────────────────────────────────┘
```

### 1.1 Warehouse 구성 요소

| 구성 요소 | 역할 | Isaac Sim 구현 |
|-----------|------|---------------|
| **Rack** | 물품 보관대 | VisualCuboid (다층) |
| **Aisle** | AMR 주행 통로 | 2m 너비 빈 공간 |
| **Staging Area** | 집/하차 구역 | 지정된 Goal Pose |
| **Conveyor Belt** | 물품 이송 | 시각적 큐브 이동 |
| **AMR Fleet** | 물품 운반 로봇 | TurtleBot3 x 3대 |
| **WMS** | 주문 관리 시스템 | Python Node |

### 1.2 Warehouse Layout (예: 12m x 10m)

```
    ┌──────────────────────────────────────────────────────┐
    │  Loading Dock                                        │
    ├──────────┬──────────┬──────────┬──────────┬──────────┤
    │          │          │          │          │          │
    │  Rack A1 │  Rack A2 │  Rack A3 │  Rack A4 │  Rack A5 │
    │          │          │          │          │          │
    ├──────────┴──────────┴──────────┴──────────┴──────────┤
    │              Aisle 1 (2m wide)                       │
    ├──────────┬──────────┬──────────┬──────────┬──────────┤
    │          │          │          │          │          │
    │  Rack B1 │  Rack B2 │  Rack B3 │  Rack B4 │  Rack B5 │
    │          │          │          │          │          │
    ├──────────┴──────────┴──────────┴──────────┴──────────┤
    │              Aisle 2 (2m wide)                       │
    ├──────────┬──────────┬──────────┬──────────┬──────────┤
    │          │          │          │          │          │
    │  Rack C1 │  Rack C2 │  Rack C3 │  Rack C4 │  Rack C5 │
    │          │          │          │          │          │
    ├──────────┴──────────┴──────────┴──────────┴──────────┤
    │              Staging Area                            │
    └──────────────────────────────────────────────────────┘
```

---

## 2. Warehouse Scene 생성

### 2.1 창고 구조물

```python
def create_warehouse_scene():
    """창고 Scene 생성"""
    
    from omni.isaac.core.objects import VisualCuboid
    
    # 바닥
    VisualCuboid(
        prim_path="/World/Warehouse/Floor",
        name="wh_floor",
        position=np.array([0.0, 0.0, -0.01]),
        scale=np.array([12.0, 10.0, 0.02]),
        color=np.array([0.25, 0.25, 0.25]),
    )
    
    # 벽
    wall_configs = [
        ("Wall_N", (6.0, 0.0, 1.5), (0.1, 10.0, 3.0)),
        ("Wall_S", (-6.0, 0.0, 1.5), (0.1, 10.0, 3.0)),
        ("Wall_E", (0.0, 5.0, 1.5), (12.0, 0.1, 3.0)),
        ("Wall_W", (0.0, -5.0, 1.5), (12.0, 0.1, 3.0)),
    ]
    for name, pos, scale in wall_configs:
        VisualCuboid(
            prim_path=f"/World/Warehouse/{name}",
            position=np.array(pos),
            scale=np.array(scale),
            color=np.array([0.5, 0.5, 0.5]),
        )
    
    # Rack 시스템 (Aisle 기준 양쪽)
    rack_positions = []
    for aisle_x in [-4, 0, 4]:
        for rack_y in [-4, -2, 0, 2, 4]:
            rack_positions.append((aisle_x, rack_y))
    
    for i, (x, y) in enumerate(rack_positions):
        VisualCuboid(
            prim_path=f"/World/Warehouse/Rack_{i:02d}",
            name=f"rack_{i}",
            position=np.array([x, y, 0.75]),
            scale=np.array([0.8, 0.6, 1.5]),
            color=np.array([0.4, 0.3, 0.2]),
        )
        # 상단 표시
        VisualCuboid(
            prim_path=f"/World/Warehouse/Rack_{i:02d}_Top",
            name=f"rack_{i}_top",
            position=np.array([x, y, 1.55]),
            scale=np.array([0.6, 0.4, 0.05]),
            color=np.array([0.2, 0.5, 0.8]),
        )
    
    print(f"  + Warehouse: 12m x 10m, {len(rack_positions)} racks")
    return len(rack_positions)
```

### 2.2 AMR + Manipulator 배치

```python
def deploy_warehouse_robots():
    """창고용 AMR 로봇 배치"""
    
    from omni.isaac.core.robots import Robot
    
    AMR_CONFIGS = [
        {"name": "Forklift_1", "pos": np.array([-7.0, -2.0, 0.1]),
         "path": "/World/Robots/FL1"},
        {"name": "Forklift_2", "pos": np.array([-7.0, 0.0, 0.1]),
         "path": "/World/Robots/FL2"},
        {"name": "Forklift_3", "pos": np.array([-7.0, 2.0, 0.1]),
         "path": "/World/Robots/FL3"},
    ]
    
    robots = []
    for cfg in AMR_CONFIGS:
        if not is_prim_path_valid(cfg["path"]):
            add_reference_to_stage(
                "/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd",
                cfg["path"],
            )
        robot = Robot(
            prim_path=cfg["path"],
            name=cfg["name"],
            position=cfg["pos"],
        )
        world.scene.add(robot)
        robots.append(robot)
    
    print(f"  + {len(robots)} AMRs deployed")
    return robots
```

---

## 3. Order Fulfillment Pipeline

### 3.1 WMS (Warehouse Management System)

```python
class WarehouseManagementSystem(Node):
    """주문 → 할당 → 실행 → 완료 Pipeline"""
    
    def __init__(self):
        super().__init__('warehouse_management')
        
        # 창고 상태
        self.inventory = {}  # rack_id → [items]
        self.orders = []     # pending orders
        self.active_tasks = {}  # robot → task
        self.completed_orders = []
        
        # AMR 상태
        self.robot_status = {}
        for ns in ['/amr1', '/amr2', '/amr3']:
            self.create_subscription(
                Odometry, f'{ns}/odom',
                lambda msg, robot=ns: self.on_robot_pose(msg, robot),
                10,
            )
        
        # Task Publisher
        self.goal_publishers = {
            ns: self.create_publisher(
                PoseStamped, f'{ns}/goal_pose', 10)
            for ns in ['/amr1', '/amr2', '/amr3']
        }
        
        # 주문 타이머
        self.timer = self.create_timer(5.0, self.process_orders)
        
        self.get_logger().info('WMS initialized')
    
    def generate_order(self, item_id, from_rack, to_rack):
        """새 주문 생성"""
        order = {
            'id': len(self.orders) + 1,
            'item': item_id,
            'from': from_rack,
            'to': to_rack,
            'status': 'pending',
            'assigned_robot': None,
        }
        self.orders.append(order)
        self.get_logger().info(f'New order #{order["id"]}: {item_id} '
                               f'{from_rack} → {to_rack}')
        return order
    
    def assign_task(self, order):
        """가장 가까운 AMR에 주문 할당"""
        rack_positions = {
            'R01': (-4, -4), 'R02': (-4, -2), 'R03': (-4, 0),
            'R04': (-4, 2), 'R05': (-4, 4), 'R06': (0, -4),
            'R07': (0, -2), 'R08': (0, 0), 'R09': (0, 2),
            'R10': (0, 4), 'R11': (4, -4), 'R12': (4, -2),
            'R13': (4, 0), 'R14': (4, 2), 'R15': (4, 4),
        }
        
        # 가장 가까운 사용 가능 로봇 찾기
        best_robot = None
        best_dist = float('inf')
        
        from_pos = rack_positions.get(order['from'], (0, 0))
        
        for ns, status in self.robot_status.items():
            if ns in self.active_tasks:
                continue  # 이미 작업 중
            rx, ry = status.get('position', (0, 0))
            dist = np.sqrt((rx - from_pos[0])**2 + (ry - from_pos[1])**2)
            if dist < best_dist:
                best_dist = dist
                best_robot = ns
        
        if best_robot:
            order['assigned_robot'] = best_robot
            order['status'] = 'assigned'
            self.active_tasks[best_robot] = order
            
            # Goal Pose 발행
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = from_pos[0]
            goal.pose.position.y = from_pos[1]
            goal.pose.orientation.w = 1.0
            self.goal_publishers[best_robot].publish(goal)
            
            self.get_logger().info(
                f'  → {best_robot} assigned to {order["from"]}')
    
    def process_orders(self):
        """주기적으로 대기 주문 처리"""
        for order in self.orders:
            if order['status'] == 'pending':
                self.assign_task(order)
```

### 3.2 Conveyor Belt Simulation

```python
class ConveyorBelt:
    """컨베이어 벨트 시뮬레이션"""
    
    def __init__(self, world, belt_path="/World/Warehouse/Conveyor"):
        self.world = world
        self.belt_path = belt_path
        self.items = []
        self.speed = 0.1  # m/s
        
        # 컨베이어 벨트 생성
        VisualCuboid(
            prim_path=belt_path,
            name="conveyor",
            position=np.array([0.0, -3.5, 0.1]),
            scale=np.array([4.0, 0.3, 0.05]),
            color=np.array([0.2, 0.2, 0.2]),
        )
    
    def spawn_item(self):
        """컨베이어에 새 물품 생성"""
        from omni.isaac.core.objects import VisualCuboid
        
        item = VisualCuboid(
            prim_path=f"{self.belt_path}/Item_{len(self.items)}",
            name=f"conveyor_item_{len(self.items)}",
            position=np.array([-2.0, -3.5, 0.12]),
            scale=np.array([0.05, 0.05, 0.05]),
            color=np.random.rand(3),
        )
        self.items.append({
            'prim': item,
            'position': -2.0,
            'color': np.random.rand(3),
        })
    
    def update(self, dt=1/60.0):
        """물품 이동"""
        for item in self.items:
            item['position'] += self.speed * dt
            if item['position'] > 2.0:
                item['position'] = -2.0  # Loop
            # USD 위치 업데이트
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(
                f"{self.belt_path}/Item_{self.items.index(item)}")
            if prim:
                xform = UsdGeom.Xformable(prim)
                op = xform.ClearXformOpOrder()
                op = xform.AddTranslateOp()
                op.Set(Gf.Vec3d(item['position'], -3.5, 0.12))
```

---

## 4. Multi-AMR Warehouse Navigation

### 4.1 Namespaced Nav2 AMR

```bash
# 각 AMR 별 Nav2 인스턴스
for robot in amr1 amr2 amr3; do
    ros2 launch nav2_bringup navigation_launch.py \
      namespace:=/${robot} \
      params_file:=~/isaac-step-curriculum/config/warehouse_nav2.yaml \
      use_sim_time:=True &
done
```

### 4.2 Warehouse Nav2 파라미터

```yaml
# 창고 전용 Nav2 설정
amcl:
  ros__parameters:
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"

global_costmap:
  global_costmap:
    ros__parameters:
      robot_base_frame: base_footprint
      footprint: "[ [0.18, 0.14], [0.18, -0.14], [-0.18, -0.14], [-0.18, 0.14] ]"
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        map_topic: /map
      inflation_layer:
        inflation_radius: 0.4  # 좁은 통로용
      width: 12.0
      height: 10.0
      resolution: 0.05

local_costmap:
  local_costmap:
    ros__parameters:
      rolling_window: true
      width: 2.0  # 좁은 창고 통로
      height: 2.0
      resolution: 0.05

controller_server:
  ros__parameters:
    FollowPath:
      desired_linear_vel: 0.3  # 창고 주행 속도
      lookahead_dist: 0.4
      max_linear_vel: 0.5
```

### 4.3 Collision Avoidance in Warehouse

```python
class WarehouseCollisionAvoidance(Node):
    """창고 내 AMR 충돌 회피"""
    
    AISLE_WIDTH = 2.0
    ROBOT_RADIUS = 0.2
    
    def __init__(self):
        super().__init__('warehouse_collision_avoider')
        
        self.poses = {}
        for ns in ['/amr1', '/amr2', '/amr3']:
            self.create_subscription(
                Odometry, f'{ns}/odom',
                lambda msg, robot=ns: self.on_odom(msg, robot), 10)
        
        self.vel_pubs = {
            ns: self.create_publisher(Twist, f'{ns}/cmd_vel_priority', 10)
            for ns in ['/amr1', '/amr2', '/amr3']
        }
        
        self.timer = self.create_timer(0.1, self.check_aisle_conflict)
    
    def on_odom(self, msg, robot_ns):
        self.poses[robot_ns] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
    
    def check_aisle_conflict(self):
        """좁은 통로에서 마주칠 경우 속도 조정"""
        for r1, p1 in self.poses.items():
            for r2, p2 in self.poses.items():
                if r1 >= r2: continue
                dist = np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                if dist < 1.0:
                    self.get_logger().warn(
                        f'⚠ Aisle conflict: {r1} ↔ {r2} ({dist:.2f}m)')
                    # 한쪽 정지
                    twist = Twist()
                    twist.linear.x = 0.0
                    self.vel_pubs[r2].publish(twist)
```

---

## 5. 실행 절차

### 5.1 Terminal Setup

```bash
# ════════════════════════════════════════════════════════
# Warehouse Automation — 5 Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim Warehouse
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-3/step21_warehouse.py

# 터미널 2: WMS + Task Dispatcher
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python ~/isaac-step-curriculum/code/phase-3/warehouse_wms.py

# 터미널 3-5: 각 AMR Nav2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch nav2_bringup navigation_launch.py \
  namespace:=/amr1 use_sim_time:=True &
# ... amr2, amr3

# 터미널 6: 모니터링
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list | grep /amr
watch -n 1 "ros2 topic echo /wms/status --once"
```

### 5.2 WMS 명령어

```bash
# 수동 주문 생성
ros2 service call /wms/new_order warehouse_msgs/Order \
  "{item_id: 'BOX-001', from_rack: 'R01', to_rack: 'R08'}"

# 창고 상태 확인
ros2 topic echo /wms/inventory --once

# AMR 상태 확인
ros2 topic echo /amr1/odom --once | head
```

---

## 6. 문제 해결

### 문제 1: 좁은 통로에서 AMR이 충돌합니다.

**해결:**
- Nav2 footprint를 실제보다 작게 설정
- Local Costmap rolling window = 2m
- 속도 제한: max_vel = 0.3 m/s

### 문제 2: WMS 주문이 할당되지 않습니다.

**해결:**
```python
# AMR 상태 수신 확인
ros2 topic echo /amr1/odom --once
# Goal Pose 발행 확인
ros2 topic echo /amr1/goal_pose --once
```

### 문제 3: Conveyor Belt 물품이 보이지 않습니다.

**해결:**
```python
# Visual 속성 확인
item_prim.GetAttribute("visibility").Set("inherited")
```

---

## 7. 정리

| 항목 | 내용 |
|------|------|
| ✅ Warehouse Scene | Rack, Aisle, Staging Area |
| ✅ AMR Fleet | 3x TurtleBot3 Namespaced |
| ✅ WMS | Order → Assign → Execute |
| ✅ Conveyor Belt | 아이템 이동 시뮬레이션 |
| ✅ Nav2 Warehouse | 좁은 통로 주행 최적화 |
| ✅ Collision Avoidance | Aisle conflict resolution |

---

## 8. 다음 Step 예고

**Step 22 — AI Worker**에서는:
- AI Worker 개념 (인간 + AI 협업)
- Perception Pipeline (YOLO + Isaac Sim)
- Task Planning with Behavior Tree
- Human-Robot Interaction
- Warehouse AI Worker 통합

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim Warehouse | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/tutorial_robotics_warehouse.html |
| ROS2 Multi-Robot | https://docs.nav2.org/tutorials/docs/navigation2_with_multiple_robots.html |
| WMS Design | https://en.wikipedia.org/wiki/Warehouse_management_system |
| Conveyor Belt | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robotics/tutorial_conveyor.html |
