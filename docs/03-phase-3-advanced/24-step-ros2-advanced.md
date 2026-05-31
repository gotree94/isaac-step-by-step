# Step 24 — ROS2 Advanced

> **소요 시간**: 120분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 11 (ROS2 Bridge), Step 16 (Multi-Robot)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **ROS2 Lifecycle Nodes**를 Isaac Sim과 연동한다
2. **Node Composition**으로 단일 프로세스에서 여러 노드를 실행한다
3. **ROS2 Actions**를 사용한 장기 실행 태스크를 구현한다
4. **ros2_control** 하드웨어 인터페이스를 Isaac Sim에 연결한다
5. **ROS2 Parameters** 동적 재설정을 구현한다
6. **Security (SROS2)** 를 설정한다
7. **Real Robot ↔ Isaac Sim** 통신 브리지를 구축한다

---

## 1. ROS2 Advanced 아키텍처

```
┌────────────────────────────────────────────────────────────┐
│                    ROS2 Advanced System                      │
│                                                              │
│  ┌──────────────────┐   ┌──────────────────┐               │
│  │  Lifecycle Node   │   │  Composable Node  │              │
│  │  (Managed Node)    │   │  (Component)      │              │
│  │                    │   │                    │              │
│  │  Unconfigured ────▶│   │  container.exe ──▶│              │
│  │  Inactive ────────▶│   │  loader ─────────▶│              │
│  │  Active ──────────▶│   │  composable       │              │
│  │  Finalized         │   │  node             │              │
│  └──────────────────┘   └──────────────────┘               │
│                                                              │
│  ┌──────────────────┐   ┌──────────────────┐               │
│  │  ROS2 Actions     │   │  ros2_control     │              │
│  │                    │   │                    │              │
│  │  Goal ──▶ Execute │   │  Controller ──▶   │              │
│  │  Feedback ────────│   │  Hardware Interface│              │
│  │  Result ──────────│   │  ──▶ Isaac Sim    │              │
│  └──────────────────┘   └──────────────────┘               │
│                                                              │
│  ┌──────────────────┐   ┌──────────────────┐               │
│  │  SROS2 Security   │   │  Real Robot       │              │
│  │  (DDS Security)    │   │  Bridge            │              │
│  │  keystore ────────│   │  Isaac Sim ────▶  │              │
│  │  permissions      │   │  Real Robot       │              │
│  └──────────────────┘   └──────────────────┘               │
└──────────────────────────────────────────────────────────────┘
```

### 1.1 ROS2 Advanced 기능 비교

| 기능 | 기본 ROS2 | Advanced | Isaac Sim 연동 |
|------|-----------|----------|---------------|
| **Node Lifecycle** | 항상 Active | Managed States | Start/Stop 제어 |
| **Composition** | 개별 프로세스 | 단일 프로세스 | 성능 최적화 |
| **Actions** | 단순 Pub/Sub | Goal/Feedback/Result | MoveIt2, Navigation |
| **ros2_control** | 직접 제어 | Hardware Interface | Real Robot 연동 |
| **Parameters** | Static | Dynamic Reconfigure | 실시간 튜닝 |
| **Security** | 미사용 | Encrypted DDS | Enterprise |

---

## 2. Lifecycle Nodes

### 2.1 Managed Node 구현

```cpp
// lifecycle_isaac_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using LifecycleCallbackReturn = 
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class IsaacSimLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    IsaacSimLifecycleNode() 
        : rclcpp_lifecycle::LifecycleNode("isaac_lifecycle_node")
    {
        RCLCPP_INFO(get_logger(), "Lifecycle Node created");
    }

    // ── Configure: 리소스 할당 ──
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring...");
        
        // Isaac Sim 파라미터 로드
        this->declare_parameter("robot_name", "franka");
        this->declare_parameter("update_rate", 100.0);
        
        // Publisher (Lifecycle-aware)
        joint_state_pub_ = this->create_publisher<JointState>(
            "joint_states", 10);
        
        RCLCPP_INFO(get_logger(), "Configured successfully");
        return LifecycleCallbackReturn::SUCCESS;
    }

    // ── Activate: 통신 시작 ──
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating...");
        
        // Isaac Sim ROS2 Bridge 활성화
        activate_isaac_bridge();
        
        // Publisher 활성화
        joint_state_pub_->on_activate();
        
        // Timer 시작
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&IsaacSimLifecycleNode::timer_callback, this));
        
        RCLCPP_INFO(get_logger(), "Activated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    // ── Deactivate: 통신 중지 ──
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        
        timer_->cancel();
        joint_state_pub_->on_deactivate();
        deactivate_isaac_bridge();
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    // ── Cleanup: 리소스 해제 ──
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        joint_state_pub_.reset();
        timer_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    // ── Shutdown: 종료 ──
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        auto msg = JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint1", "joint2", "joint3"};
        msg.position = {0.1, -0.5, 1.2};
        joint_state_pub_->publish(msg);
    }

    void activate_isaac_bridge()
    {
        // Graph 활성화
        system("ros2 service call /isaac/activate_ros2_bridge std_srvs/Empty");
    }

    void deactivate_isaac_bridge()
    {
        system("ros2 service call /isaac/deactivate_ros2_bridge std_srvs/Empty");
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<JointState>> 
        joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### 2.2 Lifecycle Node 명령

```bash
# Node 상태 전이
ros2 lifecycle set /isaac_lifecycle_node configure
ros2 lifecycle set /isaac_lifecycle_node activate
ros2 lifecycle set /isaac_lifecycle_node deactivate
ros2 lifecycle set /isaac_lifecycle_node cleanup
ros2 lifecycle set /isaac_lifecycle_node shutdown

# 상태 확인
ros2 lifecycle get /isaac_lifecycle_node
# 출력: primary_state: active

# Transition 이벤트 모니터링
ros2 topic echo /isaac_lifecycle_node/transition_event
```

### 2.3 Lifecycle Manager

```python
class LifecycleManager(Node):
    """Isaac Sim Lifecycle Manager"""
    
    def __init__(self):
        super().__init__('lifecycle_manager')
        
        # Managed nodes
        self.managed_nodes = {
            '/isaac/perception': 'inactive',
            '/isaac/control': 'inactive',
            '/isaac/navigation': 'inactive',
        }
        
        # Service clients for each managed node
        self.clients = {}
        for node_name in self.managed_nodes:
            self.clients[node_name] = self.create_client(
                ChangeState, f'{node_name}/change_state')
        
        self.timer = self.create_timer(10.0, self.check_all_nodes)
        self.get_logger().info('Lifecycle Manager started')
    
    def transition_node(self, node_name, transition_id):
        """Node state transition"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        self.clients[node_name].call_async(req)
        self.get_logger().info(
            f'Transition {node_name}: {transition_id}')
    
    def start_pipeline(self):
        """전체 Pipeline 시작"""
        self.transition_node('/isaac/perception', 1)   # configure
        self.transition_node('/isaac/control', 1)
        self.transition_node('/isaac/navigation', 1)
        # Wait...
        self.transition_node('/isaac/perception', 3)   # activate
        self.transition_node('/isaac/control', 3)
        self.transition_node('/isaac/navigation', 3)
    
    def stop_pipeline(self):
        """전체 Pipeline 정지"""
        for node in self.managed_nodes:
            self.transition_node(node, 4)  # deactivate
        for node in self.managed_nodes:
            self.transition_node(node, 6)  # cleanup
    
    def check_all_nodes(self):
        """모든 노드 상태 확인"""
        for node_name, _ in self.managed_nodes.items():
            client = self.create_client(
                GetState, f'{node_name}/get_state')
            if client.wait_for_service(timeout_sec=0.5):
                req = GetState.Request()
                future = client.call_async(req)
```

---

## 3. Node Composition

### 3.1 Composable Node 구현

```cpp
// composable_isaac_node.hpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace isaac_composition
{

class IsaacSensorNode : public rclcpp::Node
{
public:
    IsaacSensorNode(const rclcpp::NodeOptions & options)
        : Node("isaac_sensor", options)
    {
        RCLCPP_INFO(get_logger(), "Isaac Sensor Node (Composable)");
        
        // Camera publisher
        camera_pub_ = this->create_publisher<Image>("camera/image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]() { publish_camera_data(); });
    }

private:
    void publish_camera_data()
    {
        auto msg = Image();
        msg.header.stamp = this->now();
        msg.height = 480;
        msg.width = 640;
        msg.encoding = "rgb8";
        camera_pub_->publish(msg);
    }

    rclcpp::Publisher<Image>::SharedPtr camera_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class IsaacControlNode : public rclcpp::Node
{
public:
    IsaacControlNode(const rclcpp::NodeOptions & options)
        : Node("isaac_control", options)
    {
        RCLCPP_INFO(get_logger(), "Isaac Control Node (Composable)");
        
        joint_cmd_sub_ = this->create_subscription<JointState>(
            "joint_commands", 10,
            [this](JointState::ConstSharedPtr msg) {
                execute_joint_commands(msg);
            }
        );
    }

private:
    void execute_joint_commands(std::shared_ptr<const JointState> msg)
    {
        // Isaac Sim으로 명령 전달
        RCLCPP_DEBUG(get_logger(), 
            "Executing %zu joint commands", msg->name.size());
    }

    rclcpp::Subscription<JointState>::SharedPtr joint_cmd_sub_;
};

}  // namespace isaac_composition

RCLCPP_COMPONENTS_REGISTER_NODE(isaac_composition::IsaacSensorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_composition::IsaacControlNode)
```

### 3.2 Composition 실행

```bash
# 단일 프로세스에서 여러 노드 실행
ros2 component standalone \
  isaac_components \
  isaac_composition::IsaacSensorNode \
  isaac_composition::IsaacControlNode

# 또는 수동 컴포지션
ros2 component container /isaac_container
ros2 component load /isaac_container \
  isaac_components isaac_composition::IsaacSensorNode
ros2 component load /isaac_container \
  isaac_components isaac_composition::IsaacControlNode

# 컴포넌트 목록
ros2 component list /isaac_container
```

### 3.3 Python Composition (rclpy)

```python
# composable_isaac.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class ComposableIsaacNode(Node):
    """Isaac Sim Composable Node (Python)"""
    
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.cb_group = ReentrantCallbackGroup()
        self.components = {}
    
    def add_component(self, name, component_class, **kwargs):
        """동적 컴포넌트 추가"""
        component = component_class(self, **kwargs)
        self.components[name] = component
        self.get_logger().info(f'Component added: {name}')
        return component

class SensorComponent:
    def __init__(self, node, topic="camera/image"):
        self.node = node
        self.pub = node.create_publisher(
            Image, topic, 10)
        self.timer = node.create_timer(
            0.033, self.publish, 
            callback_group=node.cb_group)
    
    def publish(self):
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(msg)

class ControlComponent:
    def __init__(self, node, topic="joint_commands"):
        self.node = node
        self.sub = node.create_subscription(
            JointState, topic, self.callback, 10,
            callback_group=node.cb_group)
    
    def callback(self, msg):
        self.node.get_logger().debug(
            f'Received: {len(msg.name)} joints')

# Composition 실행
def main():
    rclpy.init()
    
    container = ComposableIsaacNode("isaac_container")
    container.add_component("sensor", SensorComponent)
    container.add_component("control", ControlComponent)
    
    executor = MultiThreadedExecutor()
    executor.add_node(container)
    executor.spin()

if __name__ == '__main__':
    main()
```

---

## 4. ROS2 Actions

### 4.1 Action Server (장기 실행 태스크)

```python
class PickAndPlaceActionServer(Node):
    """Pick-and-Place Action Server for Isaac Sim"""
    
    def __init__(self):
        super().__init__('pick_and_place_action_server')
        
        self.action_server = ActionServer(
            self,
            PickAndPlace,  # 사용자 정의 Action
            'pick_and_place',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        
        self.get_logger().info('Pick-and-Place Action Server ready')
    
    def goal_callback(self, goal_request):
        """Goal 수신 확인"""
        self.get_logger().info(
            f'Received goal: pick {goal_request.object} '
            f'from ({goal_request.pick_pose.position.x:.2f}, '
            f'{goal_request.pick_pose.position.y:.2f})')
        
        # Goal 유효성 검사
        if goal_request.object_id < 0:
            self.get_logger().warn('Invalid object ID')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Goal 취소 처리"""
        self.get_logger().info('Cancelling goal')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """실제 실행 로직 (async)"""
        self.get_logger().info('Executing pick-and-place...')
        
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        
        # Phase 1: Move to pick position
        feedback.current_phase = "moving_to_pick"
        goal_handle.publish_feedback(feedback)
        
        # Isaac Sim: MoveIt2 trajectory
        await self.move_to_position(
            goal_handle.request.pick_pose.position)
        
        # Phase 2: Pick (close gripper)
        feedback.current_phase = "grasping"
        goal_handle.publish_feedback(feedback)
        
        await self.grasp_object(
            goal_handle.request.object_id)
        
        # Phase 3: Move to place position
        feedback.current_phase = "moving_to_place"
        goal_handle.publish_feedback(feedback)
        
        await self.move_to_position(
            goal_handle.request.place_pose.position)
        
        # Phase 4: Place (open gripper)
        feedback.current_phase = "placing"
        goal_handle.publish_feedback(feedback)
        await self.release_object()
        
        # Complete
        result.success = True
        result.execution_time = 10.0
        goal_handle.succeed()
        
        self.get_logger().info('Pick-and-place completed')
        return result
    
    async def move_to_position(self, target):
        """Isaac Sim 모션 플래닝"""
        # ROS2 Action → Isaac Sim Graph 호출
        for i in range(10):
            await asyncio.sleep(0.1)
            self.get_logger().debug(
                f'Moving to target... ({i*10}%)')
    
    async def grasp_object(self, object_id):
        await asyncio.sleep(0.5)
    
    async def release_object(self):
        await asyncio.sleep(0.3)
```

### 4.2 Action Client

```python
class MissionControlClient(Node):
    """Pick-and-Place Action Client"""
    
    def __init__(self):
        super().__init__('mission_control_client')
        
        self.action_client = ActionClient(
            self, PickAndPlace, 'pick_and_place')
        
        # Goal 큐
        self.goal_queue = []
        self.current_goal = None
        self.timer = self.create_timer(5.0, self.process_queue)
    
    def send_goal(self, pick_pose, place_pose, object_id):
        """새 Goal 전송"""
        goal = PickAndPlace.Goal()
        goal.pick_pose = pick_pose
        goal.place_pose = place_pose
        goal.object_id = object_id
        goal.object = f"box_{object_id:03d}"
        
        self.goal_queue.append(goal)
        self.get_logger().info(f'Goal queued: box_{object_id}')
    
    def process_queue(self):
        """큐 처리"""
        if self.current_goal is not None:
            return  # 현재 Goal 실행 중
        
        if not self.goal_queue:
            return
        
        goal = self.goal_queue.pop(0)
        
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server unavailable')
            return
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(
            self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Feedback 처리"""
        phase = feedback_msg.feedback.current_phase
        self.get_logger().info(f'  Phase: {phase}')
    
    def goal_response_callback(self, future):
        """Goal 응답 처리"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.current_goal = None
            return
        
        self.get_logger().info('Goal accepted')
        self.current_goal = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """결과 처리"""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'Goal completed in {result.execution_time}s')
        else:
            self.get_logger().error('Goal failed')
        self.current_goal = None
```

---

## 5. ros2_control Hardware Interface

### 5.1 Hardware Interface (Isaac Sim)

```cpp
// isaac_sim_hardware_interface.cpp
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

class IsaacSimHardwareInterface : public hardware_interface::SystemInterface
{
public:
    IsaacSimHardwareInterface() = default;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                       "joint5", "joint6", "joint7"};
        
        position_commands_.resize(joint_names_.size(), 0.0);
        position_states_.resize(joint_names_.size(), 0.0);
        velocity_states_.resize(joint_names_.size(), 0.0);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(rclcpp::get_logger("IsaacSimHW"), 
            "Configuring...");
        reset_ros2_bridge();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(rclcpp::get_logger("IsaacSimHW"), 
            "Activating...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(rclcpp::get_logger("IsaacSimHW"), 
            "Deactivating...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Read from Isaac Sim
    hardware_interface::return_type read(
        const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Isaac Sim으로부터 Joint State 읽기
        auto joint_state_msg = get_isaac_joint_states();
        for (size_t i = 0; i < joint_names_.size(); i++)
        {
            position_states_[i] = joint_state_msg.position[i];
            velocity_states_[i] = joint_state_msg.velocity[i];
        }
        return hardware_interface::return_type::OK;
    }

    // Write to Isaac Sim
    hardware_interface::return_type write(
        const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // Isaac Sim으로 Joint Command 전송
        send_isaac_joint_commands(position_commands_);
        return hardware_interface::return_type::OK;
    }

private:
    void reset_ros2_bridge()
    {
        system("ros2 service call /isaac/reset_bridge std_srvs/Empty");
    }

    sensor_msgs::msg::JointState get_isaac_joint_states()
    {
        // Isaac Sim Graph에서 데이터 수집
        return sensor_msgs::msg::JointState();
    }

    void send_isaac_joint_commands(const std::vector<double> & commands)
    {
        // Isaac Sim Graph로 명령 전송
    }

    std::vector<std::string> joint_names_;
    std::vector<double> position_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(IsaacSimHardwareInterface, 
    hardware_interface::SystemInterface)
```

### 5.2 ros2_control URDF

```xml
<!-- isaac_robot.urdf -->
<robot name="franka_isaac">
    <ros2_control name="IsaacSimController" type="system">
        <hardware>
            <plugin>isaac_sim_hardware_interface/IsaacSimHardwareInterface</plugin>
        </hardware>
        
        <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <!-- ... more joints ... -->
        <joint name="joint7">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
</robot>
```

### 5.3 Controller Manager Launch

```bash
# ros2_control + Isaac Sim 연동
ros2 control load_controller \
  --controller-type joint_trajectory_controller \
  --controller-name franka_controller

ros2 control set_controller_state franka_controller active

# Joint command 발행
ros2 topic pub /franka_controller/joint_trajectory \
  trajectory_msgs/JointTrajectory \
  "{joint_names: ['joint1','joint2','joint3'],
    points: [{positions: [0.1, -0.5, 1.2], time_from_start: {sec: 2}}]}"
```

---

## 6. SROS2 Security

### 6.1 Security 설정

```bash
# ════════════════════════════════════════════════════════
# SROS2 보안 설정
# ════════════════════════════════════════════════════════

# 1. Keystore 생성
ros2 security create_keystore ~/sros2_keystore

# 2. 각 노드별 인증서 생성
ros2 security create_key ~/sros2_keystore /isaac/perception
ros2 security create_key ~/sros2_keystore /isaac/control
ros2 security create_key ~/sros2_keystore /isaac/navigation

# 3. Permission 설정 (permissions.xml)
# /isaac/permission/permissions.xml 작성
```

```xml
<!-- permissions.xml -->
<permissions>
  <grant name="isaac_perception">
    <subscribe>rcl_interfaces/msg/ParameterEvent</subscribe>
    <publish>sensor_msgs/msg/Image</publish>
    <service_call>rcl_interfaces/srv/SetParameters</service_call>
  </grant>
  
  <grant name="isaac_control">
    <subscribe>trajectory_msgs/msg/JointTrajectory</subscribe>
    <publish>sensor_msgs/msg/JointState</publish>
    <service_call>rcl_interfaces/srv/ListParameters</service_call>
  </grant>
  
  <grant name="isaac_navigation">
    <subscribe>geometry_msgs/msg/PoseStamped</subscribe>
    <publish>nav_msgs/msg/Odometry</publish>
    <service_call>nav_msgs/srv/GetPlan</service_call>
  </grant>
</permissions>
```

```bash
# 4. 보안 적용 실행
export ROS_SECURITY_KEYSTORE=~/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

ros2 run isaac_sim_ros2 isaac_perception_node \
  --ros-args --enclave /isaac/perception
```

---

## 7. Real Robot ↔ Isaac Sim Bridge

### 7.1 Real Robot 데이터 수집

```python
class RealRobotBridge(Node):
    """실제 로봇 → Isaac Sim 데이터 스트리밍"""
    
    def __init__(self):
        super().__init__('real_robot_bridge')
        
        # ROS2 Parameters (동적 재설정)
        self.declare_parameter('robot_type', 'franka')
        self.declare_parameter('update_rate', 100)
        self.declare_parameter('sim_side', True)
        
        # Dynamic reconfigure
        self.add_on_set_parameters_callback(
            self.parameters_callback)
        
        # Subscribers (Real Robot)
        self.real_joint_sub = self.create_subscription(
            JointState, '/real_robot/joint_states',
            self.on_real_joint, 10)
        
        # Publishers (→ Isaac Sim)
        self.sim_joint_pub = self.create_publisher(
            JointState, '/isaac/target_joint_states', 10)
        
        # TF Bridge
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Real Robot Bridge activated')
    
    def on_real_joint(self, msg):
        """실제 로봇 조인트 → Isaac Sim 전달"""
        # Rate limiting
        rate = self.get_parameter('update_rate').value
        if not hasattr(self, '_last_time'):
            self._last_time = self.get_clock().now()
        
        now = self.get_clock().now()
        if (now - self._last_time).nanoseconds < 1e9 / rate:
            return
        self._last_time = now
        
        # Forward to Isaac Sim
        self.sim_joint_pub.publish(msg)
    
    def parameters_callback(self, params):
        """동적 파라미터 변경"""
        for param in params:
            if param.name == 'update_rate':
                self.get_logger().info(
                    f'Update rate changed to {param.value}')
            elif param.name == 'sim_side':
                self.get_logger().info(
                    f'Sim side: {"enabled" if param.value else "disabled"}')
        return SetParametersResult(successful=True)
```

### 7.2 Digital Twin Sync

```python
class DigitalTwinSync(Node):
    """Real ↔ Sim 양방향 동기화"""
    
    SYNC_INTERVAL = 0.01  # 100Hz
    
    def __init__(self):
        super().__init__('digital_twin_sync')
        
        # Real Robot → Sim (Forward)
        self.real_to_sim = self.create_subscription(
            JointState, '/real_robot/joint_states',
            self.forward_to_sim, 10)
        self.sim_joint_pub = self.create_publisher(
            JointState, '/isaac/cmd_joint_states', 10)
        
        # Sim → Real Robot (Feedback)
        self.sim_to_real = self.create_subscription(
            JointState, '/isaac/actual_joint_states',
            self.forward_to_real, 10)
        self.real_joint_pub = self.create_publisher(
            JointState, '/real_robot/target_joint_states', 10)
        
        # Sync timer
        self.timer = self.create_timer(
            self.SYNC_INTERVAL, self.sync_check)
        
        self.sim_time_offset = 0.0
        self.sync_count = 0
    
    def forward_to_sim(self, msg):
        """실제 로봇 → Sim"""
        msg.header.stamp = self.get_clock().now().to_msg()
        self.sim_joint_pub.publish(msg)
    
    def forward_to_real(self, msg):
        """Sim → 실제 로봇"""
        self.real_joint_pub.publish(msg)
    
    def sync_check(self):
        """동기화 상태 확인"""
        self.sync_count += 1
        if self.sync_count % 100 == 0:
            # Latency 체크
            self.get_logger().info(
                f'Digital Twin sync active @100Hz')
```

---

## 8. 실행 절차

### 8.1 Lifecycle + Composition

```bash
# ════════════════════════════════════════════════════════
# ROS2 Advanced — Terminal Setup
# ════════════════════════════════════════════════════════

# 터미널 1: Isaac Sim
cd ~/isaac-sim
./python.sh ~/isaac-step-curriculum/code/phase-3/step24_ros2_advanced.py

# 터미널 2: Component Container
ros2 component container /isaac_container
ros2 component load /isaac_container isaac_components \
  isaac_composition::IsaacSensorNode
ros2 component load /isaac_container isaac_components \
  isaac_composition::IsaacControlNode

# 터미널 3: Lifecycle Management
ros2 lifecycle set /isaac_lifecycle_node configure
ros2 lifecycle set /isaac_lifecycle_node activate

# 터미널 4: Action 테스트
ros2 action send_goal /pick_and_place isaac_msgs/PickAndPlace \
  "{object_id: 1, pick_pose: {position: {x: 0.5, y: 0.0, z: 0.3}}}"
```

### 8.2 Robot Control

```bash
# ros2_control + Isaac Sim
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active
ros2 control view

# Dynamic Parameters
ros2 param set /real_robot_bridge update_rate 50
```

---

## 9. 문제 해결

### 문제 1: Lifecycle 전이가 실패합니다.

**해결:**
```bash
# Transition 로그 확인
ros2 run rclcpp_lifecycle lifecycle_tester /isaac_lifecycle_node

# Node가 올바른 ID로 생성되었는지 확인
ros2 node info /isaac_lifecycle_node
```

### 문제 2: Component 로드 실패

**해결:**
```bash
# Library 경로 확인
ldconfig -p | grep isaac_components
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/components

# Component 타입 확인
ros2 component types
```

### 문제 3: Real Robot ↔ Sim 지연

**해결:**
```python
# QoS 설정 최적화
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
```

---

## 10. 정리

| 항목 | 내용 |
|------|------|
| ✅ Lifecycle Node | Configure/Activate/Deactivate/Cleanup |
| ✅ Node Composition | Single Process Multi-Node |
| ✅ ROS2 Actions | Goal/Feedback/Result |
| ✅ ros2_control | Hardware Interface for Isaac Sim |
| ✅ SROS2 | DDS Security |
| ✅ Real Robot Bridge | Digital Twin Sync |

---

## 11. 다음 Step 예고

**Step 25 — Large-Scale Simulation**에서는:
- Multi-Robot Fleet (10+ robots)
- Distributed Simulation
- ROS2 Multi-Host
- Performance Profiling
- Cloud Rendering
- Isaac Sim + Kubernetes

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| ROS2 Lifecycle | https://design.ros2.org/articles/node_lifecycle.html |
| Node Composition | https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html |
| ROS2 Actions | https://docs.ros.org/en/humble/Tutorials/Intermediate/Actions.html |
| ros2_control | https://control.ros.org/humble/ |
| SROS2 | https://docs.ros.org/en/humble/Tutorials/Advanced/Security.html |
