# Step 09 — Sensor 기초

> **소요 시간**: 75분
> **난이도**: ★★★☆☆ (중급)
> **선수 조건**: Step 07 완료 (TurtleBot3 제어)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. Isaac Sim이 지원하는 **5가지 센서 종류**를 설명한다
2. **Camera**를 생성하고 RGB/Depth 이미지를 수집한다
3. **LiDAR**를 생성하고 Point Cloud 데이터를 수집한다
4. **Contact Sensor**로 충돌/접촉을 감지한다
5. **IMU**로 가속도/각속도를 측정한다
6. 센서 데이터를 **Python으로 저장**하고 활용한다
7. ROS2 Bridge로 센서 데이터를 **실시간 발행**한다

---

## 1. Isaac Sim의 센서 종류

### 1.1 지원 센서 개요

| 센서 | 측정 데이터 | ROS2 토픽 | 사용처 |
|------|------------|-----------|--------|
| **Camera** | RGB / Depth / Segmentation | `/camera/rgb`, `/camera/depth` | 시각 인식, SLAM |
| **LiDAR** | Point Cloud (XYZ + Intensity) | `/scan`, `/point_cloud` | 매핑, 장애물 회피 |
| **IMU** | 가속도 + 각속도 | `/imu` | 자세 추정 |
| **Contact Sensor** | 접촉 여부, 힘 | `/contact_sensor` | Grasp 감지, 충돌 |
| **RTX Sensor** | 고급 렌더링 센서 | - | 광선 추적 기반 (5.1 신규) |

### 1.2 센서 추가 방식

Isaac Sim에서 센서를 추가하는 3가지 방법:

| 방식 | 설명 | 난이도 |
|------|------|--------|
| **GUI** | Create > Sensors 메뉴, Content Browser | ★☆☆ |
| **OmniGraph** | ROS2 Bridge 노드 연결 | ★★☆ |
| **Python API** | `sensor` 모듈로 프로그래밍 방식 생성 | ★★★ |

---

## 2. Camera 센서

### 2.1 GUI로 Camera 추가

```
Create > Camera > Camera
또는
Create > Isaac > Sensors > Camera
```

Camera가 추가되면 Viewport에 Camera Prim이 나타납니다.
Camera Prim 선택 → Property Panel에서 해상도/FOV 등 설정:

| 속성 | 설명 | 권장값 |
|------|------|--------|
| `resolution` | 이미지 해상도 | `(640, 480)` |
| `focalLength` | 초점 거리 (mm) | `24.0` |
| `horizontalAperture` | 수평 조리개 (mm) | `20.955` |
| `clippingRange` | 렌더링 거리 범위 | `(0.1, 1000.0)` |
| `Stereo Role` | 좌/우/모노 | `Mono` |

### 2.2 Python으로 Camera RGB/Depth 데이터 수집

```python
import numpy as np
from omni.isaac.sensor import Camera

# Camera 생성
camera = Camera(
    prim_path="/World/Camera",
    name="MyCamera",
    position=np.array([2.0, 0.0, 1.5]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # Quaternion
    frequency=30,  # FPS
    resolution=(640, 480),
)

# 데이터 수집
camera.initialize()

for i in range(100):
    world.step(render=True)
    
    # RGB 이미지 가져오기
    rgb = camera.get_rgb()  # shape: (480, 640, 3), dtype: np.float32 (0~1)
    
    # Depth 이미지 가져오기
    depth = camera.get_depth()  # shape: (480, 640), dtype: np.float32
    
    # Point Cloud 가져오기
    pc = camera.get_pointcloud()  # shape: (480, 640, 3)
    
    print(f"Frame {i}: RGB shape={rgb.shape}, depth range=[{depth.min():.3f}, {depth.max():.3f}]")
```

### 2.3 Camera 파라미터

```python
camera = Camera(
    prim_path="/World/Camera",
    resolution=(1280, 720),      # 해상도
    frequency=30,                 # 업데이트 주파수 (Hz)
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    position=np.array([0.0, 0.0, 0.5]),
)

# Camera를 로봇에 부착 (Entity 기준)
camera.set_prim_path("/World/TurtleBot3/base_link/camera")
```

### 2.4 데이터 저장

```python
from PIL import Image

# RGB 저장
rgb_img = Image.fromarray((rgb * 255).astype(np.uint8))
rgb_img.save(f"frame_{i:04d}_rgb.png")

# Depth 저장 (16-bit PNG)
depth_mm = (depth * 1000).astype(np.uint16)
Image.fromarray(depth_mm).save(f"frame_{i:04d}_depth.png")

# 또는 NumPy 배열로 저장
np.save(f"frame_{i:04d}_rgb.npy", rgb)
np.save(f"frame_{i:04d}_depth.npy", depth)
```

---

## 3. LiDAR 센서

### 3.1 GUI로 LiDAR 추가

```
Create > Isaac > Sensors > Lidar > Lidar
```

**또는** Content Browser에서:
```
Isaac > Sensors > Lidar > lidar.usd
→ Viewport로 드래그
```

### 3.2 LiDAR 파라미터

| 속성 | 설명 | 권장값 |
|------|------|--------|
| `rotationRate` | 회전 속도 (Hz) | `10.0` |
| `horizontalResolution` | 수평 분해능 (포인트/스캔) | `360` |
| `horizontalRange` | 수평 스캔 범위 (deg) | `(0.0, 360.0)` |
| `verticalRange` | 수직 스캔 범위 (deg) | `(-30.0, 30.0)` |
| `verticalResolution` | 수직 분해능 (라인 수) | `16` |
| `maxRange` | 최대 측정 거리 (m) | `100.0` |
| `minRange` | 최소 측정 거리 (m) | `0.1` |

### 3.3 Python으로 LiDAR 데이터 수집

```python
from omni.isaac.sensor import Lidar

lidar = Lidar(
    prim_path="/World/TurtleBot3/base_link/lidar",
    name="MyLidar",
    rotation_rate=10.0,          # 10 Hz
    horizontal_resolution=360,   # 360 포인트/스캔
    horizontal_range=(0.0, 360.0),
    vertical_range=(-30.0, 30.0),
    vertical_resolution=16,      # 16 라인
    max_range=100.0,
    min_range=0.1,
)

lidar.initialize()

for i in range(200):
    world.step(render=True)
    
    # Point Cloud 데이터 (모든 포인트)
    point_cloud = lidar.get_pointcloud()  # (N, 3) xyz coordinates
    
    # Range 데이터 (거리만, 360도)
    range_data = lidar.get_range()  # (360,) 각도별 거리
    
    # Intensity 데이터
    intensity = lidar.get_intensity()  # (N,) 반사 강도
    
    if i % 20 == 0:
        print(f"Frame {i}: PointCloud={point_cloud.shape}, "
              f"Range min={range_data.min():.3f}m, max={range_data.max():.3f}m")
```

---

## 4. Contact Sensor

### 4.1 Contact Sensor란?

로봇과 환경/물체 간의 **접촉(충돌)을 감지**하는 센서입니다.
특히 그리퍼가 물체를 잡았는지 확인할 때 유용합니다.

### 4.2 Python으로 Contact Sensor 사용

```python
from omni.isaac.sensor import ContactSensor

# 그리퍼 손가락에 Contact Sensor 추가
contact_sensor = ContactSensor(
    prim_path="/World/Franka/panda_hand/panda_left_finger",
    name="GripperContact",
    min_threshold=0.0,   # 최소 접촉력
    max_threshold=1000.0, # 최대 접촉력
)

contact_sensor.initialize()

# 접촉 감지
in_contact = contact_sensor.get_current_value()  # Boolean
contact_force = contact_sensor.get_force()  # 접촉력 (N)

if in_contact:
    print(f"Gripper contacted! Force: {contact_force:.2f} N")
```

### 4.3 활용: Grasp 감지

```python
def is_grasping(contact_sensor, threshold=0.1):
    """그리퍼가 물체를 잡았는지 확인"""
    return contact_sensor.get_current_value() and contact_sensor.get_force() > threshold

# Pick & Place 중 Grasp 확인
close_gripper()
world.step(render=True)

if is_grasping(contact_sensor):
    print("✅ Object grasped successfully!")
else:
    print("❌ Grasp failed!")
```

---

## 5. IMU 센서

### 5.1 IMU 데이터

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/TurtleBot3/base_link",
    name="MyIMU",
    frequency=100,  # Hz
)

imu.initialize()

# 측정 데이터
linear_accel = imu.get_linear_acceleration()     # m/s²
angular_vel = imu.get_angular_velocity()          # rad/s
orientation = imu.get_orientation()               # Quaternion

print(f"Linear Accel: {linear_accel}")
print(f"Angular Vel: {angular_vel}")
```

---

## 6. 실습: TurtleBot3에 센서 부착 및 데이터 수집

### 6.1 시나리오

```
TurtleBot3에 Camera + LiDAR + IMU를 부착하고,
직진 주행하면서 모든 센서 데이터를 동시에 수집합니다.
```

### 6.2 구현

```python
import numpy as np
from omni.isaac.sensor import Camera, Lidar, IMUSensor

# 센서 생성
camera = Camera(
    prim_path="/World/TurtleBot3/base_link/camera",
    resolution=(640, 480),
    frequency=30,
    position=np.array([0.1, 0.0, 0.1]),  # 로봇 전방
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
)

lidar = Lidar(
    prim_path="/World/TurtleBot3/base_link/lidar",
    rotation_rate=10,
    horizontal_resolution=360,
    max_range=10.0,
)

imu = IMUSensor(
    prim_path="/World/TurtleBot3/base_link",
    frequency=100,
)

# 센서 초기화
camera.initialize()
lidar.initialize()
imu.initialize()

# 데이터 수집 루프
sensor_data = []

for i in range(300):
    world.step(render=True)
    
    # TurtleBot3 직진
    speeds = compute_wheel_speeds(linear_x=0.15, angular_z=0.0)
    controller.apply_action(ArticulationAction(joint_velocities=speeds))
    
    # 모든 센서 데이터 저장
    data = {
        'frame': i,
        'rgb': camera.get_rgb(),
        'depth': camera.get_depth(),
        'lidar': lidar.get_pointcloud(),
        'accel': imu.get_linear_acceleration(),
        'gyro': imu.get_angular_velocity(),
    }
    sensor_data.append(data)

# 수집된 데이터 분석 (예: 평균 LiDAR 거리)
avg_distances = []
for data in sensor_data:
    if data['lidar'].shape[0] > 0:
        avg_dist = np.mean(np.linalg.norm(data['lidar'], axis=1))
        avg_distances.append(avg_dist)

print(f"Average LiDAR distance: {np.mean(avg_distances):.3f}m")
```

---

## 7. ROS2 Bridge를 통한 센서 데이터 발행

### 7.1 ROS2 Bridge 활성화

Isaac Sim의 ROS2 Bridge를 사용하면 센서 데이터를 실시간 ROS2 토픽으로 발행할 수 있습니다.

**사전 준비**:
```bash
# ROS2 Humble 설치 (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# ROS2 Bridge 활성화
# Isaac Sim 실행 시 --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge

# 또는 Extension Manager에서 "ROS2 Bridge" 활성화
```

### 7.2 OmniGraph ROS2 Camera Publisher

**GUI 구성:**
```
Window > Graph Editors > Action Graph
→ New Action Graph

Nodes:
1. On Playback Tick
2. ROS2 Context
3. Isaac Create Render Product (Camera)
4. ROS2 Camera Helper

Connections:
On Playback Tick.tick → ROS2 Camera Helper.execIn
ROS2 Context.context → ROS2 Camera Helper.context
Isaac Create Render Product.renderProduct → ROS2 Camera Helper.execIn
```

**Python 구성:**
```python
import omni.graph.core as og

# ROS2 Camera Publisher Graph 생성
camera_graph_config = {
    "graph_path": "/ActionGraph/CameraPublisher",
    "evaluator_name": "execution",
}

og.Controller.edit(
    camera_graph_config,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("CreateCamera", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("ROS2Context.outputs:context", "CameraHelper.inputs:context"),
            ("CreateCamera.outputs:renderProduct", "CameraHelper.inputs:renderProduct"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("CreateCamera.inputs:cameraPrim", [Sdf.Path("/World/Camera")]),
            ("CreateCamera.inputs:width", 640),
            ("CreateCamera.inputs:height", 480),
        ],
    },
)
```

### 7.3 발행되는 ROS2 토픽

| 센서 | 토픽 이름 | 메시지 타입 |
|------|-----------|------------|
| Camera RGB | `/camera/rgb` | `sensor_msgs/Image` |
| Camera Depth | `/camera/depth` | `sensor_msgs/Image` |
| Camera Info | `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| LiDAR Scan | `/scan` | `sensor_msgs/LaserScan` |
| LiDAR PointCloud | `/point_cloud` | `sensor_msgs/PointCloud2` |
| IMU | `/imu` | `sensor_msgs/Imu` |

### 7.4 ROS2 subscriber에서 데이터 확인

```bash
# 다른 터미널에서
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic echo /camera/rgb --once
ros2 topic echo /scan --once
ros2 topic hz /camera/rgb
```

---

## 8. RTX Sensor (5.1 신규 기능)

### 8.1 RTX Sensor란?

Isaac Sim 5.1에서 도입된 **고급 렌더링 기반 센서**입니다.
기존 Camera/LiDAR보다 더 현실적인 물리 기반 센서 데이터를 생성합니다.

| 기능 | 기존 Camera | RTX Sensor |
|------|-------------|------------|
| 렌더링 방식 | Rasterization | Ray Tracing |
| 조명 효과 | 제한적 | 완전 물리 기반 |
| Semantic Segmentation | 제한적 | Object ID 자동 지원 |
| 노이즈 모델 | 없음 | 실제 센서 노이즈 모델링 |

### 8.2 RTX Sensor 사용

```python
from omni.isaac.sensor import RTXLidar

rtx_lidar = RTXLidar(
    prim_path="/World/TurtleBot3/base_link/rtx_lidar",
    name="RTXLidar",
    rotation_rate=10.0,
    horizontal_resolution=360,
    max_range=100.0,
)
```

---

## 9. 문제 해결 (Troubleshooting)

### 문제 1: Camera.get_rgb()가 None을 반환합니다.

**원인**: Camera가 아직 초기화되지 않음
**해결**: `camera.initialize()`를 호출했는지 확인.
또는 첫 몇 프레임은 데이터가 없을 수 있음 → 5프레임 정도 skip

### 문제 2: LiDAR 데이터가 너무 적습니다.

**원인**: `horizontalResolution`이 너무 낮거나 `rotationRate`가 너무 높음
**해결**: `horizontalResolution=360`, `rotationRate=10`으로 설정

### 문제 3: ROS2 Bridge가 연결되지 않습니다.

**원인**: ROS2 환경이 활성화되지 않음
**해결**:
```bash
# Isaac Sim 실행 전에 ROS2 환경 활성화
source /opt/ros/humble/setup.bash
isaacsim
```

### 문제 4: Contact Sensor가 접촉을 감지하지 못합니다.

**원인**: 
- 센서 임계값이 너무 높음
- Collision API가 활성화되지 않음
**해결**: `min_threshold=0.0`으로 설정, Collision API 확인

### 문제 5: Depth 이미지가 모두 0입니다.

**원인**: Camera가 물체를 향하지 않음
**해결**: Camera의 위치와 방향 확인. 물체가 clipping range 내에 있는지 확인.

---

## 10. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ 5가지 센서 | Camera, LiDAR, IMU, Contact, RTX Sensor |
| ✅ Camera 데이터 | RGB, Depth, PointCloud 수집 및 저장 |
| ✅ LiDAR 데이터 | PointCloud, Range, Intensity 수집 |
| ✅ Contact Sensor | 접촉 감지로 Grasp 확인 |
| ✅ IMU | 가속도, 각속도, 자세 측정 |
| ✅ ROS2 Bridge | 센서 데이터를 ROS2 토픽으로 발행 |
| ✅ 실습 | TurtleBot3 주행 + 센서 데이터 동시 수집 |

### 센서 데이터 흐름

```
Robot Sensors
    │
    ├── Camera     → RGB / Depth / Segmentation
    ├── LiDAR      → PointCloud / Range / Intensity
    ├── IMU        → Acceleration / Angular Velocity
    ├── Contact    → Is Contact / Force
    │
    ├── Python API → numpy 배열 (분석, 학습)
    └── ROS2 Bridge → ROS2 토픽 (실시간 시스템)
```

---

## 11. 다음 Step 예고

**Step 10 — Multiple Robots & Coordination**에서는:
- 여러 로봇 동시 제어
- Robot 간 통신 (간단한 Collision Avoidance)
- Scene에 로봇 Fleet 배치
- 경로 계획과 충돌 회피 기본

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Sensor API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.sensor.html |
| Camera API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.sensor.html#omni-isaac-sensor-camera |
| LiDAR API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.sensor.html#omni-isaac-sensor-lidar |
| Contact Sensor | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api/omni.isaac.sensor.html#omni-isaac-sensor-contact-sensor |
| ROS2 Bridge Tutorial | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html |
| RTX Sensor | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/rtx_sensor.html |
