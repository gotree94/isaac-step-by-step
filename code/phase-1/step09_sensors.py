"""
Step 09 — Sensor 기초: Camera, LiDAR, IMU 데이터 수집
=======================================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step09_sensors.py

목표:
    1. TurtleBot3에 Camera + LiDAR + IMU 부착
    2. 직진 주행 중 모든 센서 데이터 동시 수집
    3. RGB / Depth / PointCloud 저장
    4. 센서 데이터 통계 분석
"""

import os
import numpy as np

CONFIG = {"width": 1280, "height": 720, "headless": False, "renderer": "RayTracedLighting"}
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 09 — Sensor Data Collection Demo")
print("=" * 60)

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.sensor import Camera, Lidar, IMUSensor

# ── World ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

# ── TurtleBot3 로딩 ──
ROBOT_PATH = "/World/TurtleBot3"
if not is_prim_path_valid(ROBOT_PATH):
    add_reference_to_stage("/Isaac/Robots/TurtleBot3/turtlebot3_waffle.usd", ROBOT_PATH)

robot = Robot(prim_path=ROBOT_PATH, name="TurtleBot3", position=np.array([0.0, 0.0, 0.1]))
world.scene.add(robot)
controller = robot.get_articulation_controller()

WHEEL_DIST = 0.141
WHEEL_RADIUS = 0.033

def wheel_speeds(linear_x=0.0, angular_z=0.0):
    v_l = linear_x - angular_z * WHEEL_DIST / 2
    v_r = linear_x + angular_z * WHEEL_DIST / 2
    return np.array([v_l / WHEEL_RADIUS, v_r / WHEEL_RADIUS])

# ── 센서 생성 ──
print("\n[1/4] Creating sensors...")

# Camera (로봇 전방)
camera = Camera(
    prim_path=f"{ROBOT_PATH}/base_link/camera",
    name="MyCamera",
    position=np.array([0.1, 0.0, 0.08]),
    frequency=30,
    resolution=(640, 480),
)
camera.initialize()
print("  + Camera: 640x480 RGB, 30 FPS")

# LiDAR
lidar = Lidar(
    prim_path=f"{ROBOT_PATH}/base_link/lidar",
    name="MyLidar",
    rotation_rate=10.0,
    horizontal_resolution=360,
    horizontal_range=(0.0, 360.0),
    vertical_range=(-15.0, 15.0),
    vertical_resolution=4,
    max_range=10.0,
    min_range=0.1,
)
lidar.initialize()
print("  + LiDAR: 360° x 4 lines, 10 Hz")

# IMU
imu = IMUSensor(
    prim_path=f"{ROBOT_PATH}/base_link",
    name="MyIMU",
    frequency=100,
)
imu.initialize()
print("  + IMU: 100 Hz")

# 저장 디렉토리
output_dir = os.path.expanduser("~/isaac-step-curriculum/assets/sensor_data")
os.makedirs(output_dir, exist_ok=True)
print(f"\n  Data will be saved to: {output_dir}")

# ── 시뮬레이션 + 센서 데이터 수집 ──
print("\n[2/4] Running simulation and collecting sensor data...")

FRAMES = 200
data_log = []

for i in range(FRAMES):
    world.step(render=True)

    # TurtleBot3 직진
    speeds = wheel_speeds(linear_x=0.15, angular_z=0.0)
    controller.apply_action(ArticulationAction(joint_velocities=speeds, joint_indices=[0, 1]))

    # 센서 데이터 읽기
    rgb = camera.get_rgb()
    depth = camera.get_depth()
    pc = lidar.get_pointcloud()
    accel = imu.get_linear_acceleration()
    gyro = imu.get_angular_velocity()

    if rgb is not None:
        data_log.append({
            "frame": i,
            "rgb_mean": rgb.mean(),
            "depth_mean": depth.mean() if depth is not None else 0,
            "lidar_pts": pc.shape[0] if pc is not None else 0,
            "accel_x": accel[0] if accel is not None else 0,
            "gyro_z": gyro[2] if gyro is not None else 0,
        })

    if i % 30 == 0:
        pc_count = pc.shape[0] if pc is not None else 0
        print(f"  Frame {i:3d}: RGB={rgb.shape if rgb is not None else 'N/A'}, "
              f"LiDAR={pc_count}pts, "
              f"Accel=({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f})")

# ── 3. 첫 번째 프레임 이미지 저장 ──
print("\n[3/4] Saving sample data from first frame...")

rgb = camera.get_rgb()
depth = camera.get_depth()
pc = lidar.get_pointcloud()

if rgb is not None:
    from PIL import Image
    img = Image.fromarray((rgb * 255).astype(np.uint8))
    img.save(os.path.join(output_dir, "sample_rgb.png"))
    print(f"  + Saved: sample_rgb.png ({rgb.shape})")

if depth is not None:
    depth_mm = (depth * 1000).astype(np.uint16)
    Image.fromarray(depth_mm).save(os.path.join(output_dir, "sample_depth.png"))
    print(f"  + Saved: sample_depth.png ({depth.shape})")

if pc is not None and pc.shape[0] > 0:
    np.save(os.path.join(output_dir, "sample_lidar.npy"), pc)
    print(f"  + Saved: sample_lidar.npy ({pc.shape})")

# ── 4. 분석 ──
print("\n[4/4] Sensor data analysis...")

if data_log:
    lidar_counts = [d["lidar_pts"] for d in data_log]
    accel_x_vals = [d["accel_x"] for d in data_log]

    print(f"  Frames collected: {len(data_log)}")
    print(f"  LiDAR points per frame: avg={np.mean(lidar_counts):.0f}, "
          f"min={min(lidar_counts)}, max={max(lidar_counts)}")
    print(f"  Accel X range: [{min(accel_x_vals):.3f}, {max(accel_x_vals):.3f}] m/s²")
    print(f"  Data dir: {output_dir}")

# ── 완료 ──
print(f"\n{'='*60}")
print("Step 09 Complete!")
print(f"{'='*60}")
print("Key concepts demonstrated:")
print("  - Camera: RGB + Depth image capture")
print("  - LiDAR: PointCloud data collection")
print("  - IMU: Acceleration + Angular velocity")
print("  - Data logging and analysis")
print("  - Multiple sensors running simultaneously")

simulation_app.close()
