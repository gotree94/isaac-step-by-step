# Step 17 — Synthetic Data Generation

> **소요 시간**: 90분
> **난이도**: ★★★★☆ (고급)
> **선수 조건**: Step 09 완료 (Sensors), Python/NumPy 기초

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Isaac Sim Replicator**로 합성 데이터를 생성한다
2. **Bounding Box 2D/3D**, **Segmentation**, **Depth**, **Pose** 데이터를 추출한다
3. **데이터셋 포맷**(COCO, KITTI, NuScenes)으로 내보낸다
4. **도메인 랜덤화**(자세, 조명, 텍스처, 카메라)를 적용한다
5. ROS2 Bridge를 통해 **실시간 데이터 스트리밍**을 구현한다
6. 생성된 데이터로 **학습/검증 파이프라인**을 구축한다

---

## 1. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────┐
│                   Isaac Sim                         │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ 조명     │  │ 물체     │  │ 카메라           │  │
│  │ Randomize│  │ Pose/Rot │  │ (RGB/Depth/Seg)  │  │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
│       │             │                 │             │
│       ▼             ▼                 ▼             │
│  ┌──────────────────────────────────────────────┐   │
│  │           Replicator Writer                  │   │
│  │  (OmniGraph + Python Script)                 │   │
│  └──────────────────────┬───────────────────────┘   │
│                         │                           │
└─────────────────────────┼───────────────────────────┘
                          │
                          ▼
              ┌──────────────────────┐
              │   Output Dataset     │
              │                      │
              │  /datasets/train/    │
              │  ├── RGB images     │
              │  ├── Depth maps     │
              │  ├── Segmentation   │
              │  ├── BBox (COCO)   │
              │  └── Metadata       │
              └──────────────────────┘
```

### 1.1 생성 가능 데이터 타입

| 데이터 타입 | 설명 | 활용 |
|------------|------|------|
| **RGB** | 컬러 이미지 (1920x1080) | 객체 탐지, 분류 |
| **Depth** | 깊이 맵 (16-bit PNG) | 3D Reconstruction |
| **Segmentation** | Semantic/Instance | Semantic Segmentation |
| **Bounding Box 2D** | 이미지 좌표 (COCO) | 2D 객체 탐지 |
| **Bounding Box 3D** | 월드 좌표 (KITTI) | 3D 객체 탐지 |
| **Normal** | 표면 법선 맵 | Surface Normal |
| **Optical Flow** | 광학 흐름 | Motion Estimation |
| **Lidar Point Cloud** | 3D 포인트 (.bin) | LiDAR Perception |

### 1.2 Replicator 아키텍처

Isaac Sim Replicator는 다음 구성 요소로 작동합니다:

```
Writer (OMNI_REPLICATOR_WRITER)
  ├── BasicWriter: 기본 이미지 + bbox
  ├── CocoWriter: COCO JSON format
  ├── KittiWriter: KITTI format  
  ├── NuSceneWriter: NuScenes format
  ├── CustomWriter: 사용자 정의 format
  └── RosBridgeWriter: ROS2 토픽 발행

Randomizer
  ├── CameraRandomizer: 위치, 회전, FOV
  ├── LightRandomizer: 색상, 강도, 방향
  ├── ColorRandomizer: 물체 색상
  ├── PoseRandomizer: 물체 자세
  └── TextureRandomizer: 텍스처
```

---

## 2. OmniGraph 기반 Synthetic Data 생성

### 2.1 카메라 설정

```python
import omni.replicator.core as rep
import omni.usd
from pxr import UsdGeom, Sdf, Gf

def setup_sd_camera(camera_path="/World/Camera_SD",
                    position=(2.0, 0.0, 1.5),
                    target=(0.0, 0.0, 0.0)):
    """Synthetic Data 수집용 카메라 설정"""
    
    stage = omni.usd.get_context().get_stage()
    
    # 카메라 생성
    camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
    
    # 카메라 속성
    camera_prim.GetFocalLengthAttr().Set(24.0)  # 24mm 광각
    camera_prim.GetFocusDistanceAttr().Set(400.0)
    camera_prim.GetFStopAttr().Set(0.0)
    camera_prim.GetClippingRangeAttr().Set((0.1, 1000.0))
    camera_prim.GetHorizontalApertureAttr().Set(20.955)
    camera_prim.GetVerticalApertureAttr().Set(15.29)
    
    # 카메라 위치 설정
    camera_xform = UsdGeom.Xformable(camera_prim)
    translate_op = camera_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*position))
    
    # 타겟 바라보기
    look_at_matrix = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*position),
        Gf.Vec3d(*target),
        Gf.Vec3d(0.0, 0.0, 1.0),
    )
    rotate_op = camera_xform.AddOrientOp()
    rotate_op.Set(Gf.Quatd(look_at_matrix.ExtractRotationQuat()))
    
    print(f"  + Camera created at {position}")
    return camera_path
```

### 2.2 Replicator Writer 생성

```python
import omni.replicator.core as rep

def setup_replicator_writer(
    output_dir="/datasets/training",
    camera_path="/World/Camera_SD",
    resolution=(1920, 1080),
    num_frames=100,
):
    """Replicator Writer 설정"""
    
    # Replicator 초기화
    rep.orchestrator.set_capture_on_play(True)
    
    # 카메라 등록
    render_product = rep.create.render_product(
        camera_path,
        resolution=resolution,
    )
    
    # COCO Writer (Bounding Box 2D + Segmentation)
    writer_coco = rep.WriterRegistry.get("CocoWriter")
    writer_coco.initialize(
        output_dir=output_dir,
        rgb=True,
        bounding_box_2d=True,
        semantic_segmentation=True,
        instance_id_segmentation=True,
        distance_to_camera=True,
    )
    writer_coco.attach([render_product])
    
    # Basic Writer (Depth + Normal)
    writer_basic = rep.WriterRegistry.get("BasicWriter")
    writer_basic.initialize(
        output_dir=output_dir,
        rgb=True,
        depth_normals=True,
        include_bboxes=True,
    )
    writer_basic.attach([render_product])
    
    print(f"  + Replicator writers created:")
    print(f"    - Output: {output_dir}")
    print(f"    - Resolution: {resolution[0]}x{resolution[1]}")
    print(f"    - COCO + Basic writers attached")
    
    return render_product
```

### 2.3 Replicator 실행

```python
def run_synthetic_data_generation(num_frames=100):
    """Replicator를 통한 데이터 생성 실행"""
    
    print(f"\n  Generating {num_frames} frames of synthetic data...")
    
    with rep.trigger.on_frame(max_execs=num_frames):
        # 매 프레임마다 데이터 수집
        pass
    
    # 데이터 생성 시작
    rep.orchestrator.run()
    
    print(f"  + Data generation complete: {num_frames} frames")
```

---

## 3. 도메인 랜덤화 (Domain Randomization)

### 3.1 물체 자세 랜덤화

```python
import omni.replicator.core as rep
from pxr import Gf
import numpy as np
import random

class ObjectPoseRandomizer:
    """물체의 위치/회전을 랜덤화"""
    
    def __init__(self, prim_paths, x_range=(-0.5, 0.5), 
                 y_range=(-0.5, 0.5), z_range=(0.02, 0.2)):
        self.prim_paths = prim_paths
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
    
    def randomize(self):
        """모든 물체의 자세 랜덤화"""
        for path in self.prim_paths:
            x = random.uniform(*self.x_range)
            y = random.uniform(*self.y_range)
            z = random.uniform(*self.z_range)
            roll = random.uniform(-30, 30)  # degrees
            pitch = random.uniform(-30, 30)
            yaw = random.uniform(0, 360)
            
            self._set_pose(path, (x, y, z), (roll, pitch, yaw))
    
    def _set_pose(self, prim_path, position, rotation_deg):
        """USD prim에 자세 적용"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            # 위치
            xform.ClearXformOpOrder()
            translate = xform.AddTranslateOp()
            translate.Set(Gf.Vec3d(*position))
            
            # 회전
            r, p, y = np.radians(rotation_deg)
            quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)  # w, x, y, z
            orient = xform.AddOrientOp()
            orient.Set(Gf.Quatd(
                np.cos(y/2)*np.cos(p/2)*np.cos(r/2) + np.sin(y/2)*np.sin(p/2)*np.sin(r/2),
                np.cos(y/2)*np.cos(p/2)*np.sin(r/2) - np.sin(y/2)*np.sin(p/2)*np.cos(r/2),
                np.cos(y/2)*np.sin(p/2)*np.cos(r/2) + np.sin(y/2)*np.cos(p/2)*np.sin(r/2),
                np.sin(y/2)*np.cos(p/2)*np.cos(r/2) - np.cos(y/2)*np.sin(p/2)*np.sin(r/2),
            ))
```

### 3.2 Replicator Randomizer 사용

```python
import omni.replicator.core as rep

def setup_domain_randomization():
    """도메인 랜덤화 설정"""
    
    # 조명 랜덤화
    with rep.trigger.on_frame(max_execs=100):
        rep.randomizer.light(
            light_type="dome",
            intensity=(500, 2000),  # 조도 범위
            color=(0.8, 1.2),  # 색상 변동
            rotation=(0, 360),  # 회전
        )
    
    # 카메라 자세 랜덤화
    with rep.trigger.on_frame(max_execs=100):
        rep.randomizer.camera(
            pose=rep.distribution.uniform(
                [(0.5, -0.5, 1.0), (2.0, 0.5, 2.0)]
            ),
            look_at=(0.0, 0.0, 0.0),
        )
    
    # 물체 색상 랜덤화
    with rep.trigger.on_frame(max_execs=100):
        rep.randomizer.color(
            prims=rep.get.prims(path_pattern="/World/Objects/*"),
            colors=rep.distribution.uniform(
                [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
            ),
        )
    
    print("  + Domain randomization configured:")
    print("    - Light: dome, intensity(500-2000), rotation(0-360)")
    print("    - Camera: position randomized")
    print("    - Objects: color randomized")
```

### 3.3 Texture Randomization (Material Swap)

```python
def randomize_textures(prim_paths, texture_paths):
    """물체 텍스처 랜덤 교체"""
    
    for path in prim_paths:
        # 랜덤 텍스처 선택
        texture = random.choice(texture_paths)
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(path)
        
        if prim:
            # Material Input 설정
            mat = UsdShade.Material(prim)
            shader = UsdShade.Shader(mat.GetPrim())
            
            if shader:
                # Diffuse Texture 연결
                diffuse_input = shader.CreateInput(
                    "diffuse_texture", 
                    Sdf.ValueTypeNames.Asset
                )
                diffuse_input.Set(texture)
                
                print(f"  + Applied texture {texture} to {path}")
```

---

## 4. 데이터셋 포맷 변환

### 4.1 COCO JSON Format

```python
import json
import os
from datetime import datetime

class CocoDatasetBuilder:
    """생성된 데이터를 COCO JSON 형식으로 변환"""
    
    def __init__(self, output_dir, categories):
        self.output_dir = output_dir
        self.categories = categories
        self.images = []
        self.annotations = []
        self.annotation_id = 1
        
        os.makedirs(output_dir, exist_ok=True)
    
    def add_image(self, image_id, filename, width, height):
        """이미지 메타데이터 추가"""
        self.images.append({
            "id": image_id,
            "file_name": filename,
            "width": width,
            "height": height,
            "date_captured": datetime.now().isoformat(),
        })
    
    def add_annotation(self, image_id, category_id, bbox, segmentation=None):
        """Bounding Box 주석 추가"""
        # bbox: [x, y, width, height] (COCO format)
        annotation = {
            "id": self.annotation_id,
            "image_id": image_id,
            "category_id": category_id,
            "bbox": bbox,
            "area": bbox[2] * bbox[3],
            "iscrowd": 0,
            "ignore": 0,
        }
        
        if segmentation:
            annotation["segmentation"] = segmentation
        
        self.annotations.append(annotation)
        self.annotation_id += 1
    
    def save(self):
        """COCO JSON 저장"""
        coco_data = {
            "images": self.images,
            "annotations": self.annotations,
            "categories": self.categories,
        }
        
        json_path = os.path.join(self.output_dir, "annotations.json")
        with open(json_path, "w") as f:
            json.dump(coco_data, f, indent=2)
        
        print(f"  + COCO annotations saved: {json_path}")
        print(f"    - Images: {len(self.images)}")
        print(f"    - Annotations: {len(self.annotations)}")

# 사용 예
categories = [
    {"id": 1, "name": "TurtleBot3", "supercategory": "robot"},
    {"id": 2, "name": "Franka_Panda", "supercategory": "robot"},
    {"id": 3, "name": "Box", "supercategory": "object"},
]

builder = CocoDatasetBuilder("/datasets/train/annotations", categories)
builder.add_image(1, "frame_000001.png", 1920, 1080)
builder.add_annotation(1, 1, [500, 300, 200, 150])  # TurtleBot3
builder.save()
```

### 4.2 KITTI Format

```python
class KittiDatasetBuilder:
    """KITTI 형식 데이터셋 변환"""
    
    FORMAT = "{type} {truncated} {occluded} {alpha} {x1} {y1} {x2} {y2} {h} {w} {l} {x} {y} {z} {ry}"
    
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.labels_dir = os.path.join(output_dir, "label_2")
        self.images_dir = os.path.join(output_dir, "image_2")
        
        os.makedirs(self.labels_dir, exist_ok=True)
        os.makedirs(self.images_dir, exist_ok=True)
    
    def add_frame(self, frame_id, detections):
        """KITTI 라벨 파일 생성"""
        lines = []
        
        for det in detections:
            # 2D BBox: x1, y1, x2, y2 (픽셀 좌표)
            # 3D BBox: h, w, l, x, y, z, ry (카메라 좌표)
            
            line = self.FORMAT.format(
                type=det["type"],
                truncated=det.get("truncated", 0),
                occluded=det.get("occluded", 0),
                alpha=det.get("alpha", 0),
                x1=det["bbox_2d"][0],
                y1=det["bbox_2d"][1],
                x2=det["bbox_2d"][2],
                y2=det["bbox_2d"][3],
                h=det["dimensions"][0],
                w=det["dimensions"][1],
                l=det["dimensions"][2],
                x=det["location"][0],
                y=det["location"][1],
                z=det["location"][2],
                ry=det["rotation_y"],
            )
            lines.append(line)
        
        label_path = os.path.join(
            self.labels_dir, f"{frame_id:06d}.txt")
        with open(label_path, "w") as f:
            f.write("\n".join(lines))
        
        print(f"  + KITTI labels: {label_path} ({len(lines)} objects)")
```

---

## 5. ROS2 Bridge로 Synthetic Data 스트리밍

### 5.1 실시간 이미지 발행

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
import cv2
import numpy as np
import omni.replicator.core as rep

class SyntheticDataPublisher(Node):
    """Replicator 데이터를 ROS2 토픽으로 실시간 발행"""
    
    def __init__(self):
        super().__init__('synthetic_data_publisher')
        
        self.bridge = cv_bridge.CvBridge()
        
        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/synthetic/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/synthetic/depth', 10)
        self.seg_pub = self.create_publisher(Image, '/synthetic/segmentation', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/synthetic/camera_info', 10)
        
        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_frame)
        
        self.frame_count = 0
    
    def publish_frame(self):
        """현재 Replicator 프레임 발행"""
        self.frame_count += 1
        
        # Replicator에서 현재 데이터 가져오기
        data = rep.orchestrator.get_current_frame()
        
        if data is None:
            return
        
        # RGB 이미지
        if 'rgb' in data:
            rgb_msg = self.bridge.cv2_to_imgmsg(
                data['rgb'], encoding='rgb8')
            rgb_msg.header.frame_id = 'camera'
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(rgb_msg)
        
        # Depth 이미지
        if 'depth' in data:
            depth_normalized = (data['depth'] / data['depth'].max() * 65535).astype(np.uint16)
            depth_msg = self.bridge.cv2_to_imgmsg(
                depth_normalized, encoding='16UC1')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_pub.publish(depth_msg)
        
        # Segmentation
        if 'semantic_segmentation' in data:
            seg_msg = self.bridge.cv2_to_imgmsg(
                data['semantic_segmentation'].astype(np.uint8), 
                encoding='mono8')
            seg_msg.header.stamp = self.get_clock().now().to_msg()
            self.seg_pub.publish(seg_msg)
        
        # Camera Info
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera'
        camera_info.width = 1920
        camera_info.height = 1080
        camera_info.distortion_model = 'plumb_bob'
        self.camera_info_pub.publish(camera_info)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')
```

### 5.2 ROS2 토픽 수신

```bash
# Synthetic RGB 이미지 보기
source /opt/ros/humble/setup.bash
ros2 topic echo /synthetic/rgb  # 메타데이터
ros2 run image_view image_view image:=/synthetic/rgb

# Depth 이미지 보기
ros2 run image_view image_view image:=/synthetic/depth

# Segmentation 보기
ros2 run image_view image_view image:=/synthetic/segmentation
```

---

## 6. 전체 데이터 생성 파이프라인

### 6.1 Python 통합 스크립트

```python
import os
import sys
import json
import numpy as np
import omni.replicator.core as rep
from pxr import UsdGeom, Gf, Sdf
import omni.usd

class SyntheticDataPipeline:
    """완전한 Synthetic Data 생성 파이프라인"""
    
    def __init__(self, config):
        self.config = config
        self.output_dir = config.get('output_dir', '/datasets/training')
        self.num_frames = config.get('num_frames', 100)
        self.resolution = config.get('resolution', (1920, 1080))
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 카테고리 설정
        self.categories = [
            {"id": 1, "name": "TurtleBot3"},
            {"id": 2, "name": "Box"},
            {"id": 3, "name": "Cylinder"},
        ]
        
        # COCO Builder
        self.coco_builder = CocoDatasetBuilder(
            os.path.join(self.output_dir, 'annotations'),
            self.categories,
        )
    
    def setup_scene(self):
        """데이터 생성용 씬 설정"""
        
        print("[Setup] Creating scene for data generation...")
        
        # Ground Plane
        rep.create.plane(
            position=(0, 0, 0),
            scale=(5, 5, 1),
        )
        
        # 카메라
        self.camera = rep.create.camera(
            position=(1.5, 0.5, 1.0),
            look_at=(0, 0, 0),
            focal_length=24.0,
        )
        
        # 조명
        rep.create.light(
            light_type="dome",
            intensity=1000,
            color=(1.0, 1.0, 1.0),
        )
        
        # 물체들
        self.objects = []
        for i in range(5):
            obj = rep.create.cube(
                position=(random.uniform(-0.5, 0.5),
                         random.uniform(-0.5, 0.5),
                         0.05),
                scale=(0.1, 0.1, 0.1),
            )
            self.objects.append(obj)
        
        print(f"  + Scene ready: {len(self.objects)} objects")
    
    def setup_writer(self):
        """Writer + Randomizer 설정"""
        
        print("[Setup] Configuring writers and randomizers...")
        
        render_product = rep.create.render_product(
            self.camera, self.resolution)
        
        # COCO Writer
        writer = rep.WriterRegistry.get("CocoWriter")
        writer.initialize(
            output_dir=self.output_dir,
            rgb=True,
            bounding_box_2d=True,
            semantic_segmentation=True,
            distance_to_camera=True,
        )
        writer.attach([render_product])
        
        # Randomizers
        with rep.trigger.on_frame(max_execs=self.num_frames):
            rep.randomizer.light(
                light_type="dome",
                intensity=(800, 2000),
            )
        
        with rep.trigger.on_frame(max_execs=self.num_frames):
            for i, obj in enumerate(self.objects):
                rep.randomizer.pose(
                    prims=[obj],
                    position=rep.distribution.uniform(
                        (-0.5, -0.5, 0.02), (0.5, 0.5, 0.2)),
                    rotation=rep.distribution.uniform(
                        (0, 0, 0), (360, 360, 360)),
                )
        
        print("  + Writers and randomizers configured")
        return render_product
    
    def run(self):
        """전체 데이터 생성 실행"""
        
        print(f"\n[Run] Generating {self.num_frames} frames...")
        
        # Replicator 실행
        rep.orchestrator.run()
        
        # 데이터 후처리
        self.post_process()
        
        print(f"  + Generation complete!")
        print(f"  + Output: {self.output_dir}")
    
    def post_process(self):
        """생성된 데이터 검증 및 통계"""
        
        image_dir = os.path.join(self.output_dir, "rgb")
        if os.path.exists(image_dir):
            images = sorted(os.listdir(image_dir))
            print(f"\n[Post] Generated {len(images)} images")
            
            # 샘플 이미지 확인
            if images:
                sample = cv2.imread(os.path.join(image_dir, images[0]))
                if sample is not None:
                    print(f"  + Sample: {images[0]}, shape={sample.shape}")
    
    def generate_coco_annotations(self):
        """수동 COCO JSON 생성 (BBox 포함)"""
        
        # Replicator Writer가 이미 COCO JSON을 생성했으므로
        # 여기서는 검증/통계만 수행
        
        ann_path = os.path.join(self.output_dir, "coco_data", "annotations.json")
        if os.path.exists(ann_path):
            with open(ann_path, 'r') as f:
                data = json.load(f)
            
            print(f"\n[COCO] Annotation statistics:")
            print(f"  + Images: {len(data['images'])}")
            print(f"  + Annotations: {len(data['annotations'])}")
            print(f"  + Categories: {len(data['categories'])}")
        else:
            print(f"\n[COCO] No annotations found at {ann_path}")


# 실행
pipeline = SyntheticDataPipeline({
    'output_dir': '/datasets/training_01',
    'num_frames': 500,
    'resolution': (1280, 720),
})

pipeline.setup_scene()
pipeline.setup_writer()
pipeline.run()
pipeline.generate_coco_annotations()
```

---

## 7. 데이터 Augmentation

### 7.1 Offline Augmentation (생성 후 처리)

```python
import cv2
import numpy as np
import imgaug.augmenters as iaa
from tqdm import tqdm

class DataAugmenter:
    """생성된 합성 데이터 증강"""
    
    def __init__(self):
        self.seq = iaa.Sequential([
            iaa.Sometimes(0.5, iaa.GaussianBlur(sigma=(0, 1.0))),
            iaa.Sometimes(0.3, iaa.AdditiveGaussianNoise(scale=(0, 0.05*255))),
            iaa.Sometimes(0.4, iaa.MultiplyBrightness((0.8, 1.2))),
            iaa.Sometimes(0.3, iaa.LinearContrast((0.8, 1.2))),
            iaa.Sometimes(0.2, iaa.JpegCompression(compression=(70, 99))),
        ])
    
    def augment_batch(self, images, bboxes=None):
        """배치 증강"""
        return self.seq(images=images, bounding_boxes=bboxes)
    
    def process_dataset(self, input_dir, output_dir):
        """전체 데이터셋 증강"""
        os.makedirs(output_dir, exist_ok=True)
        
        image_files = sorted(os.listdir(os.path.join(input_dir, "rgb")))
        
        for img_file in tqdm(image_files, desc="Augmenting"):
            img_path = os.path.join(input_dir, "rgb", img_file)
            img = cv2.imread(img_path)
            
            # 증강
            augmented = self.seq(image=img)
            
            # 저장
            out_path = os.path.join(output_dir, "rgb", f"aug_{img_file}")
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            cv2.imwrite(out_path, augmented)
        
        print(f"  + Augmented {len(image_files)} images to {output_dir}")
```

---

## 8. 실행 순서

### 8.1 Terminal Setup

```bash
# ════════════════════════════════════════════════════
# Synthetic Data Generation — 2 Terminal Setup
# ════════════════════════════════════════════════════

# 터미널 1: Isaac Sim (Synthetic Data Generator)
cd ~/isaac-sim
export ROS_DOMAIN_ID=0
./python.sh ~/isaac-step-curriculum/code/phase-2/step17_synthetic_data.py

# 터미널 2: 데이터 확인
source ~/isaac-step-curriculum/env_isaacsim/bin/activate

# 생성된 이미지 확인
ls -la /datasets/training/rgb/ | head -20
ls -la /datasets/training/annotations/

# 이미지 뷰어
eog /datasets/training/rgb/frame_000001.png &

# COCO Annotation 확인
python -c "
import json
with open('/datasets/training/annotations/annotations.json') as f:
    data = json.load(f)
print(f'Images: {len(data[\"images\"])}')
print(f'Annotations: {len(data[\"annotations\"])}')
print(f'Categories: {len(data[\"categories\"])}')
"

# ROS2 스트리밍 확인 (선택)
ros2 topic echo /synthetic/rgb --once | head
```

### 8.2 생성 확인 체크리스트

| 확인 항목 | 내용 |
|-----------|------|
| □ RGB 이미지 | 각 프레임의 컬러 이미지 |
| □ Depth Map | 16-bit PNG 깊이 정보 |
| □ Segmentation | Semantic/Instance 레이블 |
| □ BBox 2D | COCO JSON 형식 |
| □ 데이터 수 | 설정한 프레임 수와 일치 |
| □ 해상도 | 설정한 해상도와 일치 |
| □ 랜덤화 | 물체 자세/조명 다양성 |
| □ 카메라 | 다양한 각도에서 촬영 |

---

## 9. 문제 해결 (Troubleshooting)

### 문제 1: Replicator Writer가 데이터를 생성하지 않습니다.

**확인:**
```python
# Replicator 활성화 확인
rep.orchestrator.get_status()
print(rep.orchestrator.is_started())

# Render Product 확인
products = rep.orchestrator.get_render_products()
print(f"Render products: {products}")
```

**해결:** `rep.orchestrator.set_capture_on_play(True)` 설정 확인

### 문제 2: Segmentation이 올바르지 않습니다.

**해결:**
```python
# Semantic Label 설정
from pxr import UsdShade
for prim_path in object_prim_paths:
    rep.modify.semantic_label(
        prim_path=prim_path,
        label="TurtleBot3",
        id=1,
    )
```

### 문제 3: COCO Bounding Box가 비어 있습니다.

**해결:**
```python
# BBox Writer 설정 확인
writer.initialize(
    bounding_box_2d=True,  # 반드시 True
    rgb=True,
)
# 카메라가 모든 물체를 포함하는지 확인
```

### 문제 4: 데이터셋 크기가 너무 큽니다.

**해결:**
```python
# 해상도 감소
resolution=(640, 480)

# 프레임 수 감소
num_frames=50

# 압축 (JPEG quality)
writer.initialize(rgb_quality=85)  # 0-100
```

---

## 10. 정리

이 Step에서 배운 내용:

| 항목 | 내용 |
|------|------|
| ✅ Replicator | Isaac Sim 합성 데이터 생성 엔진 |
| ✅ Writer | COCO/KITTI/Basic Writer |
| ✅ Domain Randomization | 조명, 자세, 색상, 텍스처 |
| ✅ Data Format | COCO JSON, KITTI TXT |
| ✅ ROS2 Streaming | 실시간 RGB/Depth/Seg 발행 |
| ✅ Augmentation | imgaug 기반 후처리 증강 |

### 최종 파이프라인

```
Scene Setup → Writer Config → Randomizer → Replicator Run
                                                    │
                                                    ▼
                                          ┌─────────────────────┐
                                          │  /datasets/training │
                                          │                     │
                                          │  ├── rgb/          │
                                          │  ├── depth/        │
                                          │  ├── seg/          │
                                          │  └── annotations/  │
                                          └─────────────────────┘
                                                    │
                                                    ▼
                                          Training (YOLO/Detectron/...
```

---

## 11. 다음 Step 예고

**Step 18 — Performance Optimization**에서는:
- Isaac Sim 성능 모니터링 (FPS, 메모리, GPU)
- Stage 규모 최적화 (Instancing, LOD, Culling)
- Physics 설정 (Substeps, Solver Iterations)
- Rendering 품질 vs 속도 trade-off
- Hydra Render Delegate
- Multi-GPU 설정

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Sim Replicator | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator/ |
| Replicator Writer API | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator/advanced_writer.html |
| COCO Format | https://cocodataset.org/#format-data |
| KITTI Format | https://www.cvlibs.net/datasets/kitti/eval_object.php |
| Domain Randomization | https://arxiv.org/abs/1703.06907 |
| imgaug Library | https://github.com/aleju/imgaug |
