# Step 23 — Deep Learning in Isaac Sim

> **소요 시간**: 150분
> **난이도**: ★★★★★ (전문가)
> **선수 조건**: Step 17 (Synthetic Data), Step 22 (AI Worker)

---

## 학습 목표

이 Step을 완료하면 다음을 할 수 있습니다:

1. **Isaac Sim + PyTorch Integration**을 이해한다
2. **Reinforcement Learning (RL)** 을 Isaac Gym 환경에서 실행한다
3. **Imitation Learning**으로 Human Demonstration을 학습한다
4. **Domain Randomization**을 적용하여 Sim-to-Real 전이를 강화한다
5. **Object Detection Model**을 합성 데이터로 학습한다
6. **ONNX Export**로 학습된 모델을 배포한다
7. **End-to-End Robot Learning Pipeline**을 구축한다

---

## 1. Robot Learning Pipeline 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Robot Learning Pipeline                          │
│                                                                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐      │
│  │ Data     │───▶│ Training │───▶│ Export   │───▶│ Deploy   │      │
│  │ Pipeline │    │ Pipeline │    │ Pipeline │    │ Pipeline │      │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘      │
│       │               │               │               │             │
│       ▼               ▼               ▼               ▼             │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐      │
│  │ Synthetic│    │ PyTorch  │    │ ONNX     │    │ Isaac    │      │
│  │ Data Gen │    │ / TF     │    │ / TensorRT│   │ Sim ROS2 │      │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘      │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  Learning Paradigms                                            │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐ │   │
│  │  │ Reinforcement│  │ Imitation    │  │ Supervised         │ │   │
│  │  │ Learning     │  │ Learning     │  │ Learning           │ │   │
│  │  │ (RL)         │  │ (BC/GAIL)    │  │ (Detection/Seg)    │ │   │
│  │  └──────────────┘  └──────────────┘  └────────────────────┘ │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.1 학습 패러다임 비교

| 방법 | 데이터 | 적용 | Isaac Sim 지원 |
|------|--------|------|---------------|
| **RL (PPO/SAC)** | Reward Signal | Locomotion, Manipulation | Isaac Gym |
| **IL (BC/GAIL)** | Expert Demo | Task Imitation | Isaac Mimic |
| **SL (Detection)** | Labeled Images | Perception | Synthetic Data |
| **Domain Randomization** | Varied Params | Sim-to-Real | Omniverse Replicator |

---

## 2. Reinforcement Learning with Isaac Gym

### 2.1 Isaac Gym 환경 설정

```python
# ⚠ Isaac Gym은 Isaac Sim 5.1에 내장 (별도 설치 불필요)
# GPU 기반 RL 학습을 위한 환경

import isaacgym
from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch

# Gym Instance 생성
gym = gymapi.acquire_gym()

# 시뮬레이션 파라미터
sim_params = gymapi.SimParams()
sim_params.dt = 1/60.0
sim_params.substeps = 2
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

# GPU Pipeline
sim_params.use_gpu_pipeline = True
sim_params.physx.use_gpu = True
sim_params.physx.num_subscenes = 4

# Scene 생성
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# Ground Plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

print(f"  + Isaac Gym initialized (GPU: {sim_params.use_gpu_pipeline})")
```

### 2.2 Franka Panda RL 환경

```python
class FrankaReachEnv:
    """Franka Panda Reaching Task (RL 환경)"""
    
    def __init__(self, num_envs=256):
        self.num_envs = num_envs
        self.max_episode_length = 200
        
        # Gym setup
        self.gym = gymapi.acquire_gym()
        self.sim = self._create_sim()
        self.envs = []
        self.frankas = []
        self.goals = []
        
        # Create parallel environments
        self._create_envs()
        
        # Tensor storage
        self._prepare_tensors()
        
        print(f"  + FrankaReachEnv: {num_envs} parallel envs")
    
    def _create_sim(self):
        sim_params = gymapi.SimParams()
        sim_params.dt = 1/60.0
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.use_gpu_pipeline = True
        
        sim = self.gym.create_sim(
            0, 0, gymapi.SIM_PHYSX, sim_params)
        
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(sim, plane_params)
        
        return sim
    
    def _create_envs(self):
        spacing = 1.5
        num_per_row = int(np.sqrt(self.num_envs))
        
        asset_root = "/Isaac/Robots/Franka"
        asset_file = "franka.usd"
        
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.flip_visual_attachments = True
        
        franka_asset = self.gym.load_asset(
            self.sim, asset_root, asset_file, asset_options)
        
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0, 0, 0)
        pose.r = gymapi.Quat(0, 0, 0, 1)
        
        for i in range(self.num_envs):
            env = self.gym.create_env(
                self.sim,
                gymapi.Vec3(-spacing/2, 0.0, -spacing/2),
                gymapi.Vec3(spacing/2, spacing, spacing/2),
                num_per_row,
            )
            
            franka = self.gym.create_actor(
                env, franka_asset, pose, f"franka_{i}", i, 1)
            
            # DOF properties
            dof_props = self.gym.get_actor_dof_properties(
                env, franka)
            dof_props['driveMode'][:7] = gymapi.DOF_MODE_POS
            dof_props['stiffness'][:7] = 100.0
            dof_props['damping'][:7] = 10.0
            self.gym.set_actor_dof_properties(
                env, franka, dof_props)
            
            # Goal (random position)
            goal_pose = gymapi.Transform()
            goal_pose.p = gymapi.Vec3(
                np.random.uniform(0.2, 0.6),
                np.random.uniform(-0.3, 0.3),
                np.random.uniform(0.2, 0.6),
            )
            goal = self.gym.create_actor(
                env, self._create_goal_asset(),
                goal_pose, f"goal_{i}", 0, 0)
            
            self.envs.append(env)
            self.frankas.append(franka)
            self.goals.append(goal)
    
    def _create_goal_asset(self):
        options = gymapi.AssetOptions()
        options.fix_base_link = True
        return self.gym.create_sphere(self.sim, 0.03, options)
    
    def _prepare_tensors(self):
        # GPU tensors for RL
        self.rb_tensor = self.gym.acquire_rigid_body_state_tensor(
            self.sim)
        self.dof_tensor = self.gym.acquire_dof_state_tensor(
            self.sim)
        
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        
        self.rb_states = gymtorch.wrap_tensor(self.rb_tensor)
        self.dof_states = gymtorch.wrap_tensor(self.dof_tensor)
    
    def reset(self, env_ids=None):
        """환경 리셋"""
        if env_ids is None:
            env_ids = np.arange(self.num_envs)
        
        # Reset Franka to home
        num_dof = 7
        dof_pos = torch.zeros((len(env_ids), num_dof),
                              device='cuda')
        dof_pos[:, :] = torch.tensor(
            [0, -0.3, 0, -2.2, 0, 2.0, 0.785], device='cuda')
        
        # Reset goals to random positions
        goal_pos = torch.rand((len(env_ids), 3), device='cuda') * 0.4 + 0.2
        goal_pos[:, 0] *= 0.4
        goal_pos[:, 1] -= 0.2
        goal_pos[:, 2] += 0.2
        
        # Apply to gym
        self.gym.set_dof_position_target_tensor(
            self.sim, None)
        
        return self._get_obs()
    
    def step(self, actions):
        """환경 Step"""
        actions = actions.clamp(-1.0, 1.0)
        
        # Apply actions (delta position control)
        self.gym.set_dof_position_target_tensor(
            self.sim, actions)
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._compute_done()
        
        return obs, reward, done, {}
    
    def _get_obs(self):
        """관측값: end-effector position + goal position"""
        # Simplified: return tensor
        return torch.rand(self.num_envs, 6, device='cuda')
    
    def _compute_reward(self):
        """Reward: 거리 기반"""
        return torch.zeros(self.num_envs, device='cuda')
    
    def _compute_done(self):
        return torch.zeros(self.num_envs, dtype=torch.bool,
                          device='cuda')
```

### 2.3 PPO Training Loop

```python
class PPO:
    """Proximal Policy Optimization"""
    
    def __init__(self, env, actor_lr=3e-4, critic_lr=1e-3):
        self.env = env
        self.actor = ActorNetwork(env.num_obs, env.num_actions)
        self.critic = CriticNetwork(env.num_obs)
        self.actor_optim = torch.optim.Adam(
            self.actor.parameters(), lr=actor_lr)
        self.critic_optim = torch.optim.Adam(
            self.critic.parameters(), lr=critic_lr)
        
        self.clip_epsilon = 0.2
        self.entropy_coef = 0.01
        self.gamma = 0.99
        self.lmbda = 0.95
    
    def collect_rollout(self, steps=2048):
        """Trajectory 수집"""
        states = []
        actions = []
        rewards = []
        dones = []
        log_probs = []
        values = []
        
        obs = self.env.reset()
        
        for _ in range(steps):
            with torch.no_grad():
                action, log_prob = self.actor(obs)
                value = self.critic(obs)
            
            next_obs, reward, done, _ = self.env.step(action)
            
            states.append(obs)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            log_probs.append(log_prob)
            values.append(value)
            
            obs = next_obs
        
        return states, actions, rewards, dones, log_probs, values
    
    def update(self, states, actions, rewards, dones, log_probs, values):
        """PPO Update"""
        # GAE 계산
        returns = []
        advantages = []
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            delta = (rewards[t] + self.gamma * next_value * 
                    (1 - dones[t]) - values[t])
            gae = delta + self.gamma * self.lmbda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
            returns.insert(0, gae + values[t])
        
        returns = torch.stack(returns)
        advantages = torch.stack(advantages)
        advantages = (advantages - advantages.mean()) / (
            advantages.std() + 1e-8)
        
        # PPO Epochs
        for _ in range(10):
            _, new_log_probs = self.actor(states)
            new_values = self.critic(states)
            
            ratio = torch.exp(new_log_probs - log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.clip_epsilon,
                                1 + self.clip_epsilon) * advantages
            
            actor_loss = -torch.min(surr1, surr2).mean()
            critic_loss = (returns - new_values).pow(2).mean()
            entropy = -(torch.exp(new_log_probs) * new_log_probs).mean()
            
            loss = actor_loss + 0.5 * critic_loss - self.entropy_coef * entropy
            
            self.actor_optim.zero_grad()
            self.critic_optim.zero_grad()
            loss.backward()
            self.actor_optim.step()
            self.critic_optim.step()
    
    def train(self, num_iterations=1000):
        """Training Loop"""
        for iteration in range(num_iterations):
            states, actions, rewards, dones, log_probs, values = \
                self.collect_rollout()
            
            self.update(
                states, actions, rewards, dones, log_probs, values)
            
            if iteration % 50 == 0:
                avg_reward = sum(r.mean().item() for r in rewards) / len(rewards)
                print(f"  Iter {iteration:4d}: avg_reward={avg_reward:.3f}")
            
            if avg_reward > 0.95:
                print(f"  ✓ Converged at iteration {iteration}")
                break
```

---

## 3. Imitation Learning

### 3.1 Human Demonstration 수집

```python
class DemonstrationCollector:
    """인간 시연 데이터 수집"""
    
    def __init__(self, save_path="demonstrations.hdf5"):
        self.save_path = save_path
        self.demonstrations = []
        self.current_demo = {
            'states': [],
            'actions': [],
            'observations': [],
        }
    
    def start_recording(self):
        """시연 녹화 시작"""
        self.current_demo = {
            'states': [],
            'actions': [],
            'observations': [],
        }
        print("  Recording started...")
    
    def record_step(self, state, action, observation):
        """Step 기록"""
        self.current_demo['states'].append(state)
        self.current_demo['actions'].append(action)
        self.current_demo['observations'].append(observation)
    
    def stop_recording(self):
        """시연 종료 및 저장"""
        self.demonstrations.append(self.current_demo)
        print(f"  Demo saved: {len(self.current_demo['states'])} steps")
        
        self._save_to_disk()
    
    def _save_to_disk(self):
        """HDF5 형식으로 저장"""
        import h5py
        
        with h5py.File(self.save_path, 'a') as f:
            demo_group = f.create_group(
                f"demo_{len(self.demonstrations)-1}")
            demo_group.create_dataset(
                'states', data=np.array(self.current_demo['states']))
            demo_group.create_dataset(
                'actions', data=np.array(self.current_demo['actions']))
            demo_group.create_dataset(
                'observations', 
                data=np.array(self.current_demo['observations']))
        
        print(f"  Saved to {self.save_path}")
    
    def load_demonstrations(self, path=None):
        """저장된 시연 로드"""
        import h5py
        
        path = path or self.save_path
        self.demonstrations = []
        
        with h5py.File(path, 'r') as f:
            for demo_name in f.keys():
                demo = f[demo_name]
                self.demonstrations.append({
                    'states': demo['states'][:],
                    'actions': demo['actions'][:],
                    'observations': demo['observations'][:],
                })
        
        print(f"  Loaded {len(self.demonstrations)} demonstrations")
```

### 3.2 Behavior Cloning

```python
class BehaviorClone:
    """Behavior Cloning (Imitation Learning)"""
    
    def __init__(self, env):
        self.env = env
        self.policy = PolicyNetwork(
            env.num_obs, env.num_actions)
        self.optimizer = torch.optim.Adam(
            self.policy.parameters(), lr=1e-4)
        self.loss_fn = nn.MSELoss()
    
    def train(self, demonstrations, epochs=1000):
        """Behavior Cloning 학습"""
        all_states = []
        all_actions = []
        
        for demo in demonstrations:
            all_states.extend(demo['states'])
            all_actions.extend(demo['actions'])
        
        states = torch.FloatTensor(np.array(all_states))
        actions = torch.FloatTensor(np.array(all_actions))
        
        dataset = TensorDataset(states, actions)
        dataloader = DataLoader(
            dataset, batch_size=128, shuffle=True)
        
        for epoch in range(epochs):
            epoch_loss = 0
            batch_count = 0
            
            for batch_states, batch_actions in dataloader:
                pred_actions = self.policy(batch_states)
                loss = self.loss_fn(pred_actions, batch_actions)
                
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                
                epoch_loss += loss.item()
                batch_count += 1
            
            if epoch % 100 == 0:
                avg_loss = epoch_loss / batch_count
                print(f"  Epoch {epoch:4d}: loss={avg_loss:.6f}")
    
    def evaluate(self, num_episodes=10):
        """학습된 정책 평가"""
        success_count = 0
        
        for episode in range(num_episodes):
            obs = self.env.reset()
            done = False
            step_count = 0
            
            while not done and step_count < 200:
                with torch.no_grad():
                    action = self.policy(
                        torch.FloatTensor(obs).unsqueeze(0))
                obs, reward, done, _ = self.env.step(
                    action.squeeze(0).numpy())
                step_count += 1
            
            if reward > 0.8:
                success_count += 1
        
        print(f"  Evaluation: {success_count}/{num_episodes} success")
```

---

## 4. Domain Randomization

### 4.1 Randomization Parameters

```python
class DomainRandomizer:
    """도메인 랜덤화 (Sim-to-Real 전이 향상)"""
    
    def __init__(self, gym, sim):
        self.gym = gym
        self.sim = sim
        
        # Randomization ranges
        self.params = {
            'gravity': (9.7, 9.9),
            'friction': (0.3, 1.5),
            'mass_scale': (0.8, 1.2),
            'joint_stiffness': (80, 150),
            'joint_damping': (5, 20),
            'light_intensity': (0.5, 1.5),
            'camera_noise': (0.0, 0.05),
        }
    
    def randomize(self):
        """물리/시각 파라미터 랜덤화"""
        changes = {}
        
        # 물리 파라미터
        for param_name in ['friction', 'mass_scale']:
            value = np.random.uniform(*self.params[param_name])
            changes[param_name] = value
        
        # 관절 파라미터
        stiffness = np.random.uniform(
            *self.params['joint_stiffness'])
        damping = np.random.uniform(
            *self.params['joint_damping'])
        
        changes['stiffness'] = stiffness
        changes['damping'] = damping
        
        # 중력
        grav = np.random.uniform(*self.params['gravity'])
        changes['gravity'] = grav
        
        self._apply_changes(changes)
        
        return changes
    
    def _apply_changes(self, changes):
        """변경 사항 적용"""
        # Gravity
        if 'gravity' in changes:
            sim_params = self.gym.get_sim_params(self.sim)
            sim_params.gravity.z = -changes['gravity']
            self.gym.set_sim_params(self.sim, sim_params)
        
        # Actor properties
        actor_handles = self.gym.get_actor_handles(self.sim)
        for handle in actor_handles:
            if 'friction' in changes:
                props = self.gym.get_actor_rigid_shape_properties(
                    handle)
                for prop in props:
                    prop.friction = changes['friction']
                self.gym.set_actor_rigid_shape_properties(
                    handle, props)
```

### 4.2 Texture & Lighting Randomization (Visual)

```python
class VisualDomainRandomizer:
    """시각적 도메인 랜덤화 (Omniverse Replicator)"""
    
    def __init__(self):
        import omni.replicator.core as rep
        self.rep = rep
    
    def setup_randomization(self, camera_prim="/World/Camera"):
        """Replicator Randomization 설정"""
        
        with self.rep.new_layer():
            
            # Lighting randomization
            with self.rep.trigger.on_frame(num_frames=1):
                self.rep.modify.look_at(
                    camera_prim, "/World/Table")
                
                # Random lighting
                self.rep.randomizer.light_intensity(
                    min=300, max=1000)
                self.rep.randomizer.light_color(
                    min=(0.8, 0.8, 0.8),
                    max=(1.2, 1.2, 1.2))
                
                # Random texture on ground
                self.rep.randomizer.texture(
                    prims=["/World/Workspace/Floor"],
                    textures=[
                        "concrete", "wood", "metal", "tile"])
                
                # Random camera pose
                self.rep.randomizer.camera_pose(
                    camera_prim,
                    distance_min=1.5, distance_max=2.5,
                    yaw_min=-30, yaw_max=30,
                    pitch_min=-20, pitch_max=20,
                )
        
        print("  + Visual Domain Randomization configured")
    
    def randomize_material(self, prim_path):
        """개별 머티리얼 랜덤화"""
        import omni.replicator.core as rep
        
        with rep.new_layer():
            with rep.trigger.on_frame(num_frames=1):
                rep.randomizer.color(
                    prims=[prim_path],
                    colors=[
                        (0.8, 0.1, 0.1),
                        (0.1, 0.8, 0.1),
                        (0.1, 0.1, 0.8),
                        (0.8, 0.8, 0.1),
                    ])
```

---

## 5. Object Detection with Synthetic Data

### 5.1 Dataset 준비

```python
def prepare_detection_dataset():
    """합성 데이터로 Detection Dataset 준비"""
    
    from omni.isaac.synthetic_utils import SyntheticDataHelper
    
    helper = SyntheticDataHelper()
    
    # 데이터 수집 설정
    helper.setup_bounding_box_2d("/World/Camera")
    helper.setup_semantic_segmentation("/World/Camera")
    helper.setup_depth("/World/Camera")
    
    # 저장 경로
    import os
    os.makedirs("datasets/detection/train/images", exist_ok=True)
    os.makedirs("datasets/detection/train/labels", exist_ok=True)
    
    print("  + Detection dataset pipeline ready")
```

### 5.2 YOLO Training

```python
def train_yolo_on_synthetic():
    """YOLO 학습 (Ultralytics)"""
    
    from ultralytics import YOLO
    
    # YOLOv8 nano 모델 로드
    model = YOLO('yolov8n.pt')
    
    # 데이터셋 YAML
    dataset_yaml = """
    train: datasets/detection/train
    val: datasets/detection/val
    nc: 4
    names: ['box', 'human', 'robot', 'rack']
    """
    
    with open('dataset.yaml', 'w') as f:
        f.write(dataset_yaml)
    
    # Training
    results = model.train(
        data='dataset.yaml',
        epochs=100,
        imgsz=640,
        batch=16,
        augment=True,
        device='cuda',
    )
    
    # Export to ONNX
    model.export(format='onnx')
    
    print(f"  + Model exported to yolov8n.onnx")
    return model
```

---

## 6. Model Export & Deployment

### 6.1 ONNX Export

```python
def export_to_onnx(model, input_size=(1, 3, 640, 640)):
    """PyTorch → ONNX Export"""
    
    import torch.onnx
    
    model.eval()
    dummy_input = torch.randn(input_size)
    
    torch.onnx.export(
        model,
        dummy_input,
        "robot_model.onnx",
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'},
        },
    )
    
    print("  + Model exported to robot_model.onnx")
```

### 6.2 TensorRT Deployment

```python
def deploy_tensorrt(onnx_path="robot_model.onnx"):
    """TensorRT 최적화 및 배포"""
    
    import tensorrt as trt
    
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    
    # ONNX 로드
    with open(onnx_path, 'rb') as f:
        parser.parse(f.read())
    
    # Build engine
    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, 1 << 30)
    config.set_flag(trt.BuilderFlag.FP16)
    
    serialized_engine = builder.build_serialized_network(
        network, config)
    
    with open("robot_model.trt", 'wb') as f:
        f.write(serialized_engine)
    
    print("  + TensorRT engine built (FP16)")
```

### 6.3 Isaac Sim에서 추론

```python
class ModelInferenceNode(Node):
    """Isaac Sim → ROS2 → Model Inference"""
    
    def __init__(self, model_path="robot_model.trt"):
        super().__init__('model_inference')
        
        # TensorRT engine 로드
        import tensorrt as trt
        import pycuda.driver as cuda
        
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(model_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(
                f.read())
        self.context = self.engine.create_execution_context()
        
        # I/O buffers
        self.inputs = []
        self.outputs = []
        self.allocate_buffers()
        
        # ROS2 Sub/Pub
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb', self.on_image, 10)
        self.pred_pub = self.create_publisher(
            Detection2DArray, '/model/predictions', 10)
        
        self.get_logger().info('Model inference node ready')
    
    def allocate_buffers(self):
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.inputs.append(host_mem) if self.engine.binding_is_input(
                binding) else self.outputs.append(host_mem)
    
    def on_image(self, msg):
        """이미지 추론"""
        # Preprocess
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        input_tensor = self.preprocess(image)
        
        # Inference
        np.copyto(self.inputs[0], input_tensor.ravel())
        self.context.execute_v2(
            [int(i) for i in [self.inputs[0], self.outputs[0]]])
        
        # Postprocess
        detections = self.postprocess(
            self.outputs[0].reshape(1, 84, 8400))
        self.pred_pub.publish(detections)
```

---

## 7. 실행 절차

### 7.1 RL Training

```bash
# Terminal 1: RL Training
cd ~/isaac-sim
./python.sh ~/isaac-step-curriculum/code/phase-3/step23_deep_learning.py \
  --mode rl

# Watch training progress
tensorboard --logdir runs/
```

### 7.2 Imitation Learning

```bash
# Terminal 2: Data Collection
./python.sh ~/isaac-step-curriculum/code/phase-3/step23_deep_learning.py \
  --mode collect_demo

# Terminal 3: Train BC
./python.sh ~/isaac-step-curriculum/code/phase-3/step23_deep_learning.py \
  --mode train_bc
```

### 7.3 Inference

```bash
# Terminal 4: Deploy trained model
./python.sh ~/isaac-step-curriculum/code/phase-3/step23_deep_learning.py \
  --mode deploy

# Verify predictions
ros2 topic echo /model/predictions
```

---

## 8. 문제 해결

### 문제 1: Isaac Gym이 OOM (Out of Memory)

**해결:**
- num_envs 감소 (256 → 64)
- Batch size 감소
- `torch.cuda.empty_cache()` 호출

### 문제 2: PPO 학습이 수렴하지 않음

**해결:**
```python
# 학습률 조정
self.actor_optim = torch.optim.Adam(
    self.actor.parameters(), lr=1e-4)

# Reward shaping:
# Sparse → Dense reward로 변경
reward = -distance_to_goal  # Dense
```

### 문제 3: 시뮬레이션 속도 느림

**해결:**
- `use_gpu_pipeline = True`
- `num_subscenes = 4` (GPU 병렬화)
- viewer 비활성화 (headless)

---

## 9. 정리

| 항목 | 내용 |
|------|------|
| ✅ RL with Isaac Gym | PPO, Parallel Envs |
| ✅ Imitation Learning | Demonstration, BC |
| ✅ Domain Randomization | Physics + Visual |
| ✅ Detection Training | Synthetic Data + YOLO |
| ✅ Model Export | ONNX + TensorRT |
| ✅ ROS2 Inference | Deploy in Isaac Sim |

---

## 10. 다음 Step 예고

**Step 24 — ROS2 Advanced**에서는:
- ROS2 Humble 고급 기능
- Lifecycle Nodes
- Multi-Node Composition
- ROS2 Actions
- ros2_control for Isaac Sim
- Real Robot ↔ Isaac Sim 통신

---

## 참고 자료

| 자료 | 링크 |
|------|------|
| Isaac Gym | https://docs.isaacsim.omniverse.nvidia.com/5.1.0/isaac_gym/index.html |
| PPO Paper | https://arxiv.org/abs/1707.06347 |
| Behavior Cloning | https://arxiv.org/abs/2101.04412 |
| Domain Randomization | https://arxiv.org/abs/1703.06907 |
| ONNX | https://onnx.ai/ |
| TensorRT | https://developer.nvidia.com/tensorrt |
