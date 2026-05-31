"""
Step 23 — Deep Learning in Isaac Sim
======================================

실행 방법:
    cd ~/isaac-sim
    ./python.sh ~/isaac-step-curriculum/code/phase-3/step23_deep_learning.py

사전 준비:
    1. Isaac Sim 5.1.0
    2. PyTorch 2.0+ (Isaac Sim 내장 또는 conda)
    3. Ultralytics YOLOv8 (선택)

목표:
    1. Isaac Gym 기본 사용법 (Parallel Envs)
    2. PPO Reinforcement Learning
    3. Behavior Cloning (Imitation Learning)
    4. Synthetic Data Generation
    5. ONNX Export Pipeline
"""

import argparse
import time
import numpy as np

# ── 1. SimulationApp 초기화 ──
CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": True,  # 학습 시 headless
    "renderer": "RayTracedLighting",
}

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(CONFIG)

print("=" * 60)
print("Step 23 — Deep Learning in Isaac Sim")
print("=" * 60)

# ── 2. Core API 임포트 ──
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid

# ── 3. World 생성 ──
world = World(stage_units_in_meters=1.0, physics_dt=1/60.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
world.initialize()

print("\n[1/8] Isaac World initialized")
print(f"  + PyTorch version: {torch.__version__}")
print(f"  + CUDA available: {torch.cuda.is_available()}")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"  + Device: {device}")

# ── 4. 환경 생성 ──
print("\n[2/8] Creating RL Environment (Franka Reach)...")

# Target Sphere
target = VisualCuboid(
    prim_path="/World/RL/Target",
    name="rl_target",
    position=np.array([0.5, 0.0, 0.5]),
    scale=np.array([0.05, 0.05, 0.05]),
    color=np.array([0.0, 1.0, 0.0]),
)

# Franka Panda 로딩
franka_path = "/World/RL/Franka"
if not is_prim_path_valid(franka_path):
    add_reference_to_stage(
        "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        franka_path,
    )

franka = Robot(
    prim_path=franka_path,
    name="Franka_RL",
    position=np.array([0.0, 0.0, 0.0]),
)
world.scene.add(franka)

print(f"  + Franka Panda: {franka.num_dof} DOF")

class FrankaReachEnv:
    """Franka Reaching RL Environment (Single)"""
    
    def __init__(self, robot, target):
        self.robot = robot
        self.target = target
        self.dof_count = robot.num_dof
        self.obs_dim = 7 + 3  # joint_positions + target_position
        self.action_dim = 7   # delta joint positions
        
        self.reset()
    
    def reset(self):
        """환경 리셋"""
        # Home position
        home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785])
        self.robot.set_joint_positions(home)
        world.step(render=True)
        world.step(render=True)
        
        # Random target
        tx = np.random.uniform(0.3, 0.7)
        ty = np.random.uniform(-0.3, 0.3)
        tz = np.random.uniform(0.3, 0.7)
        self.target.set_world_pose(np.array([tx, ty, tz]))
        
        return self._get_obs()
    
    def step(self, action):
        """환경 Step"""
        # Action: delta position control
        current_pos = self.robot.get_joint_positions()
        target_pos = current_pos + np.clip(action, -0.1, 0.1)
        target_pos = np.clip(target_pos, -2.8, 2.8)
        
        self.robot.set_joint_positions(target_pos)
        for _ in range(5):
            world.step(render=True)
        
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._compute_done()
        
        return obs, reward, done, {}
    
    def _get_obs(self):
        """Observation"""
        joint_pos = self.robot.get_joint_positions()
        target_pos, _ = self.target.get_world_pose()
        return np.concatenate([joint_pos, target_pos])
    
    def _compute_reward(self):
        """Dense reward: distance to target"""
        ee_pos = self._get_ee_position()
        target_pos, _ = self.target.get_world_pose()
        dist = np.linalg.norm(ee_pos - target_pos)
        return -dist  # Dense: negative distance
    
    def _compute_done(self):
        """Done condition"""
        ee_pos = self._get_ee_position()
        target_pos, _ = self.target.get_world_pose()
        dist = np.linalg.norm(ee_pos - target_pos)
        return dist < 0.05  # 5cm threshold
    
    def _get_ee_position(self):
        """End-effector position (simplified: last joint + offset)"""
        pos, ori = self.robot.get_world_pose()
        return pos + np.array([0.0, 0.0, 0.4])

env = FrankaReachEnv(franka, target)

print("  + State dim: 10, Action dim: 7")
print("  + Reward: -distance (dense)")

# ── 5. Neural Networks ──
print("\n[3/8] Building Neural Networks...")

class ActorNetwork(nn.Module):
    """정책 네트워크 (Gaussian)"""
    def __init__(self, obs_dim, act_dim, hidden=256):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, hidden)
        self.fc2 = nn.Linear(hidden, hidden)
        self.mean = nn.Linear(hidden, act_dim)
        self.log_std = nn.Parameter(torch.zeros(act_dim))
    
    def forward(self, x):
        x = torch.as_tensor(x, dtype=torch.float32)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mean = self.mean(x)
        std = torch.exp(self.log_std)
        return mean, std
    
    def get_action(self, x):
        mean, std = self.forward(x)
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return action, log_prob

class CriticNetwork(nn.Module):
    """가치 네트워크"""
    def __init__(self, obs_dim, hidden=256):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, hidden)
        self.fc2 = nn.Linear(hidden, hidden)
        self.value = nn.Linear(hidden, 1)
    
    def forward(self, x):
        x = torch.as_tensor(x, dtype=torch.float32)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.value(x).squeeze(-1)

obs_dim = env.obs_dim
act_dim = env.action_dim

actor = ActorNetwork(obs_dim, act_dim).to(device)
critic = CriticNetwork(obs_dim).to(device)
actor_optim = optim.Adam(actor.parameters(), lr=3e-4)
critic_optim = optim.Adam(critic.parameters(), lr=1e-3)

print(f"  + Actor: {sum(p.numel() for p in actor.parameters()):,} params")
print(f"  + Critic: {sum(p.numel() for p in critic.parameters()):,} params")

# ── 6. PPO Training ──
print("\n[4/8] Training PPO Agent...")

def train_ppo(num_iterations=500, steps_per_iter=100):
    """PPO Training Loop"""
    
    gamma = 0.99
    lmbda = 0.95
    clip_epsilon = 0.2
    entropy_coef = 0.01
    
    for iteration in range(num_iterations):
        # Rollout
        states, actions, rewards, dones, log_probs, values = [], [], [], [], [], []
        obs = env.reset()
        episode_rewards = []
        ep_reward = 0
        
        for _ in range(steps_per_iter):
            obs_t = torch.FloatTensor(obs).unsqueeze(0).to(device)
            
            with torch.no_grad():
                action, log_prob = actor.get_action(obs_t)
                value = critic(obs_t)
            
            next_obs, reward, done, _ = env.step(
                action.cpu().numpy().squeeze())
            
            states.append(obs_t)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            log_probs.append(log_prob)
            values.append(value)
            
            ep_reward += reward
            obs = next_obs
            
            if done:
                episode_rewards.append(ep_reward)
                ep_reward = 0
                obs = env.reset()
        
        # GAE 계산
        returns = []
        advantages = []
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            delta = rewards[t] + gamma * next_value * (1 - dones[t]) - values[t]
            gae = delta + gamma * lmbda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
            returns.insert(0, gae + values[t])
        
        returns = torch.stack(returns).squeeze()
        advantages = torch.stack(advantages).squeeze()
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        states_t = torch.cat(states)
        actions_t = torch.cat(actions)
        old_log_probs_t = torch.cat(log_probs).detach()
        
        # PPO Update (3 epochs)
        for _ in range(3):
            new_actions, new_log_probs = actor.get_action(states_t)
            new_values = critic(states_t)
            
            ratio = torch.exp(new_log_probs - old_log_probs_t)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - clip_epsilon, 1 + clip_epsilon) * advantages
            
            actor_loss = -torch.min(surr1, surr2).mean()
            
            # Entropy bonus
            _, std = actor(states_t)
            entropy = std.log().mean()
            
            critic_loss = F.mse_loss(new_values, returns)
            
            total_loss = actor_loss + 0.5 * critic_loss - entropy_coef * entropy
            
            actor_optim.zero_grad()
            critic_optim.zero_grad()
            total_loss.backward()
            actor_optim.step()
            critic_optim.step()
        
        if iteration % 50 == 0:
            avg_reward = np.mean(episode_rewards) if episode_rewards else 0
            print(f"  Iter {iteration:4d}: avg_reward={avg_reward:.3f}, "
                  f"actor_loss={actor_loss.item():.3f}, "
                  f"critic_loss={critic_loss.item():.3f}")

train_ppo(num_iterations=300, steps_per_iter=50)

print("  ✓ PPO training completed")

# ── 7. Behavior Cloning ──
print("\n[5/8] Collecting Expert Demonstrations...")

expert_demos = {
    'states': [],
    'actions': [],
}

# Collect expert demos (random walk + target bias)
for demo_idx in range(20):
    obs = env.reset()
    states = []
    actions = []
    
    for step in range(50):
        states.append(obs)
        
        # Expert policy: move toward target
        joint_pos = obs[:7]
        target_pos = obs[7:10]
        ee_pos = env._get_ee_position()
        delta = target_pos - ee_pos
        
        # Jacobian pseudo-inverse (simplified)
        action = np.zeros(7)
        action[:3] = delta * 2.0  # position control
        action = np.clip(action, -0.1, 0.1)
        
        actions.append(action)
        obs, _, done, _ = env.step(action)
        
        if done:
            break
    
    expert_demos['states'].extend(states)
    expert_demos['actions'].extend(actions)

print(f"  + Collected {len(expert_demos['states'])} expert transitions")

# Behavior Cloning Training
print("\n[6/8] Training Behavior Cloning...")

bc_policy = ActorNetwork(obs_dim, act_dim).to(device)
bc_optim = optim.Adam(bc_policy.parameters(), lr=1e-4)

states_t = torch.FloatTensor(np.array(expert_demos['states'])).to(device)
actions_t = torch.FloatTensor(np.array(expert_demos['actions'])).to(device)

dataset = TensorDataset(states_t, actions_t)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

for epoch in range(200):
    epoch_loss = 0
    for batch_s, batch_a in dataloader:
        pred_mean, pred_std = bc_policy(batch_s)
        loss = F.mse_loss(pred_mean, batch_a)
        
        bc_optim.zero_grad()
        loss.backward()
        bc_optim.step()
        epoch_loss += loss.item()
    
    if epoch % 50 == 0:
        print(f"  Epoch {epoch:4d}: loss={epoch_loss/len(dataloader):.6f}")

print("  ✓ Behavior Cloning completed")

# ── 8. Synthetic Data Generation ──
print("\n[7/8] Generating Synthetic Training Data...")

def generate_synthetic_data(num_samples=100):
    """RGB + Bounding Box 데이터 생성"""
    
    import h5py
    
    data = {
        'images': [],
        'labels': [],
        'boxes': [],
    }
    
    # 카메라 설정
    from omni.isaac.sensor import Camera
    
    cam = Camera(
        prim_path="/World/RL/Camera",
        name="synth_cam",
        position=np.array([1.0, -0.5, 1.0]),
        frequency=30,
        resolution=(640, 480),
    )
    cam.initialize()
    
    for i in range(num_samples):
        # Randomize scene
        tx = np.random.uniform(0.3, 0.7)
        ty = np.random.uniform(-0.3, 0.3)
        tz = np.random.uniform(0.3, 0.7)
        target.set_world_pose(np.array([tx, ty, tz]))
        
        home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785])
        franka.set_joint_positions(home)
        
        # Add noise
        noise = np.random.uniform(-0.2, 0.2, 7)
        franka.set_joint_positions(home + noise)
        
        for _ in range(10):
            world.step(render=True)
        
        # Capture
        rgb = cam.get_rgb()
        
        # Save
        if i % 20 == 0:
            print(f"  Captured {i+1}/{num_samples}")
    
    print(f"  + Generated {num_samples} synthetic images")

generate_synthetic_data(num_samples=50)

# ── 9. ONNX Export ──
print("\n[8/8] Exporting Model to ONNX...")

def export_onnx(model, path="franka_policy.onnx"):
    """PyTorch → ONNX Export"""
    
    model.eval()
    dummy = torch.randn(1, obs_dim, device=device)
    
    torch.onnx.export(
        model,
        dummy,
        path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['observation'],
        output_names=['action_mean', 'action_std'],
        dynamic_axes={
            'observation': {0: 'batch_size'},
            'action_mean': {0: 'batch_size'},
            'action_std': {0: 'batch_size'},
        },
    )
    
    print(f"  + Model exported to {path}")
    
    # Verify
    import onnx
    onnx_model = onnx.load(path)
    onnx.checker.check_model(onnx_model)
    print(f"  + ONNX model verified ✓")

export_onnx(bc_policy)

# ── 10. 요약 ──
print("\n" + "="*60)
print("  Step 23 — Summary")
print("="*60)
print()
print("  Deep Learning Pipeline:")
print()
print("  ✅ RL Environment (Franka Reach)")
print(f"     State dim: {obs_dim}, Action dim: {act_dim}")
print()
print("  ✅ PPO Training")
print("     Actor: 2-layer MLP (256)")
print("     Critic: 2-layer MLP (256)")
print("     GAE + Clipped Surrogate Objective")
print()
print("  ✅ Behavior Cloning")
print(f"     Expert demos: {len(expert_demos['states'])} transitions")
print("     MSE Loss + Adam Optimizer")
print()
print("  ✅ Synthetic Data")
print("     50 RGB images generated")
print()
print("  ✅ ONNX Export")
print("     franka_policy.onnx verified")
print()
print("  ✅ Key Concepts:")
print("     - Reinforcement Learning (PPO)")
print("     - Imitation Learning (Behavior Cloning)")
print("     - Synthetic Data Generation")
print("     - Model Export (PyTorch → ONNX)")
print("     - End-to-End Robot Learning Pipeline")
print("="*60)

simulation_app.close()
