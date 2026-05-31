# Final Project 5 — Deep Reinforcement Learning

> **예상 시간**: 16시간 | **난이도**: ★★★★★
> **관련 Step**: 23 (Deep Learning), 17 (Synthetic Data)
> **핵심 기술**: PPO, Isaac Gym, Imitation Learning, Domain Randomization

---

## 프로젝트 개요

Isaac Gym에서 Franka Panda 로봇이 PPO/Imitation Learning으로 복잡한 태스크를 학습합니다.

## 목표

1. Isaac Gym RL 환경 구축
2. PPO Training from Scratch
3. Imitation Learning (BC/GAIL)
4. Domain Randomization
5. Sim-to-Real Transfer
6. Policy Deployment

## 학습 Task

### Task 1: Reach
- End-effector → Target
- Dense reward (distance)

### Task 2: Push
- Object → Goal position
- Sparse reward

### Task 3: Peg-in-Hole
- Precision insertion
- Curriculum learning

## 구현 단계

### Step 1: Gym Environment
- 256 parallel envs
- Franka Panda articulation
- Observation/Action space

### Step 2: PPO Agent
- Actor-Critic network
- GAE, Clipped surrogate
- Hyperparameter tuning

### Step 3: Domain Randomization
- Friction, mass, lighting
- Camera noise
- Joint stiffness

### Step 4: Deployment
- ONNX/TensorRT export
- Isaac Sim inference
- ROS2 integration

## 성과 기준
- Reach: 95% success
- Push: 80% success
- Peg-in-Hole: 70% success
- Sim-to-Real gap < 10%
