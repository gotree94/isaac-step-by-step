# Final Project 2 — AI Worker Cobot Cell

> **예상 시간**: 8시간 | **난이도**: ★★★★
> **관련 Step**: 22 (AI Worker), 15 (MoveIt2)
> **핵심 기술**: Human-Robot Collaboration, Behavior Tree, MoveIt2

---

## 프로젝트 개요

AI Worker (TurtleBot3 + Franka)가 인간 작업자와 협업하여 Cobot Cell에서 Assembly 작업을 수행합니다.

## 목표

1. Cobot Cell 환경 구축
2. Franka Panda Manipulation
3. AI Worker Navigation
4. Behavior Tree Task Planning
5. Human-Robot Handover
6. Assembly Pipeline

## 구현 단계

### Step 1: Cobot Cell
- Workbench, Parts Feeder, Assembly Jig
- Safety Zones (Green/Yellow/Red)

### Step 2: Franka Manipulation
- Pick parts from feeder
- Precision assembly (peg-in-hole)
- MoveIt2 motion planning

### Step 3: AI Worker
- Mobile base navigation
- Transport parts between stations

### Step 4: Behavior Tree
- Sequence: Detect → Pick → Transport → Assemble
- Error recovery

## 성과 기준
- Assembly cycle < 60초
- 90% 조립 성공률
- 안전 거리 유지
