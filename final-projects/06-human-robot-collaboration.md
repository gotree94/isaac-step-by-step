# Final Project 6 — Human-Robot Collaboration

> **예상 시간**: 10시간 | **난이도**: ★★★★★
> **관련 Step**: 22 (AI Worker), 24 (ROS2 Advanced)
> **핵심 기술**: Human Tracking, Safety, Behavior Tree, Shared Workspace

---

## 프로젝트 개요

인간 작업자와 로봇이 동일 공간에서 안전하게 협업하는 시스템을 구현합니다.

## 목표

1. Human Pose Estimation
2. Action Recognition
3. Shared Workspace Management
4. Safety System (ISO/TS 15066)
5. Collaborative Task Execution
6. Emergency Response

## 구현 단계

### Step 1: Human Detection
- RGB-D camera setup
- Human keypoint detection
- Action classification (reaching, walking, idle)

### Step 2: Shared Workspace
- 작업 영역 분할 (Human/Robot/Shared)
- Dynamic safety zones
- Speed/Seperation Monitoring

### Step 3: Collaborative Task
- Human picks → Robot receives
- Robot places → Human inspects
- Joint assembly

### Step 4: Safety
- Emergency stop zone
- Speed reduction zone
- Collision prediction

## 시나리오

```
1. Human: Rack에서 부품 픽업
2. Robot: Handover 위치로 이동
3. Human: Robot에게 부품 전달
4. Robot: Assembly Jig에 배치
5. Robot: Safe zone으로 복귀
6. Human: Assembly 완료 확인
```

## 성과 기준
- 협업 사이클 < 40초
- Safety 위반 0건
- Handover 성공 95%+
