# Final Project 4 — Digital Twin Factory

> **예상 시간**: 12시간 | **난이도**: ★★★★★
> **관련 Step**: 19 (Digital Twin), 10 (Scene Composition)
> **핵심 기술**: Digital Twin, ROS2 Bridge, Real↔Sim Sync

---

## 프로젝트 개요

실제 공장을 Isaac Sim에 Digital Twin으로 구축하고, 실시간 데이터 동기화를 구현합니다.

## 목표

1. 공장 환경 Digital Twin 구축
2. 실시간 데이터 동기화 (ROS2)
3. 생산 라인 시뮬레이션
4. Anomaly Detection
5. What-If Analysis
6. Dashboard 시각화

## 구현 단계

### Step 1: Factory Scene
- 생산 라인, 컨베이어, 로봇
- 실제 공장 레이아웃 기반
- PBR 텍스처 적용

### Step 2: ROS2 Sync
- Real ↔ Sim 양방향 통신
- Joint State, Odom, TF 동기화
- Latency 최소화

### Step 3: Production Line
- Conveyor Belt 제어
- Robot Cell 작업
- 품질 검사

### Step 4: Analysis
- 생산량, Cycle Time 분석
- Bottleneck 식별
- What-If 시나리오

## 성과 기준
- Sync 지연 < 10ms
- 생산 라인 100% 재현
- 3개 What-If 시나리오
