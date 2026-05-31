# Final Project 3 — 자율 주행 Fleet

> **예상 시간**: 12시간 | **난이도**: ★★★★★
> **관련 Step**: 25 (Large-Scale), 16 (Multi-Robot)
> **핵심 기술**: Fleet Management, Nav2, Collision Avoidance

---

## 프로젝트 개요

10대 이상의 TurtleBot3 Fleet이 창고에서 충돌 없이 자율 주행하며 물류를 처리합니다.

## 목표

1. 10+ Robot Fleet 생성
2. Distributed Navigation (Nav2)
3. Fleet Management System
4. Dynamic Collision Avoidance
5. Task Allocation (WMS)
6. Performance Optimization

## 구현 단계

### Step 1: Fleet Scene
- 10m x 10m Warehouse
- 10+ TurtleBot3 grid 배치
- ROS2 Namespace 분리

### Step 2: Fleet Management
- 각 로봇 상태 추적
- Task Queue + Assignment
- Battery Management

### Step 3: Multi-Robot Nav2
- Namespaced Nav2 인스턴스
- 충돌 회피 우선순위
- Traffic Rules

### Step 4: Performance
- FPS 측정 및 최적화
- GPU 메모리 관리
- DDS Tuning

## 성과 기준
- 10대 동시 주행 FPS 30+
- 충돌 0건
- Task 완료율 95%+
