# Final Project 1 — 휴머노이드 창고 작업자

> **예상 시간**: 8시간 | **난이도**: ★★★★
> **관련 Step**: 20 (Humanoid), 21 (Warehouse)
> **핵심 기술**: Humanoid Control, Navigation, Manipulation

---

## 프로젝트 개요

GR1/H1 휴머노이드 로봇이 창고에서 실제 작업자처럼 물품을 집어 선반에 옮기는 시스템을 구현합니다.

## 목표

1. 휴머노이드 창고 환경 구축
2. Full-Body Task Space Control
3. 물체 인식 및 Grasping
4. Navigation to Rack
5. Pick-and-Place 작업 수행
6. 창고 작업 사이클 자동화

## 구현 단계

### Step 1: 창고 환경
- Rack, Conveyor, Item 배치
- Aisle과 Staging Area

### Step 2: 휴머노이드 제어
- T-Pose → Standing → Walking
- Arm Control for Reaching

### Step 3: Perception
- Camera → Object Detection
- Rack/Item 위치 인식

### Step 4: Task Pipeline
- Navigate → Pick → Transport → Place

## 성과 기준
- 5회 연속 성공적인 Pick-and-Place
- 사이클 타임 < 30초
- 충돌 없이 자율 주행
