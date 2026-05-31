# Final Project 7 — 대규모 물류 Warehouse

> **예상 시간**: 16시간 | **난이도**: ★★★★★
> **관련 Step**: 21 (Warehouse), 25 (Large-Scale), 24 (ROS2 Advanced)
> **핵심 기술**: Fleet Management, WMS, Multi-Agent, Performance Optimization

---

## 프로젝트 개요

대규모 물류 Warehouse에서 20+ AMR Fleet, Conveyor System, WMS가 통합된 End-to-End 물류 시스템을 구축합니다.

## 목표

1. 20+ AMR Fleet 운용
2. Warehouse Management System (WMS)
3. Conveyor Belt + Sorting System
4. Order Fulfillment Pipeline
5. Dynamic Task Allocation
6. Fleet Performance Optimization
7. Monitoring Dashboard

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────┐
│                     Warehouse                        │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│   │ Rack A1-5│  │ Rack B1-5│  │ Rack C1-5│ ...     │
│   └────┬─────┘  └────┬─────┘  └────┬─────┘         │
│        │             │             │                 │
│   ┌────▼─────────────▼─────────────▼──────┐         │
│   │           Aisle Network               │         │
│   └────┬─────────────┬─────────────┬──────┘         │
│        │             │             │                 │
│   ┌────▼─────────────▼─────────────▼──────┐         │
│   │         Staging / Sorting              │         │
│   │  AMR 1-5   AMR 6-10   AMR 11-15 ...   │         │
│   └────────────────────────────────────────┘         │
│                                                      │
│   ┌────────────────────────────────────────┐         │
│   │   WMS: Order → Assign → Execute       │         │
│   └────────────────────────────────────────┘         │
└──────────────────────────────────────────────────────┘
```

## 구현 단계

### Step 1: Mega Warehouse Scene
- 20+ Racks (5 Aisles × 4 Rows)
- 5 Conveyor Belts
- Staging Area with 20 Docks
- Loading/Unloading Zones

### Step 2: AMR Fleet (20+)
- TurtleBot3 × 20
- Grid-based spawning
- Namespace /amr001-/amr020
- Battery management

### Step 3: WMS (Warehouse Management System)
- Order queue (100+ orders)
- Nearest-robot assignment
- Dynamic rerouting
- Priority handling

### Step 4: Conveyor Network
- 5 parallel conveyor lines
- Item sorting by destination
- Merge/diverge logic

### Step 5: Performance
- FPS optimization for 20+ robots
- GPU memory < 16GB
- DDS tuning for 20+ topics

## 성과 기준
- 20 AMR 동시 운용 FPS 20+
- Order 처리량 100+/hour
- 충돌 0건
- 평균 Task 완료 시간 < 60초
- 시스템 가동률 99%
