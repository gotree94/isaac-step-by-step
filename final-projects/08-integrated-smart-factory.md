# Final Project 8 — 통합 Smart Factory

> **예상 시간**: 24시간 | **난이도**: ★★★★★ (최종)
> **관련 Step**: 전체 (01-26)
> **핵심 기술**: 전 분야 통합, End-to-End System

---

## 프로젝트 개요

26개 Step의 모든 기술을 통합한 Smart Factory를 구축합니다. Digital Twin, Humanoid, AI Worker, AMR Fleet, Deep Learning, ROS2 Advanced가 하나의 시나리오로 동작합니다.

## 목표

1. **Digital Twin** — 공장 전체 디지털 트윈
2. **Humanoid Worker** — 휴머노이드 작업자
3. **AI Worker** — 협업 로봇
4. **AMR Fleet** — 자율 물류
5. **Deep Learning** — 지능형 제어
6. **ROS2** — 전체 통신 인프라
7. **Integrated Pipeline** — End-to-End

## 시스템 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│                    Smart Factory                                │
│                                                                 │
│  ┌────────────────────┐    ┌────────────────────┐            │
│  │  Production Area     │    │  Logistics Area     │            │
│  │                      │    │                      │            │
│  │  Humanoid Worker ──▶│    │  AMR Fleet ────────▶│            │
│  │  AI Cobot ─────────▶│    │  Conveyor Network ─▶│            │
│  │  Assembly Line ────▶│    │  Warehouse Racks ──▶│            │
│  └──────────┬──────────┘    └──────────┬──────────┘            │
│             │                         │                         │
│  ┌──────────▼─────────────────────────▼──────────┐            │
│  │              ROS2 Communication Bus              │            │
│  │  /factory/*  /humanoid/*  /cobot/*  /fleet/*    │            │
│  └──────────────────────────────────────────────────┘            │
│                                                                 │
│  ┌──────────────────────────────────────────────────────┐     │
│  │              Central Control System                    │     │
│  │  WMS  │  Production Planner  │  Quality  │  Monitor  │     │
│  └──────────────────────────────────────────────────────┘     │
│                                                                 │
│  ┌──────────────────────────────────────────────────────┐     │
│  │              Deep Learning Pipeline                    │     │
│  │  Perception  │  RL Control  │  Anomaly Detection     │     │
│  └──────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────────┘
```

## 구현 단계

### Phase 1: Infrastructure (4h)
- Factory Scene (30m × 20m)
- Production Line × 3
- Logistics Warehouse
- Digital Twin Setup
- ROS2 Backbone

### Phase 2: Robotics (6h)
- Humanoid Worker 배치
- AI Cobot (Franka + AMR)
- AMR Fleet (10+)
- Conveyor System

### Phase 3: Intelligence (6h)
- YOLO Object Detection
- PPO Robot Control
- Behavior Tree Planning
- Anomaly Detection

### Phase 4: Integration (8h)
- End-to-End Pipeline
- WMS + Production Sync
- Monitoring Dashboard
- What-If Simulation
- Performance Optimization

## 통합 시나리오

```
1. [WMS] Order 수신: "생산 100개"
2. [Logistics] AMR → Warehouse → Raw Materials 픽업
3. [Production] AMR → Production Line #1 전달
4. [Cobot] Franka가 Assembly 수행
5. [Humanoid] Humanoid가 Quality Inspection
6. [Logistics] 완제품 → Warehouse 적재
7. [Monitor] 실시간 생산 현황 Dashboard
8. [AI] Anomaly 감지 → 자동 대응
```

## 성과 기준 (KPI)

| KPI | 목표 | 측정 방법 |
|-----|------|----------|
| 생산량 | 50 unit/hour | WMS 카운터 |
| Defect Rate | < 1% | Quality Inspection |
| AMR 충돌 | 0건 | Fleet Log |
| System FPS | 15+ (전체) | Profiler |
| 통신 지연 | < 50ms | ROS2 Latency |
| AI 정확도 | 95%+ | Detection/Control |

## 최종 평가 기준

- [ ] 26 Step 전체 기술 통합
- [ ] 5개 이상 Robot Type 동시 운용
- [ ] End-to-End 생산 Pipeline
- [ ] 실시간 모니터링
- [ ] 30분 연속 안정 운용
- [ ] 복구 시나리오 (충돌/오류)
