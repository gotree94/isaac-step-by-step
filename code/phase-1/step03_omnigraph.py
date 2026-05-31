"""
Step 03 — OmniGraph 기초: Python으로 Action Graph 생성하기
=========================================================

실행 방법 (Standalone):
    cd ~/isaac-sim  # or <ISAAC_ROOT>
    ./python.sh ~/isaac-step-curriculum/code/phase-1/step03_omnigraph.py

실행 방법 (GUI Extension):
    Script Editor에서 열고 Run.

목표:
    Step 03에서 GUI로 구성한 Action Graph를 Python API로 동일하게 생성합니다.
    아래 3가지 그래프를 순차적으로 생성합니다:
    
    Graph 1: OnPlaybackTick → WritePrimProperty (큐브 회전)
    Graph 2: OnImpulseEvent → WritePrimProperty (점프)
    Graph 3: KeyboardInput → Multiply → WritePrimProperty (키보드 이동)
"""

# ──────────────────────────────────────────────
# 1. 임포트
# ──────────────────────────────────────────────
import carb
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.objects import VisualCuboid

# OmniGraph Python API
import omni.graph.core as og
import omni.graph.tools as og_tools


# ──────────────────────────────────────────────
# 2. Stage 준비 (Cube + GroundPlane)
# ──────────────────────────────────────────────
print("[Step 03] ===== OmniGraph Python Scripting Demo =====")

create_new_stage()
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 제어할 큐브 생성
cube = VisualCuboid(
    prim_path="/World/Cube",
    name="MyCube",
    position=np.array([0.0, 0.0, 2.0]),
    size=0.5,
    color=np.array([0.0, 1.0, 0.0]),  # 초록색
)
world.scene.add(cube)

# 비교용 큐브 2 (키보드 제어용)
cube2 = VisualCuboid(
    prim_path="/World/Cube2",
    name="MyCube2",
    position=np.array([2.0, 0.0, 0.5]),
    size=0.5,
    color=np.array([1.0, 0.5, 0.0]),  # 주황색
)
world.scene.add(cube2)

print("[Step 03] Scene prepared: Cube (0,0,2), Cube2 (2,0,0.5).")

# ──────────────────────────────────────────────
# Helper: Action Graph 생성 함수
# ──────────────────────────────────────────────
def create_action_graph(graph_path: str, pipeline_stage: str = "PIPELINE_STAGE_EXECUTION"):
    """
    새 Action Graph를 생성하고 그래프 편집 구성을 반환합니다.
    
    Args:
        graph_path: Stage에서의 그래프 경로 (예: /ActionGraph/RotateCube)
        pipeline_stage: PIPELINE_STAGE_EXECUTION (기본) or PIPELINE_STAGE_ON_DEMAND
    
    Returns:
        og.Controller.edit()에 전달할 config dict
    """
    # 그래프 경로가 중복되지 않도록 확인
    if og.Controller.graph_exists(graph_path):
        print(f"  ⚠ Graph {graph_path} already exists. Skipping creation.")
        return None
    
    config = {
        "graph_path": graph_path,
        "evaluator_name": pipeline_stage,
    }
    print(f"  + Created Action Graph at {graph_path} (stage={pipeline_stage})")
    return config


# ──────────────────────────────────────────────
# Graph 1: OnPlaybackTick → WritePrimProperty (회전)
# ──────────────────────────────────────────────
print("\n[Step 03] Graph 1: Rotate Cube with OnPlaybackTick...")

graph1_config = create_action_graph("/ActionGraph/RotateCube")
if graph1_config:
    (
        graph1_handle,
        graph1_nodes,
        graph1_connections,
        graph1_attrs,
    ) = og.Controller.edit(
        graph1_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                # (variable_name, node_type)
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("WriteRotZ", "omni.isaac.core_nodes.IsaacWritePrimProperty"),
            ],
            og.Controller.Keys.CONNECT: [
                # (from_attribute, to_attribute)
                ("OnTick.outputs:tick", "WriteRotZ.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # WriteRotZ 설정
                ("WriteRotZ.inputs:targetPrim", Sdf.Path("/World/Cube")),
                ("WriteRotZ.inputs:propertyName", "xformOp:rotateZ"),
            ],
        },
    )
    print("  + Nodes: OnPlaybackTick → WritePrimProperty(rotateZ)")
    print("  + Cube will rotate continuously while simulation is playing.")
else:
    print("  ⚠ Skipped (already exists).")

# ──────────────────────────────────────────────
# Graph 2: OnImpulseEvent → WritePrimProperty (점프)
# ──────────────────────────────────────────────
print("\n[Step 03] Graph 2: Jump Cube with OnImpulseEvent...")

graph2_config = create_action_graph("/ActionGraph/JumpCube")
if graph2_config:
    (
        graph2_handle,
        graph2_nodes,
        graph2_connections,
        graph2_attrs,
    ) = og.Controller.edit(
        graph2_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                ("WritePosZ", "omni.isaac.core_nodes.IsaacWritePrimProperty"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulse.outputs:tick", "WritePosZ.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("WritePosZ.inputs:targetPrim", Sdf.Path("/World/Cube")),
                ("WritePosZ.inputs:propertyName", "xformOp:translateZ"),
            ],
        },
    )
    print("  + Nodes: OnImpulseEvent → WritePrimProperty(translateZ)")
    print("  + Each click of 'enableImpulse' moves Cube up by 0.5m (Z).")
else:
    print("  ⚠ Skipped (already exists).")


# ──────────────────────────────────────────────
# Graph 3: KeyboardInput → Multiply → WritePrimProperty (이동)
# ──────────────────────────────────────────────
print("\n[Step 03] Graph 3: Keyboard-controlled movement...")

graph3_config = create_action_graph("/ActionGraph/KeyboardMove")
if graph3_config:
    (
        graph3_handle,
        graph3_nodes,
        graph3_connections,
        graph3_attrs,
    ) = og.Controller.edit(
        graph3_config,
        {
            og.Controller.Keys.CREATE_NODES: [
                # Event
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                # Keyboard X axis
                ("KeyboardX", "omni.graph.action.KeyboardInput"),
                # Keyboard Y axis
                ("KeyboardY", "omni.graph.action.KeyboardInput"),
                # Math (속도 스케일)
                ("SpeedX", "omni.graph.nodes.ConstantDouble"),
                ("SpeedY", "omni.graph.nodes.ConstantDouble"),
                ("MulX", "omni.graph.nodes.Multiply"),
                ("MulY", "omni.graph.nodes.Multiply"),
                # Actions
                ("WriteX", "omni.isaac.core_nodes.IsaacWritePrimProperty"),
                ("WriteY", "omni.isaac.core_nodes.IsaacWritePrimProperty"),
            ],
            og.Controller.Keys.CONNECT: [
                # Execution flow
                ("OnTick.outputs:tick", "WriteX.inputs:execIn"),
                ("OnTick.outputs:tick", "WriteY.inputs:execIn"),
                # X axis: Keyboard → Multiply → Write
                ("KeyboardX.outputs:axis", "MulX.inputs:a"),
                ("SpeedX.outputs:value", "MulX.inputs:b"),
                ("MulX.outputs:result", "WriteX.inputs:value"),
                # Y axis: Keyboard → Multiply → Write
                ("KeyboardY.outputs:axis", "MulY.inputs:a"),
                ("SpeedY.outputs:value", "MulY.inputs:b"),
                ("MulY.outputs:result", "WriteY.inputs:value"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Keyboard X: RightArrow / LeftArrow → X축 이동
                ("KeyboardX.inputs:key1", "RightArrow"),
                ("KeyboardX.inputs:key2", "LeftArrow"),
                ("KeyboardX.inputs:useKey1AsAxis", True),
                ("KeyboardX.inputs:useKey2AsAxis", True),
                ("KeyboardX.inputs:axisOutputMode", "Key1MinusKey2"),
                # Keyboard Y: UpArrow / DownArrow → Y축 이동
                ("KeyboardY.inputs:key1", "UpArrow"),
                ("KeyboardY.inputs:key2", "DownArrow"),
                ("KeyboardY.inputs:useKey1AsAxis", True),
                ("KeyboardY.inputs:useKey2AsAxis", True),
                ("KeyboardY.inputs:axisOutputMode", "Key1MinusKey2"),
                # Speed constants
                ("SpeedX.inputs:value", 10.0),
                ("SpeedY.inputs:value", 10.0),
                # Write target: Cube2
                ("WriteX.inputs:targetPrim", Sdf.Path("/World/Cube2")),
                ("WriteX.inputs:propertyName", "xformOp:translateX"),
                ("WriteY.inputs:targetPrim", Sdf.Path("/World/Cube2")),
                ("WriteY.inputs:propertyName", "xformOp:translateY"),
            ],
        },
    )
    print("  + Nodes: KeyboardAxis → Multiply → WritePrimProperty")
    print("  + Cube2 controlled by Arrow Keys (→← = X, ↑↓ = Y)")
else:
    print("  ⚠ Skipped (already exists).")


# ──────────────────────────────────────────────
# 3. 완료 메시지 및 그래프 목록 출력
# ──────────────────────────────────────────────
print("\n[Step 03] ===== All Action Graphs Created =====")
print()

# 등록된 그래프 목록 출력
all_graphs = og.Controller.graph_ready_to_tick_list()
print(f"Graphs registered ({len(all_graphs)}):")
for g in all_graphs:
    print(f"  - {g}")

print()
print("[Step 03] How to use:")
print("  1. Press Play (▶) in the viewport")
print("  2. Observe:")
print("     - Cube (green)   : rotates continuously (Graph 1)")
print("     - Cube2 (orange) : move with Arrow Keys (Graph 3)")
print("  3. Graph 2 (Jump):")
print("     - Open Window > Graph Editors > Action Graph")
print("     - Select /ActionGraph/JumpCube")
print("     - Click OnImpulseEvent → enableImpulse = True")
print()
print("[Step 03] Tips:")
print("  - Open 'Window > Graph Editors > Action Graph' to inspect graphs")
print("  - Each graph is under /ActionGraph/ sub-path in the Stage")
print("  - Modify node values in Property Panel while simulation runs")
print("  - Save as USD: File > Save As → step03_omnigraph.usd")
