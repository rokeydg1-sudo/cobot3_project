#!/usr/bin/env python3

# ============================================================
# Isaac Sim Standalone 실행
# ============================================================

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})


# ============================================================
# Isaac Sim API
# ============================================================

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid


# ============================================================
# World 생성
# ============================================================

world = World(
    stage_units_in_meters=1.0
)


# 기본 바닥 생성
world.scene.add_default_ground_plane()


# ============================================================
# Assembly Cell 설정
# ============================================================

CELL_SIZE = np.array([
    3.0,    # X 크기
    3.0,    # Y 크기
    0.02    # Z 높이
])


CELL_LOCATIONS = {

    "A": {
        "position": np.array([0.0, 0.0, 0.01]),
        "color": np.array([0.0, 0.0, 1.0])
    },

    "B": {
        "position": np.array([4.0, 0.0, 0.01]),
        "color": np.array([1.0, 1.0, 0.0])
    },

    "C": {
        "position": np.array([8.0, 0.0, 0.01]),
        "color": np.array([0.0, 1.0, 0.0])
    }
}


# ============================================================
# Assembly Cell Area 생성
# ============================================================

for cell_id, data in CELL_LOCATIONS.items():

    world.scene.add(
        VisualCuboid(

            # Isaac Sim 내부 Prim 경로
            prim_path=f"/World/AssemblyCell_{cell_id}",

            # 객체 이름
            name=f"assembly_cell_{cell_id}",

            # 위치
            position=data["position"],

            # 기본 Cube 크기
            size=1.0,

            # 실제 영역 크기
            scale=CELL_SIZE,

            # Cell별 색상
            color=data["color"]
        )
    )


# ============================================================
# World 초기화
# ============================================================

world.reset()


# ============================================================
# 실행 정보 출력
# ============================================================

print("\n========================================")
print(" Assembly Cell Test")
print("========================================")
print(" Assembly A : BLUE")
print(" Assembly B : YELLOW")
print(" Assembly C : GREEN")
print("========================================\n")


# ============================================================
# Simulation Loop
# ============================================================

try:

    while simulation_app.is_running():

        # Isaac Sim Simulation 진행
        world.step(render=True)


finally:

    # Isaac Sim 종료
    simulation_app.close()
