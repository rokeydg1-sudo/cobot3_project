#!/usr/bin/env python3

# ============================================================
# Isaac Sim Standalone 실행
# ============================================================

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})


# ============================================================
# Python / Isaac Sim import
# ============================================================

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid

# 우리가 만든 Assembly 알고리즘
from assembly_algorithm import Task, AssemblyCell


# ============================================================
# World 생성
# ============================================================

world = World(
    stage_units_in_meters=1.0
)

# 기본 바닥 생성
world.scene.add_default_ground_plane()


# ============================================================
# Assembly Cell 영역 설정
# ============================================================

CELL_SIZE = np.array([
    3.0,     # X 크기
    3.0,     # Y 크기
    0.02     # Z 높이
])


CELL_LOCATIONS = {

    # Assembly A
    "A": {
        "position": np.array([0.0, 0.0, 0.01]),
        "color": np.array([0.0, 0.0, 1.0])
    },

    # Assembly B
    "B": {
        "position": np.array([4.0, 0.0, 0.01]),
        "color": np.array([1.0, 1.0, 0.0])
    },

    # Assembly C
    "C": {
        "position": np.array([8.0, 0.0, 0.01]),
        "color": np.array([0.0, 1.0, 0.0])
    }
}


# ============================================================
# Isaac Sim에 Assembly Cell 영역 생성
# ============================================================

for cell_id, data in CELL_LOCATIONS.items():

    world.scene.add(
        VisualCuboid(

            # Isaac Sim 내부 경로
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
# Assembly Cell 알고리즘 생성
# ============================================================

cell_a = AssemblyCell(
    cell_id="A"
)

cell_b = AssemblyCell(
    cell_id="B"
)

cell_c = AssemblyCell(
    cell_id="C"
)


# ============================================================
# Task 정의
# ============================================================

task_1 = Task(
    task_id=1,
    kit_id="KIT_A",
    processing_time=3
)

task_2 = Task(
    task_id=2,
    kit_id="KIT_B",
    processing_time=5
)

task_3 = Task(
    task_id=3,
    kit_id="KIT_C",
    processing_time=2
)


# ============================================================
# 초기 Task 등록
#
# 각 Assembly Cell에 Task를 3개씩 등록
# ============================================================

for _ in range(3):

    cell_a.add_task(task_1)

    cell_b.add_task(task_2)

    cell_c.add_task(task_3)


# ============================================================
# World 초기화
# ============================================================

world.reset()


# ============================================================
# Simulation Time
# ============================================================

simulation_time = 0.0

dt = 0.1


# ============================================================
# 실행 정보
# ============================================================

print("\n========================================")
print(" Assembly Cell Isaac Sim Test")
print("========================================")
print(" Assembly A : BLUE")
print(" Assembly B : YELLOW")
print(" Assembly C : GREEN")
print("----------------------------------------")
print(" Cell A : Task 1 / KIT_A / 3 sec")
print(" Cell B : Task 2 / KIT_B / 5 sec")
print(" Cell C : Task 3 / KIT_C / 2 sec")
print("----------------------------------------")
print(" Initial Queue Size : 3")
print("========================================\n")


# ============================================================
# Simulation Loop
# ============================================================

try:

    while simulation_app.is_running():

        # ----------------------------------------------------
        # Isaac Sim 한 Step 진행
        # ----------------------------------------------------

        world.step(render=True)


        # ----------------------------------------------------
        # Simulation Time 증가
        # ----------------------------------------------------

        simulation_time += dt


        # ----------------------------------------------------
        # Assembly Cell 업데이트
        # ----------------------------------------------------

        completed_a = cell_a.update(
            simulation_time
        )

        completed_b = cell_b.update(
            simulation_time
        )

        completed_c = cell_c.update(
            simulation_time
        )


        # ----------------------------------------------------
        # 완료된 Task 확인
        #
        # 현재는 FMS가 없으므로 출력만 함
        # ----------------------------------------------------

        if completed_a is not None:

            print(
                f"[FMS RESULT] "
                f"Cell A | "
                f"Task {completed_a.task_id} | "
                f"Kit {completed_a.kit_id}"
            )


        if completed_b is not None:

            print(
                f"[FMS RESULT] "
                f"Cell B | "
                f"Task {completed_b.task_id} | "
                f"Kit {completed_b.kit_id}"
            )


        if completed_c is not None:

            print(
                f"[FMS RESULT] "
                f"Cell C | "
                f"Task {completed_c.task_id} | "
                f"Kit {completed_c.kit_id}"
            )


finally:

    # ========================================================
    # Isaac Sim 종료
    # ========================================================

    simulation_app.close()
