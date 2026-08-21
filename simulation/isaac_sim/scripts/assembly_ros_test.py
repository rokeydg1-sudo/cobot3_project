#!/usr/bin/env python3

# ============================================================
# Isaac Sim Standalone
# Assembly Algorithm + ROS 2 Test
# ============================================================

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})


# ============================================================
# Python
# ============================================================

import sys
import os
import numpy as np


# ============================================================
# Assembly Algorithm Import
# ============================================================

# 현재 파일과 같은 디렉토리에 있는
# assembly_algorithm.py를 import하기 위한 경로
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)


from assembly_algorithm import Task
from assembly_algorithm import AssemblyCell


# ============================================================
# Isaac Sim API
# ============================================================

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid


# ============================================================
# ROS 2 Bridge 활성화
# ============================================================

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

simulation_app.update()


# ============================================================
# ROS 2
# ============================================================

import rclpy

from rclpy.node import Node

from std_msgs.msg import String


# ============================================================
# ROS 2 Assembly Node
# ============================================================

class AssemblyROSNode(Node):

    def __init__(self):

        super().__init__("isaac_assembly_node")


        # ----------------------------------------------------
        # FMS로 Task 결과를 보내는 Publisher
        # ----------------------------------------------------

        self.task_publisher = self.create_publisher(
            String,
            "/assembly/task",
            10
        )


        self.get_logger().info(
            "Isaac Assembly ROS Node started"
        )


    # --------------------------------------------------------
    # FMS로 Task 결과 전송
    # --------------------------------------------------------

    def send_task_to_fms(self, cell, task):

        result = (
            f"cell_id={cell.cell_id}, "
            f"task_id={task.task_id}, "
            f"kit_id={task.kit_id}, "
            f"processing_time={task.processing_time}"
        )


        msg = String()

        msg.data = result


        self.task_publisher.publish(msg)


        self.get_logger().info(
            f"FMS로 Task 결과 전송: {result}"
        )


# ============================================================
# World
# ============================================================

world = World(
    stage_units_in_meters=1.0
)


# 기본 바닥
world.scene.add_default_ground_plane()


# ============================================================
# Assembly Cell Area
# ============================================================

CELL_SIZE = np.array([
    3.0,
    3.0,
    0.02
])


CELL_LOCATIONS = {

    "A": {
        "position": np.array([
            0.0,
            0.0,
            0.01
        ]),
        "color": np.array([
            0.0,
            0.0,
            1.0
        ])
    },

    "B": {
        "position": np.array([
            4.0,
            0.0,
            0.01
        ]),
        "color": np.array([
            1.0,
            1.0,
            0.0
        ])
    },

    "C": {
        "position": np.array([
            8.0,
            0.0,
            0.01
        ]),
        "color": np.array([
            0.0,
            1.0,
            0.0
        ])
    }
}


# ============================================================
# Assembly Area 생성
# ============================================================

for cell_id, data in CELL_LOCATIONS.items():

    world.scene.add(

        VisualCuboid(

            prim_path=f"/World/AssemblyCell_{cell_id}",

            name=f"assembly_cell_{cell_id}",

            position=data["position"],

            size=1.0,

            scale=CELL_SIZE,

            color=data["color"]
        )
    )


# ============================================================
# Assembly Cell 생성
# ============================================================

cell_1 = AssemblyCell(
    cell_id=1
)

cell_2 = AssemblyCell(
    cell_id=2
)

cell_3 = AssemblyCell(
    cell_id=3
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
# 초기 Queue 등록
# ============================================================

for _ in range(3):

    cell_1.add_task(task_1)

    cell_2.add_task(task_2)

    cell_3.add_task(task_3)


# ============================================================
# World 초기화
# ============================================================

world.reset()


# ============================================================
# ROS 초기화
# ============================================================

rclpy.init(args=None)

ros_node = AssemblyROSNode()


# ============================================================
# Simulation Time
# ============================================================

simulation_time = 0.0

dt = 0.1


# ============================================================
# 시작 메시지
# ============================================================

print()
print("==============================================")
print(" Isaac Sim Assembly ROS Test")
print("==============================================")
print(" Assembly A : BLUE")
print(" Assembly B : YELLOW")
print(" Assembly C : GREEN")
print()
print(" ROS Topic")
print(" PUB /assembly/task")
print("==============================================")
print()


# ============================================================
# Simulation Loop
# ============================================================

try:

    while simulation_app.is_running():

        # ----------------------------------------------------
        # ROS 2 callback 처리
        # ----------------------------------------------------

        rclpy.spin_once(
            ros_node,
            timeout_sec=0.0
        )


        # ----------------------------------------------------
        # Assembly Cell 업데이트
        # ----------------------------------------------------

        completed_task = cell_1.update(
            simulation_time
        )

        if completed_task is not None:

            ros_node.send_task_to_fms(
                cell_1,
                completed_task
            )


        completed_task = cell_2.update(
            simulation_time
        )

        if completed_task is not None:

            ros_node.send_task_to_fms(
                cell_2,
                completed_task
            )


        completed_task = cell_3.update(
            simulation_time
        )

        if completed_task is not None:

            ros_node.send_task_to_fms(
                cell_3,
                completed_task
            )


        # ----------------------------------------------------
        # Isaac Sim 진행
        # ----------------------------------------------------

        world.step(
            render=True
        )


        # ----------------------------------------------------
        # Simulation Time
        # ----------------------------------------------------

        simulation_time += dt


finally:

    print()
    print("Closing Assembly ROS Test...")


    ros_node.destroy_node()


    if rclpy.ok():

        rclpy.shutdown()


    simulation_app.close()
