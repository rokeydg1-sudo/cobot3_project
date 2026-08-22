#!/usr/bin/env python3
"""Scenario 0의 Task 10개를 실행 우선순위로 정렬하는 cuOpt 클래스."""

from __future__ import annotations

import cudf
import numpy as np
from cuopt import routing

from defined import (
    LOCATION_BY_ID,
    PARTS_SUPERMARKET,
    OptimizationRequest,
    OptimizationResult,
    OrderedTask,
    Task,
)


class CuOptSolver:
    """거리, 긴급도와 완료기한을 고려해 Task 순서를 결정한다."""

    AMR_SPEED_MPS = 1.0
    PICKUP_SERVICE_TIME = 10
    # Task 10개를 AMR 1대가 순차 운송할 수 있는 Scenario 0 시간 범위다.
    URGENCY_TARGET_SECONDS = {1: 600, 2: 520, 3: 450, 4: 380, 5: 320}

    def __init__(
        self, request: OptimizationRequest, time_limit: float = 5.0
    ) -> None:
        if not 1 <= len(request.tasks) <= 10:
            raise ValueError("Scenario 0 requires 1 to 10 tasks.")

        self.request = request
        self.tasks = list(request.tasks)
        self.time_limit = float(time_limit)
        self.cost_matrix: np.ndarray | None = None
        self.transit_time_matrix: np.ndarray | None = None
        self.data_model = None
        self.solution = None
        self._validate_tasks()

    def _validate_tasks(self) -> None:
        task_ids: set[str] = set()
        for task in self.tasks:
            if task.task_id in task_ids:
                raise ValueError(f"Duplicate task_id: {task.task_id}")
            task_ids.add(task.task_id)
            if task.delivery_cell not in {"cell_a", "cell_b", "cell_c"}:
                raise ValueError(f"Unknown Assembly Cell: {task.delivery_cell}")

    @staticmethod
    def _distance(
        source_x: float, source_y: float, target_x: float, target_y: float
    ) -> float:
        return float(np.hypot(target_x - source_x, target_y - source_y))

    def build_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """Task 간 실제 단위 운송 비용을 생성한다.

        node 0은 현재 AMR 위치이고, node 1~10은 입력 순서의 Task이다.
        Task i 다음 Task j의 비용은 Cell i -> Supermarket -> Cell j이다.
        """
        node_count = len(self.tasks) + 1
        matrix = np.zeros((node_count, node_count), dtype=np.float32)
        supermarket = PARTS_SUPERMARKET
        amr = self.request.amr_state

        for target_node, target_task in enumerate(self.tasks, start=1):
            target_cell = LOCATION_BY_ID[target_task.delivery_cell]
            matrix[0, target_node] = self._distance(
                amr.x, amr.y, supermarket.x, supermarket.y
            ) + self._distance(
                supermarket.x, supermarket.y, target_cell.x, target_cell.y
            )

        for source_node, source_task in enumerate(self.tasks, start=1):
            source_cell = LOCATION_BY_ID[source_task.delivery_cell]

            # 모든 Task 완료 후 Scenario 0의 AMR 시작 위치로 복귀한다.
            matrix[source_node, 0] = self._distance(
                source_cell.x, source_cell.y, amr.x, amr.y
            )

            for target_node, target_task in enumerate(self.tasks, start=1):
                if source_node == target_node:
                    continue
                target_cell = LOCATION_BY_ID[target_task.delivery_cell]
                matrix[source_node, target_node] = self._distance(
                    source_cell.x,
                    source_cell.y,
                    supermarket.x,
                    supermarket.y,
                ) + self._distance(
                    supermarket.x,
                    supermarket.y,
                    target_cell.x,
                    target_cell.y,
                )

        self.cost_matrix = matrix
        self.transit_time_matrix = np.ceil(
            matrix / self.AMR_SPEED_MPS
        ).astype(np.float32)
        return self.cost_matrix, self.transit_time_matrix

    def _latest_service_start(self, task: Task) -> int:
        """Pickup과 Delivery가 deadline 안에 끝나는 최종 서비스 시작시각."""
        urgency_deadline = (
            task.requested_at + self.URGENCY_TARGET_SECONDS[task.urgency]
        )
        completion_deadline = min(task.deadline, urgency_deadline)
        latest_start = completion_deadline - (
            self.PICKUP_SERVICE_TIME + task.service_time
        )
        if latest_start < task.requested_at:
            raise ValueError(
                f"Task {task.task_id} cannot finish before its deadline."
            )
        return latest_start

    def build_data_model(self):
        """각 Task를 고유한 cuOpt node 하나로 구성한다."""
        if self.cost_matrix is None or self.transit_time_matrix is None:
            self.build_matrices()

        task_count = len(self.tasks)
        self.data_model = routing.DataModel(task_count + 1, 1, task_count)
        self.data_model.add_cost_matrix(cudf.DataFrame(self.cost_matrix))
        self.data_model.add_transit_time_matrix(
            cudf.DataFrame(self.transit_time_matrix)
        )
        self.data_model.set_vehicle_locations(
            cudf.Series([0], dtype="int32"),
            cudf.Series([0], dtype="int32"),
        )

        # node 1~10이 Task 01~10과 1:1로 대응하므로 결과를 추정하지 않는다.
        self.data_model.set_order_locations(
            cudf.Series(range(1, task_count + 1), dtype="int32")
        )
        self.data_model.set_order_time_windows(
            cudf.Series(
                [task.requested_at for task in self.tasks], dtype="int32"
            ),
            cudf.Series(
                [self._latest_service_start(task) for task in self.tasks],
                dtype="int32",
            ),
        )
        self.data_model.set_order_service_times(
            cudf.Series(
                [
                    self.PICKUP_SERVICE_TIME + task.service_time
                    for task in self.tasks
                ],
                dtype="int32",
            )
        )
        return self.data_model

    def run_optimizer(self):
        if self.data_model is None:
            self.build_data_model()

        settings = routing.SolverSettings()
        settings.set_time_limit(self.time_limit)
        self.solution = routing.Solve(self.data_model, settings)
        status = int(self.solution.get_status())
        if status != 0:
            raise RuntimeError(
                f"cuOpt failed: status={status}, message={self.solution.get_message()}"
            )
        return self.solution

    def format_result(self) -> OptimizationResult:
        if self.solution is None or self.cost_matrix is None:
            raise RuntimeError("Optimizer has not been run.")

        route_df = self.solution.get_route()
        route_nodes = [
            int(value) for value in route_df["location"].to_arrow().to_pylist()
        ]
        task_nodes = [node for node in route_nodes if node != 0]
        if len(task_nodes) != len(self.tasks):
            raise RuntimeError(
                "cuOpt result does not contain every Task exactly once."
            )

        ordered_tasks = tuple(
            OrderedTask(
                sequence=sequence,
                task_id=self.tasks[node - 1].task_id,
                delivery_cell=self.tasks[node - 1].delivery_cell,
                x=LOCATION_BY_ID[self.tasks[node - 1].delivery_cell].x,
                y=LOCATION_BY_ID[self.tasks[node - 1].delivery_cell].y,
                yaw=LOCATION_BY_ID[self.tasks[node - 1].delivery_cell].yaw,
            )
            for sequence, node in enumerate(task_nodes, start=1)
        )
        total_distance = sum(
            float(self.cost_matrix[source, target])
            for source, target in zip(route_nodes, route_nodes[1:])
        )
        return OptimizationResult(
            success=True,
            message=f"Optimized {len(ordered_tasks)} tasks.",
            ordered_tasks=ordered_tasks,
            total_distance=total_distance,
        )

    def solve(self) -> OptimizationResult:
        self.build_matrices()
        self.build_data_model()
        self.run_optimizer()
        return self.format_result()
