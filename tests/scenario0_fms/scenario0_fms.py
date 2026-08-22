#!/usr/bin/env python3
"""Scenario 0의 작업큐와 cuOpt 호출을 관리하는 FMS ROS 2 Node."""

from __future__ import annotations

from collections import deque
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from defined import (
    INITIAL_AMR_STATE,
    TASKS,
    OptimizationRequest,
    OptimizationResult,
    Task,
)
from scenario0_cuopt_solver import CuOptSolver


class Scenario0FMSNode(Node):
    QUEUE_CAPACITY = 10
    TASK_REQUEST_TOPIC = "/assembly/task_request"
    ROUTE_REQUEST_SERVICE = "/amr/request_route"

    def __init__(self) -> None:
        super().__init__("scenario0_fms")

        # Scenario 0 재현용 Task를 실제 수신 큐와 같은 큐에 순서대로 적재한다.
        self.task_queue: deque[Task] = deque(TASKS, maxlen=self.QUEUE_CAPACITY)
        self.latest_plan: OptimizationResult | None = None
        self.is_optimizing = False

        self.task_subscription = self.create_subscription(
            String,
            self.TASK_REQUEST_TOPIC,
            self.task_request_callback,
            self.QUEUE_CAPACITY,
        )
        self.route_request_service = self.create_service(
            Trigger,
            self.ROUTE_REQUEST_SERVICE,
            self.amr_route_request_callback,
        )

        self.get_logger().info(
            f"Loaded {len(self.task_queue)}/{self.QUEUE_CAPACITY} demo tasks"
        )
        self.get_logger().info(
            f"Waiting for AMR route request on {self.ROUTE_REQUEST_SERVICE}"
        )

    def add_task(self, task: Task) -> None:
        """검증된 Assembly Cell Task를 큐에 추가한다."""
        if len(self.task_queue) >= self.QUEUE_CAPACITY:
            raise OverflowError("Task queue is full (capacity=10).")
        if task.delivery_cell not in {"cell_a", "cell_b", "cell_c"}:
            raise ValueError(f"Unknown Assembly Cell: {task.delivery_cell}")
        if any(queued.task_id == task.task_id for queued in self.task_queue):
            raise ValueError(f"Duplicate task_id: {task.task_id}")
        self.task_queue.append(task)

    def task_request_callback(self, message: String) -> None:
        """임시 문자열 요청을 Task로 변환해 큐에 적재한다."""
        try:
            # task_id,cell_id,kit_id,urgency,requested_at,deadline,service_time
            fields = [field.strip() for field in message.data.split(",")]
            if len(fields) != 7:
                raise ValueError("Task request requires 7 fields.")

            task = Task(
                task_id=fields[0],
                kit_id=fields[2],
                delivery_cell=fields[1].lower(),
                urgency=int(fields[3]),
                requested_at=int(fields[4]),
                deadline=int(fields[5]),
                service_time=int(fields[6]),
            )
            self.add_task(task)
        except (ValueError, OverflowError) as error:
            self.get_logger().warning(f"Rejected Assembly Cell task: {error}")
            return

        self.get_logger().info(
            f"Queued {task.task_id} from {task.delivery_cell}: "
            f"{len(self.task_queue)}/{self.QUEUE_CAPACITY}"
        )

    def amr_route_request_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """AMR 요청 시 현재 작업큐 전체를 cuOpt로 최적화한다."""
        del request  # Trigger에는 AMR 상태 필드가 아직 정의되지 않았다.

        if not self.task_queue:
            response.success = False
            response.message = "Task queue is empty."
            return response
        if self.is_optimizing:
            response.success = False
            response.message = "Route optimization is already in progress."
            return response

        self.is_optimizing = True
        tasks = list(self.task_queue)
        self.get_logger().info(f"Sending {len(tasks)} tasks to cuOpt")

        try:
            optimization_request = OptimizationRequest(
                tasks=tuple(tasks),
                amr_state=INITIAL_AMR_STATE,
            )
            self.latest_plan = CuOptSolver(optimization_request).solve()

            self.get_logger().info("=== Optimized Assembly Cell Order ===")
            for task in self.latest_plan.ordered_tasks:
                self.get_logger().info(
                    f"{task.sequence:02d}. {task.task_id} -> "
                    f"{task.delivery_cell} ({task.x:.1f}, {task.y:.1f})"
                )

            response.success = self.latest_plan.success
            response.message = self.latest_plan.message
            self.get_logger().info("Optimized route stored in FMS")
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"cuOpt optimization failed: {error}")
        finally:
            self.is_optimizing = False

        return response


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Scenario0FMSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
