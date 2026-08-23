#!/usr/bin/env python3
"""Scenario 0의 작업큐, cuOpt 호출, AMR Pull 요청을 관리하는 FMS ROS 2 Node."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Sequence

import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from interfaces.msg import AMRStatus
from interfaces.srv import RequestTask

from fms.defined import (
    AMRState,
    LOCATION_BY_ID,
    PARTS_SUPERMARKET,
    OptimizationRequest,
    OptimizationResult,
    Task,
)

from fms.scenario0_cuopt_solver import CuOptSolver


@dataclass
class AMRRuntimeState:
    """Latest runtime state for one registered AMR."""

    state: str
    status: str
    current_task_id: str
    load_state: str
    kit_id: str = ""
    x: float = 0.0
    y: float = 0.0


class Scenario0FMSNode(Node):

    # =====================================================
    # 기본 설정
    # =====================================================

    QUEUE_CAPACITY = 10

    # 실제 Assembly Node에서 사용 중인 Topic
    TASK_REQUEST_TOPIC = "/assembly/request"

    # AMR Pull 요청 Service
    TASK_REQUEST_SERVICE = "/fms/request_task"

    # AMR 상태 이벤트
    AMR_STATUS_TOPIC = "/amr/status"


    def __init__(self) -> None:

        super().__init__(
            "scenario0_fms"
        )


        # =================================================
        # FMS Task Queue
        #
        # 실제 E2E에서는 Assembly 요청으로 채워짐
        # =================================================

        self.task_queue: deque[Task] = deque(
            maxlen=self.QUEUE_CAPACITY
        )

        # All Scenario 0 task times use FMS startup as time zero.
        self.fms_started_at = time.monotonic()


        # =================================================
        # 마지막 최적화 결과
        # =================================================

        self.latest_plan: OptimizationResult | None = None


        # =================================================
        # cuOpt 실행 중 여부
        # =================================================

        self.is_optimizing = False


        # =================================================
        # AMR 상태 저장
        #
        # key:
        #   AMR_01
        #
        # value:
        #   {
        #       "state": ...,
        #       "status": ...,
        #       "current_task_id": ...,
        #       "load_state": ...
        #   }
        #
        # 상태 이벤트가 들어올 때만 갱신
        # =================================================

        self.amr_states: dict[str, AMRRuntimeState] = {}


        # =================================================
        # Assembly -> FMS
        #
        # Task 요청
        # =================================================

        self.task_subscription = self.create_subscription(
            String,
            self.TASK_REQUEST_TOPIC,
            self.task_request_callback,
            self.QUEUE_CAPACITY,
        )


        # =================================================
        # AMR -> FMS
        #
        # AMR 상태 변화 Event
        # =================================================

        self.amr_status_subscription = (
            self.create_subscription(
                AMRStatus,
                self.AMR_STATUS_TOPIC,
                self.amr_status_callback,
                10,
            )
        )


        # =================================================
        # AMR -> FMS
        #
        # Pull 방식 Task 요청
        # =================================================

        self.task_request_service = (
            self.create_service(
                RequestTask,
                self.TASK_REQUEST_SERVICE,
                self.request_task_callback,
            )
        )


        # =================================================
        # 시작 로그
        # =================================================

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "Scenario 0 FMS started"
        )

        self.get_logger().info(
            f"Assembly Topic : "
            f"{self.TASK_REQUEST_TOPIC}"
        )

        self.get_logger().info(
            f"Task Service   : "
            f"{self.TASK_REQUEST_SERVICE}"
        )

        self.get_logger().info(
            f"AMR Status     : "
            f"{self.AMR_STATUS_TOPIC}"
        )

        self.get_logger().info(
            "Task Queue     : EMPTY"
        )

        self.get_logger().info(
            "================================="
        )


    # =====================================================
    # 문자열 key=value 메시지 Parser
    # =====================================================

    @staticmethod
    def parse_key_value_message(
        data: str,
    ) -> dict[str, str]:

        result: dict[str, str] = {}


        for item in data.split(","):

            item = item.strip()


            if not item:
                continue


            if "=" not in item:
                continue


            key, value = item.split(
                "=",
                1,
            )


            result[
                key.strip()
            ] = value.strip()


        return result


    # =====================================================
    # Assembly -> FMS
    #
    # Task 요청 수신
    #
    # 현재 Assembly 메시지 예:
    #
    # cell_id=A,
    # task_id=1,
    # kit_id=KIT_STAR,
    # shape=STAR,
    # processing_time=3.0
    #
    # 현재 Assembly에는 아직 urgency / deadline이 없으므로
    # Scenario 0 기본값을 임시 적용한다.
    # requested_at은 메시지 값이 아니라 FMS 수신 시각을 사용한다.
    #
    # 향후 custom interface에서 실제 값으로 교체
    # =====================================================

    def task_request_callback(
        self,
        message: String,
    ) -> None:

        try:

            fields = (
                self.parse_key_value_message(
                    message.data
                )
            )


            # =============================================
            # 필수 값 확인
            # =============================================

            cell_id = fields.get(
                "cell_id"
            )

            task_id = fields.get(
                "task_id"
            )

            kit_id = fields.get(
                "kit_id"
            )

            processing_time = fields.get(
                "processing_time"
            )


            if (
                cell_id is None
                or task_id is None
                or kit_id is None
                or processing_time is None
            ):

                raise ValueError(
                    "Assembly request requires "
                    "cell_id, task_id, kit_id, "
                    "processing_time."
                )


            # =============================================
            # Cell 이름 통일
            #
            # A -> cell_a
            # B -> cell_b
            # C -> cell_c
            # =============================================

            delivery_cell = (
                f"cell_{cell_id.lower()}"
            )


            # =============================================
            # Scenario 0 임시 metadata
            #
            # requested_at은 FMS 시작 시각을 0으로 한 작업 수신 시각이다.
            # urgency/deadline은 custom message 적용 전까지 기본값을 쓴다.
            # =============================================

            urgency = int(
                fields.get(
                    "urgency",
                    1,
                )
            )


            requested_at = int(time.monotonic() - self.fms_started_at)


            deadline = int(
                fields.get(
                    "deadline",
                    requested_at + 600,
                )
            )


            # =============================================
            # Task 생성
            # =============================================

            task = Task(
                task_id=str(
                    task_id
                ),

                kit_id=str(
                    kit_id
                ),

                delivery_cell=(
                    delivery_cell
                ),

                urgency=urgency,

                requested_at=(
                    requested_at
                ),

                deadline=deadline,

                processing_time=float(
                    processing_time
                ),
            )


            self.add_task(
                task
            )


        except (
            ValueError,
            OverflowError,
        ) as error:

            self.get_logger().warning(
                f"Rejected Assembly Task: "
                f"{error}"
            )

            return


        self.get_logger().info(
            f"[TASK QUEUED] "
            f"{task.task_id} "
            f"{task.kit_id} "
            f"-> {task.delivery_cell} "
            f"(processing="
            f"{task.processing_time:.1f}s) "
            f"queue="
            f"{len(self.task_queue)}/"
            f"{self.QUEUE_CAPACITY}"
        )


    # =====================================================
    # Task Queue 추가
    # =====================================================

    def add_task(
        self,
        task: Task,
    ) -> None:

        # =================================================
        # Queue Full
        # =================================================

        if (
            len(self.task_queue)
            >= self.QUEUE_CAPACITY
        ):

            raise OverflowError(
                "Task queue is full "
                f"(capacity="
                f"{self.QUEUE_CAPACITY})."
            )


        # =================================================
        # Cell 확인
        # =================================================

        if task.delivery_cell not in {
            "cell_a",
            "cell_b",
            "cell_c",
        }:

            raise ValueError(
                f"Unknown Assembly Cell: "
                f"{task.delivery_cell}"
            )


        # =================================================
        # 중복 Task 방지
        # =================================================

        if any(
            queued.task_id
            == task.task_id

            for queued
            in self.task_queue
        ):

            raise ValueError(
                f"Duplicate task_id: "
                f"{task.task_id}"
            )


        self.task_queue.append(
            task
        )


    # =====================================================
    # AMR 상태 Event 수신
    #
    # polling하지 않고
    # Event가 들어올 때만 마지막 상태 갱신
    # =====================================================

    def amr_status_callback(
        self,
        message: AMRStatus,
    ) -> None:

        amr_id = message.amr_id


        if not amr_id:

            self.get_logger().warning(
                f"Invalid AMR status: "
                "amr_id is empty"
            )

            return


        self.amr_states[amr_id] = AMRRuntimeState(
            state=message.state,
            status=message.event,
            current_task_id=message.task_id,
            load_state=message.load_state,
            kit_id=message.kit_id,
            x=float(message.x),
            y=float(message.y),
        )


        self.get_logger().info(
            f"[AMR EVENT] "
            f"{amr_id} "
            f"state="
            f"{self.amr_states[amr_id].state} "
            f"status="
            f"{self.amr_states[amr_id].status} "
            f"task="
            f"{self.amr_states[amr_id].current_task_id or '-'} "
            f"load="
            f"{self.amr_states[amr_id].load_state} "
            f"position=({message.x:.2f}, {message.y:.2f})"
        )


    # =====================================================
    # AMR -> FMS
    #
    # 다음 Task 요청
    #
    # Pull 방식 핵심 Callback
    # =====================================================

    def request_task_callback(
        self,
        request: RequestTask.Request,
        response: RequestTask.Response,
    ) -> RequestTask.Response:

        self.get_logger().info(
            f"[TASK REQUEST] "
            f"{request.amr_id} "
            f"state={request.state} "
            f"position="
            f"({request.x:.2f}, "
            f"{request.y:.2f}) "
            f"load="
            f"{request.load_state}"
        )

        if not request.amr_id:
            response.has_task = False
            response.message = "Rejected: amr_id is empty."
            return response

        if request.state != "IDLE":
            response.has_task = False
            response.message = "Rejected: AMR state must be IDLE."
            return response

        if request.current_task_id:
            response.has_task = False
            response.message = "Rejected: AMR already has a task."
            return response

        if request.load_state != "EMPTY":
            response.has_task = False
            response.message = "Rejected: AMR load state must be EMPTY."
            return response

        stored_state = self.amr_states.get(request.amr_id)
        if stored_state is not None and stored_state.state not in {
            "IDLE",
            "DELIVERED",
        }:
            response.has_task = False
            response.message = (
                "Rejected: FMS records AMR as "
                f"{stored_state.state}."
            )
            return response


        # =================================================
        # Request 자체를 AMR 최신 상태로 반영
        # =================================================

        self.amr_states[request.amr_id] = AMRRuntimeState(
            state=request.state,
            status="REQUESTING_TASK",
            current_task_id=request.current_task_id,
            load_state=request.load_state,
            x=float(request.x),
            y=float(request.y),
        )


        # =================================================
        # 대기 Task 없음
        # =================================================

        if not self.task_queue:

            response.has_task = False

            response.message = (
                "No waiting task."
            )


            self.get_logger().info(
                f"[NO TASK] "
                f"{request.amr_id}"
            )


            return response


        # =================================================
        # 이미 cuOpt 실행 중
        # =================================================

        if self.is_optimizing:

            response.has_task = False

            response.message = (
                "Optimization is already "
                "in progress."
            )

            return response


        # =================================================
        # 현재 AMR 상태 생성
        #
        # AMR이 직접 보낸 현재 상태/위치를 사용
        # =================================================

        amr_state = AMRState(

            amr_id=(
                request.amr_id
            ),

            state=(
                request.state
            ),

            x=float(
                request.x
            ),

            y=float(
                request.y
            ),

            # 현재 RequestTask.srv에는
            # yaw가 없으므로 Scenario 0에서는
            # 0으로 가정
            yaw=0.0,

            load_state=(
                request.load_state
            ),

            current_task_id=(
                request.current_task_id
            ),
        )


        # =================================================
        # 현재 Queue 전체를 cuOpt에 전달
        # =================================================

        tasks = list(
            self.task_queue
        )


        self.is_optimizing = True


        self.get_logger().info(
            f"[CUOPT] "
            f"Sending "
            f"{len(tasks)} tasks "
            f"for {request.amr_id}"
        )


        try:

            optimization_request = (
                OptimizationRequest(

                    tasks=tuple(
                        tasks
                    ),

                    amr_state=(
                        amr_state
                    ),
                )
            )


            self.latest_plan = (
                CuOptSolver(
                    optimization_request
                ).solve()
            )


            # =============================================
            # cuOpt 결과 확인
            # =============================================

            if (
                not self.latest_plan.success
                or
                not self.latest_plan.ordered_tasks
            ):

                response.has_task = False

                response.message = (
                    self.latest_plan.message
                )

                return response


            self.get_logger().info(
                "=== Optimized Task Order ==="
            )


            for ordered_task in (
                self.latest_plan.ordered_tasks
            ):

                self.get_logger().info(
                    f"{ordered_task.sequence:02d}. "
                    f"{ordered_task.task_id} "
                    f"-> "
                    f"{ordered_task.delivery_cell}"
                )


            # =============================================
            # cuOpt 결과의 첫 번째 Task를
            # 현재 요청한 AMR에 할당
            # =============================================

            selected_order = (
                self.latest_plan
                .ordered_tasks[0]
            )


            selected_task = next(

                task

                for task in tasks

                if (
                    task.task_id
                    == selected_order.task_id
                )
            )


            # =============================================
            # FMS Location DB
            #
            # cuOpt Logical Location
            #      ↓
            # FMS
            #      ↓
            # Physical Coordinate
            # =============================================

            pickup_location = (
                PARTS_SUPERMARKET
            )


            delivery_location = (
                LOCATION_BY_ID[
                    selected_task.delivery_cell
                ]
            )


            # =============================================
            # AMR Response
            # =============================================

            response.has_task = True


            response.task_id = (
                selected_task.task_id
            )


            response.kit_id = (
                selected_task.kit_id
            )


            response.processing_time = float(
                selected_task.processing_time
            )


            # =============================================
            # Pickup
            # =============================================

            response.pickup_id = (
                pickup_location.location_id
            )


            response.pickup_x = float(
                pickup_location.x
            )


            response.pickup_y = float(
                pickup_location.y
            )


            # =============================================
            # Delivery
            # =============================================

            response.delivery_id = (
                delivery_location.location_id
            )


            response.delivery_x = float(
                delivery_location.x
            )


            response.delivery_y = float(
                delivery_location.y
            )


            response.message = (
                f"Assigned "
                f"{selected_task.task_id} "
                f"to "
                f"{request.amr_id}"
            )


            # =============================================
            # Task 상태 변경
            # =============================================

            selected_task.status = (
                "ASSIGNED"
            )


            # =============================================
            # 현재 Scenario 0에서는
            # AMR 한 대가 한 Task씩 Pull하므로
            # Queue에서 할당된 Task 제거
            #
            # 추후 실패 복구 / 재할당을 넣을 때는
            # 별도 Active Task Registry로
            # 이동시키는 구조로 확장 가능
            # =============================================

            self.task_queue.remove(
                selected_task
            )


            # =============================================
            # FMS AMR 상태 갱신
            # =============================================

            self.amr_states[request.amr_id] = AMRRuntimeState(
                state="BUSY",
                status="TASK_ASSIGNED",
                current_task_id=selected_task.task_id,
                load_state=request.load_state,
                kit_id=selected_task.kit_id,
                x=float(request.x),
                y=float(request.y),
            )


            self.get_logger().info(
                f"[TASK ASSIGNED] "
                f"{selected_task.task_id} "
                f"-> {request.amr_id}"
            )


            self.get_logger().info(
                f"Pickup   : "
                f"{pickup_location.location_id} "
                f"({pickup_location.x:.2f}, "
                f"{pickup_location.y:.2f})"
            )


            self.get_logger().info(
                f"Delivery : "
                f"{delivery_location.location_id} "
                f"({delivery_location.x:.2f}, "
                f"{delivery_location.y:.2f})"
            )


            self.get_logger().info(
                f"Remaining Queue: "
                f"{len(self.task_queue)}"
            )


            return response


        except Exception as error:

            response.has_task = False

            response.message = str(
                error
            )


            self.get_logger().error(
                f"cuOpt optimization failed: "
                f"{error}"
            )


            return response


        finally:

            self.is_optimizing = False


def main(
    args: Sequence[str] | None = None,
) -> None:

    rclpy.init(
        args=args
    )


    node = (
        Scenario0FMSNode()
    )


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
