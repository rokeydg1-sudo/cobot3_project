import math
import threading
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path

from interfaces.msg import AMRStatus
from interfaces.srv import RequestTask


# =========================================================
# 이동 설정
# =========================================================

ARRIVAL_TOLERANCE_M = 0.20
MOVE_TIMEOUT_SEC = 60.0
CHECK_INTERVAL_SEC = 0.05
NAV2_SERVER_TIMEOUT_SEC = 10.0
NAV2_RESULT_TIMEOUT_SEC = 30.0
GLOBAL_FRAME_ID = "map"

# FMS에 다음 작업을 요청하는 주기
TASK_REQUEST_INTERVAL_SEC = 1.0

# 현재는 실제 Pick 동작이 없으므로
# Supermarket 도착 후 적재 시간을 임시로 사용
LOADING_TIME_SEC = 2.0


class AMRNode(Node):

    def __init__(self):

        super().__init__("amr_node")

        # =================================================
        # AMR Identity
        #
        # 현재 Scenario 0에서는 AMR 1대
        # 추후 Multi-AMR에서는
        # AMR_02, AMR_03 ... 으로 확장
        # =================================================

        self.amr_id = "AMR_01"


        # =================================================
        # AMR 상태
        # =================================================

        self.state = "IDLE"
        self.load_state = "EMPTY"

        self.current_task_id = ""
        self.current_kit_id = ""


        # =================================================
        # 내부 동작 상태
        # =================================================

        # FMS에 Task 요청이 진행 중인지
        self.task_request_pending = False

        # 현재 Task를 실제 수행 중인지
        self.task_running = False


        # =================================================
        # Callback Group
        #
        # Task Service 응답 처리 중에도
        # /amr/odom이 계속 들어와야 함
        # =================================================

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup()


        # =================================================
        # AMR -> FMS
        #
        # Pull 방식:
        # AMR이 IDLE일 때 다음 Task 요청
        # =================================================

        self.task_client = self.create_client(
            RequestTask,
            "/fms/request_task",
            callback_group=self.service_group,
        )


        # =================================================
        # AMR -> Isaac Sim
        #
        # 현재 임시 이동 방식
        #
        # 향후 Nav2 NavigateToPose로 교체 예정
        # =================================================

        self.path_command_publisher = self.create_publisher(
            Path,
            "/amr/path_command",
            10,
        )

        # Nav2 planner는 FMS가 준 목적지까지의 중간 경로만 계산한다.
        # 계산된 Path는 Isaac Sim 측 주행 제어기가 소비한다.
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            callback_group=self.nav2_group,
        )


        # =================================================
        # Isaac Sim -> AMR
        #
        # Isaac ROS2 Bridge가 발행하는 Odometry
        # =================================================

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/amr/odom",
            self.odom_callback,
            10,
            callback_group=self.odom_group,
        )


        # =================================================
        # AMR -> FMS
        #
        # 상태가 변경될 때 이벤트 발행
        # =================================================

        self.status_publisher = self.create_publisher(
            AMRStatus,
            "/amr/status",
            10,
        )


        # =================================================
        # 최신 AMR 위치
        # =================================================

        self.pose_lock = threading.Lock()
        self.latest_xy = None
        self.isaac_state_received = False


        # =================================================
        # Task 상태 보호
        # =================================================

        self.task_lock = threading.Lock()


        # =================================================
        # IDLE일 때 FMS에 다음 작업 요청
        # =================================================

        self.task_request_timer = self.create_timer(
            TASK_REQUEST_INTERVAL_SEC,
            self.try_request_task,
            callback_group=self.timer_group,
        )


        # =================================================
        # 시작 로그
        # =================================================

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "AMR Node started"
        )

        self.get_logger().info(
            f"AMR ID        : {self.amr_id}"
        )

        self.get_logger().info(
            "Task Service  : /fms/request_task"
        )

        self.get_logger().info(
            "Path Command  : /amr/path_command"
        )

        self.get_logger().info(
            "Odom Topic    : /amr/odom"
        )

        self.get_logger().info(
            "Status Topic  : /amr/status"
        )

        self.get_logger().info(
            "ExecuteMission: REMOVED"
        )

        self.get_logger().info(
            "================================="
        )


        # 최초 상태는 Isaac Sim의 odometry를 받은 뒤 알린다.


    # =====================================================
    # /amr/odom 수신
    # =====================================================

    def odom_callback(self, msg):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )


        first_state = False

        with self.pose_lock:

            self.latest_xy = (
                x,
                y,
            )

            if not self.isaac_state_received:

                self.isaac_state_received = True
                first_state = True

        if first_state:

            self.get_logger().info(
                "Isaac Sim state received; AMR communication is ready"
            )

            self.publish_status("READY")


    # =====================================================
    # 최신 위치 반환
    # =====================================================

    def get_current_position(self):

        with self.pose_lock:

            if self.latest_xy is None:

                return None


            return (
                self.latest_xy[0],
                self.latest_xy[1],
            )


    # =====================================================
    # AMR 상태 Event 발행
    #
    # FMS는 이 Topic을 계속 polling하는 것이 아니라
    # 상태가 바뀔 때 발생하는 Event를 받아
    # 마지막 상태만 관리하면 됨
    # =====================================================

    def publish_status(
        self,
        status,
    ):

        msg = AMRStatus()
        msg.amr_id = self.amr_id
        msg.state = self.state
        msg.event = status
        msg.task_id = self.current_task_id
        msg.kit_id = self.current_kit_id
        msg.load_state = self.load_state

        current_position = self.get_current_position()

        if current_position is not None:

            msg.x, msg.y = current_position


        self.status_publisher.publish(
            msg
        )


        self.get_logger().info(
            f"[AMR STATUS] "
            f"amr_id={self.amr_id}, "
            f"state={self.state}, "
            f"status={status}, "
            f"task_id={self.current_task_id or '-'}, "
            f"load={self.load_state}"
        )


    def transition_to(self, state, event=None):

        """Change runtime state and publish one state-change event."""

        with self.task_lock:

            self.state = state

        self.publish_status(event or state)


    # =====================================================
    # IDLE 상태일 때 FMS에 다음 Task 요청
    # =====================================================

    def try_request_task(self):

        # =================================================
        # 현재 Task 수행 중
        # =================================================

        if self.task_running:

            return


        # =================================================
        # 이미 Service 요청 중
        # =================================================

        if self.task_request_pending:

            return


        # =================================================
        # ERROR 상태에서는 자동으로 새 Task를 받지 않음
        # =================================================

        if self.state == "ERROR":

            return


        # =================================================
        # IDLE 상태에서만 다음 Task 요청
        # =================================================

        if self.state != "IDLE":

            return


        # =================================================
        # 현재 위치가 아직 안 들어온 경우
        #
        # FMS에 잘못된 위치를 보내지 않도록
        # /amr/odom 수신 후 요청 시작
        # =================================================

        current_position = (
            self.get_current_position()
        )


        if current_position is None:

            return


        # =================================================
        # FMS Service가 아직 실행되지 않은 경우
        # =================================================

        if not self.task_client.service_is_ready():

            self.get_logger().info(
                "Waiting for FMS Task Service..."
            )

            return


        current_x, current_y = (
            current_position
        )


        # =================================================
        # Request 생성
        # =================================================

        request = RequestTask.Request()

        request.amr_id = self.amr_id
        request.state = self.state
        request.current_task_id = (
            self.current_task_id
        )

        request.x = float(
            current_x
        )

        request.y = float(
            current_y
        )

        request.load_state = (
            self.load_state
        )


        # =================================================
        # 비동기 Service 요청
        # =================================================

        self.task_request_pending = True


        self.get_logger().info(
            f"[TASK REQUEST] "
            f"{self.amr_id} -> FMS "
            f"(x={current_x:.2f}, "
            f"y={current_y:.2f})"
        )


        future = self.task_client.call_async(
            request
        )


        future.add_done_callback(
            self.task_response_callback
        )


    # =====================================================
    # FMS Task 응답
    # =====================================================

    def task_response_callback(
        self,
        future,
    ):

        self.task_request_pending = False


        try:

            response = future.result()


        except Exception as error:

            self.get_logger().error(
                f"Task request failed: {error}"
            )

            return


        # =================================================
        # FMS에 현재 대기 Task가 없음
        # =================================================

        if not response.has_task:

            self.get_logger().info(
                f"[NO TASK] "
                f"{response.message}"
            )

            return


        # =================================================
        # 새로운 Task 수신
        # =================================================

        with self.task_lock:

            if self.task_running:

                self.get_logger().warning(
                    "Task is already running. "
                    "Ignoring duplicated assignment."
                )

                return


            self.task_running = True

            self.current_task_id = (
                response.task_id
            )

            self.current_kit_id = (
                response.kit_id
            )

            self.state = "BUSY"


        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "NEW TASK RECEIVED"
        )

        self.get_logger().info(
            f"Task ID         : "
            f"{response.task_id}"
        )

        self.get_logger().info(
            f"Kit ID          : "
            f"{response.kit_id}"
        )

        self.get_logger().info(
            f"Processing Time : "
            f"{response.processing_time:.1f}s"
        )

        self.get_logger().info(
            f"Pickup          : "
            f"{response.pickup_id} "
            f"({response.pickup_x:.2f}, "
            f"{response.pickup_y:.2f})"
        )

        self.get_logger().info(
            f"Delivery        : "
            f"{response.delivery_id} "
            f"({response.delivery_x:.2f}, "
            f"{response.delivery_y:.2f})"
        )

        self.get_logger().info(
            "================================="
        )


        self.publish_status(
            "TASK_ASSIGNED"
        )


        # =================================================
        # 실제 이동은 Blocking Loop를 사용하므로
        # 별도 Worker Thread에서 수행
        #
        # 그래야 Executor가 /amr/odom과
        # Service Callback을 계속 처리할 수 있음
        # =================================================

        worker = threading.Thread(
            target=self.execute_task,
            args=(response,),
            daemon=True,
        )

        worker.start()


    # =====================================================
    # Task 실행
    #
    # FMS가 이미:
    #
    # Logical Location
    #      ↓
    # Physical Coordinate
    #
    # 변환을 끝낸 상태이므로
    # AMR은 받은 좌표를 그대로 사용
    # =====================================================

    def execute_task(
        self,
        task,
    ):

        try:

            # =================================================
            # 1. Pickup 위치로 이동
            # =================================================

            self.transition_to("MOVING_TO_PICKUP")


            pickup_success = (
                self.move_to_destination(

                    task.pickup_id,

                    task.pickup_x,

                    task.pickup_y,
                )
            )


            if not pickup_success:

                self.handle_task_failure(
                    f"Failed to move to "
                    f"{task.pickup_id}"
                )

                return


            # =================================================
            # Pickup 도착
            # =================================================

            self.transition_to("ARRIVED_PICKUP")


            # =================================================
            # 2. Kit 적재
            #
            # 현재는 실제 Pick 장비가 없으므로
            # 시간으로 가정
            # =================================================

            self.publish_status("LOADING")


            time.sleep(
                LOADING_TIME_SEC
            )


            self.load_state = "LOADED"

            self.transition_to("LOADED", "LOAD_COMPLETE")


            # =================================================
            # 3. Delivery 위치로 이동
            # =================================================

            self.transition_to("MOVING_TO_DELIVERY")


            delivery_success = (
                self.move_to_destination(

                    task.delivery_id,

                    task.delivery_x,

                    task.delivery_y,
                )
            )


            if not delivery_success:

                self.handle_task_failure(
                    f"Failed to move to "
                    f"{task.delivery_id}"
                )

                return


            # =================================================
            # Delivery 도착
            # =================================================

            self.publish_status("ARRIVED_DELIVERY")


            # =================================================
            # 4. Delivery 완료
            #
            # 현재는 Cell 영역 도착 =
            # 부품 배송 완료로 가정
            # =================================================

            self.load_state = "EMPTY"

            self.transition_to("DELIVERED", "DELIVERY_COMPLETE")


            # =================================================
            # 5. Mission 완료
            # =================================================

            self.publish_status("MISSION_COMPLETE")


            self.get_logger().info(
                f"[TASK COMPLETE] "
                f"{self.current_task_id}"
            )


            # =================================================
            # 6. AMR을 다시 IDLE 상태로 변경
            #
            # 다음 Timer Tick에서
            # FMS에 새로운 Task를 Pull 요청하게 됨
            # =================================================

            with self.task_lock:

                self.current_task_id = ""
                self.current_kit_id = ""

                self.state = "IDLE"

                self.task_running = False


            self.publish_status("IDLE")


        except Exception as error:

            self.handle_task_failure(
                str(error)
            )


    # =====================================================
    # Task 실패 처리
    # =====================================================

    def handle_task_failure(
        self,
        message,
    ):

        self.state = "ERROR"


        self.publish_status(
            "TASK_FAILED"
        )


        self.get_logger().error(
            f"[TASK FAILED] {message}"
        )


        with self.task_lock:

            self.task_running = False


    # =====================================================
    # Nav2에 경로를 요청하고 Isaac Sim에 Path 명령 발행
    # =====================================================

    def plan_path(
        self,
        goal_x,
        goal_y,
    ):

        current_position = self.get_current_position()

        if current_position is None:

            self.get_logger().error("Cannot plan without Isaac odometry")
            return None

        if not self.path_client.wait_for_server(
            timeout_sec=NAV2_SERVER_TIMEOUT_SEC
        ):

            self.get_logger().error("Nav2 planner action server is unavailable")
            return None

        current_x, current_y = current_position
        stamp = self.get_clock().now().to_msg()

        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.start = PoseStamped()
        goal.start.header.frame_id = GLOBAL_FRAME_ID
        goal.start.header.stamp = stamp
        goal.start.pose.position.x = float(current_x)
        goal.start.pose.position.y = float(current_y)
        goal.start.pose.orientation.w = 1.0
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = GLOBAL_FRAME_ID
        goal.goal.header.stamp = stamp
        goal.goal.pose.position.x = float(goal_x)
        goal.goal.pose.position.y = float(goal_y)
        goal.goal.pose.orientation.w = 1.0

        completed = threading.Event()
        outcome = {}

        def result_callback(future):

            try:

                wrapped_result = future.result()
                outcome["status"] = wrapped_result.status
                outcome["result"] = wrapped_result.result

            except Exception as error:

                outcome["error"] = error

            completed.set()

        def goal_callback(future):

            try:

                goal_handle = future.result()

                if not goal_handle.accepted:

                    outcome["error"] = RuntimeError(
                        "Nav2 rejected the path-planning request"
                    )
                    completed.set()
                    return

                goal_handle.get_result_async().add_done_callback(
                    result_callback
                )

            except Exception as error:

                outcome["error"] = error
                completed.set()

        self.path_client.send_goal_async(goal).add_done_callback(goal_callback)

        if not completed.wait(NAV2_RESULT_TIMEOUT_SEC):

            self.get_logger().error("Timed out while waiting for a Nav2 path")
            return None

        if "error" in outcome:

            self.get_logger().error(f"Nav2 path request failed: {outcome['error']}")
            return None

        result = outcome["result"]

        if outcome["status"] != GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().error(
                f"Nav2 planning failed: code={result.error_code}, "
                f"message={result.error_msg}"
            )
            return None

        if not result.path.poses:

            self.get_logger().error("Nav2 returned an empty path")
            return None

        return result.path


    def send_path_to_isaac(self, path):

        """Publish a Nav2-planned path for the Isaac Sim AMR controller."""

        self.path_command_publisher.publish(path)
        self.get_logger().info(
            f"Path command sent to Isaac Sim: {len(path.poses)} poses"
        )


    # =====================================================
    # 목적지 이동
    #
    # 중요:
    # 예전처럼 AMR Node 내부 LOCATIONS를 조회하지 않음
    #
    # FMS가 논리 위치를 물리좌표로 변환해서
    # x / y를 전달함
    # =====================================================

    def move_to_destination(
        self,
        destination_id,
        goal_x,
        goal_y,
    ):

        path = self.plan_path(
            goal_x,
            goal_y,
        )

        if path is None:

            return False

        self.send_path_to_isaac(path)


        start_time = (
            time.monotonic()
        )

        last_log_time = 0.0


        # =================================================
        # Isaac Sim odometry를 기준으로 실제 도착을 확인한다.
        # =================================================

        while rclpy.ok():

            elapsed = (
                time.monotonic()
                - start_time
            )


            # =================================================
            # Timeout
            # =================================================

            if elapsed >= MOVE_TIMEOUT_SEC:

                self.get_logger().error(
                    f"Timeout while moving "
                    f"to {destination_id}"
                )

                return False


            # =================================================
            # 최신 위치
            # =================================================

            current_position = (
                self.get_current_position()
            )


            if current_position is None:

                time.sleep(
                    CHECK_INTERVAL_SEC
                )

                continue


            current_x, current_y = (
                current_position
            )


            # =================================================
            # 목표까지 거리
            # =================================================

            distance = math.hypot(

                goal_x - current_x,

                goal_y - current_y,
            )


            # =================================================
            # 도착
            # =================================================

            if (
                distance
                <= ARRIVAL_TOLERANCE_M
            ):

                self.get_logger().info(
                    f"Reached {destination_id}: "
                    f"x={current_x:.2f}, "
                    f"y={current_y:.2f}, "
                    f"distance={distance:.3f}m"
                )

                return True


            # =================================================
            # 이동 중 로그
            # =================================================

            now = time.monotonic()


            if (
                now - last_log_time
                >= 1.0
            ):

                last_log_time = now


                self.get_logger().info(
                    f"Moving {destination_id}: "
                    f"x={current_x:.2f}, "
                    f"y={current_y:.2f}, "
                    f"distance={distance:.2f}m"
                )


            time.sleep(
                CHECK_INTERVAL_SEC
            )


        return False


def main(args=None):

    rclpy.init(
        args=args
    )


    node = AMRNode()


    # =====================================================
    # 여러 Callback을 동시에 처리해야 함
    #
    # - /amr/odom
    # - RequestTask Service response
    # - Task Request Timer
    # =====================================================

    executor = MultiThreadedExecutor(
        num_threads=3
    )


    executor.add_node(
        node
    )


    try:

        executor.spin()


    except KeyboardInterrupt:

        pass


    finally:

        executor.shutdown()

        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
