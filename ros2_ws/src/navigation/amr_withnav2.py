import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from interfaces.msg import AMRStatus
from interfaces.srv import RequestTask
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


NAV2_ACTION_NAME = "/navigate_to_pose"
NAV2_FRAME_ID = "map"
NAV2_SERVER_TIMEOUT_SEC = 10.0
NAV2_RESULT_TIMEOUT_SEC = 120.0
TASK_REQUEST_INTERVAL_SEC = 1.0
LOADING_TIME_SEC = 2.0


class AMRNav2Node(Node):
    """Pull FMS missions and execute them through Nav2 NavigateToPose."""

    def __init__(self):
        super().__init__("amr_nav2_node")
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self.amr_id = "AMR_01"
        self.state = "IDLE"
        self.load_state = "EMPTY"
        self.current_task_id = ""
        self.current_kit_id = ""
        self.task_request_pending = False
        self.task_running = False

        self.pose_lock = threading.Lock()
        self.task_lock = threading.Lock()
        self.latest_xy = None
        self.isaac_state_received = False

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup()

        # 현재 Workspace의 AMR -> FMS Pull 인터페이스
        self.task_client = self.create_client(
            RequestTask,
            "/fms/request_task",
            callback_group=self.service_group,
        )
        self.status_publisher = self.create_publisher(
            AMRStatus, "/amr/status", 10
        )

        # 유지하는 Nav2 통신: NavigateToPose가 경로 계획과 주행을 담당한다.
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            NAV2_ACTION_NAME,
            callback_group=self.nav2_group,
        )

        # Isaac Sim -> AMR
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/amr/odom",
            self.odom_callback,
            10,
            callback_group=self.odom_group,
        )
        self.task_request_timer = self.create_timer(
            TASK_REQUEST_INTERVAL_SEC,
            self.try_request_task,
            callback_group=self.timer_group,
        )

        self.get_logger().info("=================================")
        self.get_logger().info("AMR Nav2 Node started")
        self.get_logger().info(f"AMR ID       : {self.amr_id}")
        self.get_logger().info("Task Service : /fms/request_task")
        self.get_logger().info("Nav2 Action  : /navigate_to_pose")
        self.get_logger().info("Odom Topic   : /amr/odom")
        self.get_logger().info("Status Topic : /amr/status")
        self.get_logger().info("=================================")

    def odom_callback(self, message):
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        first_state = False

        with self.pose_lock:
            self.latest_xy = (x, y)
            if not self.isaac_state_received:
                self.isaac_state_received = True
                first_state = True

        if first_state:
            self.get_logger().info(
                "Isaac Sim state received; AMR communication is ready"
            )
            self.publish_status("READY")

    def get_current_position(self):
        with self.pose_lock:
            return self.latest_xy

    def publish_status(self, event):
        message = AMRStatus()
        message.amr_id = self.amr_id
        message.state = self.state
        message.event = event
        message.task_id = self.current_task_id
        message.kit_id = self.current_kit_id
        message.load_state = self.load_state

        current_position = self.get_current_position()
        if current_position is not None:
            message.x, message.y = current_position

        self.status_publisher.publish(message)
        self.get_logger().info(
            f"[AMR STATUS] amr_id={self.amr_id}, state={self.state}, "
            f"event={event}, task_id={self.current_task_id or '-'}, "
            f"load={self.load_state}"
        )

    def transition_to(self, state, event=None):
        with self.task_lock:
            self.state = state
        self.publish_status(event or state)

    def try_request_task(self):
        if self.task_running or self.task_request_pending:
            return
        if self.state != "IDLE":
            return

        current_position = self.get_current_position()
        if current_position is None:
            return
        if not self.task_client.service_is_ready():
            self.get_logger().info("Waiting for FMS Task Service...")
            return

        current_x, current_y = current_position
        request = RequestTask.Request()
        request.amr_id = self.amr_id
        request.state = self.state
        request.current_task_id = self.current_task_id
        request.x = float(current_x)
        request.y = float(current_y)
        request.load_state = self.load_state

        self.task_request_pending = True
        self.get_logger().info(
            f"[TASK REQUEST] {self.amr_id} -> FMS "
            f"(x={current_x:.2f}, y={current_y:.2f})"
        )
        future = self.task_client.call_async(request)
        future.add_done_callback(self.task_response_callback)

    def task_response_callback(self, future):
        self.task_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Task request failed: {error}")
            return

        if not response.has_task:
            self.get_logger().info(f"[NO TASK] {response.message}")
            return

        with self.task_lock:
            if self.task_running:
                self.get_logger().warning(
                    "Task is already running. Ignoring duplicated assignment."
                )
                return
            self.task_running = True
            self.current_task_id = response.task_id
            self.current_kit_id = response.kit_id
            self.state = "BUSY"

        self.get_logger().info("=================================")
        self.get_logger().info("NEW TASK RECEIVED")
        self.get_logger().info(f"Task ID  : {response.task_id}")
        self.get_logger().info(f"Kit ID   : {response.kit_id}")
        self.get_logger().info(
            f"Pickup   : {response.pickup_id} "
            f"({response.pickup_x:.2f}, {response.pickup_y:.2f})"
        )
        self.get_logger().info(
            f"Delivery : {response.delivery_id} "
            f"({response.delivery_x:.2f}, {response.delivery_y:.2f})"
        )
        self.get_logger().info("=================================")
        self.publish_status("TASK_ASSIGNED")

        threading.Thread(
            target=self.execute_task, args=(response,), daemon=True
        ).start()

    def execute_task(self, task):
        try:
            self.transition_to("MOVING_TO_PICKUP")
            if not self.move_to_destination(
                task.pickup_id, task.pickup_x, task.pickup_y
            ):
                self.handle_task_failure(f"Failed to move to {task.pickup_id}")
                return

            self.transition_to("ARRIVED_PICKUP")
            self.publish_status("LOADING")
            time.sleep(LOADING_TIME_SEC)
            self.load_state = "LOADED"
            self.transition_to("LOADED", "LOAD_COMPLETE")

            self.transition_to("MOVING_TO_DELIVERY")
            if not self.move_to_destination(
                task.delivery_id, task.delivery_x, task.delivery_y
            ):
                self.handle_task_failure(f"Failed to move to {task.delivery_id}")
                return

            self.publish_status("ARRIVED_DELIVERY")
            self.load_state = "EMPTY"
            self.transition_to("DELIVERED", "DELIVERY_COMPLETE")
            self.publish_status("MISSION_COMPLETE")

            with self.task_lock:
                self.current_task_id = ""
                self.current_kit_id = ""
                self.state = "IDLE"
                self.task_running = False
            self.publish_status("IDLE")
        except Exception as error:
            self.handle_task_failure(str(error))

    def handle_task_failure(self, message):
        with self.task_lock:
            self.state = "ERROR"
            self.task_running = False
        self.publish_status("TASK_FAILED")
        self.get_logger().error(f"[TASK FAILED] {message}")

    def move_to_destination(self, destination_id, goal_x, goal_y):
        """Send one physical FMS destination to Nav2 and await its result."""
        if not self.nav2_client.wait_for_server(
            timeout_sec=NAV2_SERVER_TIMEOUT_SEC
        ):
            self.get_logger().error("Nav2 NavigateToPose server is unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(goal_x, goal_y)
        goal.behavior_tree = ""
        completed = threading.Event()
        outcome = {}

        def feedback_callback(feedback_message):
            feedback = feedback_message.feedback
            self.get_logger().debug(
                f"Nav2 {destination_id}: "
                f"remaining={feedback.distance_remaining:.2f}m"
            )

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
                        f"Nav2 rejected goal: {destination_id}"
                    )
                    completed.set()
                    return
                goal_handle.get_result_async().add_done_callback(result_callback)
            except Exception as error:
                outcome["error"] = error
                completed.set()

        self.get_logger().info(
            f"Sending NavigateToPose goal: {destination_id} "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )
        self.nav2_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        ).add_done_callback(goal_callback)

        if not completed.wait(NAV2_RESULT_TIMEOUT_SEC):
            self.get_logger().error(
                f"Timed out while navigating to {destination_id}"
            )
            return False
        if "error" in outcome:
            self.get_logger().error(f"Nav2 navigation failed: {outcome['error']}")
            return False

        result = outcome["result"]
        if (
            outcome["status"] != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != NavigateToPose.Result.NONE
        ):
            self.get_logger().error(
                f"Nav2 navigation failed: status={outcome['status']}, "
                f"code={result.error_code}, message={result.error_msg}"
            )
            return False

        self.get_logger().info(f"Nav2 reached destination: {destination_id}")
        return True

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = NAV2_FRAME_ID
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = AMRNav2Node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

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
