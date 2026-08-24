import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from interfaces.msg import AMRStatus, Location
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
NAV2_RESULT_TIMEOUT_SEC = 120.0
TASK_REQUEST_INTERVAL_SEC = 1.0
LOADING_TIME_SEC = 2.0
ASSEMBLY_LOCATIONS = {
    Location.ASSEMBLY_CELL_A,
    Location.ASSEMBLY_CELL_B,
    Location.ASSEMBLY_CELL_C,
}


class AMRNode(Node):
    """Pull FMS missions and execute them through Nav2 NavigateToPose."""

    def __init__(self, amr_id):
        super().__init__("amr_node")
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self.amr_id = amr_id
        self.state = "IDLE"
        self.load_state = "EMPTY"
        self.current_task_id = ""
        self.current_kit_id = ""
        self.task_request_pending = False
        self.task_running = False
        self.destinations = []
        self.destination_index = 0
        self.navigation_token = 0
        self.navigation_timeout_timer = None
        self.loading_timer = None

        self.pose_lock = threading.Lock()
        self.task_lock = threading.RLock()
        self.latest_xy = None
        self.isaac_state_received = False

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup()

        # AMR -> FMS
        self.task_client = self.create_client(
            RequestTask,
            "/fms/request_task",
            callback_group=self.service_group,
        )
        self.status_publisher = self.create_publisher(
            AMRStatus, "/amr/status", 10
        )

        # Nav2 plans and drives the AMR by publishing /cmd_vel to Isaac Sim.
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
        self.get_logger().info("AMR Node started")
        self.get_logger().info(f"AMR ID        : {self.amr_id}")
        self.get_logger().info("Task Service  : /fms/request_task")
        self.get_logger().info("Nav2 Action   : /navigate_to_pose")
        self.get_logger().info("Odom Topic    : /amr/odom")
        self.get_logger().info("Status Topic  : /amr/status")
        self.get_logger().info("=================================")

    def odom_callback(self, msg):
        """Store Isaac Sim odometry and announce readiness once."""
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
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
        """Publish the latest AMR runtime state to FMS."""
        message = AMRStatus()
        with self.task_lock:
            message.amr_id = self.amr_id
            message.state = self.state
            message.event = event
            message.task_id = self.current_task_id
            message.kit_id = self.current_kit_id
            message.load_state = self.load_state
            current_position = self.get_current_position()
            if current_position is not None:
                message.x, message.y = current_position
            packed_at_ns = time.time_ns()
            message.timestamp.sec = packed_at_ns // 1_000_000_000
            message.timestamp.nanosec = packed_at_ns % 1_000_000_000

        self.status_publisher.publish(message)
        self.get_logger().info(
            f"[AMR STATUS] amr_id={self.amr_id}, state={self.state}, "
            f"event={event}, task_id={self.current_task_id or '-'}, "
            f"load={self.load_state}"
        )

    def transition_to(self, state, event=None):
        """Change state and publish one state-change event."""
        with self.task_lock:
            self.state = state
            self.publish_status(event or state)

    def try_request_task(self):
        """Ask FMS for work when this AMR can accept a mission."""
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
        """Store an FMS assignment and start its mission worker."""
        self.task_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Task request failed: {error}")
            return

        if not response.has_task:
            self.get_logger().info(f"[NO TASK] {response.message}")
            return
        if not response.destinations:
            self.get_logger().error("FMS returned a task without destinations")
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
            self.destinations = list(response.destinations)
            self.destination_index = 0

        self.get_logger().info("=================================")
        self.get_logger().info("NEW TASK RECEIVED")
        self.get_logger().info(f"Task ID         : {response.task_id}")
        self.get_logger().info(f"Kit ID          : {response.kit_id}")
        self.get_logger().info(
            f"Processing Time : {response.processing_time:.1f}s"
        )
        for index, destination in enumerate(response.destinations, start=1):
            self.get_logger().info(
                f"Destination {index:02d} : {destination.name} "
                f"({destination.x:.2f}, {destination.y:.2f}, "
                f"yaw={destination.yaw:.2f})"
            )
        self.get_logger().info("=================================")
        self.publish_status("TASK_ASSIGNED")
        self.start_next_destination()

    def start_next_destination(self):
        """Send the next destination without blocking an executor thread."""
        with self.task_lock:
            if not self.task_running:
                return
            if self.destination_index >= len(self.destinations):
                destination = None
            else:
                destination = self.destinations[self.destination_index]

        if destination is None:
            self.complete_task()
            return

        location_name = destination.name
        if location_name == Location.PARTS_SUPERMARKET:
            self.transition_to("MOVING_TO_PICKUP")
        elif location_name in ASSEMBLY_LOCATIONS:
            self.transition_to("MOVING_TO_DELIVERY")
        else:
            self.transition_to("NAVIGATING")

        self.send_navigation_goal(destination)

    def complete_task(self):
        self.publish_status("MISSION_COMPLETE")
        self.get_logger().info(f"[TASK COMPLETE] {self.current_task_id}")
        with self.task_lock:
            self.current_task_id = ""
            self.current_kit_id = ""
            self.destinations = []
            self.destination_index = 0
            self.state = "IDLE"
            self.task_running = False
        self.publish_status("IDLE")

    def handle_task_failure(self, message):
        """Move the AMR to ERROR and stop automatic task requests."""
        with self.task_lock:
            self.state = "ERROR"
            self.task_running = False
        self.publish_status("TASK_FAILED")
        self.get_logger().error(f"[TASK FAILED] {message}")

    def create_pose(self, destination):
        """Create a map-frame pose for a Nav2 navigation goal."""
        pose = PoseStamped()
        pose.header.frame_id = NAV2_FRAME_ID
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(destination.x)
        pose.pose.position.y = float(destination.y)
        pose.pose.orientation.z = math.sin(float(destination.yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(destination.yaw) / 2.0)
        return pose

    def send_navigation_goal(self, destination):
        """Send one destination and continue from asynchronous callbacks."""
        destination_id = destination.name
        if not self.nav2_client.server_is_ready():
            self.get_logger().error("Nav2 NavigateToPose server is unavailable")
            self.handle_task_failure(
                f"Nav2 is unavailable for {destination_id}"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(destination)
        goal.behavior_tree = ""

        with self.task_lock:
            self.navigation_token += 1
            token = self.navigation_token

        def feedback_callback(feedback_message):
            feedback = feedback_message.feedback
            self.get_logger().debug(
                f"Nav2 {destination_id}: "
                f"remaining={feedback.distance_remaining:.2f}m"
            )

        def goal_callback(future):
            if not self.is_current_navigation(token):
                return
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    if not self.claim_navigation(token):
                        return
                    self.finish_navigation_timer()
                    self.handle_task_failure(
                        f"Nav2 rejected goal: {destination_id}"
                    )
                    return
                goal_handle.get_result_async().add_done_callback(
                    lambda result_future: self.navigation_result_callback(
                        result_future, destination, token
                    )
                )
            except Exception as error:
                if not self.claim_navigation(token):
                    return
                self.finish_navigation_timer()
                self.handle_task_failure(str(error))

        self.get_logger().info(
            f"Sending NavigateToPose goal: {destination_id} "
            f"({destination.x:.2f}, {destination.y:.2f}, "
            f"yaw={destination.yaw:.2f})"
        )
        self.finish_navigation_timer()
        self.navigation_timeout_timer = self.create_timer(
            NAV2_RESULT_TIMEOUT_SEC,
            lambda: self.navigation_timeout_callback(destination_id, token),
            callback_group=self.timer_group,
        )
        self.nav2_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        ).add_done_callback(goal_callback)

    def is_current_navigation(self, token):
        with self.task_lock:
            return self.task_running and token == self.navigation_token

    def claim_navigation(self, token):
        """Allow only one result or timeout callback to finish this goal."""
        with self.task_lock:
            if not self.task_running or token != self.navigation_token:
                return False
            self.navigation_token += 1
            return True

    def finish_navigation_timer(self):
        if self.navigation_timeout_timer is not None:
            self.navigation_timeout_timer.cancel()
            self.destroy_timer(self.navigation_timeout_timer)
            self.navigation_timeout_timer = None

    def navigation_timeout_callback(self, destination_id, token):
        if not self.claim_navigation(token):
            return
        self.finish_navigation_timer()
        self.handle_task_failure(
            f"Timed out while navigating to {destination_id}"
        )

    def navigation_result_callback(self, future, destination, token):
        if not self.claim_navigation(token):
            return
        self.finish_navigation_timer()

        try:
            wrapped_result = future.result()
        except Exception as error:
            self.handle_task_failure(str(error))
            return

        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != NavigateToPose.Result.NONE
        ):
            self.handle_task_failure(
                f"Nav2 navigation failed: status={wrapped_result.status}, "
                f"code={result.error_code}, message={result.error_msg}"
            )
            return

        self.get_logger().info(
            f"Nav2 reached destination: {destination.name}"
        )
        self.handle_destination_arrival(destination)

    def handle_destination_arrival(self, destination):
        location_name = destination.name
        if location_name == Location.PARTS_SUPERMARKET:
            self.transition_to("ARRIVED_PICKUP")
            self.publish_status("LOADING")
            self.finish_loading_timer()
            self.loading_timer = self.create_timer(
                LOADING_TIME_SEC,
                self.loading_complete_callback,
                callback_group=self.timer_group,
            )
            return

        if location_name in ASSEMBLY_LOCATIONS:
            with self.task_lock:
                self.load_state = "EMPTY"
            self.transition_to("DELIVERED", "DELIVERY_COMPLETE")
        else:
            self.transition_to("ARRIVED", f"ARRIVED_{location_name}")
        self.advance_destination()

    def finish_loading_timer(self):
        if self.loading_timer is not None:
            self.loading_timer.cancel()
            self.destroy_timer(self.loading_timer)
            self.loading_timer = None

    def loading_complete_callback(self):
        self.finish_loading_timer()
        with self.task_lock:
            self.load_state = "LOADED"
        self.transition_to("LOADED", "LOAD_COMPLETE")
        self.advance_destination()

    def advance_destination(self):
        with self.task_lock:
            self.destination_index += 1
        self.start_next_destination()


def main(args=None):
    rclpy.init(args=args)
    amr_id = "AMR_01"
    node = AMRNode(amr_id)
    executor = MultiThreadedExecutor(num_threads=2)
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
