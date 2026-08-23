import math
import threading
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from interfaces.action import ExecuteMission


# =========================================================
# 목적지 좌표
# =========================================================

LOCATIONS = {
    "SP": (-7.0, 0.0),
    "A":  (7.0, 3.5),
    "B":  (7.0, 0.0),
    "C":  (7.0, -3.5),
}


# =========================================================
# 이동 설정
# =========================================================

ARRIVAL_TOLERANCE_M = 0.20
MOVE_TIMEOUT_SEC = 60.0
CHECK_INTERVAL_SEC = 0.05


class AMRMissionNode(Node):

    def __init__(self):

        super().__init__("amr_mission_node")

        # Action 실행 중에도 /amr/odom을 받아야 하므로
        # Callback Group을 분리한다.
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.odom_group = MutuallyExclusiveCallbackGroup()


        # =================================================
        # FMS -> AMR Mission
        # =================================================

        self.action_server = ActionServer(
            self,
            ExecuteMission,
            "/amr/execute_mission",
            self.execute_callback,
            callback_group=self.action_group,
        )


        # =================================================
        # AMR Mission Node -> Isaac Sim
        #
        # TCP 5005 대신 ROS2 Topic
        # =================================================

        self.goal_publisher = self.create_publisher(
            Point,
            "/amr/goal",
            10,
        )


        # =================================================
        # Isaac Sim -> AMR Mission Node
        #
        # Isaac ROS2 Bridge가 발행
        # =================================================

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/amr/odom",
            self.odom_callback,
            10,
            callback_group=self.odom_group,
        )


        # =================================================
        # AMR 상태
        # =================================================

        self.status_publisher = self.create_publisher(
            String,
            "/amr/status",
            10,
        )


        # 최신 AMR 좌표
        self.pose_lock = threading.Lock()
        self.latest_xy = None


        # 같은 위치를 연속 명령해도
        # Isaac이 새 명령인지 구분하기 위한 ID
        self.goal_command_id = 0


        self.get_logger().info(
            "AMR Mission Action Server started"
        )

        self.get_logger().info(
            "Goal command  : /amr/goal"
        )

        self.get_logger().info(
            "Pose feedback : /amr/odom"
        )

        self.get_logger().info(
            "TCP 5005      : REMOVED"
        )


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

        with self.pose_lock:

            self.latest_xy = (
                x,
                y,
            )


    # =====================================================
    # 상태 발행
    # =====================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_publisher.publish(
            msg
        )

        self.get_logger().info(
            f"STATUS: {status}"
        )


    # =====================================================
    # Isaac Sim으로 Goal 발행
    # =====================================================

    def publish_goal(
        self,
        goal_x,
        goal_y,
    ):

        self.goal_command_id += 1


        msg = Point()

        msg.x = float(
            goal_x
        )

        msg.y = float(
            goal_y
        )


        # 지금은 z를 실제 높이로 사용하지 않으므로
        # 새 Goal을 구분하는 command_id로 사용
        msg.z = float(
            self.goal_command_id
        )


        self.goal_publisher.publish(
            msg
        )


        self.get_logger().info(

            f"Goal published: "
            f"x={goal_x:.2f}, "
            f"y={goal_y:.2f}, "
            f"id={self.goal_command_id}"
        )


    # =====================================================
    # 목적지 이동
    # =====================================================

    def move_to_destination(
        self,
        destination,
    ):

        if destination not in LOCATIONS:

            self.get_logger().error(
                f"Unknown destination: {destination}"
            )

            return False


        goal_x, goal_y = (
            LOCATIONS[destination]
        )


        self.publish_goal(
            goal_x,
            goal_y,
        )


        start_time = (
            time.monotonic()
        )

        last_log_time = 0.0


        # =================================================
        # /amr/odom을 이용해 도착 판단
        # =================================================

        while rclpy.ok():

            elapsed = (
                time.monotonic()
                - start_time
            )


            # ---------------------------------------------
            # Timeout
            # ---------------------------------------------

            if elapsed >= MOVE_TIMEOUT_SEC:

                self.get_logger().error(

                    f"Timeout while moving "
                    f"to {destination}"
                )

                return False


            # ---------------------------------------------
            # 최신 위치 확인
            # ---------------------------------------------

            with self.pose_lock:

                latest_xy = (
                    self.latest_xy
                )


            if latest_xy is None:

                time.sleep(
                    CHECK_INTERVAL_SEC
                )

                continue


            current_x, current_y = (
                latest_xy
            )


            # ---------------------------------------------
            # 목표까지 거리
            # ---------------------------------------------

            distance = math.hypot(

                goal_x - current_x,

                goal_y - current_y,
            )


            # ---------------------------------------------
            # 도착
            # ---------------------------------------------

            if (
                distance
                <= ARRIVAL_TOLERANCE_M
            ):

                self.get_logger().info(

                    f"Reached {destination}: "
                    f"x={current_x:.2f}, "
                    f"y={current_y:.2f}, "
                    f"distance={distance:.3f}m"
                )

                return True


            # ---------------------------------------------
            # 이동 중 로그
            # ---------------------------------------------

            now = time.monotonic()


            if (
                now - last_log_time
                >= 1.0
            ):

                last_log_time = now


                self.get_logger().info(

                    f"Moving {destination}: "
                    f"x={current_x:.2f}, "
                    f"y={current_y:.2f}, "
                    f"distance={distance:.2f}m"
                )


            time.sleep(
                CHECK_INTERVAL_SEC
            )


        return False


    # =====================================================
    # Mission 수행
    # =====================================================

    def execute_callback(
        self,
        goal_handle,
    ):

        route = list(
            goal_handle.request.route
        )


        self.get_logger().info(

            f'Mission received: '
            f'{" -> ".join(route)}'
        )


        self.publish_status(
            "MISSION_RECEIVED"
        )


        feedback_msg = (
            ExecuteMission.Feedback()
        )


        # =================================================
        # Route 순서대로 실행
        # =================================================

        for destination in route:


            # =============================================
            # 이동 시작
            # =============================================

            status = (
                f"MOVING_TO_{destination}"
            )


            self.publish_status(
                status
            )


            feedback_msg.status = status

            goal_handle.publish_feedback(
                feedback_msg
            )


            # =============================================
            # 이동
            # =============================================

            success = (
                self.move_to_destination(
                    destination
                )
            )


            # =============================================
            # 실패
            # =============================================

            if not success:

                status = (
                    f"MOVE_FAILED_{destination}"
                )


                self.publish_status(
                    status
                )


                feedback_msg.status = (
                    status
                )


                goal_handle.publish_feedback(
                    feedback_msg
                )


                goal_handle.abort()


                result = (
                    ExecuteMission.Result()
                )


                result.success = False

                result.message = (
                    f"Failed to move "
                    f"to {destination}"
                )


                return result


            # =============================================
            # 도착
            # =============================================

            status = (
                f"ARRIVED_{destination}"
            )


            self.publish_status(
                status
            )


            feedback_msg.status = status

            goal_handle.publish_feedback(
                feedback_msg
            )


            # =============================================
            # Supermarket
            # =============================================

            if destination == "SP":

                self.publish_status(
                    "LOADING"
                )


                feedback_msg.status = (
                    "LOADING"
                )


                goal_handle.publish_feedback(
                    feedback_msg
                )


                time.sleep(
                    2.0
                )


                self.publish_status(
                    "LOAD_COMPLETE"
                )


                feedback_msg.status = (
                    "LOAD_COMPLETE"
                )


                goal_handle.publish_feedback(
                    feedback_msg
                )


            # =============================================
            # Assembly Cell
            # =============================================

            else:

                status = (

                    f"DELIVERY_COMPLETE_"
                    f"{destination}"
                )


                self.publish_status(
                    status
                )


                feedback_msg.status = (
                    status
                )


                goal_handle.publish_feedback(
                    feedback_msg
                )


        # =================================================
        # Mission 완료
        # =================================================

        self.publish_status(
            "MISSION_COMPLETE"
        )


        goal_handle.succeed()


        result = (
            ExecuteMission.Result()
        )


        result.success = True

        result.message = (

            f'Mission complete: '
            f'{" -> ".join(route)}'
        )


        return result


def main(args=None):

    rclpy.init(
        args=args
    )


    node = AMRMissionNode()


    # 중요:
    # Action Callback이 목적지 도착을 기다리는 동안
    # /amr/odom Callback도 계속 돌아야 한다.
    executor = MultiThreadedExecutor(
        num_threads=2
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