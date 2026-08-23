import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose
from interfaces.action import ExecuteMission


# =========================================================
# 목적지 좌표
#
# Nav2의 "map" 좌표계 기준
# =========================================================

LOCATIONS = {
    "SP": (-7.0, 0.0),
    "A":  (7.0, 3.5),
    "B":  (7.0, 0.0),
    "C":  (7.0, -3.5),
}


# =========================================================
# Nav2 설정
# =========================================================

NAV2_ACTION_NAME = "/navigate_to_pose"
NAV2_FRAME_ID = "map"

# Nav2 서버가 올라올 때까지 기다리는 최대 시간
NAV2_SERVER_WAIT_TIMEOUT = 5.0


class AMRNode(Node):

    def __init__(self):
        super().__init__("amr_node")

        # =================================================
        # 같은 Node에서
        #
        # ExecuteMission Action Server
        # +
        # NavigateToPose Action Client
        #
        # 를 동시에 사용하기 때문에 Reentrant 사용
        # =================================================

        self.callback_group = ReentrantCallbackGroup()

        # =================================================
        # FMS → AMR
        #
        # FMS가 보내는 Mission을 받는 Action Server
        # =================================================

        self.action_server = ActionServer(
            self,
            ExecuteMission,
            "/amr/execute_mission",
            self.execute_callback,
            callback_group=self.callback_group,
        )

        # =================================================
        # AMR → Nav2
        #
        # NavigateToPose Action Client
        # =================================================

        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            NAV2_ACTION_NAME,
            callback_group=self.callback_group,
        )

        # Nav2 feedback 로그 throttling용
        self.last_nav2_feedback_log_ns = 0

        # =================================================
        # AMR 상태 Publisher
        # =================================================

        self.status_publisher = self.create_publisher(
            String,
            "/amr/status",
            10,
        )

        self.get_logger().info(
            "AMR Node started"
        )

        self.get_logger().info(
            "ExecuteMission Action Server: /amr/execute_mission"
        )

        self.get_logger().info(
            "Nav2 Action Client: /navigate_to_pose"
        )

        self.get_logger().info(
            "Waiting for mission from FMS..."
        )


    # =====================================================
    # AMR 상태 발행
    # =====================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_publisher.publish(msg)

        self.get_logger().info(
            f"STATUS: {status}"
        )


    # =====================================================
    # Nav2 Feedback
    # =====================================================

    def nav2_feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        # ---------------------------------------------
        # Nav2는 feedback을 매우 자주 보내므로
        # 로그는 약 1초에 한 번만 출력
        # ---------------------------------------------

        now_ns = self.get_clock().now().nanoseconds

        if (
            now_ns - self.last_nav2_feedback_log_ns
            < 1_000_000_000
        ):
            return

        self.last_nav2_feedback_log_ns = now_ns

        current_pose = feedback.current_pose.pose.position

        self.get_logger().info(
            "Nav2 feedback | "
            f"position=({current_pose.x:.2f}, "
            f"{current_pose.y:.2f}) | "
            f"remaining={feedback.distance_remaining:.2f} m"
        )


    # =====================================================
    # Nav2로 실제 이동 명령 전달
    # =====================================================

    async def move_to_destination(
        self,
        destination,
    ):

        # ---------------------------------------------
        # 목적지 검사
        # ---------------------------------------------

        if destination not in LOCATIONS:

            self.get_logger().error(
                f"Unknown destination: {destination}"
            )

            return False


        goal_x, goal_y = LOCATIONS[destination]

        self.get_logger().info(
            f"Navigation requested: "
            f"{destination} -> "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )


        # =================================================
        # 1. Nav2 Action Server 확인
        # =================================================

        self.get_logger().info(
            "Checking Nav2 NavigateToPose server..."
        )

        server_ready = self.nav2_client.wait_for_server(
            timeout_sec=NAV2_SERVER_WAIT_TIMEOUT
        )

        if not server_ready:

            self.get_logger().error(
                "Nav2 NavigateToPose server is not available"
            )

            return False


        # =================================================
        # 2. NavigateToPose Goal 생성
        # =================================================

        nav_goal = NavigateToPose.Goal()


        # ---------------------------------------------
        # 좌표계
        #
        # 이 x, y는 map frame 기준이라는 의미
        # ---------------------------------------------

        nav_goal.pose.header.frame_id = (
            NAV2_FRAME_ID
        )

        nav_goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )


        # ---------------------------------------------
        # 위치
        # ---------------------------------------------

        nav_goal.pose.pose.position.x = goal_x
        nav_goal.pose.pose.position.y = goal_y
        nav_goal.pose.pose.position.z = 0.0


        # ---------------------------------------------
        # 방향
        #
        # 현재 최소 구현에서는 yaw = 0 rad
        #
        # quaternion:
        # x = 0
        # y = 0
        # z = 0
        # w = 1
        # ---------------------------------------------

        nav_goal.pose.pose.orientation.x = 0.0
        nav_goal.pose.pose.orientation.y = 0.0
        nav_goal.pose.pose.orientation.z = 0.0
        nav_goal.pose.pose.orientation.w = 1.0


        # ---------------------------------------------
        # Behavior Tree를 비워두면
        # Nav2의 default BT 사용
        # ---------------------------------------------

        nav_goal.behavior_tree = ""


        # =================================================
        # 3. Nav2로 Goal 전송
        # =================================================

        self.get_logger().info(
            f"Sending NavigateToPose goal: "
            f"{destination}"
        )

        send_goal_future = (
            self.nav2_client.send_goal_async(
                nav_goal,
                feedback_callback=(
                    self.nav2_feedback_callback
                ),
            )
        )

        nav_goal_handle = await send_goal_future


        # =================================================
        # 4. Goal 수락 여부 확인
        # =================================================

        if not nav_goal_handle.accepted:

            self.get_logger().error(
                f"Nav2 rejected goal: {destination}"
            )

            return False


        self.get_logger().info(
            f"Nav2 accepted goal: {destination}"
        )


        # =================================================
        # 5. Nav2 이동 결과 대기
        # =================================================

        result_future = (
            nav_goal_handle.get_result_async()
        )

        wrapped_result = await result_future


        # ROS Action 자체 상태
        action_status = wrapped_result.status

        # NavigateToPose Result
        nav_result = wrapped_result.result


        # =================================================
        # 6. 성공 판단
        # =================================================

        if (
            action_status
            == GoalStatus.STATUS_SUCCEEDED
            and
            nav_result.error_code
            == NavigateToPose.Result.NONE
        ):

            self.get_logger().info(
                f"Nav2 reached destination: "
                f"{destination}"
            )

            return True


        # =================================================
        # 7. 이동 실패
        # =================================================

        self.get_logger().error(
            "Nav2 navigation failed | "
            f"destination={destination} | "
            f"action_status={action_status} | "
            f"error_code={nav_result.error_code} | "
            f"error_msg={nav_result.error_msg}"
        )

        return False


    # =====================================================
    # FMS Mission 수행
    # =====================================================

    async def execute_callback(
        self,
        goal_handle,
    ):

        route = list(
            goal_handle.request.route
        )


        self.get_logger().info(
            f'Mission received: {" -> ".join(route)}'
        )


        self.publish_status(
            "MISSION_RECEIVED"
        )


        feedback_msg = (
            ExecuteMission.Feedback()
        )


        # =================================================
        # Mission 경로 순서대로 수행
        #
        # 예:
        #
        # ["SP", "A"]
        #
        # SP NavigateToPose
        #        ↓
        # 성공
        #        ↓
        # A NavigateToPose
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
            # Nav2 NavigateToPose
            # =============================================

            success = (
                await self.move_to_destination(
                    destination
                )
            )


            # =============================================
            # 이동 실패
            # =============================================

            if not success:

                status = (
                    f"MOVE_FAILED_{destination}"
                )

                self.publish_status(
                    status
                )

                feedback_msg.status = status

                goal_handle.publish_feedback(
                    feedback_msg
                )

                goal_handle.abort()


                result = (
                    ExecuteMission.Result()
                )

                result.success = False

                result.message = (
                    f"Failed to move to "
                    f"{destination}"
                )

                return result


            # =============================================
            # Nav2 도착 완료
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
            # Supermarket 도착
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


                # -----------------------------------------
                # 기존 적재 시간 유지
                # -----------------------------------------

                time.sleep(2.0)


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
            # Assembly Cell 도착
            # =============================================

            else:

                status = (
                    f"DELIVERY_COMPLETE_"
                    f"{destination}"
                )

                self.publish_status(
                    status
                )

                feedback_msg.status = status

                goal_handle.publish_feedback(
                    feedback_msg
                )


        # =================================================
        # 전체 Mission 완료
        # =================================================

        self.publish_status(
            "MISSION_COMPLETE"
        )


        goal_handle.succeed()


        result = ExecuteMission.Result()

        result.success = True

        result.message = (
            f'Mission complete: '
            f'{" -> ".join(route)}'
        )


        return result


# =========================================================
# main
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = AMRNode()


    # =====================================================
    # 중요
    #
    # 이 Node는 동시에:
    #
    # 1. ExecuteMission Action Server
    # 2. NavigateToPose Action Client
    #
    # 역할을 수행한다.
    #
    # 따라서 MultiThreadedExecutor 사용
    # =====================================================

    executor = MultiThreadedExecutor(
        num_threads=4
    )

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