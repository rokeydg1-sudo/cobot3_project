import time
import socket

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

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
# Isaac Sim TCP Server
# =========================================================

ISAAC_HOST = "127.0.0.1"
ISAAC_PORT = 5005


class AMRMissionNode(Node):

    def __init__(self):
        super().__init__('amr_mission_node')

        # =================================================
        # FMS가 보내는 Mission을 받는 Action Server
        # =================================================

        self.action_server = ActionServer(
            self,
            ExecuteMission,
            '/amr/execute_mission',
            self.execute_callback
        )

        # =================================================
        # AMR 상태 Publisher
        # =================================================

        self.status_publisher = self.create_publisher(
            String,
            '/amr/status',
            10
        )

        self.get_logger().info(
            'AMR Mission Action Server started'
        )

        self.get_logger().info(
            'Waiting for mission from FMS...'
        )


    # =====================================================
    # AMR 상태 발행
    # =====================================================

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_publisher.publish(msg)

        self.get_logger().info(
            f'STATUS: {status}'
        )


    # =====================================================
    # Isaac Sim에 실제 이동 명령 전달
    # =====================================================

    def move_to_destination(self, destination):

        # ---------------------------------------------
        # 목적지 확인
        # ---------------------------------------------

        if destination not in LOCATIONS:

            self.get_logger().error(
                f'Unknown destination: {destination}'
            )

            return False


        goal_x, goal_y = LOCATIONS[destination]

        self.get_logger().info(
            f'Sending Isaac goal: '
            f'{destination} ({goal_x}, {goal_y})'
        )


        # ---------------------------------------------
        # Isaac Sim TCP 연결
        # ---------------------------------------------

        try:

            with socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            ) as sock:

                # 최대 60초 동안 이동 완료 기다림
                sock.settimeout(60.0)

                sock.connect(
                    (ISAAC_HOST, ISAAC_PORT)
                )


                # 예:
                # "-7.0 0.0"
                command = (
                    f'{goal_x} {goal_y}\n'
                )


                sock.sendall(
                    command.encode('utf-8')
                )


                self.get_logger().info(
                    f'Goal sent to Isaac Sim: '
                    f'{command.strip()}'
                )


                # =========================================
                # Isaac Sim의 REACHED 응답 대기
                # =========================================

                while True:

                    response = sock.recv(1024)

                    if not response:

                        self.get_logger().error(
                            'Isaac Sim connection closed'
                        )

                        return False


                    message = (
                        response
                        .decode('utf-8')
                        .strip()
                    )


                    self.get_logger().info(
                        f'Isaac response: {message}'
                    )


                    if message == 'REACHED':

                        return True


        # ---------------------------------------------
        # Isaac Sim이 실행되지 않은 경우
        # ---------------------------------------------

        except ConnectionRefusedError:

            self.get_logger().error(
                'Cannot connect to Isaac Sim. '
                'standalone_amr_world.py를 먼저 실행하세요.'
            )

            return False


        # ---------------------------------------------
        # 이동 시간이 너무 오래 걸린 경우
        # ---------------------------------------------

        except socket.timeout:

            self.get_logger().error(
                f'Timeout while moving to {destination}'
            )

            return False


        # ---------------------------------------------
        # 기타 TCP 오류
        # ---------------------------------------------

        except Exception as error:

            self.get_logger().error(
                f'Isaac TCP error: {error}'
            )

            return False


    # =====================================================
    # FMS Mission 수행
    # =====================================================

    def execute_callback(self, goal_handle):

        route = list(
            goal_handle.request.route
        )


        self.get_logger().info(
            f'Mission received: {" -> ".join(route)}'
        )


        self.publish_status(
            'MISSION_RECEIVED'
        )


        feedback_msg = ExecuteMission.Feedback()


        # =================================================
        # Mission 경로 순서대로 수행
        # =================================================

        for destination in route:

            # =============================================
            # 이동 시작
            # =============================================

            status = f'MOVING_TO_{destination}'

            self.publish_status(status)

            feedback_msg.status = status

            goal_handle.publish_feedback(
                feedback_msg
            )


            # =============================================
            # 실제 Isaac Sim 이동
            # =============================================

            success = self.move_to_destination(
                destination
            )


            # =============================================
            # 이동 실패
            # =============================================

            if not success:

                status = (
                    f'MOVE_FAILED_{destination}'
                )

                self.publish_status(status)

                feedback_msg.status = status

                goal_handle.publish_feedback(
                    feedback_msg
                )


                goal_handle.abort()


                result = ExecuteMission.Result()

                result.success = False

                result.message = (
                    f'Failed to move to {destination}'
                )

                return result


            # =============================================
            # 실제 도착 완료
            # =============================================

            status = f'ARRIVED_{destination}'

            self.publish_status(status)

            feedback_msg.status = status

            goal_handle.publish_feedback(
                feedback_msg
            )


            # =============================================
            # Supermarket 도착
            # =============================================

            if destination == 'SP':

                self.publish_status(
                    'LOADING'
                )

                feedback_msg.status = 'LOADING'

                goal_handle.publish_feedback(
                    feedback_msg
                )


                # 적재 시간 2초
                # 이 sleep은 유지
                time.sleep(2.0)


                self.publish_status(
                    'LOAD_COMPLETE'
                )

                feedback_msg.status = (
                    'LOAD_COMPLETE'
                )

                goal_handle.publish_feedback(
                    feedback_msg
                )


            # =============================================
            # Assembly Cell 도착
            # =============================================

            else:

                status = (
                    f'DELIVERY_COMPLETE_{destination}'
                )

                self.publish_status(status)

                feedback_msg.status = status

                goal_handle.publish_feedback(
                    feedback_msg
                )


        # =================================================
        # 전체 Mission 완료
        # =================================================

        self.publish_status(
            'MISSION_COMPLETE'
        )


        goal_handle.succeed()


        result = ExecuteMission.Result()

        result.success = True

        result.message = (
            f'Mission complete: {" -> ".join(route)}'
        )


        return result


def main(args=None):

    rclpy.init(args=args)

    node = AMRMissionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()