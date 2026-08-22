import socket
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


# =========================================================
# Isaac Sim Position TCP Server
# =========================================================

POSE_HOST = "127.0.0.1"
POSE_PORT = 5006


class AMRPoseBridgeNode(Node):

    def __init__(self):

        super().__init__("amr_pose_bridge_node")

        # =================================================
        # /amr/odom Publisher
        # =================================================

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/amr/odom",
            10
        )

        # =================================================
        # 최신 AMR Pose
        # =================================================

        self.latest_pose = None

        self.pose_lock = threading.Lock()

        # 디버그 로그 출력용 카운터
        self.pose_receive_count = 0

        # =================================================
        # TCP Server Thread
        # =================================================

        self.running = True

        self.server_socket = None

        self.server_thread = threading.Thread(
            target=self.tcp_server_loop,
            daemon=True
        )

        self.server_thread.start()

        # =================================================
        # ROS2 Publish Timer
        #
        # 20Hz
        # =================================================

        self.publish_timer = self.create_timer(
            0.05,
            self.publish_odometry
        )

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "AMR Pose Bridge Node started"
        )

        self.get_logger().info(
            f"TCP Pose Server : "
            f"{POSE_HOST}:{POSE_PORT}"
        )

        self.get_logger().info(
            "ROS2 Topic      : /amr/odom"
        )

        self.get_logger().info(
            "================================="
        )


    # =====================================================
    # TCP Server
    # =====================================================

    def tcp_server_loop(self):

        try:

            self.server_socket = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            self.server_socket.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1
            )

            self.server_socket.bind(
                (POSE_HOST, POSE_PORT)
            )

            self.server_socket.listen(1)

            self.server_socket.settimeout(1.0)

        except Exception as error:

            self.get_logger().error(
                f"Pose TCP Server 시작 실패: {error}"
            )

            return


        while self.running:

            try:

                client_socket, client_address = (
                    self.server_socket.accept()
                )

                self.get_logger().info(
                    f"Isaac Sim Pose 연결됨: "
                    f"{client_address}"
                )

                self.handle_client(
                    client_socket
                )

            except socket.timeout:

                continue

            except OSError:

                # 종료 과정에서 socket이 닫히면 정상적으로 빠져나감
                if not self.running:
                    break

            except Exception as error:

                if self.running:

                    self.get_logger().error(
                        f"Pose TCP Server 오류: {error}"
                    )


    # =====================================================
    # Isaac Sim Client 처리
    # =====================================================

    def handle_client(self, client_socket):

        buffer = ""

        client_socket.settimeout(1.0)

        try:

            while self.running:

                try:

                    data = client_socket.recv(
                        4096
                    )

                    if not data:
                        break


                    buffer += data.decode(
                        "utf-8"
                    )


                    # 여러 Pose가 한 번에 들어올 수 있으므로
                    # 줄 단위로 처리
                    while "\n" in buffer:

                        line, buffer = buffer.split(
                            "\n",
                            1
                        )

                        self.parse_pose(
                            line.strip()
                        )


                except socket.timeout:

                    continue


        except Exception as error:

            if self.running:

                self.get_logger().warning(
                    f"Isaac Pose 연결 오류: {error}"
                )


        finally:

            try:

                client_socket.close()

            except Exception:

                pass


            if self.running:

                self.get_logger().warning(
                    "Isaac Sim Pose 연결 종료"
                )


    # =====================================================
    # Pose 문자열 파싱
    #
    # Isaac Sim 전송 형식:
    #
    # x y z qw qx qy qz
    # =====================================================

    def parse_pose(self, line):

        if not line:
            return


        values = line.split()


        if len(values) != 7:

            self.get_logger().warning(
                f"잘못된 Pose 데이터: {line}"
            )

            return


        try:

            (
                x,
                y,
                z,
                qw,
                qx,
                qy,
                qz
            ) = map(
                float,
                values
            )

        except ValueError:

            self.get_logger().warning(
                f"Pose 숫자 변환 실패: {line}"
            )

            return


        # =================================================
        # 최신 Pose 저장
        # =================================================

        with self.pose_lock:

            self.latest_pose = (
                x,
                y,
                z,
                qw,
                qx,
                qy,
                qz
            )


        # =================================================
        # 디버그 로그
        #
        # 20개 수신마다 한 번 출력
        # =================================================

        self.pose_receive_count += 1


        if self.pose_receive_count >= 20:

            self.pose_receive_count = 0

            self.get_logger().info(
                f"[POSE RX] "
                f"x={x:.2f}, "
                f"y={y:.2f}, "
                f"z={z:.2f}"
            )


    # =====================================================
    # /amr/odom Publish
    # =====================================================

    def publish_odometry(self):

        with self.pose_lock:

            if self.latest_pose is None:
                return


            (
                x,
                y,
                z,
                qw,
                qx,
                qy,
                qz
            ) = self.latest_pose


        msg = Odometry()


        # -------------------------------------------------
        # Header
        # -------------------------------------------------

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "world"

        msg.child_frame_id = "base_link"


        # -------------------------------------------------
        # Position
        # -------------------------------------------------

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z


        # -------------------------------------------------
        # Orientation
        #
        # Isaac:
        # qw qx qy qz
        #
        # ROS:
        # x y z w
        # -------------------------------------------------

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw


        # -------------------------------------------------
        # Publish
        # -------------------------------------------------

        self.odom_publisher.publish(
            msg
        )


    # =====================================================
    # 종료 처리
    # =====================================================

    def destroy_node(self):

        self.running = False


        if self.server_socket is not None:

            try:

                self.server_socket.close()

            except Exception:

                pass


        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = AMRPoseBridgeNode()


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