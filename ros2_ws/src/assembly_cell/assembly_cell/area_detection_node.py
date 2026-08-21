import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import String


class AreaDetectionNode(Node):

    def __init__(self):

        super().__init__("area_detection_node")


        # ====================================================
        # Cell Detection Area
        #
        # 실제 Isaac Sim 좌표 기준
        #
        # Cell A : (7.0,  3.5)
        # Cell B : (7.0,  0.0)
        # Cell C : (7.0, -3.5)
        #
        # 도착 판정이 너무 빡빡하지 않도록
        # 중심 좌표 주변을 약간 넓게 설정
        # ====================================================

        self.cell_areas = {

            "A": {
                "x_min": 6.3,
                "x_max": 7.7,
                "y_min": 2.8,
                "y_max": 4.2,
            },

            "B": {
                "x_min": 6.3,
                "x_max": 7.7,
                "y_min": -0.7,
                "y_max": 0.7,
            },

            "C": {
                "x_min": 6.3,
                "x_max": 7.7,
                "y_min": -4.2,
                "y_max": -2.8,
            },
        }


        # ====================================================
        # 현재 AMR이 들어가 있는 Cell Area
        #
        # 같은 Area에서 part_arrived가
        # 계속 발행되는 것을 방지
        # ====================================================

        self.current_area = None


        # ====================================================
        # 디버그 로그용 카운터
        #
        # /amr/odom이 20Hz 정도이므로
        # 너무 많은 로그를 방지
        # ====================================================

        self.odom_log_count = 0


        # ====================================================
        # AMR 위치 Subscriber
        #
        # Pose Bridge가 발행하는 실제 Topic:
        #
        # /amr/odom
        # ====================================================

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/amr/odom",
            self.odom_callback,
            10
        )


        # ====================================================
        # Assembly Node로 도착 이벤트 Publisher
        # ====================================================

        self.arrival_publisher = self.create_publisher(
            String,
            "/assembly/part_arrived",
            10
        )


        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "Area Detection Node started"
        )

        self.get_logger().info(
            "Listening Topic : /amr/odom"
        )

        self.get_logger().info(
            "Publish Topic   : /assembly/part_arrived"
        )

        self.get_logger().info(
            "================================="
        )


    # ========================================================
    # Odometry Callback
    # ========================================================

    def odom_callback(self, msg):

        # ====================================================
        # AMR 실제 위치
        # ====================================================

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y


        # ====================================================
        # 디버그 로그
        #
        # 대략 1초에 한 번 출력
        # ====================================================

        self.odom_log_count += 1

        if self.odom_log_count >= 20:

            self.odom_log_count = 0

            self.get_logger().info(
                f"[ODOM] "
                f"x={x:.2f}, "
                f"y={y:.2f}"
            )


        # ====================================================
        # 현재 위치가 어느 Cell Area인지 판정
        # ====================================================

        detected_area = self.detect_cell_area(
            x,
            y
        )


        # ====================================================
        # 아무 Cell Area에도 없음
        # ====================================================

        if detected_area is None:

            # 이전에는 Area 안에 있었는데
            # 지금은 빠져나온 경우
            if self.current_area is not None:

                self.get_logger().info(
                    f"[AREA EXIT] "
                    f"Cell {self.current_area}"
                )


            self.current_area = None

            return


        # ====================================================
        # 이미 같은 Area 안에 있음
        #
        # 중복 도착 이벤트 방지
        # ====================================================

        if detected_area == self.current_area:

            return


        # ====================================================
        # 새로운 Cell Area 진입
        # ====================================================

        self.current_area = detected_area


        self.get_logger().info(
            f"[AREA ENTER] "
            f"AMR -> Cell {detected_area} "
            f"(x={x:.2f}, y={y:.2f})"
        )


        # Assembly Node에 부품 도착 알림
        self.publish_arrival(
            detected_area
        )


    # ========================================================
    # Cell Area 판정
    # ========================================================

    def detect_cell_area(self, x, y):

        for cell_id, area in self.cell_areas.items():

            if (
                area["x_min"] <= x <= area["x_max"]
                and
                area["y_min"] <= y <= area["y_max"]
            ):

                return cell_id


        return None


    # ========================================================
    # Assembly Node에 도착 이벤트 전달
    # ========================================================

    def publish_arrival(self, cell_id):

        msg = String()

        msg.data = f"cell_id={cell_id}"


        self.arrival_publisher.publish(
            msg
        )


        self.get_logger().info(
            f"[PART ARRIVED] "
            f"Cell {cell_id} 도착 이벤트 전송"
        )


def main(args=None):

    rclpy.init(args=args)

    node = AreaDetectionNode()


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