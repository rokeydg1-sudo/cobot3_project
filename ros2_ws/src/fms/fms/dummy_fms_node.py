import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DummyFMS(Node):

    def __init__(self):

        super().__init__("dummy_fms")

        # ====================================================
        # Assembly Cell에서 작업 완료 결과를 받음
        # ====================================================

        self.task_subscriber = self.create_subscription(
            String,
            "/assembly/task",
            self.receive_task_result,
            10
        )

        # ====================================================
        # FMS가 받은 결과를 저장할 파일
        # ====================================================

        self.result_file = os.path.expanduser(
            "~/cobot3_project/fms_results.txt"
        )

        self.get_logger().info(
            "Dummy FMS started"
        )

        self.get_logger().info(
            "Waiting for Assembly Task results..."
        )


    def receive_task_result(self, msg):

        # ====================================================
        # Assembly Node에서 전달받은 결과
        # ====================================================

        result = msg.data

        # 화면 출력
        self.get_logger().info(
            f"Assembly 결과 수신: {result}"
        )

        # ====================================================
        # 결과 파일에 저장
        # ====================================================

        with open(self.result_file, "a") as file:

            file.write(result + "\n")


def main(args=None):

    rclpy.init(args=args)

    node = DummyFMS()

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