import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .task import Task
from .assembly_cell import AssemblyCell


class AssemblyNode(Node):

    def __init__(self):

        super().__init__("assembly_node")

        # ====================================================
        # FMS로 Task 결과를 보내는 Publisher
        # ====================================================

        self.task_publisher = self.create_publisher(
            String,
            "/assembly/task",
            10
        )


        # ====================================================
        # Assembly Cell 생성
        # ====================================================

        self.cell_1 = AssemblyCell(cell_id=1)
        self.cell_2 = AssemblyCell(cell_id=2)
        self.cell_3 = AssemblyCell(cell_id=3)


        # ====================================================
        # Task 정의
        # ====================================================

        self.task_1 = Task(
            task_id=1,
            kit_id="KIT_A",
            processing_time=3
        )

        self.task_2 = Task(
            task_id=2,
            kit_id="KIT_B",
            processing_time=5
        )

        self.task_3 = Task(
            task_id=3,
            kit_id="KIT_C",
            processing_time=2
        )


        # ====================================================
        # 초기 Task 등록
        # 각 Cell의 Queue에 Task 3개씩 등록
        # ====================================================

        for _ in range(3):

            self.cell_1.add_task(self.task_1)

            self.cell_2.add_task(self.task_2)

            self.cell_3.add_task(self.task_3)


        # ====================================================
        # Simulation Time
        # ====================================================

        self.simulation_time = 0.0

        self.dt = 0.1


        # ====================================================
        # 주기적인 Assembly 업데이트
        #
        # 0.1초마다 실행
        # ====================================================

        self.timer = self.create_timer(
            self.dt,
            self.update
        )


        self.get_logger().info(
            "Assembly Node started"
        )


    def update(self):

        # Simulation Time 증가
        self.simulation_time += self.dt


        # ====================================================
        # Cell 1
        # ====================================================

        completed_task = self.cell_1.update(
            self.simulation_time
        )

        if completed_task is not None:

            self.send_task_to_fms(
                self.cell_1,
                completed_task
            )


        # ====================================================
        # Cell 2
        # ====================================================

        completed_task = self.cell_2.update(
            self.simulation_time
        )

        if completed_task is not None:

            self.send_task_to_fms(
                self.cell_2,
                completed_task
            )


        # ====================================================
        # Cell 3
        # ====================================================

        completed_task = self.cell_3.update(
            self.simulation_time
        )

        if completed_task is not None:

            self.send_task_to_fms(
                self.cell_3,
                completed_task
            )


    def send_task_to_fms(self, cell, task):

        # ====================================================
        # FMS로 보낼 결과 문자열 생성
        # ====================================================

        result = (
            f"cell_id={cell.cell_id}, "
            f"task_id={task.task_id}, "
            f"kit_id={task.kit_id}, "
            f"processing_time={task.processing_time}"
        )


        # ROS 2 String 메시지 생성
        msg = String()

        msg.data = result


        # FMS로 Publish
        self.task_publisher.publish(msg)


        self.get_logger().info(
            f"FMS로 Task 결과 전송: {result}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = AssemblyNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()
