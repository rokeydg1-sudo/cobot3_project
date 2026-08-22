import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .assembly_cell import AssemblyCell


class AssemblyNode(Node):

    def __init__(self):

        super().__init__("assembly_node")


        # ====================================================
        # ROS2 Publisher
        #
        # Assembly -> FMS
        # "Cell A에 STAR 부품이 필요합니다."
        # ====================================================

        self.request_publisher = self.create_publisher(
            String,
            "/assembly/request",
            10
        )


        # ====================================================
        # ROS2 Subscriber
        #
        # Area Detection Node -> Assembly
        #
        # 예:
        # cell_id=A
        # ====================================================

        self.arrival_subscriber = self.create_subscription(
            String,
            "/assembly/part_arrived",
            self.part_arrived_callback,
            10
        )


        # ====================================================
        # Assembly Cell 생성
        #
        # AssemblyCell 생성 시 각각 랜덤 Task 3개 자동 생성
        # ====================================================

        self.cells = {
            "A": AssemblyCell(cell_id="A"),
            "B": AssemblyCell(cell_id="B"),
            "C": AssemblyCell(cell_id="C"),
        }


        # ====================================================
        # 각 Cell의 현재 Task를
        # FMS에 이미 요청했는지 저장
        #
        # False = 아직 요청 안 함
        # True  = 이미 요청함
        # ====================================================

        self.request_sent = {
            "A": False,
            "B": False,
            "C": False,
        }


        # ====================================================
        # Simulation Time
        # ====================================================

        self.simulation_time = 0.0

        self.dt = 0.1


        # ====================================================
        # 0.1초마다 Assembly 상태 업데이트
        # ====================================================

        self.timer = self.create_timer(
            self.dt,
            self.update
        )


        # ====================================================
        # 시작 로그
        # ====================================================

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "Assembly Node started"
        )

        self.get_logger().info(
            "================================="
        )


        # 처음 생성된 Queue 출력
        for cell in self.cells.values():
            cell.print_queue()


    # ========================================================
    # Main Update
    # ========================================================

    def update(self):

        # Simulation Time 증가
        self.simulation_time += self.dt


        # ====================================================
        # Cell A / B / C 순회
        # ====================================================

        for cell_id, cell in self.cells.items():

            # =================================================
            # Cell 상태 업데이트
            #
            # PROCESSING이면 작업 시간 체크
            # WAITING이면 그냥 대기
            # =================================================

            completed_task = cell.update(
                self.simulation_time
            )


            # =================================================
            # 작업 완료
            # =================================================

            if completed_task is not None:

                self.get_logger().info(
                    f"Cell {cell_id} - "
                    f"Task {completed_task.task_id} "
                    f"({completed_task.shape}) 완료"
                )


                # AssemblyCell 내부에서
                # 이미 새로운 랜덤 Task가 생성되어
                # Queue는 다시 3개가 되어 있음

                cell.print_queue()


                # =================================================
                # 이제 Queue 맨 앞이 새로운 현재 작업이므로
                # FMS에 다시 요청할 수 있도록 초기화
                # =================================================

                self.request_sent[cell_id] = False


            # =================================================
            # WAITING_FOR_PART 상태라면
            # 필요한 부품을 FMS에 요청
            # =================================================

            if (
                cell.state == cell.WAITING_FOR_PART
                and not self.request_sent[cell_id]
            ):

                self.send_request_to_fms(
                    cell_id,
                    cell
                )


    # ========================================================
    # FMS로 부품 요청
    # ========================================================

    def send_request_to_fms(self, cell_id, cell):

        # Queue 맨 앞 Task 확인
        task = cell.get_next_task()


        if task is None:
            return


        # ====================================================
        # FMS로 보낼 메시지
        #
        # 예:
        #
        # cell_id=A,
        # task_id=1,
        # kit_id=KIT_STAR,
        # shape=STAR,
        # processing_time=3.0
        # ====================================================

        request = (
            f"cell_id={cell_id},"
            f"task_id={task.task_id},"
            f"kit_id={task.kit_id},"
            f"shape={task.shape},"
            f"processing_time={task.processing_time}"
        )


        msg = String()

        msg.data = request


        self.request_publisher.publish(msg)


        # 중복 요청 방지
        self.request_sent[cell_id] = True


        self.get_logger().info(
            f"[FMS REQUEST] "
            f"Cell {cell_id} -> "
            f"{task.shape} "
            f"(Task {task.task_id})"
        )


    # ========================================================
    # AMR 부품 도착 Callback
    # ========================================================

    def part_arrived_callback(self, msg):

        # ====================================================
        # 예상 메시지
        #
        # cell_id=A
        #
        # 나중에는:
        #
        # cell_id=A,amr_id=AMR_1
        #
        # 같은 형식으로 확장 가능
        # ====================================================

        data = msg.data.strip()


        # ====================================================
        # 문자열에서 cell_id 추출
        # ====================================================

        cell_id = None


        for item in data.split(","):

            key_value = item.split("=")

            if len(key_value) != 2:
                continue


            key = key_value[0].strip()
            value = key_value[1].strip()


            if key == "cell_id":
                cell_id = value


        # ====================================================
        # Cell ID 확인
        # ====================================================

        if cell_id is None:

            self.get_logger().warning(
                f"잘못된 도착 메시지: {msg.data}"
            )

            return


        if cell_id not in self.cells:

            self.get_logger().warning(
                f"존재하지 않는 Cell: {cell_id}"
            )

            return


        cell = self.cells[cell_id]


        # ====================================================
        # AMR 도착 처리
        # ====================================================

        success = cell.part_arrived(
            self.simulation_time
        )


        if success:

            task = cell.current_task


            self.get_logger().info(
                f"[PART ARRIVED] "
                f"Cell {cell_id} - "
                f"{task.shape} 부품 도착"
            )

            self.get_logger().info(
                f"[PROCESSING] "
                f"Cell {cell_id} - "
                f"{task.processing_time:.0f}초 작업 시작"
            )


        else:

            self.get_logger().warning(
                f"Cell {cell_id} - "
                f"부품 도착 이벤트 처리 실패"
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


if __name__ == "__main__":
    main()