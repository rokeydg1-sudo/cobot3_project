import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .assembly_cell import AssemblyCell


class AssemblyNode(Node):

    # ========================================================
    # FMS 요청 재전송 주기
    #
    # 부품이 도착할 때까지 같은 Task를
    # 2초마다 다시 요청
    # ========================================================

    REQUEST_RETRY_INTERVAL = 2.0


    def __init__(self):

        super().__init__("assembly_node")


        # ====================================================
        # ROS2 Publisher
        #
        # Assembly -> FMS
        # ====================================================

        self.request_publisher = self.create_publisher(
            String,
            "/assembly/request",
            10
        )


        # ====================================================
        # ROS2 Subscriber
        #
        # Area Detection -> Assembly
        # ====================================================

        self.arrival_subscriber = self.create_subscription(
            String,
            "/assembly/part_arrived",
            self.part_arrived_callback,
            10
        )


        # ====================================================
        # Assembly Cell 생성
        # ====================================================

        self.cells = {
            "A": AssemblyCell(cell_id="A"),
            "B": AssemblyCell(cell_id="B"),
            "C": AssemblyCell(cell_id="C"),
        }


        # ====================================================
        # 마지막으로 요청을 보낸 시간
        #
        # 기존 request_sent=True/False 방식 제거
        #
        # 이제:
        #
        # WAITING_FOR_PART 상태라면
        # 일정 시간마다 요청을 다시 보냄
        # ====================================================

        self.last_request_time = {
            "A": -self.REQUEST_RETRY_INTERVAL,
            "B": -self.REQUEST_RETRY_INTERVAL,
            "C": -self.REQUEST_RETRY_INTERVAL,
        }


        # ====================================================
        # 현재 요청 중인 Task ID
        #
        # Queue의 맨 앞 Task가 바뀌었는지 확인하기 위함
        # ====================================================

        self.last_request_task_id = {
            "A": None,
            "B": None,
            "C": None,
        }


        # ====================================================
        # FMS 연결 대기 로그 중복 방지
        # ====================================================

        self.waiting_for_fms_logged = False


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
            f"FMS retry interval : "
            f"{self.REQUEST_RETRY_INTERVAL:.1f}s"
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


                # 새로운 Task가 Queue 뒤에 자동 추가됨
                cell.print_queue()


                # =================================================
                # Queue의 맨 앞 Task가 바뀌었으므로
                # 요청 기록 초기화
                # =================================================

                self.last_request_task_id[
                    cell_id
                ] = None


                self.last_request_time[
                    cell_id
                ] = (
                    self.simulation_time
                    - self.REQUEST_RETRY_INTERVAL
                )


            # =================================================
            # WAITING_FOR_PART
            #
            # 필요한 부품이 아직 도착하지 않았다면
            # FMS에 요청
            # =================================================

            if (
                cell.state
                == cell.WAITING_FOR_PART
            ):

                self.check_and_send_request(
                    cell_id,
                    cell
                )


    # ========================================================
    # FMS 요청 여부 확인
    # ========================================================

    def check_and_send_request(
        self,
        cell_id,
        cell,
    ):

        task = cell.get_next_task()


        if task is None:

            return


        # ====================================================
        # 새로운 Task가 Queue 맨 앞으로 온 경우
        #
        # 즉시 요청할 수 있도록 시간 초기화
        # ====================================================

        if (
            self.last_request_task_id[cell_id]
            != task.task_id
        ):

            self.last_request_task_id[
                cell_id
            ] = task.task_id


            self.last_request_time[
                cell_id
            ] = (
                self.simulation_time
                - self.REQUEST_RETRY_INTERVAL
            )


        # ====================================================
        # 아직 재전송 시간이 안 됐다면 대기
        # ====================================================

        elapsed = (
            self.simulation_time
            - self.last_request_time[cell_id]
        )


        if (
            elapsed
            < self.REQUEST_RETRY_INTERVAL
        ):

            return


        # ====================================================
        # FMS Subscriber 연결 확인
        #
        # FMS가 아직 /assembly/request에 붙지 않았다면
        # 메시지를 보내지 않음
        #
        # 기존 코드의 핵심 문제였던
        # "FMS가 준비되기 전에 한 번 보내고 끝"
        # 상황 방지
        # ====================================================

        subscriber_count = (
            self.request_publisher
            .get_subscription_count()
        )


        if subscriber_count == 0:

            if not self.waiting_for_fms_logged:

                self.get_logger().warning(
                    "[FMS WAIT] "
                    "/assembly/request subscriber가 "
                    "아직 없습니다. "
                    "FMS 연결을 기다립니다."
                )

                self.waiting_for_fms_logged = True


            return


        # FMS 연결 확인됨
        self.waiting_for_fms_logged = False


        # ====================================================
        # 실제 FMS 요청
        # ====================================================

        self.send_request_to_fms(
            cell_id,
            cell
        )


        # 마지막 전송 시간 기록
        self.last_request_time[
            cell_id
        ] = self.simulation_time


    # ========================================================
    # FMS로 부품 요청
    # ========================================================

    def send_request_to_fms(
        self,
        cell_id,
        cell,
    ):

        # Queue 맨 앞 Task
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


        self.request_publisher.publish(
            msg
        )


        self.get_logger().info(
            f"[FMS REQUEST] "
            f"Cell {cell_id} -> "
            f"{task.shape} "
            f"(Task {task.task_id})"
        )


    # ========================================================
    # AMR 부품 도착 Callback
    # ========================================================

    def part_arrived_callback(
        self,
        msg,
    ):

        # ====================================================
        # 예상 메시지
        #
        # cell_id=A
        # ====================================================

        data = msg.data.strip()


        # ====================================================
        # cell_id 추출
        # ====================================================

        cell_id = None


        for item in data.split(","):

            key_value = item.split(
                "=",
                1,
            )


            if len(key_value) != 2:

                continue


            key = (
                key_value[0]
                .strip()
            )

            value = (
                key_value[1]
                .strip()
            )


            if key == "cell_id":

                cell_id = value


        # ====================================================
        # Cell ID 확인
        # ====================================================

        if cell_id is None:

            self.get_logger().warning(
                f"잘못된 도착 메시지: "
                f"{msg.data}"
            )

            return


        if cell_id not in self.cells:

            self.get_logger().warning(
                f"존재하지 않는 Cell: "
                f"{cell_id}"
            )

            return


        cell = self.cells[
            cell_id
        ]


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
                f"{task.processing_time:.0f}초 "
                f"작업 시작"
            )


            # =================================================
            # PROCESSING 상태가 되었으므로
            # update()에서 더 이상 FMS 요청을 보내지 않음
            # =================================================


        else:

            self.get_logger().warning(
                f"Cell {cell_id} - "
                f"부품 도착 이벤트 처리 실패"
            )


def main(args=None):

    rclpy.init(
        args=args
    )


    node = AssemblyNode()


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()