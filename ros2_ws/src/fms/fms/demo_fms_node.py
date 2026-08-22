from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from interfaces.action import ExecuteMission


class DemoFMSNode(Node):

    def __init__(self):

        super().__init__('demo_fms_node')


        # ====================================================
        # 테스트 설정
        #
        # 성공한 배송 5번까지만 수행
        # ====================================================

        self.max_deliveries = 5

        self.completed_deliveries = 0


        # ====================================================
        # Assembly 요청 Queue
        #
        # 먼저 들어온 요청부터 처리 (FIFO)
        # ====================================================

        self.request_queue = deque()


        # ====================================================
        # 현재 AMR이 Mission 수행 중인지 여부
        # ====================================================

        self.mission_active = False


        # ====================================================
        # 현재 수행 중인 요청
        # ====================================================

        self.current_request = None


        # ====================================================
        # 중복 Task 방지
        #
        # 같은 task_id가 실수로 여러 번 들어오는 경우
        # 한 번만 Queue에 넣기 위함
        # ====================================================

        self.received_tasks = set()


        # ====================================================
        # Assembly Request Subscriber
        # ====================================================

        self.subscription = self.create_subscription(
            String,
            '/assembly/request',
            self.request_callback,
            10
        )


        # ====================================================
        # AMR Action Client
        # ====================================================

        self.action_client = ActionClient(
            self,
            ExecuteMission,
            '/amr/execute_mission'
        )


        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'Demo FMS Node started'
        )

        self.get_logger().info(
            f'Max deliveries: {self.max_deliveries}'
        )

        self.get_logger().info(
            'Waiting for /assembly/request ...'
        )

        self.get_logger().info(
            '================================='
        )


    # ========================================================
    # Assembly Request 수신
    # ========================================================

    def request_callback(self, msg):

        # 이미 배송 5번 끝났으면 새로운 요청은 받지 않음
        if self.completed_deliveries >= self.max_deliveries:

            self.get_logger().info(
                '5번 배송 테스트 완료 - 추가 요청 무시'
            )

            return


        self.get_logger().info(
            f'[ASSEMBLY REQUEST] {msg.data}'
        )


        # ====================================================
        # 메시지 파싱
        #
        # 예:
        #
        # cell_id=A,
        # task_id=1,
        # kit_id=KIT_STAR,
        # shape=STAR,
        # processing_time=3.0
        # ====================================================

        request = self.parse_request(
            msg.data
        )


        if request is None:

            self.get_logger().warning(
                f'잘못된 Assembly 요청: {msg.data}'
            )

            return


        task_id = request['task_id']


        # ====================================================
        # 중복 Task 방지
        # ====================================================

        if task_id in self.received_tasks:

            self.get_logger().warning(
                f'Task {task_id} 중복 요청 무시'
            )

            return


        self.received_tasks.add(
            task_id
        )


        # ====================================================
        # FIFO Queue에 요청 저장
        # ====================================================

        self.request_queue.append(
            request
        )


        self.get_logger().info(
            f'[QUEUE ADD] '
            f'Cell {request["cell_id"]} / '
            f'{request["shape"]} / '
            f'Task {request["task_id"]}'
        )


        self.get_logger().info(
            f'Waiting requests: '
            f'{len(self.request_queue)}'
        )


        # ====================================================
        # AMR이 놀고 있다면 바로 다음 Mission 시작
        # ====================================================

        self.try_start_next_mission()


    # ========================================================
    # Assembly 메시지 파싱
    # ========================================================

    def parse_request(self, data):

        values = {}


        for item in data.split(','):

            if '=' not in item:
                continue


            key, value = item.split(
                '=',
                1
            )


            values[
                key.strip()
            ] = value.strip()


        # 반드시 필요한 값
        required_fields = [
            'cell_id',
            'task_id',
            'kit_id',
            'shape',
        ]


        for field in required_fields:

            if field not in values:
                return None


        # Cell 검증
        if values['cell_id'] not in [
            'A',
            'B',
            'C',
        ]:

            return None


        return values


    # ========================================================
    # 다음 Mission 실행
    # ========================================================

    def try_start_next_mission(self):

        # ----------------------------------------------------
        # 이미 5번 완료
        # ----------------------------------------------------

        if self.completed_deliveries >= self.max_deliveries:
            return


        # ----------------------------------------------------
        # 현재 AMR이 Mission 수행 중
        # ----------------------------------------------------

        if self.mission_active:
            return


        # ----------------------------------------------------
        # 대기 중인 Assembly 요청 없음
        # ----------------------------------------------------

        if not self.request_queue:

            self.get_logger().info(
                'FMS Queue empty - waiting for next request'
            )

            return


        # ====================================================
        # 가장 먼저 들어온 요청 꺼내기
        # ====================================================

        self.current_request = (
            self.request_queue.popleft()
        )


        cell_id = (
            self.current_request['cell_id']
        )


        # ====================================================
        # AMR은 Kit 하나만 운반 가능
        #
        # 따라서 Mission은 무조건
        #
        # SP -> Cell
        #
        # 형태
        # ====================================================

        route = [
            'SP',
            cell_id,
        ]


        self.mission_active = True


        self.get_logger().info(
            '---------------------------------'
        )

        self.get_logger().info(
            f'[MISSION {self.completed_deliveries + 1}'
            f'/{self.max_deliveries}]'
        )

        self.get_logger().info(
            f'Cell: {cell_id}'
        )

        self.get_logger().info(
            f'Part: {self.current_request["shape"]}'
        )

        self.get_logger().info(
            f'Task: {self.current_request["task_id"]}'
        )

        self.get_logger().info(
            f'Route: {" -> ".join(route)}'
        )

        self.get_logger().info(
            '---------------------------------'
        )


        self.send_mission(
            route
        )


    # ========================================================
    # AMR Mission 전송
    # ========================================================

    def send_mission(self, route):

        self.get_logger().info(
            'Waiting for AMR Action Server...'
        )


        self.action_client.wait_for_server()


        goal_msg = ExecuteMission.Goal()

        goal_msg.route = route


        self.get_logger().info(
            f'Sending mission to AMR: '
            f'{" -> ".join(route)}'
        )


        send_goal_future = (
            self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
        )


        send_goal_future.add_done_callback(
            self.goal_response_callback
        )


    # ========================================================
    # AMR Goal 응답
    # ========================================================

    def goal_response_callback(self, future):

        goal_handle = future.result()


        if not goal_handle.accepted:

            self.get_logger().warning(
                'AMR rejected mission'
            )


            self.mission_active = False

            self.current_request = None


            # 다음 요청 진행
            self.try_start_next_mission()

            return


        self.get_logger().info(
            'AMR accepted mission'
        )


        result_future = (
            goal_handle.get_result_async()
        )


        result_future.add_done_callback(
            self.result_callback
        )


    # ========================================================
    # AMR Feedback
    # ========================================================

    def feedback_callback(
        self,
        feedback_msg
    ):

        feedback = (
            feedback_msg.feedback
        )


        self.get_logger().info(
            f'AMR Feedback: '
            f'{feedback.status}'
        )


    # ========================================================
    # AMR Mission Result
    # ========================================================

    def result_callback(self, future):

        result = (
            future.result().result
        )


        self.get_logger().info(
            f'AMR Result: '
            f'success={result.success}'
        )


        self.get_logger().info(
            f'AMR Message: '
            f'{result.message}'
        )


        # ====================================================
        # Mission 성공
        # ====================================================

        if result.success:

            self.completed_deliveries += 1


            self.get_logger().info(
                f'[DELIVERY COMPLETE] '
                f'{self.completed_deliveries}'
                f'/{self.max_deliveries}'
            )


        else:

            self.get_logger().warning(
                'Mission failed - '
                'delivery count not increased'
            )


        # 현재 Mission 종료
        self.mission_active = False

        self.current_request = None


        # ====================================================
        # 5번 완료 확인
        # ====================================================

        if (
            self.completed_deliveries
            >= self.max_deliveries
        ):

            self.get_logger().info(
                '================================='
            )

            self.get_logger().info(
                'Demo FMS 테스트 완료'
            )

            self.get_logger().info(
                '총 5번 배송 완료'
            )

            self.get_logger().info(
                '================================='
            )

            return


        # ====================================================
        # Queue에 다음 요청이 있으면 바로 실행
        # ====================================================

        self.try_start_next_mission()


def main(args=None):

    rclpy.init(args=args)

    node = DemoFMSNode()


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