import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from interfaces.action import ExecuteMission


class DemoFMSNode(Node):

    def __init__(self):
        super().__init__('demo_fms_node')

        self.subscription = self.create_subscription(
            String,
            '/assembly/request',
            self.request_callback,
            10
        )

        self.action_client = ActionClient(
            self,
            ExecuteMission,
            '/amr/execute_mission'
        )

        self.get_logger().info('Demo FMS Node started')
        self.get_logger().info('Waiting for /assembly/request ...')


    def request_callback(self, msg):

        self.get_logger().info(
            f'Assembly request received: {msg.data}'
        )

        if msg.data.strip().lower() != 'need':
            self.get_logger().warning(
                f'Unknown request: {msg.data}'
            )
            return

        target_cells = random.sample(
            ['A', 'B', 'C'],
            2
        )

        route = []

        for cell in target_cells:
            route.append('SP')
            route.append(cell)

        self.get_logger().info(
            f'Random targets selected: {target_cells}'
        )

        self.get_logger().info(
            f'Mission created: {" -> ".join(route)}'
        )

        self.send_mission(route)


    def send_mission(self, route):

        self.get_logger().info(
            'Waiting for AMR Action Server...'
        )

        self.action_client.wait_for_server()

        goal_msg = ExecuteMission.Goal()

        goal_msg.route = route

        self.get_logger().info(
            f'Sending mission to AMR: {" -> ".join(route)}'
        )

        self.send_goal_future = (
            self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
        )

        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )


    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning(
                'AMR rejected mission'
            )
            return

        self.get_logger().info(
            'AMR accepted mission'
        )

        self.result_future = (
            goal_handle.get_result_async()
        )

        self.result_future.add_done_callback(
            self.result_callback
        )


    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'AMR Feedback: {feedback.status}'
        )


    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info(
            f'AMR Result: success={result.success}'
        )

        self.get_logger().info(
            f'AMR Message: {result.message}'
        )


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