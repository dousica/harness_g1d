import argparse
import math
import os
import sys

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node


MAX_OPENING_M = 0.09


class GripperClient(Node):
    def __init__(self, side):
        super().__init__('lzh_gripper_client')
        self._client = ActionClient(
            self,
            GripperCommand,
            f'/lzh_{side}_dex1/gripper_command',
        )

    def send(self, opening_m):
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f'Action server unavailable for {self.get_name()}'
            )

        goal = GripperCommand.Goal()
        goal.command.position = opening_m
        goal.command.max_effort = 0.0

        goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback,
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Gripper goal was rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError('No result received from gripper Action')

        result = wrapped_result.result
        self.get_logger().info(
            f'Action finished: reached_goal={result.reached_goal}, '
            f'position={result.position:.4f} m'
        )
        return wrapped_result.status == 4 and result.reached_goal

    def _feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f'feedback: position={feedback.position:.4f} m, '
            f'reached_goal={feedback.reached_goal}, '
            f'stalled={feedback.stalled}'
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Send a G1-D Dex1-1 gripper opening goal.'
    )
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument(
        '--opening', type=float, required=True, metavar='METERS',
        help='total opening distance, from 0.0 (closed) to 0.09 (open)',
    )
    parser.add_argument(
        '--execute', action='store_true',
        help='send the Action goal; without this, only validate and print it',
    )
    return parser.parse_args(argv)


def main(args=None):
    cli = parse_args(args)
    if not math.isfinite(cli.opening) or not 0.0 <= cli.opening <= MAX_OPENING_M:
        print('opening must be finite and between 0.0 and 0.09 meters', file=sys.stderr)
        return 2

    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if domain_id != '42':
        print(
            f'ROS_DOMAIN_ID must be 42 (current: {domain_id or "unset"}); '
            'source ~/g1d_ros2_lzh/setup_env.sh first',
            file=sys.stderr,
        )
        return 2

    action_name = f'/lzh_{cli.side}_dex1/gripper_command'
    print(
        f'Validated target: side={cli.side}, opening={cli.opening:.4f} m, '
        f'action={action_name}, ROS_DOMAIN_ID=42'
    )
    if not cli.execute:
        print('Dry check only; no ROS Action goal was sent. Add --execute to send.')
        return 0

    confirmation = (
        f'SEND {cli.side.upper()} GRIPPER {cli.opening:.4f}M'
    )
    print('This will command the physical gripper if a real-execution server is active.')
    try:
        typed = input(f'Type exactly "{confirmation}" to continue: ')
    except (EOFError, KeyboardInterrupt):
        print('\nCancelled; no goal sent.')
        return 1
    if typed != confirmation:
        print('Confirmation did not match; no goal sent.')
        return 1

    rclpy.init(args=None)
    node = GripperClient(cli.side)
    try:
        succeeded = node.send(cli.opening)
        return 0 if succeeded else 1
    except (RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
