#!/usr/bin/env python3
"""Simulate the door detector service used by Check and GPSR startup."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DoorSim(Node):
    def __init__(self):
        super().__init__('door_sim')
        self.declare_parameter('door_open', True)
        self.closed_checks = self.declare_parameter('closed_checks_before_open', 0).value
        if self.closed_checks < 0:
            raise ValueError('closed_checks_before_open must be nonnegative')
        self.checks = 0
        self.service = self.create_service(Trigger, '/hardware/door_detector/check', self.check)
        self.get_logger().info(
            f'Simulated door ready: closed_checks_before_open={self.closed_checks}; '
            'door_open can be changed with ros2 param set'
        )

    def check(self, request, response):
        self.checks += 1
        opened = self.get_parameter('door_open').value and self.checks > self.closed_checks
        response.success = opened
        response.message = 'Door is open (simulated)' if opened else 'Door is closed (simulated)'
        self.get_logger().info(f'check={self.checks}: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DoorSim()
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
