import sys

import rclpy
from rclpy.node import Node
from planing_interfaces.msg import WorldState


class Planer(Node):

    def __init__(self):
        super().__init__('planer')
        self.publisher = self.create_publisher(
            WorldState,
            'target_state'
            , 10)

    def publish(self, left, middle, right):
        msg = WorldState()
        msg.left = left
        msg.middle = middle
        msg.right = right
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    left = sys.argv[1]
    middle = sys.argv[2]
    right = sys.argv[3]

    planer = Planer()

    planer.get_logger().info('left: %s ' % left)
    planer.get_logger().info('middle: %s ' % middle)
    planer.get_logger().info('right: %s ' % right)

    planer.publish(left, middle, right)

    rclpy.spin_once(planer)

    rclpy.shutdown()
    # print('Hi from move_test.')


if __name__ == '__main__':
    main()
