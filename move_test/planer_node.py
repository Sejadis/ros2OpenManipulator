import string
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

    def publish(self, left, middle, right, level):
        msg = WorldState()
        msg.left = left
        msg.middle = middle
        msg.right = right
        msg.level = level
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    planer = Planer()

    """
    left = sys.argv[1]
    middle = sys.argv[2]
    right = sys.argv[3]


    planer.get_logger().info('left: %s ' % left)
    planer.get_logger().info('middle: %s ' % middle)
    planer.get_logger().info('right: %s ' % right)

    planer.publish(left, middle, right, 0)
    """
    print(
        "Usage: <position> <level>\n"
        "<position>: [ l | m | r ] for [ left | middle right ] \n"
        "<level>: integer [ 0 - 2 ] for height, 0 being ground level")
    input_line = input("--> ")
    # rclpy.spin_once(planer)

    while rclpy.ok() and input_line:
        # print('got input %s' % input_line)
        args = input_line.split()
        pos = args[0]
        level = int(args[1])
        if pos == "l":
            planer.publish("a", "0", "0", level)
        if pos == "m":
            planer.publish("0", "a", "0", level)
        if pos == "r":
            planer.publish("0", "0", "a", level)
        # rclpy.spin_once()
        input_line = input("--> ")

    rclpy.shutdown()
    # print('Hi from move_test.')


if __name__ == '__main__':
    main()
