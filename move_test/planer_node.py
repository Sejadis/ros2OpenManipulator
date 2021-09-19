import rclpy
from rclpy.node import Node
from planing_interfaces.msg import WorldState
from planing_interfaces.action import ActionPlan


def calculate_plan(initial_state: WorldState, target_state: WorldState):
    return []


class Planer(Node):

    def __init__(self):
        super().__init__('planer')
        self.publisher = self.create_publisher(
            WorldState,
            'target_state'
            , 10)
        self.plan_request_action_server = ActionServer(self,
                                                       ActionPlan,
                                                       'action_plan',
                                                       self.plan_request_callback)

    def plan_request_callback(self, goal_handle):
        self.get_logger().info('Received plan request')
        goal_handle.succeed()

        result = ActionPlan.Result()
        result.plan = calculate_plan(goal_handle.request.initial_state, goal_handle.request.target_state)
        return result

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
