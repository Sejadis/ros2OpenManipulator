import rclpy
from rclpy.action import ActionServer
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
        print('Planer Node running')

    def plan_request_callback(self, goal_handle):
        self.get_logger().info('Received plan request')

        result = ActionPlan.Result()
        result.plan = calculate_plan(goal_handle.request.initial_state, goal_handle.request.target_state)
        goal_handle.succeed()
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

    rclpy.spin(planer)

    rclpy.shutdown()
    # print('Hi from move_test.')


if __name__ == '__main__':
    main()
