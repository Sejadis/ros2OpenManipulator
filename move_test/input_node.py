import rclpy
from rclpy.node import Node
import ast
from .world_node import WorldState

class InputHandler(Node):

    def __init__(self):
        super(InputHandler, self).__init__('input')
        self.world_action_client = self.create_action_client(...)
        self.world_state_client = self.create_action_client(...)

    def set_target_action(self, obj, target):
        goal = TargetAction.Goal()
        goal.object = obj
        goal.target = target
        self.world_action_client.call_async(goal)

    def set_target_state(self, state: WorldState):
        goal = SetWorldState.Goal()
        goal.target_state = state
        self.world_state_client.call_async(goal)


def main(args=None):
    rclpy.init(args=args)
    input_handler = InputHandler()

    print(
        "Usage: <position> <level>\n"
        "<position>: [ l | m | r ] for [ left | middle right ] \n"
        "<level>: integer [ 0 - 2 ] for height, 0 being ground level")
    input_line = input("--> ")
    (operator, data) = parse_input(input_line)

    while rclpy.ok() and operator:

        if operator == "set":
            data = ast.literal_eval(args[1])
            input_handler.set_target_state(data)
        elif operator == "do":

            input_handler.set_target_action(data[0], data[1])
        else:
            input_handler.get_logger().info("Invalid operator")
        input_line = input("--> ")
        (operator, data) = parse_input(input_line)

    rclpy.shutdown()
    # print('Hi from move_test.')


def parse_input(input):
    args = input.split()
    if len(args) >= 2:
        operator = args[0]
        data = args[1:] if len(args) > 2 else args[1]
        return operator, data
    return None, None


if __name__ == '__main__':
    main()