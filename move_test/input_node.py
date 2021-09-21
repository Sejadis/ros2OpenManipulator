import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import ast
from .world_node import WorldState
from planing_interfaces.action import MovementAction, SetWorldState
from planing_interfaces.msg import StackData


class InputHandler(Node):

    def __init__(self):
        super(InputHandler, self).__init__('input')
        self.world_action_client = ActionClient(self,
                                                MovementAction,
                                                'world_action')
        self.world_state_client = ActionClient(self,
                                               SetWorldState,
                                               'world_action')

    def set_target_action(self, obj, target):
        goal = MovementAction.Goal()
        goal.object = obj
        goal.target_stack = target
        self.world_action_client.send_goal_async(goal)

    def set_target_state(self, state):
        goal = SetWorldState.Goal()
        goal.target_state = state
        self.world_state_client.send_goal_async(goal)


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
            input_handler.set_target_action(data[0], int(data[1]))
        elif operator == "state":
            state = None
            if data == "default":
                state = default_state
                # state = WorldState(default_state)
            elif data == "1":
                state = WorldState(custom_state_1)
            elif data == "2":
                state = WorldState(custom_state_2)
            elif data == "3":
                state = WorldState(custom_state_3)
            input_handler.set_target_state(state)
        else:
            input_handler.get_logger().info("Invalid operator")
        input_line = input("--> ")
        (operator, data) = parse_input(input_line)

    rclpy.shutdown()


default_state = [["blue", "black"], ["yellow", "grey"], ["white"]]
custom_state_1 = [["black", "blue"], ["yellow", "grey"], ["white"]]
custom_state_2 = [["black", "blue", "white"], [], ["yellow", "grey"]]
custom_state_3 = [["yellow", "blue"], ["black", "grey"], ["white"]]


def parse_input(user_input):
    args = user_input.split()
    if len(args) >= 2:
        operator = args[0]
        data = args[1:] if len(args) > 2 else args[1]
        return operator, data
    return None, None


if __name__ == '__main__':
    main()
