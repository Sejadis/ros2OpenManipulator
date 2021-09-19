import rclpy
from rclpy.node import Node
from enum import Enum

class WorldActionType(Enum):
    GRAB = 1
    RELEASE = 2


class WorldAction():

    def __init__(self, from_stack: int, to_stack: int, action_type: WorldActionType):
        self.from_stack = from_stack
        self.to_stack = to_stack
        self.action_type = action_type


class WorldState:

    def __init__(self, state):
        self.state = state
        self.max_name_length = 0
        for i in range(len(self.state)):
            for j in range(len(self.state[i])):
                if len(self.state[i][j]) > self.max_name_length:
                    self.max_name_length = len(self.state[i][j])
        #print(self.max_name_length)

    def stacks(self):
        return len(self.state)

    def stack_size(self, stack: int):
        return len(self.state[stack])

    def __str__(self):
        max_stack_size = 0
        stack_count = self.stacks()
        for stack_index in range(stack_count):
            if max_stack_size < self.stack_size(stack_index):
                max_stack_size = self.stack_size(stack_index)
        output = ''
        for stack_level in reversed(range(max_stack_size)):
            for stack_index in range(stack_count):
                stack = self.state[stack_index]
                current_element = ' '
                #print('at level %d with stack having length %d' % (stack_level, self.stack_size(stack_index)))
                if self.stack_size(stack_index) -1 >= stack_level:
                    #print(" %d >= %d" % (self.stack_size(stack_index), stack_level))
                    current_element += stack[stack_level]
                current_element = current_element.ljust(self.max_name_length + 2)
                current_element += '|'
                output += current_element
            output += '\n'
        return output

class World(Node):

    def __init__(self, world_state: WorldState):
        super(World, self).__init__("world")
        self.world_state = world_state
        #action server requested world state
        #action client movement
        #action client planning or service?
        self.planning_client = ActionClient(self,ActionPlan, 'action_plan')
        self.world_state_publisher = self.create_publisher(WorldState, 'world_state',10)

    def world_action_callback(self):
        # do stuff

    def world_state_callback(self):
        #do other stuff

    def request_plan(self, target_state: WorldState):
        goal = ActionPlan.Goal()
        goal.initial_state = self.world_state
        goal.target_state = target_state
        self.planning_client.wait_for_server()
        future = self.planning_client.send_goal_async(goal)
        future.add_done_callback(self.plan_goal_callback)

    def plan_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future):
        result = future.result()
        plan = result.plan
        self.get_logger().info('received plan with %d steps' % len(plan))
        #make move request

def main(args=None):
    rclpy.init(args=args)
    #default world state
    world_state = WorldState([["blue", "black"], ["yellow", "grey"], ["white"]])
    world = World(world_state)

    rclpy.spin(world)
    rclpy.shutdown()


if __name__ == '__main__':
    main()