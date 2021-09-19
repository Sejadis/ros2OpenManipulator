import time
import rclpy
from rclpy.node import Node
from enum import Enum
from planing_interfaces.action import ActionPlan, MovementAction, SetWorldState, SetMovementAction


class WorldActionType(Enum):
    GRAB = 1
    RELEASE = 2


class WorldAction:

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
        # print(self.max_name_length)

    def stacks(self):
        return len(self.state)

    def stack_size(self, stack: int):
        return len(self.state[stack])

    def is_top_level_object(self, name):
        for i in range(self.stacks()):
            stack_size = self.stack_size(i)
            if self.state[i][stack_size - 1] == name:
                return True, i

        # none of the top objects on any stack match the requested object
        return False, None

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
                # print('at level %d with stack having length %d' % (stack_level, self.stack_size(stack_index)))
                if self.stack_size(stack_index) - 1 >= stack_level:
                    # print(" %d >= %d" % (self.stack_size(stack_index), stack_level))
                    current_element += stack[stack_level]
                current_element = current_element.ljust(self.max_name_length + 2)
                current_element += '|'
                output += current_element
            output += '\n'
        return output


class World(Node):
    active_planning_action_completed = True
    active_movement_action_completed = True

    def __init__(self, world_state: WorldState):
        super(World, self).__init__("world")
        self.world_state = world_state
        # action server requested world state
        self.world_action_server = ActionServer(self,
                                                MovementAction,
                                                'world_action',
                                                self.world_action_callback)
        self.world_state_server = ActionServer(self,
                                               SetWorldState,
                                               'world_state',
                                               self.world_state_callback)
        # action client movement
        self.movement_client = ActionClient(self,
                                            SetMovementAction,
                                            'movement_action')
        # action client planning or service?
        self.planning_client = ActionClient(self,
                                            ActionPlan,
                                            'action_plan')
        self.world_state_publisher = self.create_publisher(WorldState,
                                                           'world_state',
                                                           10)

    def world_action_callback(self, goal_handle):
        # we got a specific action, no need for a plan, directly call movement
        # split into 2 parts:
        # move to target object and grab
        # move to target position and release
        self.get_logger().info('received request for action: %s to %s' %
                               (goal_handle.request.object,
                                goal_handle.request.target))
        if not self.active_movement_action_completed:
            self.get_logger().info('There is still an active movement action running, rejecting new request')
            goal_handle.abort()
        goal_handle.execute()
        self.active_movement_action_completed = False
        is_top_level, stack = self.world_state.is_top_level_object(goal_handle.request.object)
        if not is_top_level:
            goal_handle.abort()
            return
        current_stack_size = self.world_state.stack_size(stack)
        self.request_movement(WorldActionType.GRAB, current_stack_size - 1, stack)
        while not self.active_movement_action_completed:
            time.sleep(0.5)
        self.active_movement_action_completed = False
        target_stack_size = self.world_state.stack_size(goal_handle.request.target_stack)
        self.request_movement(WorldActionType.RELEASE, target_stack_size, goal_handle.request.target_stack)
        while not self.active_movement_action_completed:
            time.sleep(0.5)
        goal_handle.succeed()
        return

    def request_movement(self, action_type: WorldActionType, target_level, target_stack):
        goal = SetMovementAction.Goal()
        goal.type = action_type
        goal.level = target_level
        goal.target_stack = target_stack
        self.movement_client.wait_for_server()
        future = self.movement_client.send_goal_async(goal)
        future.add_done_callback(self.movement_goal_callback)

    def movement_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('movement goal rejected')
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self.movement_result_callback)

    def movement_result_callback(self, future):
        result = future.result()
        self.get_logger().info('movement result received')
        self.active_movement_action_completed = True

    def world_state_callback(self, goal_handle):
        self.get_logger().info('received request for state %s' % goal_handle.request.target_state)
        if not self.active_planning_action_completed:
            self.get_logger().info('There is still an active planning action running, rejecting new request')
            goal_handle.abort()
        goal_handle.execute()
        self.active_planning_action_completed = False
        self.request_plan(goal_handle.request.target_state)
        while not self.active_planning_action_completed:
            time.sleep(0.5)
        goal_handle.succeed()
        return

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
            self.get_logger().info('planning goal rejected')
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future):
        result = future.result()
        plan = result.plan
        self.get_logger().info('received plan with %d steps' % len(plan))
        self.active_planning_action_completed = True
        # make move request


def main(args=None):
    rclpy.init(args=args)
    # default world state
    world_state = WorldState([["blue", "black"], ["yellow", "grey"], ["white"]])
    world = World(world_state)

    rclpy.spin(world)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
