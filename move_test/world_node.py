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


class WorldState():

    def __init__(self, state):
        self.state = state


class World(Node):

    def __init__(self, world_state: WorldState):
        super(World, self).__init__("world")
        self.world_state = world_state
        #action server requested world state
        #action client movement
        #action client planning
        #publisher world state

    def world_action_callback(self):
        # do stuff

    def world_state_callback(self):
        #do other stuff

    def request_plan(self, target_state: WorldState):
        goal = ActionPlan.Goal()
        goal.initial_state = self.world_state
        goal.target_state = target_state
        future = self.planning_client.call_async(goal)
        future.add_done_callback(lambda : print('plan received'))

def main(args=None):
    rclpy.init(args=args)
    #default world state
    world_state = WorldState([["blue", "black"], ["yellow"], ["white"]])
    world = World(world_state)

    rclpy.spin(world)
    rclpy.shutdown()


if __name__ == '__main__':
    main()