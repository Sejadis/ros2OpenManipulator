import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from planing_interfaces.msg import WorldState
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from open_manipulator_msgs.msg import OpenManipulatorState, KinematicsPose
from planing_interfaces.action import SetMovementAction

from move_test.world_node import WorldActionType


class Point(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

    def set_from_values(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def set_from_point(self, p):
        self.x = p.x
        self.y = p.y
        self.z = p.z


class Mover(Node):

    def __init__(self):
        super().__init__("mover")
        self.kinematics_pose: KinematicsPose = None
        self.max_z = 0.25
        self.z_levels = [0.04, 0.085, 0.13]
        self.path_time = 3.0
        self.data_group = MutuallyExclusiveCallbackGroup()
        self.work_group = MutuallyExclusiveCallbackGroup()
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10,
            callback_group=self.data_group
        )
        self.manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.manipulator_state_callback,
            10,
            callback_group=self.data_group
        )
        self.target_state_subscription = self.create_subscription(
            WorldState,
            'target_state',
            self.target_state_callback,
            10,
            callback_group=self.data_group
        )
        self.movement_server = ActionServer(self,
                                            SetMovementAction,
                                            'movement_action',
                                            self.movement_action_callback,
                                            callback_group=self.work_group)
        self.kinematic_pose_client = self.create_client(SetKinematicsPose, 'goal_task_space_path_position_only')
        while not self.kinematic_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kinematics service')
        self.tool_client = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for tool service')

        self.left_position = Point()
        self.middle_position = Point()
        self.right_position = Point()

        self.left_position.set_from_values(0.25, -0.1, 0.04)
        self.middle_position.set_from_values(0.25, 0.0, 0.04)
        self.right_position.set_from_values(0.25, 0.1, 0.04)

        self.stacks = [self.left_position, self.middle_position, self.right_position]

        self.kinematics_future = None
        self.tool_future = None
        self.grabber_open: bool = False
        self.is_moving = False
        self.queue = []

        print('Mover node started')

    def movement_action_callback(self, goal_handle):
        target_stack = self.stacks[goal_handle.request.target_stack]
        grabber_target_state = True if goal_handle.request.type == WorldActionType.RELEASE else False
        self.synthesize_movement(target_stack, goal_handle.request.level, grabber_target_state)
        while not len(self.queue) == 0:
            time.sleep(0.5)
        goal_handle.succeed()
        print('succeeded')
        response = SetMovementAction.Result()
        return response

    def kinematics_pose_callback(self, msg: KinematicsPose):
        self.kinematics_pose = msg

    def manipulator_state_callback(self, msg: OpenManipulatorState):
        if msg.open_manipulator_moving_state == OpenManipulatorState.IS_MOVING:
            self.is_moving = True
        elif self.is_moving:
            self.is_moving = False
            print('stopped moving, continue to next item on plan')
            if len(self.queue) > 0:
                self.queue.pop(0)()
                print('%d items left in queue ' % len(self.queue))

    def send_pose_request(self, position: Point):
        print('requested pose is (%s, %s, %s)' % (position.x, position.y, position.z))

        request = SetKinematicsPose.Request()
        request.end_effector_name = 'gripper'
        request.kinematics_pose.pose.position.x = position.x
        request.kinematics_pose.pose.position.y = position.y
        request.kinematics_pose.pose.position.z = position.z
        request.path_time = self.path_time
        self.kinematics_future = self.kinematic_pose_client.call_async(request)

    def set_grabber_state(self, should_open: bool):
        request = SetJointPosition.Request()
        request.path_time = self.path_time
        request.joint_position.joint_name = ['gripper']
        request.joint_position.position = [0.01] if should_open else [0.005]
        self.grabber_open = not self.grabber_open
        self.tool_future = self.tool_client.call_async(request)

    def target_state_callback(self, msg):
        self.get_logger().info("Received target state %s" % msg)
        if msg.left == 'a':
            print('left recognized')
            position = self.left_position
        if msg.middle == 'a':
            print('middle recognized')
            position = self.middle_position
        if msg.right == 'a':
            print('right recognized')
            position = self.right_position

        self.synthesize_movement(position, msg.level, not self.grabber_open)

    def synthesize_movement(self, target_position, level, target_grabber_state):
        target_point = Point()
        target_point.set_from_point(target_position)
        target_point.z = self.z_levels[level]
        self.queue.clear()
        # move upwards at current position first, to avoid collision
        current_clear_position = Point()
        current_clear_position.set_from_values(
            self.kinematics_pose.pose.position.x,
            self.kinematics_pose.pose.position.y,
            self.max_z
        )
        self.send_pose_request(current_clear_position)
        target_clear_position = Point()
        target_clear_position.set_from_point(target_point)
        target_clear_position.z = self.max_z
        # enqueue top most position for target pos to avoid collision
        self.queue.append(lambda: self.send_pose_request(target_clear_position))
        # enqueue target point
        self.queue.append(lambda: self.send_pose_request(target_point))
        # enqueue grabber
        self.queue.append(lambda: self.set_grabber_state(target_grabber_state))


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    mover = Mover()
    executor.add_node(mover)
    executor.spin()
    # rclpy.spin(mover)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
