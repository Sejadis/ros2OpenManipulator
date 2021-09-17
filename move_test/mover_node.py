import rclpy
from rclpy.node import Node
from planing_interfaces.msg import WorldState
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from open_manipulator_msgs.msg import OpenManipulatorState, KinematicsPose
from rclpy.qos import QoSProfile


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
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            10
        )
        self.manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.manipulator_state_callback,
            10
        )
        self.target_state_subscription = self.create_subscription(
            WorldState,
            'target_state',
            self.target_state_callback,
            10
        )
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

        self.kinematics_future = None
        self.tool_future = None
        self.grabber_open: bool = False
        self.is_moving = False
        self.queue = []

        print('Mover node started')

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
        request.joint_position.position = [0.01] if should_open else [0.0]
        self.grabber_open = not self.grabber_open
        self.tool_future = self.tool_client.call_async(request)
        # self.tool_future.add_done_callback(lambda future: print('tool state done %s' % future.done()))

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

        target_point = Point()
        target_point.set_from_point(position)
        target_point.z = self.z_levels[msg.level]
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
        self.queue.append(lambda: self.set_grabber_state(not self.grabber_open))

        # self.send_pose_request(target_point)
        # self.set_grabber_state(not self.grabber_open)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    rclpy.spin(mover)

    rclpy.shutdown()
    print("did shutdown")


if __name__ == '__main__':
    main()
