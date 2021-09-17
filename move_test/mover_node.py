import rclpy
from rclpy.node import Node
from planing_interfaces.msg import WorldState
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition


class Point(object):
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class Mover(Node):

    def __init__(self):
        super().__init__("mover")
        self.subscription = self.create_subscription(
            WorldState,
            'target_state',
            self.target_state_callback,
            10)
        self.kinematic_pose_client = self.create_client(SetKinematicsPose, 'goal_task_space_path_position_only')
        while not self.kinematic_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kinematics service')
        self.tool_client = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for tool service')

        self.left_position = Point(0.25, -0.1, 0.05)
        self.middle_position = Point(0.25, 0.0, 0.05)
        self.right_position = Point(0.25, 0.1, 0.05)
        self.kinematics_future = None
        self.tool_future = None
        self.grabber_open: bool = False

        print('Mover node started')

    def send_pose_request(self, position: Point):
        request = SetKinematicsPose.Request()
        request.end_effector_name = 'gripper'
        request.kinematics_pose.pose.position.x = position.x
        request.kinematics_pose.pose.position.y = position.y
        request.kinematics_pose.pose.position.z = position.z
        request.path_time = 2.0
        self.kinematics_future = self.kinematic_pose_client.call_async(request)
        # self.kinematics_future.add_done_callback(self.set_grabber_state)

    def set_grabber_state(self, should_open: bool):
        request = SetJointPosition.Request()
        request.path_time = 1.0 if should_open else 3.0
        request.joint_position.joint_name = ['gripper']
        request.joint_position.position = [0.01] if should_open else [-0.01]
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

        print('target position is (%s, %s, %s)' % (position.x, position.y, position.z))
        self.send_pose_request(position)
        self.set_grabber_state(not self.grabber_open)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    rclpy.spin(mover)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
