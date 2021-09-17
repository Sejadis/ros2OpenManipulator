import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetKinematicsPose


class StateLogger(Node):

    def __init__(self):
        super().__init__('state_logger')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.cli = self.create_client(SetKinematicsPose, 'goal_task_space_path_position_only')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        self.req = SetKinematicsPose.Request()

    def send_request(self, x, y, z):
        self.req.end_effector_name = 'gripper'
        self.req.kinematics_pose.pose.position.x = x
        self.req.kinematics_pose.pose.position.y = y
        self.req.kinematics_pose.pose.position.z = z
        self.req.path_time = 2.0
        self.future = self.cli.call_async(self.req)

    def listener_callback(self, msg):
        self.get_logger().info('Joint State: "%s"' % msg.effort[0])


def main(args=None):
    rclpy.init(args=args)

    state_logger = StateLogger()
    state_logger.send_request(0.135, -0.0, 0.24)
    while rclpy.ok():
        rclpy.spin_once(state_logger)
        if state_logger.future.done():
            try:
                response = state_logger.future.result()
            except Exception as e:
                state_logger.get_logger().info('call failed %r' % (e,))
            else:
                state_logger.get_logger().info('result %s' % response.is_planned)
            break
    rclpy.shutdown()
    # print('Hi from move_test.')


if __name__ == '__main__':
    main()
