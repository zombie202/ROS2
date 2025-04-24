import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class Calculator(Node):
    def __init__(self):
        super().__init__('ForwardKin')
        self.JointSubscription = self.create_subscription(JointState, '/dobot_joint_states', self.JointSubscription_callback, 10)
        self.JointSubscription

        self.PosePublisher = self.create_publisher(JointState, '/joint_states', 10)

    def JointSubscription_callback(self, jointState: JointState):
        jointStateOut = JointState()
        jointStateOut.header.stamp = self.get_clock().now().to_msg()
        jointStateOut.header.frame_id = 'base'

        angles = jointState.position

        alpha1, alpha2, alpha3, alpha4 = angles

        alpha34 = math.pi/2 - alpha3
        alpha3 = -alpha2 + alpha3

        jointStateOut.name = ["rotating_arm_joint", "lower_arm_joint", "upper_arm_joint", "mount_joint", "gripper_joint"]
        jointStateOut.position = [alpha1, alpha2, alpha3, alpha34, alpha4]


        self.PosePublisher.publish(jointStateOut)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Calculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
