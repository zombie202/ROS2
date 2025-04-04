import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math


class ForwardKinNode(Node):
    def __init__(self):
        super().__init__('ForwardKin')
        self.JointSubscription = self.create_subscription(JointState, '/joint_states', self.JointSubscription_callback, 10)
        self.JointSubscription

        self.PosePublisher = self.create_publisher(PoseStamped, '/pose', 10)

    def JointSubscription_callback(self, jointState: JointState):
        poseStamped = PoseStamped()
        poseStamped.header.stamp = self.get_clock().now().to_msg()
        poseStamped.header.frame_id = 'base'

        angles = jointState.position
        l0, l1, l2, l3, l4 = [0.138, 0.135, 0.147, 0.041/2, 0.041]
        alpha1, alpha2, alpha3, _, alpha4 = angles

        L = l1*math.sin(alpha2) + l2*math.cos(alpha3 + alpha2) + l3

        x = math.cos(alpha1) * L
        y = math.sin(alpha1) * L
        z = l1*math.cos(alpha2) - l2*math.sin(alpha3 + alpha2) + l0 - l4
        r = alpha1 + alpha4

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        q0, q1, q2, q3 = [math.cos(r/2), 0, 0, math.sin(r/2)]

        quaternion = Quaternion()
        quaternion.w = float(q0)
        quaternion.x = float(q1)
        quaternion.y = float(q2)
        quaternion.z = float(q3)

        pose = Pose()
        pose.position = point
        pose.orientation = quaternion


        poseStamped.pose = pose

        self.PosePublisher.publish(poseStamped)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
