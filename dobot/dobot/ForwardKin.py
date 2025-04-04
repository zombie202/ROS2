import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math
import numpy as np

def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


class ForwardKinNode(Node):
    def __init__(self):
        super().__init__('ForwardKin')
        self.JointSubscription = self.create_subscription(JointState, '/joint_states', self.JointSubscription_callback, 10)
        self.JointSubscription

        self.PosePublisher = self.create_publisher(PoseStamped, '/gripper_pose', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('link_sizes.base.z', rclpy.Parameter.Type.DOUBLE),
                ('link_sizes.rotating_arm', rclpy.Parameter.Type.DOUBLE),
                ('link_sizes.lower_arm', rclpy.Parameter.Type.DOUBLE),
                ('link_sizes.upper_arm', rclpy.Parameter.Type.DOUBLE),
                ('link_sizes.mount.xz', rclpy.Parameter.Type.DOUBLE)
            ]
        )

    def JointSubscription_callback(self, jointState: JointState):
        poseStamped = PoseStamped()
        poseStamped.header.stamp = self.get_clock().now().to_msg()
        poseStamped.header.frame_id = 'base'

        angles = jointState.position
        # l0, l1, l2, l3, l4 = [0.138, 0.135, 0.147, 0.041/2, 0.041]

        l0 = self.get_parameter('link_sizes.base.z').value + self.get_parameter('link_sizes.rotating_arm').value
        l1 = self.get_parameter('link_sizes.lower_arm').value
        l2 = self.get_parameter('link_sizes.upper_arm').value
        l3 = self.get_parameter('link_sizes.mount.xz').value/2
        l4 = self.get_parameter('link_sizes.mount.xz').value

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

        quaternion1 = [math.cos(r/2), 0, 0, math.sin(r/2)]


        quaternion2 = [math.cos(math.pi/2), math.sin(math.pi/2), 0, 0]

        [q0, q1, q2, q3] = quaternion_multiply(quaternion1, quaternion2)

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
