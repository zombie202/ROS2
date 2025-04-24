import rclpy
import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from math import sin, cos, atan2, sqrt, pi

# Komenda do publikowania współrzędnych punktu
# ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base'}, point: {x: 0.3, y: 0, z: 0.1}}" -1

clock = rclpy.clock.Clock()

class InverseKinNode(Node):
    def __init__(self):
        super().__init__('InverseKin')
        self.JointSubscription = self.create_subscription(PointStamped, '/clicked_point', self._callback, 10)
        self.JointSubscription

        self.JointPublisher = self.create_publisher(JointState, '/joint_states', 10)

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

    def is_point_good(self, p, w, z) -> bool:
        p_w, p_z = p

        alpha2 = atan2(p_w, p_z)
        alpha3 = atan2(p_z - z, w - p_w)

        return (alpha2 >= 0 and alpha2 <= pi/2 and alpha3 >= -10/180*pi and alpha3 <= pi/2)


    def _callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        r = 0

        self.get_logger().info("Clicked :x=%f y=%f z=%f" %(x, y, z))

        l0 = self.get_parameter('link_sizes.base.z').value + self.get_parameter('link_sizes.rotating_arm').value
        l1 = self.get_parameter('link_sizes.lower_arm').value
        l2 = self.get_parameter('link_sizes.upper_arm').value
        l3 = self.get_parameter('link_sizes.mount.xz').value/2
        l4 = self.get_parameter('link_sizes.mount.xz').value

        z = z - l0 + l4

        alpha1 = atan2(y,x)

        if alpha1 < -pi/2 or alpha1 > pi/2:
            self.get_logger().info('Brak rozwiązań. Ograniczenie na zakresach stawów')
            return

        alpha4 = r - alpha1

        w = x/cos(alpha1) - l3

        mi = l1*l1 - l2*l2+ w*w + z*z
        delta = 16*w*w*mi*mi-4*(mi*mi-4*z*z*l1*l1)*(4*z*z+4*w*w)

        self.get_logger().info("mi=%f delta=%f" %(mi, delta))

        if delta < 0:
            self.get_logger().info('Brak rozwiązań. Zakrótkie ramię')
            return

        p1_w = (4*w*mi+sqrt(delta))/(8*(z*z+w*w))
        p1 = ( p1_w , (mi-2*p1_w*w)/(2*z))

        p2_w = (4*w*mi-sqrt(delta))/(8*(z*z+w*w))
        p2 = ( p2_w , (mi-2*p2_w*w)/(2*z))

        self.get_logger().info("P1: (%f, %f)" %(p1[0], p1[1]))
        self.get_logger().info("P2: (%f, %f)" %(p2[0], p2[1]))

        p = None

        if self.is_point_good(p2, w, z):
            p = p2

        if self.is_point_good(p1, w, z):
            p = p1

        if p is None:
            self.get_logger().info('Brak rozwiązań. Ograniczenie na zakresach stawów')
            return

        p_w, p_z = p
        alpha2 = atan2(p_w, p_z)
        alpha3 = atan2(p_z - z, w - p_w)

        joint_state = JointState()

        alpha34 = pi/2 - alpha3
        alpha3 = -alpha2 + alpha3

        joint_state.name = ["rotating_arm_joint", "lower_arm_joint", "upper_arm_joint", "mount_joint", "gripper_joint"]
        joint_state.position = [alpha1, alpha2, alpha3, alpha34, alpha4]

        joint_state.header.stamp = clock.now().to_msg()

        self.JointPublisher.publish(joint_state)

        pass


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
