import rclpy
import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

from math import atan2, sin, cos

# Komenda do publikowania współrzędnych punktu
# ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base'}, point: {x: 0.3, y: 0, z: 0.1}}" -1

clock = rclpy.clock.Clock

class Lab4(Node):
    def __init__(self):
        super().__init__('Lab4Publisher')
        self.JointSubscription = self.create_subscription(PointStamped, '/clicked_point', self._callback, 10)
        self.JointSubscription

        self.PTP_cliente = ActionClient(self, PointToPoint, '/PTP_action')
        self.PTP_cliente.wait_for_server()

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


    def on_motion_done(self, goal_future, x, y, z, r):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion request was rejected')
            return

        future = goal_handle.get_result_async()
        future.add_done_callback(lambda future: self.get_logger().info("Wykonano ruch do punktu [%f, %f, %f, %f]" %(x,y,z,r)))

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
        alpha1 = atan2(y, x)
        y = y - sin(alpha1)*l3
        x = x - cos(alpha1)*l3

        goal = PointToPoint.Goal()
        goal.motion_type = 2
        goal.target_pose = [float(x*1000) , float(y*1000), float(z*1000), float(r)]

        goal_future = self.PTP_cliente.send_goal_async(goal)
        goal_future.add_done_callback(lambda future: self.on_motion_done(future, x, y, z, r))

        # self.get_logger().info("Wykonano ruch do punktu [%f, %f, %f, %f]" %(x,y,z,r))

        # alpha1 = atan2(y,x)

        # if alpha1 < -pi/2 or alpha1 > pi/2:
        #     self.get_logger().info('Brak rozwiązań. Ograniczenie na zakresach stawów')
        #     return

        # alpha4 = r - alpha1

        # w = x/cos(alpha1) - l3

        # mi = l1*l1 - l2*l2+ w*w + z*z
        # delta = 16*w*w*mi*mi-4*(mi*mi-4*z*z*l1*l1)*(4*z*z+4*w*w)

        # self.get_logger().info("mi=%f delta=%f" %(mi, delta))

        # if delta < 0:
        #     self.get_logger().info('Brak rozwiązań. Zakrótkie ramię')
        #     return

        # p1_w = (4*w*mi+sqrt(delta))/(8*(z*z+w*w))
        # p1 = ( p1_w , (mi-2*p1_w*w)/(2*z))

        # p2_w = (4*w*mi-sqrt(delta))/(8*(z*z+w*w))
        # p2 = ( p2_w , (mi-2*p2_w*w)/(2*z))

        # self.get_logger().info("P1: (%f, %f)" %(p1[0], p1[1]))
        # self.get_logger().info("P2: (%f, %f)" %(p2[0], p2[1]))

        # p = None

        # if self.is_point_good(p2, w, z):
        #     p = p2

        # if self.is_point_good(p1, w, z):
        #     p = p1

        # if p is None:
        #     self.get_logger().info('Brak rozwiązań. Ograniczenie na zakresach stawów')
        #     return

        # p_w, p_z = p
        # alpha2 = atan2(p_w, p_z)
        # alpha3 = atan2(p_z - z, w - p_w)

        # joint_state = JointState()

        # alpha34 = pi/2 - alpha3
        # alpha3 = -alpha2 + alpha3

        # joint_state.name = ["rotating_arm_joint", "lower_arm_joint", "upper_arm_joint", "mount_joint", "gripper_joint"]
        # joint_state.position = [alpha1, alpha2, alpha3, alpha34, alpha4]

        # joint_state.header.stamp = clock.now().to_msg()

        # self.JointPublisher.publish(joint_state)

        pass


def main(args=None):
    rclpy.init(args=args)
    node = Lab4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
