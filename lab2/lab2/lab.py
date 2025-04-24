import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

class Movement(Node):
    def __init__(self):
        super().__init__('movement')

        self.rotate_cliente = ActionClient(self, PointToPoint, '/PTP_action')
        self.rotate_cliente.wait_for_server()

        self.spawn_service = self.create_client(GripperControl, '/dobot_gripper_service')
        self.spawn_service.wait_for_service()

        self.execute()

    def execute(self):
        self.get_logger().info('Executing movement node')
        self.move(116.56, -53.148, 75, 0, self.patch_one)

    def patch_one(self):
        self.move(116.56, -53.148, 1, 0, self.one)

    def one(self):
        self.gripper("close", self.two)

    def two(self):
        self.move(116.56, -53.148, 75, 0, self.three)
        pass

    def three(self):
        self.move(115.503, 52.473, 1, 0, self.four)
        pass

    def four(self):
        self.gripper("open", lambda: self.get_logger().info('Sequence completed'))
        pass

    def gripper(self, state:str, callback=None):
        request = GripperControl.Request()
        request.gripper_state = state

        future = self.spawn_service.call_async(request)
        future.add_done_callback(lambda future: callback())

    def move(self, x, y, z, r, callback=None):
        goal = PointToPoint.Goal()
        goal.motion_type = 2
        goal.target_pose = [x ,y, z, r]

        goal_future = self.rotate_cliente.send_goal_async(goal)
        goal_future.add_done_callback(lambda future: self.on_motion_done(future, callback))

    def on_motion_done(self, goal_future, callback=None):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion request was rejected')
            return

        future = goal_handle.get_result_async()
        future.add_done_callback(lambda future: callback())


def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
