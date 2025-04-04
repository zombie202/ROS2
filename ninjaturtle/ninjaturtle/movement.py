import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn

class Movement(Node):
    def __init__(self):
        super().__init__('movement')
        self.declare_parameter('turtle2_name', 'Leonardo')
        self.declare_parameter('turtle3_name', 'Donatello')

        self.rotate_cliente = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')
        self.rotate_cliente.wait_for_server()

        self.spawn_service = self.create_client(Spawn, 'spawn')
        self.spawn_service.wait_for_service()

        self.execute()

    def execute(self):
        self.get_logger().info('Executing movement node')
        self.rotate_turtle(3.14159, self.spawn_second_turtle)

    def spawn_second_turtle(self):
        self.spawn_turtle(
            2, 2, 0,
            self.get_parameter('turtle2_name').get_parameter_value().string_value,
            lambda: self.rotate_turtle(0, self.rotate_from_0)
            )
    def rotate_from_0(self):
        self.rotate_turtle(4.71239, self.spawn_third_turtle)

    def spawn_third_turtle(self):
        self.spawn_turtle(
            4, 4, 0,
            self.get_parameter('turtle3_name').get_parameter_value().string_value,
            lambda: self.get_logger().info('Sequence completed')
            )

    def spawn_turtle(self, x:float, y:float, theta:float, name:str, callback=None):
        request = Spawn.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        request.name = name

        future = self.spawn_service.call_async(request)
        future.add_done_callback(lambda future: callback())

    def rotate_turtle(self, angle: float, callback=None):
        goal = RotateAbsolute.Goal()
        goal.theta = float(angle)

        goal_future = self.rotate_cliente.send_goal_async(goal)
        goal_future.add_done_callback(lambda future: self.on_rotation_done(future, callback))

    def on_rotation_done(self, goal_future, callback=None):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Rotation request was rejected')
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
