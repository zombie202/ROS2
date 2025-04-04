import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

class Movement(Node):
    def __init__(self):
        super().__init__('tower')

        self.x = 116
        self.pick_y = -53 
        self.place_y = 52
        self.can_go = True

        self.declare_parameter('height', 4)
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        self.rotate_cliente = ActionClient(self, PointToPoint, '/PTP_action')
        self.rotate_cliente.wait_for_server()

        self.spawn_service = self.create_client(GripperControl, '/dobot_gripper_service')
        self.spawn_service.wait_for_service()

        self.execute()

    def calc_z(self, block_no: int):
        return block_no*20+1

    def move_bloc_pick(self, block_no: int,  callback=None):       
        def pick():
            self.move(self.x, self.pick_y, self.calc_z(block_no), 0, grip)

        def grip():
            self.get_logger().info('Grip')
            self.gripper("close", up)
        
        def up():
            self.move(self.x, self.pick_y, 80, 0, callback)

        self.move(self.x, self.pick_y, 80, 0, pick)

    def move_block_place(self, block_no: int, callback=None):

        def down():
            self.move(self.x, self.place_y, self.calc_z(block_no), 0, place)
        
        def place():
            self.gripper("open", up)
        
        def up():
            self.move(self.x, self.place_y, 80, 0, callback)

        self.move(self.x, self.place_y, 80, 0, down)

    def release(self):
        self.can_go=True

    def move_block(self, block_no: int):
        self.move_bloc_pick(self.height-1- block_no, lambda: self.move_block_place(block_no, self.release))

    def execute(self):
        self.get_logger().info('Executing movement node')

        for i in range(0, self.height+1):
            while not self.can_go:
                rclpy.spin_once(self)
            
            self.get_logger().info(f'{i}')
            self.can_go = False
            self.move_block(i)

        # def move_fourth():
        #     if self.height <
        #     self.move_block(3)
        
        # def move_third():
        #     self.move_block(2, move_fourth)

        # def move_secound():
        #     if self.height < 2: return
        #     self.move_block(1, move_third)
        
        # def move_first():
        #     self.move_block(0, move_secound)

        # move_first()


    def gripper(self, state:str, callback=None):
        request = GripperControl.Request()
        request.gripper_state = state

        future = self.spawn_service.call_async(request)
        future.add_done_callback(lambda future: callback())


    def move(self, x, y, z, r, callback=None):
        goal = PointToPoint.Goal()
        goal.motion_type = 1
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
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
