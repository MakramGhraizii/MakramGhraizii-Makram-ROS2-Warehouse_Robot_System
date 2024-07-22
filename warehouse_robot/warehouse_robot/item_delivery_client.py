import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_tutorial.action import DeliveryItem

class ItemDeliveryClient(Node):

    def __init__(self):
        super().__init__('item_delivery_client')
        self._action_client = ActionClient(self, DeliveryItem, 'deliver_item')
        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None
        self.declare_parameter('item_name','default')
        self.declare_parameter('quantity',1)

    def send_goal(self, item_name, quantity):
        
        goal_msg = DeliveryItem.Goal()
        goal_msg.item_name = item_name
        goal_msg.quantity = quantity

        self.get_logger().info(f'Sending goal with item_name="{item_name}" and quantity={quantity}')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    item_delivery_client = ItemDeliveryClient()
    
    item_name = item_delivery_client.get_parameter('item_name').get_parameter_value().string_value
    quantity = item_delivery_client.get_parameter('quantity').value
    item_delivery_client.send_goal(item_name, quantity)

    rclpy.spin(item_delivery_client)

if __name__ == '__main__':
    main()
