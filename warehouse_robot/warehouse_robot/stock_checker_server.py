import rclpy
from rclpy.node import Node
from interface_tutorial.srv import CheckStock

class StockCheckerServer(Node):

    def __init__(self):
        super().__init__('stock_checker_server')
        self.srv = self.create_service(CheckStock, 'check_stock', self.check_stock_callback)

        self.stock_levels = {
            'item1': 50,
            'item2': 30,
            'item3': 75,
            'item4': 10,
        }

    def check_stock_callback(self, request, response):
        item_name = request.item_name
        if item_name in self.stock_levels:
            response.stock_level = self.stock_levels[item_name]
        else:
            response.stock_level = 0  # Default stock level for unknown items
        self.get_logger().info(f'Checked stock for {item_name}: {response.stock_level}')
        return response

def main(args=None):
    rclpy.init(args=args)
    stock_checker_server = StockCheckerServer()
    rclpy.spin(stock_checker_server)

if __name__ == '__main__':
    main()
