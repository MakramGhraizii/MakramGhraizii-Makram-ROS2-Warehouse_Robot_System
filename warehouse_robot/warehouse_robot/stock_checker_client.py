import rclpy
from rclpy.node import Node
from interface_tutorial.srv import CheckStock
import matplotlib.pyplot as plt
import time
class StockCheckerClient(Node):

    def __init__(self):
        super().__init__('stock_checker_client')
        self.cli = self.create_client(CheckStock, 'check_stock')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.items = ['item1', 'item2', 'item3', 'item4']
        self.stock_levels = {}

    def send_request(self, item_name):
        request = CheckStock.Request()
        request.item_name = item_name
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error(f'Failed to get stock level for {item_name}')
            return 0
        return response.stock_level
    
    def check_stock(self, item_name):
        self.get_logger().info(f'Retrieving stock level for {item_name}')
        return self.send_request(item_name)
    
    def visualize_stock(self):
        for item in self.items:
            stock_level = self.check_stock(item)
            self.stock_levels[item] = stock_level
            self.get_logger().info(f'{item}: {stock_level}')
            time.sleep(0.5)
        self.plot_stock()

    def plot_stock(self):
        items = list(self.stock_levels.keys())
        levels = list(self.stock_levels.values())

        plt.figure(figsize=(10, 6)) 
        plt.bar(items, levels, color='blue')
        plt.xlabel('Items')
        plt.ylabel('Stock Levels')
        plt.title('Stock Levels of Items')
        plt.xticks(rotation=45)  
        plt.tight_layout()  
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    client = StockCheckerClient()
    client.visualize_stock()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
