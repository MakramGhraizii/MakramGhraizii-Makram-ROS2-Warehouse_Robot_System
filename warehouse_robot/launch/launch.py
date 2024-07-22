from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_robot',
            executable='item_delivery_server_node',
            name='item_delivery_server',
        ),
        Node(
            package='warehouse_robot',
            executable='stock_checker_server_node',
            name='stock_checker_server',
        ),
        Node(
            package='warehouse_robot',
            executable='item_delivery_client_node',
            name='item_delivery_client',
            parameters=[{
    'item_name': LaunchConfiguration('item_name'),  
    'quantity': LaunchConfiguration('quantity')
}],
        ),
        Node(
            package='warehouse_robot',
            executable='stock_checker_client_node',
            name='stock_checker_client',
        ),
    ])
