from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makramghraizi',
    maintainer_email='makramghraizi14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'item_delivery_server_node = warehouse_robot.item_delivery_server:main',
            'item_delivery_client_node = warehouse_robot.item_delivery_client:main',
            'stock_checker_server_node = warehouse_robot.stock_checker_server:main',
        ],
    },
)
