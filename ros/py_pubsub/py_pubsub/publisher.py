#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_1 = self.create_publisher(Float64, 'data_1', 10)
        self.publisher_2 = self.create_publisher(Float64, 'data_2', 10)
        self.publisher_3 = self.create_publisher(Float64, 'data_3', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.shutdown_timer = self.create_timer(300, self.shutdown)
        self.data_1 = 0.0
        self.data_2 = 0.0
        self.data_3 = 0.0

    def publish_data(self):
        self.data_1 += random.uniform(-0.5, 0.5)
        self.data_2 += random.uniform(-0.5, 0.5)
        self.data_3 += random.uniform(-0.5, 0.5)
        self.publisher_1.publish(Float64(data=self.data_1))
        self.publisher_2.publish(Float64(data=self.data_2))
        self.publisher_3.publish(Float64(data=self.data_3))
        self.get_logger().info(f'Publishing: {self.data_1}, {self.data_2}, {self.data_3}')

    def shutdown(self):
        self.get_logger().info('Shutting down...')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()