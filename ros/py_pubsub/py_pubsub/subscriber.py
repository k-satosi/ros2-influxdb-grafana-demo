#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from influxdb_client import InfluxDBClient, Point
import os

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription_1 = self.create_subscription(Float64, 'data_1', self.listener_callback_1, 10)
        self.subscription_2 = self.create_subscription(Float64, 'data_2', self.listener_callback_2, 10)
        self.subscription_3 = self.create_subscription(Float64, 'data_3', self.listener_callback_3, 10)
        db_host = os.environ['INFLUXDB_HOST']
        db_port = os.environ['INFLUXDB_PORT']
        db_token = os.environ['INFLUXDB_TOKEN']
        db_org = os.environ['INFLUXDB_ORG']
        db_bucket = os.environ['INFLUXDB_BUCKET']
        self.influxdb_client = InfluxDBClient(
            url=f"http://{db_host}:{db_port}",
            token=db_token,
            org=db_org,
        )
        self.bucket = db_bucket
        self.get_logger().info("Subscriber initialized")

    def listener_callback_1(self, msg):
        self.write_to_influxdb('data_1', msg.data)

    def listener_callback_2(self, msg):
        self.write_to_influxdb('data_2', msg.data)

    def listener_callback_3(self, msg):
        self.write_to_influxdb('data_3', msg.data)

    def write_to_influxdb(self, measurement, value):
        point = Point(measurement).field("value", value)
        self.influxdb_client.write_api().write(bucket=self.bucket, record=point)
        self.get_logger().info(f'Written to InfluxDB: {measurement} -> {value}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
