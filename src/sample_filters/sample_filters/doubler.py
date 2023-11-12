import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class Doubler(Node):

    def __init__(self):
        super().__init__('doubler')
        self.get_logger().info('doubler initializing...')
        self.subscription = self.create_subscription(
            Float64,
            'sensorvalue',
            self.listener_callback,
            10)
        self.subscription  # avoid warning that subscription is not used

    def listener_callback(self, msg):
        self.get_logger().info('received new sensor value: "%.1f"' % msg.data)
        self.process_value(msg.data)

    def process_value(self, value):
        processed_value = 2 * value;
        self.get_logger().info('output: "%.1f"' % processed_value)
        # TODO: publish processed value as new topic

def main(args=None):
    rclpy.init(args=args)

    # create a data processor
    rnd_sensor = Doubler()

    # start working (and block until context is shutdown)
    rclpy.spin(rnd_sensor)