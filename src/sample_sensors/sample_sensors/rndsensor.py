import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class RndSensor(Node):

    def __init__(self):
        super().__init__('rnd_sensor')
        self.get_logger().info('random data sensor initializing...')
        self.publisher_ = self.create_publisher(Float64, 'sensorvalue', 10)
        timer_period = 1.0  # time-interval for our periodic timer [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.value = 40.0

    def timer_callback(self):
        self.update_value()
        msg = Float64() # create a new message to be published
        msg.data = self.value
        self.publisher_.publish(msg)
        self.get_logger().info('sensor value: "%.1f"' % msg.data)

    def update_value(self):
        self.value = self.value + 0.1

def main(args=None):
    rclpy.init(args=args)

    # create a new sensor
    rnd_sensor = RndSensor()

    # start working (and block until context is shutdown)
    rclpy.spin(rnd_sensor)

    # destroy the node
    rnd_sensor.destroy_node()

    # shutdown our context
    rclpy.shutdown()


if __name__ == '__main__':
    main()
