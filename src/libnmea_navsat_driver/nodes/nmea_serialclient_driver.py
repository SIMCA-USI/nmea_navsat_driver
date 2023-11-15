from rclpy.node import Node
from libnmea_navsat_driver.driver import Ros2NMEADriver
from std_msgs.msg import Byte
import rclpy


class SerialClient(Node):
    def __init__(self):

        super().__init__(node_name='serial_client_satnav')

        self.raw = None
        self.sub = self.create_subscription(Byte, '/gps/connect_satnav', self.callback_sub, 10)
        self.logger = self.get_logger()

    def callback_sub(self, msg: Byte):
        driver = Ros2NMEADriver()
        frame_id = driver.get_frame_id()

        self.raw = msg.data
        data = self.raw.decode('utf-16le')
        self.get_logger().info(f'{data}')

        driver.add_sentence(data, frame_id)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = SerialClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
