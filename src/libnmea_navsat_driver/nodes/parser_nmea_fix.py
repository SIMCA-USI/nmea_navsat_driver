from sensors.gps.gps_topics import *
from rclpy.node import Node
from libnmea_navsat_driver.driver import Ros2NMEADriver
import rclpy
import os
import yaml
from yaml.loader import SafeLoader
from rclpy.qos import HistoryPolicy


class ParserNmeaFix(Node):
    """
    This class represents a ROS 2 node that subscribes to a topic named '/gps/connect' of type String.
    It receives NMEA sentences, processes them, and forwards them to a ROS 2 driver.
    """
    def __init__(self):

        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__(node_name='parser_nmea_fix', namespace=vehicle_parameters['id_vehicle'], allow_undeclared_parameters=True,
                         start_parameter_services=True, automatically_declare_parameters_from_overrides=True)

        self.sub = self.create_subscription(String, 'gps/connect', self.callback_sub, HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        self.logger = self.get_logger()
        self.driver = Ros2NMEADriver()
        self.frame_id = self.driver.get_frame_id()
        self.raw = None

    def callback_sub(self, msg: String):
        """
        Callback function that calls the method add_sentence to process the incoming NMEA sentences.

        :param msg: The NMEA sentence received from the topic '/gps/connect'.
        :type msg: String
        """

        self.raw = str(msg.data)
        data = self.raw
        try:
            if self.driver.add_sentence(data, self.frame_id) != False:
                self.driver.get_logger().debug("Received sentence: %s" % data)
            else:
                self.driver.get_logger().warn("Error with sentence: %s" % data)

        except ValueError as e:
            self.driver.get_logger().warn(
                "Value error, likely due to missing fields in the NMEA message. "
                "Error was: %s. Please report this issue to me. " % e)


def main(args=None):
    """
    The main function for initializing and running the TcpClientSub node.

    :param args: Command-line arguments.
    :type args: list
    :return: None
    """
    try:
        rclpy.init(args=args)
        node = ParserNmeaFix()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
