import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Byte
import serial


class Connexion(Node):

    def __init__(self):
        super().__init__(node_name='connexion_serial_satnav')
        self.pub = self.create_publisher(Byte, '/gps/connect_satnav', HistoryPolicy.KEEP_LAST)
        self.logger = self.get_logger()
        self._log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=10)).value
        self.logger.set_level(self._log_level)

    def run(self):
        serial_port = self.declare_parameter('port', '/dev/ttyUSB0').value
        serial_baud = self.declare_parameter('baud', 4800).value
        try:
            gps = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
            self.get_logger().info("Successfully connected to {0} at {1}.".format(serial_port, serial_baud))
            try:
                while rclpy.ok():
                    data = gps.readline().strip()

                    try:
                        ''' puede que el error que sale se deba a que no tiene conexión el gps probar fuera, si no, 
                        averiguar el tipo de mensages que recibimos y convertir a byte o mandar ese tipo directamente y 
                        despues ver como decodificar'''
                        self.get_logger().info(f'{data}')
                        self.get_logger().info(f'{data.decode("utf-8")}')
                        if isinstance(data, Byte):
                            data = data.decode("utf-8")
                            msg = Byte()
                            msg.data = data
                            self.pub.publish(msg)
                    except ValueError as e:
                        self.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                            "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                            "with the NMEA sentences that caused it." % e)

            except Exception as e:
                self.get_logger().error("Ros error: {0}".format(e))
                gps.close()  # Close GPS serial port
        except serial.SerialException as ex:
            self.get_logger().fatal(
                "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))


def main(args=None):
    print('Hi from sensors.')
    rclpy.init(args=args)
    gps = None
    try:
        gps = Connexion()
        gps.run()
        rclpy.spin(gps)
    finally:
        if gps is not None:
            gps.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
