from geographiclib.geodesic import Geodesic
import math
from math import atan2
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3
from libnmea_navsat_driver import parser
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
import errno
import socket
import threading
import time
from traceback import format_exc
import pynmea2
import rclpy
import utm
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from sensors.gps.gps_topics import *



from ros2_waypoints import *
import os
import yaml
from yaml.loader import SafeLoader

class Ros2NMEA_UTMDriver(Node):
    """
    This class represents a ROS 2 node responsible for processing NMEA and UTM sentences and publishing related data.
    """

    def __init__(self):
        """
        Constructor for the Ros2NMEADriver class.
        """
        with open(os.getenv('ROS_WS') + '/vehicle.yaml') as f:
            vehicle_parameters = yaml.load(f, Loader=SafeLoader)
        super().__init__('Ros2NMEA_UTMDriver', namespace=vehicle_parameters['id_vehicle']+'/gps')

        self.logger = self.get_logger()
        self._log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=20)).value
        self.logger.set_level(self._log_level)
        self.__mode = self.get_parameter_or('gps_params.mode', Parameter(name='gps_params.mode', value='tcp')).value
        self.__ip = self.get_parameter_or('gps_params.ip',
                                          Parameter(name='gps_params.ip', value='192.168.1.100')).value
        self.__port = self.get_parameter_or('gps_params.port', Parameter(name='gps_params.port', value=5017)).value
        self.__nmea_messages = self.get_parameter_or('gps_params.nmea_messages',
                                                     Parameter(name='gps_params.nmea_messages',
                                                               value=['GGA', 'GSA', 'GST'])).value
        self.__thread_connection = None
        self.active = True
        self.utm_pub = self.create_publisher(Vector3, 'utm', 10)
        self.GPS_Position = self.create_publisher(Vector3,'position', 10)
        self.GPS_UTM = self.create_publisher(Vector3,'utm', 10)
        self.GPS_Quality = self.create_publisher(Int16,'quality', 10)
        self.GPS_Satellites = self.create_publisher(Int16,'satellites', 10)
        self.GPS_STD_DEV = self.create_publisher(Vector3,'std_dev', 10)
        self.GPS_STD_DEV_AXIS = self.create_publisher(Vector3,'std_dev_axis', 10)
        self.GPS_Orientation = self.create_publisher(Float64,'orientation_gps', 10)
        self.GPS_Heading = self.create_publisher(Float64,'heading/trimble', 10)
        self.GPS_RMS = self.create_publisher(Float64,'orientation_gps', 10)
        self.GPS_Date = self.create_publisher(String, 'date', 10)
        self.GPS_Speed = self.create_publisher(Float64,'speed', 10)
        self.GPS_HDOP = self.create_publisher(Float64,'hdop', 10)


        #Publishers del nmea
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.orientation_pub = self.create_publisher(Float64, 'orientation', 10)
        self.orientation_two_p = self.create_publisher(Float64, 'orientation2p', 10)
        self.dist_two_p = self.create_publisher(Float64, 'distance2p', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)


        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False

        self.frame_id = self.get_frame_id()
        self.raw = None

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")
        self.last_lon_lat = None

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def shutdown(self, reason=None):
        """
        This method sets the 'active' flag to False, indicating that the node is being shut down.
        It logs a shutdown message and an optional reason for the shutdown.

        :param reason: Optional reason for the shutdown.
        :type reason: str or None
        :return: None
        """
        self.active = False
        self.logger.info('{} Node is DOWN'.format(self.get_name()))
        if reason:
            self.logger.info(reason)

    def __run_tcp(self):
        """
        This method creates a new thread that runs the __listen_nmea function to establish and maintain a connection to
        the GPS device and receive NMEA data.

        :return: None
        """
        self.__thread_connection: threading.Thread = threading.Thread(target=self.__listen_nmea, args=())
        self.__thread_connection.start()

    def __listen_nmea(self):
        """
        This method connects to the GPS device, continuously receives NMEA data, and calls the method `__process_nmea`
        to handle the received data.

        :return: None
        """

        _socket_nmea = None
        connected_nmea = False

        while self.active:
            try:
                if not connected_nmea:
                    _socket_nmea = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    print("Connecting to GPS at {}:{} ...".format(self.__ip, self.__port))
                    _socket_nmea.settimeout(4)
                    _socket_nmea.connect((self.__ip, self.__port))
                    print("Connection established [{}]".format(self.__port))
                    _socket_nmea.settimeout(0.5)
                    connected_nmea = True
                else:
                    data = _socket_nmea.recv(1024)
                    self.__process_nmea_utm(data.decode(errors='replace'))

            except IOError as e:
                if e.errno == errno.EPIPE:
                    self.logger.error('Connection broken')
                elif e.errno == errno.ECONNREFUSED:
                    self.logger.error('There is no GPS ready on {}:{}...\n'.format(self.__ip, self.__port))
                elif e.errno == errno.ENETUNREACH:
                    self.logger.error(
                        'GPS IP not valid. Please use an IP of your network, not {}...\n'.format(self.__ip))
                    rclpy.try_shutdown()
                    # rospy.signal_shutdown("IP configuration not valid")
                    self.active = False
                elif e.errno == errno.ECONNRESET:
                    self.logger.error('GPS disconnected from port {}...\n'.format(self.__port))

                elif e.errno == errno.EHOSTUNREACH:
                    self.logger.error('No GPS detected in {}...\n'.format(self.__ip))
                elif e.errno == errno.ETIMEDOUT:
                    self.logger.error("Connection to GPS lost. Trying to recover...")
                    connected_nmea = False
                    _socket_nmea.close()
                else:
                    self.logger.error('Connection problem resetting...\n{}'.format(format_exc()))
                time.sleep(2)

            except Exception:
                print(format_exc())
                connected_nmea = False
                if _socket_nmea:
                    _socket_nmea.close()
                time.sleep(2)

        _socket_nmea.close()

    def __process_nmea_utm(self, data: str) -> None:
        """
        This function takes both raw NMEA and UTM data string, processes them, and publishes them to a specific topic.

        :param data: A string containing the raw NMEA data received from the GPS device.
        :type data: str
        :return: None
        :raises pynmea2.ParseError: If an error occurs while parsing the NMEA data.
        """
        try:
            if self.__mode == 'tcp':
                msgs = data.split('\r\n')[:-1]
            else:
                msgs = list([data])

            for m in msgs:
                msg_raw = String()
                msg_raw.data = m
                self.logger.debug(msg_raw)
                data = msg_raw.data
                #for publishing fix
                try:
                    if self.add_sentence(data, self.frame_id) != False:
                        self.get_logger().debug("Received sentence: %s" % data)
                    else:
                        self.get_logger().warn("Error with sentence: %s" % data)

                except ValueError as e:
                    self.get_logger().warn(
                        "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue to me. " % e)
                #for publishing utm
                try:
                    raw=String()
                    raw.data=m
                    # self.raw = str(m.data)
                    self.logger.error(f'Received msg {raw.data}')
                    msg = pynmea2.parse(raw.data)

                    if msg.sentence_type in self.__nmea_messages:
                        if msg.sentence_type == 'GGA':
                            lat = msg.latitude  # msg.latitude
                            lon = msg.longitude  # msg.longitude
                            alt = float(msg.altitude)  # gps_qual
                            self.GPS_Position.publish(
                                Vector3(x=float(lat), y=float(lon), z=float(alt)))
                            to_utm = utm.from_latlon(lat, lon)  # can get "T" too
                            self.utm_pub.publish(
                                Vector3(x=float(to_utm[0]), y=float(to_utm[1]), z=float(to_utm[2])))
                            quality = msg.gps_qual
                            if quality == 0:
                                self.logger.info("GPS POSITION NO FIX")
                            self.GPS_Quality.publish(Int16(data=quality))

                            if len(msg.num_sats):
                                num_sats = int(msg.num_sats)  # msg.altitude
                            else:
                                num_sats = 0
                                self.logger.info("NO AVAILABLE SATELLITES")
                            self.GPS_Satellites.publish(Int16(data=num_sats))

                        elif msg.sentence_type == 'GST':
                            # rms RMS value of the standard deviation of the range inputs to the navigation process.
                            # Range inputs include preudoranges & DGNSS corrections.',
                            self.GPS_RMS.publish(Float64(data=float(msg.rms)))

                            # 'Standard deviation of semi-major axis of error ellipse (meters)',
                            # 'std_dev_major',

                            # 'Standard deviation of semi-minor axis of error ellipse (meters)',
                            # 'std_dev_minor',
                            self.GPS_STD_DEV_AXIS.publish(
                                Vector3(x=msg.std_dev_major, y=msg.std_dev_minor, z=0.))

                            # 'Orientation of semi-major axis of error ellipse (degrees from true north)',
                            # 'orientation',
                            self.GPS_Orientation.publish(Float64(data=float(msg.orientation)))

                            # 'Standard deviation of latitude error (meters)',
                            # 'std_dev_latitude',
                            # 'Standard deviation of longitude error (meters)',
                            # 'std_dev_longitude',
                            # 'Standard deviation of altitude error (meters)',
                            # 'std_dev_altitude',
                            self.GPS_STD_DEV.publish(
                                Vector3(x=msg.std_dev_latitude, y=msg.std_dev_longitude, z=msg.std_dev_altitude))
                        elif msg.sentence_type == 'HDT':
                            self.GPS_Heading.publish(Float64(data=float(msg.heading)))

                        elif msg.sentence_type == 'RMC':
                            self.GPS_Date.publish(String(data=str(msg)))

                        elif msg.sentence_type == 'VTG':
                            self.GPS_Speed.publish(Float64(data=float(msg.spd_over_grnd_kmph)))

                        elif msg.sentence_type == 'GSA':
                            self.GPS_HDOP.publish(Float64(data=float(msg.hdop)))
                        else:
                            print(msg.sentence_type)

                except pynmea2.ParseError:
                    self.logger.error('pynmea error')
                    return

        except pynmea2.ParseError:
            self.logger.error(f'{Exception("Error in parse nmea")}')

    def run(self):
        """
        This method starts the Connexion node by executing the appropriate method based on the selected mode.
        In this case, it calls the __run_tcp method if the mode is set to 'tcp'.

        :return: None
        """

        if self.__mode == 'tcp':
            self.__run_tcp()


    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        """
        Add an NMEA sentence for processing.

        :param nmea_string: NMEA sentence to be processed.
        :type nmea_string: str
        :param frame_id: Frame ID for the ROS messages.
        :type frame_id: str
        :param timestamp: Timestamp for the sentence (optional).
        :type timestamp: None

        :return: True if the sentence was successfully processed, False if there was an issue.
        """
        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn("Received a sentence with an invalid checksum. " +
                                   "Sentence was: %s" % nmea_string)
            return False

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = self.get_clock().now().to_msg()
        utm_msg = Vector3()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            if current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if self.last_lon_lat is None:
                self.last_lon_lat = (current_fix.longitude, current_fix.latitude)
            else:
                geo = Geodesic.WGS84.Inverse(self.last_lon_lat[1], self.last_lon_lat[0],
                                              current_fix.latitude, current_fix.longitude)
                brng = geo['azi1']
                dist = geo['s12']

                if brng < 0:
                    brng += 360
                orient = Float64()
                orient.data = brng
                distance=Float64()
                distance.data = dist
                self.orientation_two_p.publish(orient)
                self.dist_two_p.publish(distance)
                self.last_lon_lat = (current_fix.longitude, current_fix.latitude)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
                orientation = Float64()
                orientation.data = 2 * atan2(q[2], q[3])
                orientation.data = math.degrees(orientation.data)
                self.orientation_pub.publish(orientation)

        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    def get_frame_id(self):
        """
        Get the frame ID.

        :return: The frame ID.
        """
        frame_id = self.declare_parameter('frame_id', 'gps').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id


def main(args=None):
    """
        This function initializes the ROS 2 framework, creates an instance of the Connexion class, starts the GPS node,
        enters the ROS 2 event loop to process messages, and handles manual interruption. It ensures proper
        cleanup before shutting down the program.

        :param args: Command-line arguments.
        :type args: list
        :return: None
        """
    print('Hi from sensors.')
    rclpy.init(args=args)
    manager = None
    try:
        manager = Ros2NMEA_UTMDriver()
        manager.run()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.shutdown('Manually interrupted')
    finally:
        if manager is not None:
            manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()