import os

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from yaml.loader import SafeLoader

def generate_launch_description():
    parameters_file_path = '{}/../conf/gps_serial.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))

    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='parser_nmea_fix',
            parameters=[parameters_file_path]
        )
    ])
