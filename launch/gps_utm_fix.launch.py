import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    parameters_file_path = '{}/../conf/nmea_tcpclient_driver.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='gps_utm_fix',
            parameters=[parameters_file_path],
            output='screen'
        )
    ])