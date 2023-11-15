import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


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
