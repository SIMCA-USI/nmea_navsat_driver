from glob import glob
import os
from setuptools import setup

package_name = "nmea_navsat_driver"
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version='2.0.0',
    packages=["libnmea_navsat_driver", "libnmea_navsat_driver.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}'.format(package_name), glob('launch/*.launch.py')),
        ('share/conf/', glob('conf/*')),
    ],
    package_dir={'': 'src', },
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'numpy',
                      'pyyaml',
                      'utm'],
    author='Eric Perko',
    maintainer='Ed Venator',
    keywords=['ROS2'],
    description='Package to parse NMEA strings and publish a very simple GPS message.',
    license='BSD',
    entry_points={
        'console_scripts': ['nmea_serial_driver = libnmea_navsat_driver.nodes.nmea_serial_driver:main',
                            'nmea_socket_driver = libnmea_navsat_driver.nodes.nmea_socket_driver:main',
                            'nmea_tcpclient_driver = libnmea_navsat_driver.nodes.nmea_tcpclient_driver:main',
                            'nmea_topic_driver = libnmea_navsat_driver.nodes.nmea_topic_driver:main',
                            'nmea_topic_serial_reader = libnmea_navsat_driver.nodes.nmea_topic_serial_reader:main',
                            'parser_nmea_fix =  libnmea_navsat_driver.nodes.parser_nmea_fix:main',
                            'gps_utm_fix = libnmea_navsat_driver.nodes.gps_utm_fix:main'
                            ],
    }
)
