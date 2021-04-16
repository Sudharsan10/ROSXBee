"""
XBeeInterface.py exposes the XBee module within ROS environment

AUTHORS: Sudharsan
GitHub: Sudharsan10
"""
# ==================================================================================================================== #
# Meta Data and future imports
# ==================================================================================================================== #
from __future__ import print_function

# ---> Metadata <--- #
__author__ = 'Sudharsan'
__version__ = '1.0'
__data__ = 'Mar 12, 2021'

# ==================================================================================================================== #
# Import Section
# ==================================================================================================================== #

# ---> ROS2 import <--- #
from launch import LaunchDescription
from launch_ros.actions import Node


# ==================================================================================================================== #
def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    xbee_interface_node = Node(
        package='rosxbeepy',
        executable='XBeeInterface',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baud_rate': 115200}
        ]
    )
    ld.add_action(xbee_interface_node)
    return ld

# ==================================================================================================================== #
