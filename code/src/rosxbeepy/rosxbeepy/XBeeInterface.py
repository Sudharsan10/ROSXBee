#!/usr/bin/env python3
"""
XBeeInterface.py exposes the XBee module within ROS environment

"""
# ==================================================================================================================== #
# Meta Data and future imports
# ==================================================================================================================== #
from __future__ import print_function

# ---> Metadata <--- #
__author__ = 'Sudharsan'
__version__ = '1.0'
__data__ = 'Mar 8, 2021'

# ==================================================================================================================== #
# Import Section
# ==================================================================================================================== #

# ---> ROS2 import <--- #
import rclpy
from rclpy.node import Node

# ---> Digi-Xbee Imports <--- #
from digi.xbee.packets.base import DictKeys
from digi.xbee.packets.common import TransmitStatusPacket, TransmitStatus
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress, XBeeMessage

# ---> Miscellaneous import <--- #
import json
import cbor2
import platform

# ---> Exceptions <--- #
from rclpy.exceptions import ROSInterruptException
from digi.xbee.exception import TransmitException, XBeeException
from serial.serialutil import SerialException


# ==================================================================================================================== #
# Class definitions
# ==================================================================================================================== #

class XBeeInterfaceNode(Node):
    coordinator: str = '0'
    __platform__: str = platform.system()

    """"

    """

    def __init__(self):
        """
        Creates a ROS2 Node for interfacing with XBee Device
        """
        super().__init__('XBeeInterface')

        # ---> Declare ROS Parameters <--- #
        self.declare_parameter('xbee_device_port_id', '/dev/ttyUSB0' if self.__platform__ == 'Linux' else 'COM0')
        self.declare_parameter('baud_rate', 115200)

        # ---> Get Xbee Device Object <--- #
        try:
            self.xbee = XBeeDevice(self.get_parameter('xbee_device_port_id').value,
                                   self.get_parameter('baud_rate').value)
            self.xbee.open()
            self.get_logger().info('Successfully interfaced XBee module with node ID: ', self.xbee.get_node_id())
        except XBeeException as error:
            self.get_logger().info('XBee Exception Occurred, ERROR: ', error)
        except SerialException as error:
            self.get_logger().info('Cannot establish  connection with the XBee Module,: ' + str(error))
        except Exception as error:
            self.get_logger().info('Exception occurred,  Error: ', error)
        finally:
            # self.set_call_back()
            pass

    def transmit(self, data: bytes, recipient_address: str = coordinator) -> TransmitStatusPacket:
        """
        Takes the address of the receiver xbee module and transmit the data to the receiver

        :param data: Data in bytes to be transmitted to the desired xbee module/device
        :type data: bytes
        :param recipient_address: 64 bit xbee address of the recipient module/device
        :type recipient_address: str
        :return: Digi-xbee 'TransmitStatusPacket' object, which gives us the transmit status of the transmitted data
        """
        try:
            if type(data) is not bytes or type(recipient_address) is not str:
                raise TypeError('Args "data" must be bytes & "recipient_address" must be a str')
            if not self.xbee.is_open():
                self.xbee.open()

                # Instantiate a remote XBee device object
                target = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(recipient_address))

                # Send data using the remote object
                status_packet: TransmitStatusPacket = self.xbee.send_data(target, data)
                return status_packet
        except TypeError as e:
            self.get_logger().info('Bad arg types, ERROR: ', e)
        except TransmitException as e:
            self.get_logger().info('Failed to transmit data, ERROR: ', e.status)


# ==================================================================================================================== #
# ==================================================================================================================== #
# Main function declaration
# ==================================================================================================================== #
def main(args=None) -> None:
    """
    ROS2 Entry point for the Node creation
    Args:
        args: None

    Returns: None

    """
    try:
        rclpy.init(args=args)
        node = XBeeInterfaceNode()
        rclpy.spin(node)
    except ROSInterruptException as e:
        print('Failed to create a ROS2 Node: ', e)
    finally:
        rclpy.shutdown()


# ==================================================================================================================== #
if __name__ == '__main__':
    main()
