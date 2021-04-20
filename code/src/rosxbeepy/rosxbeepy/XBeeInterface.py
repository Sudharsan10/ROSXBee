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
__data__ = 'Mar 8, 2021'

# ==================================================================================================================== #
# Import Section
# ==================================================================================================================== #

# ---> ROS2 import <--- #
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# ---> Digi-Xbee Imports <--- #
from digi.xbee.packets.base import DictKeys
from serial.serialutil import SerialException
from digi.xbee.exception import TransmitException, XBeeException
from digi.xbee.packets.common import TransmitStatusPacket, TransmitStatus
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress, XBeeMessage

# ---> Miscellaneous import <--- #
import platform

# ---> Other Imports <--- #
from rosxbeepy.action import Tx


# ==================================================================================================================== #
# Class definitions
# ==================================================================================================================== #

class XBeeInterfaceNode(Node):
    """

    """
    coordinator: str = '0'
    __platform__: str = platform.system()

    def __init__(self):
        """
        Creates a ROS2 Node for interfacing with XBee Device
        """
        super().__init__('XBeeInterface')

        # ---> Declare ROS Parameters <--- #
        self.declare_parameter('port', '/dev/ttyUSB0' if self.__platform__ == 'Linux' else 'COM0')
        self.declare_parameter('baud_rate', 115200)

        # ---> Get Xbee Device Object <--- #
        try:
            self.xbee = XBeeDevice(self.get_parameter('port').value,
                                   self.get_parameter('baud_rate').value)
            self.xbee.open()
            self.get_logger().info('Successfully interfaced XBee module with node ID: ' + self.xbee.get_node_id())
        except XBeeException as error:
            self.get_logger().info('XBee Exception Occurred, ERROR: ' + str(error))
        except SerialException as error:
            self.get_logger().info('Cannot establish  connection with the XBee Module,: ' + str(error))
        except Exception as error:
            self.get_logger().info('Exception occurred,  Error: ' + str(error))
        finally:
            # self.set_call_back()
            pass

        try:
            self.transmission_server = ActionServer(
                self,
                Tx,
                'XBeeTransmit',
                execute_callback='',
                callback_group=ReentrantCallbackGroup(),
                handle_accepted_callback=self.handle_accepted_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback
            )
        except ROSInterruptException as e:
            self.get_logger().info('ROS Exception Occurred, ERROR: ' + str(e))
        finally:
            pass

    def destroy(self) -> None:
        """
        Destroy transmission action server along with the XBeeInterface node

        :return: None
        """
        self.transmission_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_handle) -> GoalResponse:
        """
        Accept or reject a client request to begin an action.
        :param goal_handle:
        :return:
        """
        self.get_logger().info('Received Goal Request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Accept or reject a client request to begin an action.
        :param goal_handle:
        :return:
        """
        self.get_logger().info('Received Cancel Request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        Provide a handle to an accepted goal.
        :param goal_handle:
        :return:
        """
        self.get_logger().info('Deferring Execution ...')

    def execute_callback(self, goal_handle) -> Tx.Result:
        """
        Execute goal

        :param goal_handle:
        :return:
        """
        self.get_logger().info('Executing the goal')

        # ---> Feedback & Result <--- #
        feedback_msg = Tx.Feedback()
        result_msg = Tx.Result()

        # ---> Executing the Goal <--- #

        # ---> If all the msg are sent successfully to the recipient <--- #
        if True:
            goal_handle.succeed()
        # ---> Return the Result <--- #
        self.get_logger().info('Finished executing goal, Returning result')
        return result_msg

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
    try:
        rclpy.init(args=args)
        node = XBeeInterfaceNode()
        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
        node.transmission_server.destroy()
    except ROSInterruptException as e:
        print('Failed to create a ROS2 Node: ', e)
    except KeyboardInterrupt as e:
        print('Exiting due to keyboard interrupt: ', e)
    finally:
        rclpy.shutdown()


# ==================================================================================================================== #
if __name__ == '__main__':
    main()
