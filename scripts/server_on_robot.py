#!/usr/bin/env python
from irobot_create.console_interfaces import remote_serial
import rospy

if __name__ == "__main__":
    rospy.init_node("serial_server_on_robot")
    s = remote_serial.RemoteSerialServer()
    s.wait_for_connection()
    s.receive()

