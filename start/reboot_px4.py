#!/usr/bin/env python3

"""
Example of connecting to an autopilot via serial communication using pymavlink
"""

# Import mavutil
from pymavlink import mavutil

# Create the connection
# Need to provide the serial port and baudrate
master = mavutil.mavlink_connection("/dev/ttyTHS1", baud=921600)

# Restart the ArduSub board !
master.reboot_autopilot()
