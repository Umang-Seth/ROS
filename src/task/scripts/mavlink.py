from pymavlink import mavutil
from pymavlink import mavwp
import time

master = mavutil.mavlink_connection("udp:localhost:14550")

master.wait_heartbeat()

wp = mavwp.MAVWPLoader()

master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        0, # lat
        0, # lon
        0)
 