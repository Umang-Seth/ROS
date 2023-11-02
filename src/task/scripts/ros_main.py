#!/usr/bin/env python
from pickle import TRUE
from mavros_msgs import msg
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from ros_library import vehicle_ros,FCU_state_alt
import time
from threading import *



class read(Thread):                 #class to read message from vehicle_state
    def run(self):
        #while True:
        FCU_state_alt.FCU_callback
              

if __name__ == '__main__':

    vehicle=vehicle_ros()
    Read=read()
    Read.start()   
    vehicle.setGuided()
    time.sleep(3)
    vehicle.setArm()
    time.sleep(2)
    vehicle.setTakeoff(5)
    time.sleep(8)
    vehicle.setNextpos()
    time.sleep(10)
    vehicle.setLand()

    
    
    #vehicle.move(-35.36224591,149.16445259,10)
    #vehicle.setNextpos()
    
    