#!/usr/bin/env python
#from pickle import TRUE
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,Imu
from mavros_msgs.srv import *

import sys
import rospy
from mavros_msgs.msg import State
from std_msgs.msg import String,Float64
import time

#global variable
latitude =0.0
longitude=0.0


class vehicle_ros:

    def setGuided(self):
        flightModeService1 = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService1(custom_mode='GUIDED')

    '''def setCommand(self):
        commandservice=rospy.ServiceProxy('/mavros/cmd/command',mavros_msgs.srv.CommandLong)
        command=commandservice('')


    def move(self, lat: float, lon: float, alt: float):
        data = mavros_msgs.msg.GlobalPositionTarget()
        data.latitude = lat
        data.longitude = lon
        data.altitude = alt
        self.TOPIC_SET_POSE_GLOBAL.set_data(data)
        self.topic_publisher(topic=self.TOPIC_SET_POSE_GLOBAL)

    def fileopen(self):
        command'''
        
    def setArm(self):
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    
    def setDisarm(self):
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    
    def setStabilize(self):
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='STABILIZE')

    def setTakeoff(self,alt):
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = alt, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)

    def setNextpos(self):
        nextposService = rospy.ServiceProxy('/mavros/cmd/command', mavros_msgs.srv.CommandLong) 
        nextposService(latitude = -35.36230206, longitude = 149.16428736)

    def setDisarm():
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)

    def setLand(self):
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)

class FCU_state_alt:

    def __init__(self):
        
        rospy.init_node('FCU_state_alt')
        self.FCU_sub=rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.FCU_callback)   
        self.FCU_status ="Initialising"
        print("height")

    def FCU_callback(self,msg):
        self.FCU_status=msg.data    
        print ("height= : ", self.FCU_status)
