#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOLRequest, CommandTOL
import time
import math

current_state = State()
current_pos = Odometry()

def state_cb(msg):
    global current_state
    current_state = msg
    
def state_pos(msg):
    global current_pos
    current_pos = msg

# def draw_circle(center_x, center_y, radius):
#     x = 0
#     y = radius
#     d = 1 - radius

#     points = []
#     while x <= y:
#         points.append((x + center_x, y + center_y))
#         points.append((y + center_x, x + center_y))
#         points.append((y + center_x, -x + center_y))
#         points.append((x + center_x, -y + center_y))
#         points.append((-x + center_x, -y + center_y))
#         points.append((-y + center_x, -x + center_y))
#         points.append((-y + center_x, x + center_y))
#         points.append((-x + center_x, y + center_y))

#         if d < 0:
#             d += 2 * x + 3
#         else:
#             d += 2 * (x - y) + 5
#             y -= 1
#         x += 1

#     return points

if __name__ == "__main__":

    center_x = int(0)
    center_y = int(0)
    radius = int(10)

    rospy.init_node("helix_node_takeoff")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    state_sub_pos = rospy.Subscriber("mavros/global_position/local", Odometry, callback = state_pos)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/takeoff")
    takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'GUIDED'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    takeoff_msg = CommandTOLRequest()
    takeoff_msg.min_pitch = 0.0
    takeoff_msg.yaw = 0.0
    takeoff_msg.latitude = 0.0
    takeoff_msg.longitude = 0.0
    takeoff_msg.altitude = 5.0

    last_req = rospy.Time.now()
    time_now = int(time.time())
    while(not rospy.is_shutdown()):
        if(current_state.mode != "GUIDED" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("GUIDED enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    rospy.loginfo("Sending takeoff command...")
                    takeoff_client.call(takeoff_msg)
                    time.sleep(7)

                    # setpoint_msg = PositionTarget()
                    
                    # setpoint_msg.header.stamp = rospy.Time.now()
                    # setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    # setpoint_msg.velocity.x = 5.0*math.sin((2*math.pi)*((int(time.time())-time_now)/4))  
                    # setpoint_msg.velocity.y = 5.0*math.cos((2*math.pi)*((int(time.time())-time_now)/4))
                    # setpoint_msg.velocity.z = 5.0
                    # setpoint_msg.yaw = 0.0
                    # setpoint_pub.publish(setpoint_msg)
                last_req = rospy.Time.now()
        if(current_state.armed and current_pos.pose.pose.position.z > 1.0):    
            setpoint_msg = PositionTarget()
                        
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_msg.velocity.x = 5.0*math.sin((2*math.pi)*((int(time.time())-time_now)/4))  
            setpoint_msg.velocity.y = 5.0*math.cos((2*math.pi)*((int(time.time())-time_now)/4))
            setpoint_msg.velocity.z = 5.0
            setpoint_msg.yaw = 0.0
            setpoint_pub.publish(setpoint_msg)
        # print(int(time.time()))
        # print(type(int(time.time())))
        # print(draw_circle(center_x, center_y, radius))
        rate.sleep()