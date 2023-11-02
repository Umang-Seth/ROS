#!/usr/bin/env python

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped  # Add this line for the PoseStamped message
from mavros_msgs.srv import CommandBool, SetMode,CommandBoolRequest

# Define the destination waypoint (10, 10, 2) in local coordinates
DESTINATION_X = 10.0
DESTINATION_Y = 10.0
DESTINATION_Z = 2.0

# Global variables for the current position
current_x = 0.0
current_y = 0.0
current_z = 0.0

# Callback function to receive the current position of the drone
def local_position_cb(msg):
    global current_x, current_y, current_z
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y
    current_z = msg.pose.position.z

# Function to arm and takeoff the drone
def arm_and_takeoff():
    rospy.loginfo("Arming the drone...")
    while not rospy.is_shutdown():
        if current_z > 1.0:  # Check if the drone is already in the air
            rospy.loginfo("Drone is already in the air.")
            break

        # Call the arming service
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True  # Set the value field as a boolean (True for arming)
        if arming_client(arm_cmd):
            rospy.loginfo("Drone armed.")
            break
        rospy.sleep(1.0)

    rospy.loginfo("Taking off...")
    while not rospy.is_shutdown():
        if current_z > 1.0:  # Check if the drone has taken off
            rospy.loginfo("Drone in the air.")
            break
        setpoint_target(DESTINATION_X, DESTINATION_Y, DESTINATION_Z)
        rospy.sleep(0.1)

# Function to set the target position for the drone
def setpoint_target(x, y, z):
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                       PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                       PositionTarget.IGNORE_YAW_RATE

    target.position.x = x
    target.position.y = y
    target.position.z = z

    target.velocity.x = 0.0
    target.velocity.y = 0.0
    target.velocity.z = 0.0

    target.yaw = 0.0

    target_pub.publish(target)

    rospy.loginfo(f"Setpoint target set to: ({x}, {y}, {z})")

if __name__ == "__main__":
    rospy.init_node("local_position_navigation")

    # Subscribe to the local position topic
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_cb)

    # Initialize publisher for the setpoint target
    target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    # Initialize clients for services
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # Wait for services to become available
    arming_client.wait_for_service()
    set_mode_client.wait_for_service()

    # Arm and takeoff
    arm_and_takeoff()

    # Send the target position and navigate to the destination
    setpoint_target(DESTINATION_X, DESTINATION_Y, DESTINATION_Z)

    rospy.spin()
