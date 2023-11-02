#!/usr/bin/env python

import rospy
from mavros_msgs.msg import PositionTarget
import math
import time

def main():
    # Initialize the ROS node
    rospy.init_node('setpoint_publisher', anonymous=True)

    # Create a publisher to publish setpoints to "/mavros/setpoint_raw/local" topic
    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)
    time_now = int(time.time())
    while not rospy.is_shutdown():
        # Create a PositionTarget message
        setpoint_msg = PositionTarget()

        # Fill in the setpoint message fields
        setpoint_msg.header.stamp = rospy.Time.now()
        setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Set the position setpoints (in meters)
        # setpoint_msg.position.x = 0.0  # desired x position (adjust as needed)
        # setpoint_msg.position.y = 0.0  # desired y position (adjust as needed)
        # setpoint_msg.position.z = 0.0  # desired z position (adjust as needed)

        # Set the velocity setpoints (in m/s)
        setpoint_msg.velocity.x = 5.0*math.sin((2*math.pi)*((int(time.time())-time_now)/4))  # desired x velocity (adjust as needed)
        setpoint_msg.velocity.y = 5.0*math.cos((2*math.pi)*((int(time.time())-time_now)/4))  # desired y velocity (adjust as needed)
        setpoint_msg.velocity.z = 5.0  # desired z velocity (adjust as needed)

        # Set the yaw setpoint (in radians)
        setpoint_msg.yaw = 0.0  # desired yaw angle (adjust as needed)

        # Publish the setpoint message
        setpoint_pub.publish(setpoint_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
