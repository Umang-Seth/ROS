#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

pose = [1,2,3]          

def euclidean_distance(goal_pose):
    return math.sqrt(math.pow((goal_pose.pose.pose.position.x - pose[0]), 2) + math.pow((goal_pose.pose.pose.position.y - pose[1]), 2))
    
def steering_angle(goal_pose): 
    return abs(math.atan2(goal_pose.pose.pose.position.y , goal_pose.pose.pose.position.x) - pose[2])




def linear_vel(goal_pose, constant=1.5):
    return constant * euclidean_distance(goal_pose)

def angular_vel(goal_pose, constant=6):
    return constant * (steering_angle(goal_pose))
    

def odom_callback(data):
    global pose
    pose = data
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]





def control_loop():
    rospy.init_node('ebot_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)
    global pose
    pose[0] = Odometry()
    pose[1] = Odometry()
    pose[2] = Odometry()
    vel_msg = Twist()

    goal_pose = Odometry()
    goal_pose.pose.pose.position.x = input('x') 
    goal_pose.pose.pose.position.y = input('y')
    distance_tolerance = 0.1
    angle_curr = 0.01

    while steering_angle(goal_pose)>=angle_curr :
        vel_msg.angular.z = angular_vel(goal_pose)
        pub.publish(vel_msg)
        rospy.loginfo('posew: {}'.format(steering_angle(goal_pose)))
        rate.sleep()

    while euclidean_distance(goal_pose) >= distance_tolerance:
        vel_msg.linear.x = linear_vel(goal_pose)
        pub.publish(vel_msg)
        rospy.loginfo('euclidean_distance: {}'.format(euclidean_distance(goal_pose)))
        rate.sleep()
        
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

