#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import tf
from object_msgs.msg import ObjectPose
import roslaunch
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import cv2


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm_planning_group" 
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        self._gripper_planning_group = "gripper_planning_group"
        self._gripper_group = moveit_commander.MoveGroupCommander(self._gripper_planning_group)

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        self._listener = tf.TransformListener()
        self._now = rospy.Time.now()
        
        self._client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self._client.wait_for_server()
        
        rospy.loginfo("waiting for server")
        
        self._goal = MoveBaseGoal()
        self._goal.target_pose.header.frame_id = "map"
        self._goal.target_pose.header.stamp = rospy.Time.now()
        
    def go_to_pose(self, arg_pose):

        self._group.set_pose_target(arg_pose)
        self._group.go(wait=True) 

        
    def set_joint_angles(self, arg_list_joint_angles):

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        self._group.go(wait=True)

    def gripper(self, val):
        self._gripper_group.set_joint_value_target(val)
        self._gripper_group.go(wait=True)
        rospy.loginfo("Object picked")

    def turn(self,direction):
        switcher = { 
        0: [0, 0, -0.00152321936501, 0.999998839893],
        1: [0, 0, 0.999999241034, 0.00123201417136], 
        2: [0, 0, -0.70905213339, -0.705156062202],
        3: [0, 0, -0.706662129946, 0.70755115295],
        4: [0, 0, -0.89154808732, 0.452926051287]
        }
        return switcher.get(direction, "nothing")
    
    def navigation(self,w_x,w_y,w_align):
        
        self._goal.target_pose.pose.position.x = w_x
        self._goal.target_pose.pose.position.y = w_y

        self._goal.target_pose.pose.orientation.x = self.turn(w_align)[0]
        self._goal.target_pose.pose.orientation.y = self.turn(w_align)[1]
        self._goal.target_pose.pose.orientation.z = self.turn(w_align)[2]
        self._goal.target_pose.pose.orientation.w = self.turn(w_align)[3]
        self._client.send_goal(self._goal)
        wait = self._client.wait_for_result()
    
    #get pose of coke can
    def detection_coke(self):
        flag = 0
        while(flag!=1):
            try:
                (trans1,rot1) = self._listener.lookupTransform('/ebot_base', '/object_1', rospy.Time(0))
                flag=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 

        return trans1
        
    #get pose of battery
    def detection_battery(self):
        flag = 0
        while(flag!=1):
            try:
                (trans2,rot2) = self._listener.lookupTransform('/ebot_base', '/object_2', rospy.Time(0))
                flag=1
            except (tf.ExtrapolationException):
                continue 

        return trans2
    
    #get pose of coke can
    def detection_glue(self):
        flag = 0
        while(flag!=1):
            try:
                (trans3,rot3) = self._listener.lookupTransform('/ebot_base', '/object_3', rospy.Time(0))
                flag=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 

        return trans3
    
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



def main():
   
    ur5 = Ur5Moveit()

    carry_object = [math.radians(-54),
                    math.radians(11),
                    math.radians(43),
                    math.radians(-85),
                    math.radians(35),
                    math.radians(0)]

    zero = [math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(0)]

    dropbox_1 = [math.radians(98),
                 math.radians(27),
                 math.radians(-106),
                 math.radians(62),
                 math.radians(80),
                 math.radians(0)]

    dropbox_2 = [math.radians(-98),
                 math.radians(27),
                 math.radians(-106),
                 math.radians(62),
                 math.radians(80),
                 math.radians(0)]

    look_left = [math.radians(88),
                 math.radians(-3),
                 math.radians(-28),
                 math.radians(-1),
                 math.radians(90),
                 math.radians(0)]

    look_right = [math.radians(-88),
                  math.radians(-3),
                  math.radians(-28),
                  math.radians(-1),
                  math.radians(90),
                  math.radians(0)]
    
    detect_joint_angles = [math.radians(0),
                          math.radians(-2),
                          math.radians(-12),
                          math.radians(-16),
                          math.radians(0),
                          math.radians(0)]


    detect_glue = [math.radians(25),
                   math.radians(-61),
                   math.radians(3),
                   math.radians(-102),
                   math.radians(-41),
                   math.radians(138)]

    detect_store1 = [math.radians(3),
                   math.radians(-42),
                   math.radians(-12),
                   math.radians(-72),
                   math.radians(-27),
                   math.radians(97)]

    detect_store2 = [math.radians(149),
                   math.radians(-6),
                   math.radians(-24),
                   math.radians(-135),
                   math.radians(-140),
                   math.radians(218)]

    gripper_open = {"gripper_finger1_joint": math.radians(0)}
    gripper_close_coke = {"gripper_finger1_joint": math.radians(15)} 
    gripper_close_battery = {"gripper_finger1_joint": math.radians(18)}
    gripper_close_glue = {"gripper_finger1_joint": math.radians(20)}

    #implemint the various poses defined above 
    while not rospy.is_shutdown():
        ur5.set_joint_angles(carry_object)
        #navigate to store room
        ur5.navigation(25.9, -3.11, 4)
        rospy.sleep(2)

        #object detection in store
        ur5.set_joint_angles(zero)
        ur5.set_joint_angles(detect_store1)
        rospy.sleep(2)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/ebot_ws/src/ebot_final/launch/find_object.launch"])
        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)
    
        battery = ur5.detection_battery()       
        
        launch.shutdown()
        ur5.set_joint_angles(detect_store2)
        rospy.sleep(2)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/ebot_ws/src/ebot_final/launch/find_object.launch"])
        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)
    
        battery = ur5.detection_battery()        
        
        launch.shutdown()
                
        #picking battery
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = battery[0] 
        ur5_pose_1.position.y = battery[1] - 0.3
        ur5_pose_1.position.z = 0.9#glue[2] + 0.1
        ur5_pose_1.orientation.x = -0.00301245486045
        ur5_pose_1.orientation.y = 0.948249832483
        ur5_pose_1.orientation.z = -0.317508830355
        ur5_pose_1.orientation.w = 0.00115019913628
        ur5.go_to_pose(ur5_pose_1)

        ur5_pose_2 = geometry_msgs.msg.Pose()
        ur5_pose_2.position.x = battery[0]
        ur5_pose_2.position.y = battery[1] - 0.2
        ur5_pose_2.position.z = 0.9#glue[2] + 0.1
        ur5_pose_2.orientation.x = -0.00326370214739
        ur5_pose_2.orientation.y = 0.976543956567
        ur5_pose_2.orientation.z = -0.215291981583
        ur5_pose_2.orientation.w = 0.000782181268205
        ur5.go_to_pose(ur5_pose_2)

        ur5.gripper(gripper_close_battery)

        ur5_pose_3 = geometry_msgs.msg.Pose()
        ur5_pose_3.position.x = battery[0]
        ur5_pose_3.position.y = battery[1] - 0.25
        ur5_pose_3.position.z = 0.92#glue[2] + 0.2
        ur5_pose_3.orientation.x = -0.00326370214739
        ur5_pose_3.orientation.y = 0.976543956567
        ur5_pose_3.orientation.z = -0.215291981583
        ur5_pose_3.orientation.w = 0.000782181268205
        ur5.go_to_pose(ur5_pose_3)
        ur5.set_joint_angles(zero)
        ur5.set_joint_angles(carry_object)

        #navigate research lab
        ur5.navigation(25.9, -3.11, 1)
        rospy.sleep(2)
        ur5.navigation(10.75, 6.5, 2)
        rospy.sleep(2)
        ur5.navigation(10.75, 9.5, 2)
        rospy.sleep(2)
        #drop battery
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(dropbox_2)
        rospy.sleep(2)
        ur5.gripper(gripper_open)
        rospy.sleep(2)  
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(carry_object)
        rospy.sleep(2)
        
        """
        #navigate pantry room
        ur5.navigation(13, 1, 3)
        rospy.sleep(2)
        ur5.navigation(13, -1, 3)
        rospy.sleep(2)

        #object detection in pantry

        #picking coke

        #navigate to confrence Room
        ur5.navigation(5.14, 0.89, 3)
        rospy.sleep(2)
        ur5.navigation(5.19, -0.59, 0)
        rospy.sleep(2)

        #drop coke

        #navigate to start
        ur5.navigation(5.14, 0.89, 1)
        rospy.sleep(2)
        ur5.navigation(0, 0, 1)
        rospy.sleep(2)
        """
        """
        ur5.set_joint_angles(carry_object)
        
        #navigate pantry
        ur5.navigation(13, 1, 3)
        rospy.sleep(2)
        ur5.navigation(13, -1, 3)
        rospy.sleep(2)
        
        #detection object in pantry
        #ur5.set_joint_angles(look_left)
        #rospy.sleep(2)
        #ur5.set_joint_angles(look_right)
        #rospy.sleep(2)
        #ur5.set_joint_angles(carry_object)
        #rospy.sleep(2)
        #ur5.navigation(14.6, -1, 0)
        #rospy.sleep(2)
        #ur5.set_joint_angles(zero)
        #ur5.set_joint_angles(detect_coke)        
        #ur5.navigation(13, -1, 3)
        #rospy.sleep(2)
        
        #navigate meeting
        ur5.navigation(13, 2, 3)
        rospy.sleep(2)
        ur5.navigation(13, 1, 1)
        rospy.sleep(2)
        
        
        ur5.navigation(8.7, 1, 2)
        rospy.sleep(2)
        ur5.navigation(8.7, 2.6, 0)
        rospy.sleep(2)
        ur5.navigation(7, 2.6, 0)
        rospy.sleep(2)
        
        #dropping coke
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        #ur5.go_to_pose(detect_dropcoke)
        #rospy.sleep(2)
        ur5.set_joint_angles(dropbox_1)
        rospy.sleep(2)
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(carry_object)
        rospy.sleep(2)
        
        #detection object in meeting 
        ur5.navigation(8, 2.6, 0)
        rospy.sleep(2)
        """
        """
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        
        #ur5.go_to_pose(detect_glue)
        ur5.set_joint_angles(detect_joint_angles)
        ur5.set_joint_angles(detect_glue)
        rospy.sleep(2)
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/ebot_ws/src/ebot_final/launch/find_object.launch"])
        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)
    
        #glue = ur5.detection_glue()
        
        launch.shutdown()
        rospy.sleep(5)
        #print glue
        
        
        ur5_pose_7 = geometry_msgs.msg.Pose()
        ur5_pose_7.position.x = glue[0] 
        ur5_pose_7.position.y = glue[1] - 0.3
        ur5_pose_7.position.z = 0.9#glue[2] + 0.1
        ur5_pose_7.orientation.x = -0.00301245486045
        ur5_pose_7.orientation.y = 0.948249832483
        ur5_pose_7.orientation.z = -0.317508830355
        ur5_pose_7.orientation.w = 0.00115019913628

        ur5_pose_8 = geometry_msgs.msg.Pose()
        ur5_pose_8.position.x = glue[0]
        ur5_pose_8.position.y = glue[1] - 0.2
        ur5_pose_8.position.z = 0.9#glue[2] + 0.1
        ur5_pose_8.orientation.x = -0.00326370214739
        ur5_pose_8.orientation.y = 0.976543956567
        ur5_pose_8.orientation.z = -0.215291981583
        ur5_pose_8.orientation.w = 0.000782181268205

        ur5_pose_9 = geometry_msgs.msg.Pose()
        ur5_pose_9.position.x = glue[0]
        ur5_pose_9.position.y = glue[1] - 0.25
        ur5_pose_9.position.z = 0.92#glue[2] + 0.2
        ur5_pose_9.orientation.x = -0.00326370214739
        ur5_pose_9.orientation.y = 0.976543956567
        ur5_pose_9.orientation.z = -0.215291981583
        ur5_pose_9.orientation.w = 0.000782181268205

        ur5.go_to_pose(ur5_pose_7)
        ur5.go_to_pose(ur5_pose_8)
        ur5.gripper(gripper_close_glue)
        ur5.go_to_pose(ur5_pose_9)
        ur5.set_joint_angles(detect_glue)
        ur5.set_joint_angles(detect_joint_angles)
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(carry_object)
        rospy.sleep(2)
        #ur5.go_to_pose(ur5_pose_1)
        
                
        #navigate research
        ur5.navigation(8.7, 2.6, 3)
        rospy.sleep(2)
        ur5.navigation(8.7, 1, 0)
        rospy.sleep(2)
        ur5.navigation(10.75, 9.5, 2)
        rospy.sleep(2)
    
        #dropping glue
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(dropbox_2)
        rospy.sleep(2)
        ur5.gripper(gripper_open)
        rospy.sleep(2)  
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(carry_object)
        rospy.sleep(2)
        

        #navigating store
        #ur5.navigation(11, 5, 3)
        #rospy.sleep(2)
        #ur5.navigation(25.9, -3.11, 4)
        #rospy.sleep(2)
        #ur5.set_joint_angles(zero)
        #rospy.sleep(2)

        #object detection in store
        #ur5.go_to_pose(detect_store1)
        #rospy.sleep(2)
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/ebot_ws/src/ebot_final/launch/find_object.launch"])
        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)
    
        
        
        launch.shutdown()
        
        #ur5.go_to_pose(detect_store2)
        #rospy.sleep(2)
        #ur5.set_joint_angles(zero)
        #rospy.sleep(2)

        #ur5.set_joint_angles(carry_object)
        #rospy.sleep(2)
        """
        """
        #navigate confrence
        ur5.navigation(5.14, 0.89, 3)
        rospy.sleep(2)
        ur5.navigation(5.19, -0.59, 0)
        rospy.sleep(2)
        
        #dropping FPGA board
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(dropbox_1)
        rospy.sleep(2)
        ur5.set_joint_angles(zero)
        rospy.sleep(2)
        ur5.set_joint_angles(carry_object)
        rospy.sleep(2)
        
        #navigate end
        ur5.navigation(5.14, 0.89, 1)
        rospy.sleep(2)
        ur5.navigation(0, 0, 1)
        rospy.sleep(2)
        """       
        
        #


        break
    del ur5


if __name__ == '__main__':
    main()

"""
wtr ebot_base
x: 0.00204150091959
y: -0.321844714622
z: 1.00244009548

q_x: -0.965363078658
q_y: -0.00425616058315
q_z: -9.86255806213e-05
q_W: 0.260875452533

0: [-1.41033694091e-06, 3.56947591957e-06, -0.00152321936501, 0.999998839893],
1: [-7.12779797728e-06, -4.68549619016e-06, 0.999999241034, 0.00123201417136], 
2: [8.45054925629e-06, -1.92801666487e-06, -0.70905213339, -0.705156062202],
3: [-2.3994573094e-06, 7.34160739442e-06, -0.706662129946, 0.70755115295],
4: [1.70291024232e-07, -3.36456766153e-06, -0.285511727824, 0.95837521528]
        
"""