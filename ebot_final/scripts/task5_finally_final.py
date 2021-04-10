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

    def display(self, detected_ob):

        for ob in detected_ob:
            h = ob['height']
            w = ob['width']
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            print(pts)
            homography_matrix = np.array([[ob['homography'][0], ob['homography'][1], ob['homography'][2]],
                                         [ob['homography'][3], ob['homography'][4], ob['homography'][5]],
                                         [ob['homography'][6], ob['homography'][7], ob['homography'][8]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(pts, homography_matrix)

            print(dst)
            self.cv_image = cv2.polylines(self.cv_image, [np.int32(dst)], True, (255, 0, 0), 3)
            self.cv_image = cv2.putText(self.cv_image, ob['name'], tuple(np.int32(dst[0][0])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("Detected Objects", self.cv_image)
        cv2.waitKey(0)

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
    """
    #get pose of battery
    def detection_battery(self):
        flag = 0
        while(flag!=1):
            try:
                (trans2,rot2) = self._listener.lookupTransform('/ebot_base', '/object_2', rospy.Time(0))
                flag=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return trans2
    """
    
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

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/ebot_ws/src/ebot_final/launch/find_object.launch"])

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

    dropbox_1 = [math.radians(90),
                 math.radians(8),
                 math.radians(-85),
                 math.radians(61),
                 math.radians(86),
                 math.radians(0)]

    dropbox_2 = [math.radians(-92),
                 math.radians(8),
                 math.radians(-85),
                 math.radians(61),
                 math.radians(86),
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
    """
    detect_glue = [math.radians(-146),
                   math.radians(-35),
                   math.radians(-9),
                   math.radians(58),
                   math.radians(41),
                   math.radians(-37)]
    """
    detect_dropcoke = geometry_msgs.msg.Pose()
    detect_dropcoke.position.x = 0.326550221367
    detect_dropcoke.position.y = -0.377683366984
    detect_dropcoke.position.z = 1.16725582614
    detect_dropcoke.orientation.x = -0.999763997673
    detect_dropcoke.orientation.y = -0.0214736360203
    detect_dropcoke.orientation.z = 0.0025386768878
    detect_dropcoke.orientation.w = 0.00209452423633

    detect_dropglue = geometry_msgs.msg.Pose()
    detect_dropglue.position.x = 0.326550221367
    detect_dropglue.position.y = -0.377683366984
    detect_dropglue.position.z = 1.16725582614
    detect_dropglue.orientation.x = -0.999763997673
    detect_dropglue.orientation.y = -0.0214736360203
    detect_dropglue.orientation.z = 0.0025386768878
    detect_dropglue.orientation.w = 0.00209452423633
    """
    detect_glue = geometry_msgs.msg.Pose()
    detect_glue.position.x = -0.0234310605697
    detect_glue.position.y = -0.234722350945
    detect_glue.position.z = 0.997379746652
    detect_glue.orientation.x = -0.967825130008
    detect_glue.orientation.y = 0.00304489785874
    detect_glue.orientation.z = 0.00543105138463
    detect_glue.orientation.w = 0.251546715348
    """
    detect_store1 = geometry_msgs.msg.Pose()
    detect_store1.position.x = 0.160503532017
    detect_store1.position.y = -0.201101669171
    detect_store1.position.z = 0.886793430379
    detect_store1.orientation.x = 0.700890347937
    detect_store1.orientation.y = 0.71321644149
    detect_store1.orientation.z = -0.0014438240673
    detect_store1.orientation.w = 0.0085406749432

    detect_store2 = geometry_msgs.msg.Pose()
    detect_store2.position.x = 0.151037606716
    detect_store2.position.y = 0.347212587069
    detect_store2.position.z = 0.878908266532
    detect_store2.orientation.x = 0.700954721993
    detect_store2.orientation.y = 0.713152173748
    detect_store2.orientation.z = -0.00158471660944
    detect_store2.orientation.w = 0.00859903879276

    detect_pantry_left = geometry_msgs.msg.Pose()
    detect_pantry_left.position.x = 0.109282385115
    detect_pantry_left.position.y = -0.0158989961745
    detect_pantry_left.position.z = 0.981413451929
    detect_pantry_left.orientation.w = 0.142067601256
    detect_pantry_left.orientation.x = -0.765647772729
    detect_pantry_left.orientation.y = -0.613819740684
    detect_pantry_left.orientation.z = 0.129713571895

    detect_pantry_right = [math.radians(72),
                          math.radians(-48),
                          math.radians(-16),
                          math.radians(53),
                          math.radians(78),
                          math.radians(-5)]


    gripper_open = {"gripper_finger1_joint": math.radians(0)}
    gripper_close_coke = {"gripper_finger1_joint": math.radians(15)}
    gripper_close_battery = {"gripper_finger1_joint": math.radians(18)}
    gripper_close_glue = {"gripper_finger1_joint": math.radians(15)}
    """
    #roslaunch api
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/umang/catkin_ws/src/ebot_percep/launch/find_object_3d.launch"])

    #pose for detection of object
    detect_joint_angles = [math.radians(0),
                          math.radians(-2),
                          math.radians(-12),
                          math.radians(-16),
                          math.radians(0),
                          math.radians(0)]

    #pose for droping object
    drop_joint_angles = [math.radians(-18),
                          math.radians(-17),
                          math.radians(-27),
                          math.radians(14),
                          math.radians(70),
                          math.radians(0)]

    joint_angles = [math.radians(24),
                    math.radians(10),
                    math.radians(-45),
                    math.radians(67),
                    math.radians(43),
                    math.radians(-54)]

    ur5.set_joint_angles(detect_joint_angles)
    rospy.sleep(2)

    launch.start()#launching find object 3d node and detection info node
    rospy.loginfo("started find object 3d")
    rospy.sleep(2)

    coke = ur5.detection_coke()
    battery = ur5.detection_battery()
    glue = ur5.detection_glue()

    launch.shutdown()#shutting down find object 3d node

    #various poses to grab and drop object

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = coke[0]
    ur5_pose_1.position.y = coke[1] - 0.3
    ur5_pose_1.position.z = 0.9#coke[2] + 0.1
    ur5_pose_1.orientation.x = -0.00301245486045
    ur5_pose_1.orientation.y = 0.948249832483
    ur5_pose_1.orientation.z = -0.317508830355
    ur5_pose_1.orientation.w = 0.00115019913628

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = coke[0]
    ur5_pose_2.position.y = coke[1] - 0.17
    ur5_pose_2.position.z = 0.9#coke[2] + 0.1
    ur5_pose_2.orientation.x = -0.00326370214739
    ur5_pose_2.orientation.y = 0.976543956567
    ur5_pose_2.orientation.z = -0.215291981583
    ur5_pose_2.orientation.w = 0.000782181268205

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = coke[0]
    ur5_pose_3.position.y = coke[1] - 0.25
    ur5_pose_3.position.z = 1.0#coke[2] + 0.2
    ur5_pose_3.orientation.x = -0.00326370214739
    ur5_pose_3.orientation.y = 0.976543956567
    ur5_pose_3.orientation.z = -0.215291981583
    ur5_pose_3.orientation.w = 0.000782181268205

    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = battery[0]
    ur5_pose_4.position.y = battery[1] - 0.3
    ur5_pose_4.position.z = 0.9#battery[2] + 0.1
    ur5_pose_4.orientation.x = -0.00360182775062
    ur5_pose_4.orientation.y = 0.966394888837
    ur5_pose_4.orientation.z = -0.257034633868
    ur5_pose_4.orientation.w = 0.00106895184449

    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = battery[0]
    ur5_pose_5.position.y = battery[1] - 0.18
    ur5_pose_5.position.z = 0.93#battery[2] + 0.1
    ur5_pose_5.orientation.x = -0.00360182775062
    ur5_pose_5.orientation.y = 0.966394888837
    ur5_pose_5.orientation.z = -0.257034633868
    ur5_pose_5.orientation.w = 0.00106895184449

    ur5_pose_6 = geometry_msgs.msg.Pose()
    ur5_pose_6.position.x = battery[0]
    ur5_pose_6.position.y = battery[1] - 0.18
    ur5_pose_6.position.z = 0.95#battery[2] + 0.2
    ur5_pose_6.orientation.x = -0.00360182775062
    ur5_pose_6.orientation.y = 0.966394888837
    ur5_pose_6.orientation.z = -0.257034633868
    ur5_pose_6.orientation.w = 0.00106895184449

    """

    #implemint the various poses defined above
    while not rospy.is_shutdown():

        ur5.set_joint_angles(carry_object)

        #navigate pantry
        ur5.navigation(12.9, 1, 3)
        rospy.sleep(2)

        #detection object in pantry
        '''
        ur5.navigation(14.4, -1, 0)
        rospy.sleep(2)
        ur5.go_to_pose(detect_pantry_left)
        rospy.sleep(2)
        coke = ur5.detection_coke()
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = coke[0]
        ur5_pose_1.position.y = coke[1] - 0.3
        ur5_pose_1.position.z = 1#coke[2] + 0.1
        ur5_pose_1.orientation.x = -0.00301245486045
        ur5_pose_1.orientation.y = 0.948249832483
        ur5_pose_1.orientation.z = -0.317508830355
        ur5_pose_1.orientation.w = 0.00115019913628
        print("coke pose: {}".format(coke))
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)
        #ur5.set_joint_angles(carry_object)
        #rospy.sleep(2)
        '''

        ur5.navigation(11.25, -1, 2)
        rospy.sleep(2)
        ur5.set_joint_angles(detect_pantry_left)
        rospy.sleep(2)

        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)

        coke = ur5.detection_coke()


        launch.shutdown()
        #coke = ur5.detection_coke()

        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = coke[0]
        ur5_pose_1.position.y = coke[1] - 0.3
        ur5_pose_1.position.z = 1#coke[2] + 0.1
        ur5_pose_1.orientation.x = -0.00301245486045
        ur5_pose_1.orientation.y = 0.948249832483
        ur5_pose_1.orientation.z = -0.317508830355
        ur5_pose_1.orientation.w = 0.00115019913628
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)

        ur5_pose_2 = geometry_msgs.msg.Pose()
        ur5_pose_2.position.x = coke[0]
        ur5_pose_2.position.y = coke[1] - 0.17
        ur5_pose_2.position.z = 0.9#coke[2] + 0.1
        ur5_pose_2.orientation.x = -0.00326370214739
        ur5_pose_2.orientation.y = 0.976543956567
        ur5_pose_2.orientation.z = -0.215291981583
        ur5_pose_2.orientation.w = 0.000782181268205
        ur5.go_to_pose(ur5_pose_2)
        rospy.sleep(2)

        ur5.gripper(gripper_close_coke)
        rospy.sleep(2)

        ur5_pose_3 = geometry_msgs.msg.Pose()
        ur5_pose_3.position.x = coke[0]
        ur5_pose_3.position.y = coke[1] - 0.25
        ur5_pose_3.position.z = 1.0#coke[2] + 0.2
        ur5_pose_3.orientation.x = -0.00326370214739
        ur5_pose_3.orientation.y = 0.976543956567
        ur5_pose_3.orientation.z = -0.215291981583
        ur5_pose_3.orientation.w = 0.000782181268205
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)

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
        ur5.set_joint_angles(zero)
        rospy.sleep(2)

        #ur5.go_to_pose(detect_glue)
        ur5.set_joint_angles(detect_joint_angles)
        ur5.set_joint_angles(detect_glue)
        rospy.sleep(2)

        launch.start()#launching find object 3d node and detection info node
        rospy.loginfo("started find object 3d")
        rospy.sleep(2)

        glue = ur5.detection_glue()


        launch.shutdown()
        print glue

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
        ur5_pose_8.position.y = glue[1] - 0.17
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

        #ur5.go_to_pose(detect_store2)
        #rospy.sleep(2)
        #ur5.set_joint_angles(zero)
        #rospy.sleep(2)

        #ur5.set_joint_angles(carry_object)
        #rospy.sleep(2)

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
