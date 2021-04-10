#! /usr/bin/env python
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



detected_ob = Float32()
img = np.empty((480, 640, 3))


def callback(d):
	global detected_ob
	detected_ob = d.data
	return detected_ob

def im_callback(i):
	global img
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(i, "bgr8")
	return img


def display(ob, cv_image):
	obj_names = {77:"Coke",8:"battery"}
	h = ob[1]
	w = ob[2]
	pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)

	homography_matrix = np.array([[ob[3], ob[6], ob[9]],
	                              [ob[4], ob[7], ob[10]],
	                              [ob[5], ob[8], ob[11]]], dtype=np.float32)

	dst = cv2.perspectiveTransform(pts, homography_matrix)
	cv_image = cv2.polylines(cv_image, [np.int32(dst)], True, (255, 0, 0), 3)
	cv_image = cv2.putText(cv_image, obj_names.get(ob[0]), tuple(np.int32(dst[0][0])),
						cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

	cv2.imshow("Detected image", cv_image)
	cv2.waitKey(5000) #5seconds
	cv2.destroyAllWindows()


def main():
	rospy.init_node('bounding_box', anonymous=True)
	rospy.Subscriber('/objects', Float32MultiArray, callback)
	rospy.Subscriber('/camera/color/image_raw2', Image, im_callback)
	rospy.sleep(2)
	counter = len(detected_ob)//12

	for i in range(counter):
		display(detected_ob[i:i+12],img)




if __name__ == '__main__':
    main()