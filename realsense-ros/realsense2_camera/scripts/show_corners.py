#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import numpy as np
import cv2
import time

from racecar_flexbe_states.msg import Twist_float 

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
	self.pub = rospy.Publisher('/cmd_vel', Twist_float , queue_size=1)
	self.msg = Twist_float()
	

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
	    start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	    gray = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)

	    # FIND HARRIS CORNERS
	    gray = np.float32(gray)
	    dst = cv2.cornerHarris(gray,8,27,0.10)
	    dst = cv2.dilate(dst, None)
	    res, dst = cv2.threshold(dst, 0.01*dst.max(),255,0)
	    '''
	    dst = np.uint8(dst)

	    # find centroids
	    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

	    # define the criteria to stop and refine the corners
	    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
	    corners = cv2.cornerSubPix(gray, np.float32(centroids),(5,5), (-1,-1), criteria)

	    # now draw them
	    res = np.hstack((centroids,corners))
	    res = np.int0(res)
	    cv_image[res[:,1], res[:,0]] = [0,0,255]
	    cv_image[res[:,3], res[:,2]] = [0,255,0]
	    '''
        except CvBridgeError as e:
            print(e)
	
	
	#print(self.follow_corridor.curr_steering_angle)
	# self.pub.publish(self.msg)
	cv2.imshow("Image window", dst)
	cv2.waitKey(1)
	print( time.time() - start_time)

def main():
    rospy.init_node('image_listener', anonymous=True)
    topic = '/camera/color/image_raw'
    listener = ImageListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
	print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
