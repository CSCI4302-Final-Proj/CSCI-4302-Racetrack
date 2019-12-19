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

import my_follow_corridor

from racecar_flexbe_states.msg import Twist_float 

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
	self.pub = rospy.Publisher('/cmd_vel', Twist_float , queue_size=1)
	self.msg = Twist_float()
	
	self.follow_corridor = my_follow_corridor.HandCodedLaneFollower()

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
	    start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
	    #start_time = time.time()
	    res_image = self.follow_corridor.follow_lane(cv_image)
	    # self.msg.vel = 0
	    # self.msg.angle = angle
        except CvBridgeError as e:
            print(e)
	
	
	#print(self.follow_corridor.curr_steering_angle)
	# self.pub.publish(self.msg)
	cv2.imshow("Image window", res_image)
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
