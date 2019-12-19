#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import math
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
	self.rate = rospy.Rate(0.2)
	self.count = 0
	

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
	    start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red = np.array([100, 100, 100])
            upper_red = np.array([179, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)
	    size = 0	
	    for x in range(mask.shape[0]):
		for y in range(mask.shape[1]):
			size += mask[x,y]	
	    print(size)
            # calculate moments of binary frame
            M = cv2.moments(mask)

            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # put text and highlight the center

#            cv2.circle(cv_image, (cX, cY), 5, (0, 255, 0), -1)
#            cv2.putText(cv_image, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            def calculate_heading(centroid):
                # calculate heading
                height, width, _ = cv_image.shape

                camera_mid_offset_percent = 0.00  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
                mid = int((width / 2 )* (1 + camera_mid_offset_percent))

                # calculate centroid offset from mid
                x_offset = cX - mid ; y_offset = height - cY

                angle_to_mid_radian = math.atan(x_offset / float(y_offset))

                print('angle heading: %s' % (np.rad2deg(angle_to_mid_radian)))

#                cv2.line(cv_image, (mid , height), (centroid[0], centroid[1]), (0,255,0), 2)

                return angle_to_mid_radian

            def calculate_speed(speed):
		return speed

	    heading = calculate_heading((cX,cY))
	    speed = calculate_speed(-0.1)
	    self.count += 1
	    #if size < 100:
	#	self.publish_msg(0, heading)
	    if not self.count % 180:
		count = step = self.count
		while count < step + 150:
	    	    self.publish_msg(speed,heading) 
		    count +=1	
	    else:
		self.publish_msg(-0.1 ,heading)
 
        except CvBridgeError as e:
            print(e)

	except ZeroDivisionError as e:
	    print(e)	
	
	#print(self.follow_corridor.curr_steering_angle)
	# self.pub.publish(self.msg)
#	cv2.imshow("original image", cv_image)
#        cv2.waitKey(1)
	'''
	cv2.imshow("Image window", thresh)
	cv2.waitKey(1)
	'''
#	cv2.imshow("mask", mask)
#	cv2.waitKey(1)
	#print( time.time() - start_time)

    def publish_msg(self, speed, heading):
        self.msg.vel = speed
        self.msg.angle = -heading

        self.pub.publish(self.msg)

   

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
