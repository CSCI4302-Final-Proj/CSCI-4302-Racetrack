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
        self.waitInterval = 0
        self.topic = topic
        self.bridge = CvBridge()
        self.centerDepth = 3000
        self.turnOne = False
        self.turnTwo = False
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
	self.pub = rospy.Publisher('/cmd_vel', Twist_float , queue_size=1)
	self.msg = Twist_float()
	self.thresh1 = 100
	self.thresh2 = 200
	self.state = 0

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	    # cal forward depth
	    def center_depth():
		pix = (data.width/2, data.height/2)
		l = [ cv_image[x,y] for x  in range(26,32) for y in range(37,41)]
		center_depth = sum(l)/len(l)
#                print(cv_image)
#                print(data);
#		print('Depth at center %d' % center_depth)
		return center_depth

	   
	    self.centerDepth = center_depth() # print center depth
            if self.state == 0 and self.centerDepth < 7500 and self.waitInterval > 100:
                self.msg.vel = -0.08
                self.msg.angle = 0.65
                self.waitInterval = 0
                self.turnOne = True
                print(self.state)
            elif self.turnOne and self.centerDepth > 7500:
		self.state = 1
                self.turnOne = False


            if self.state == 1 and self.centerDepth < 6500 and self.waitInterval > 40:
                self.msg.vel = -0.08
                self.msg.angle = 0.65
                self.waitInterval = 0
                self.turnTwo = True
                print(self.state)
            elif self.turnTwo and self.centerDepth > 7500:
		self.state = 2
                self.turnTwo = False
                
            if not self.turnOne and not self.turnTwo:
                self.waitInterval += 1
	        minVal, maxVal, _, _= cv2.minMaxLoc(cv_image)
	        #print(maxVal)
	        maxVal = 6959.0
	        cv_image = cv2.convertScaleAbs(cv_image,alpha=(255/maxVal))
	        blurred = cv2.GaussianBlur(cv_image, (31,31),0)
	        if self.state == 0:
			ret, thresh = cv2.threshold(blurred, self.thresh1, 255, 0)# hallway: 1:100 , hallway 2: 
	        elif self.state == 1:
			ret, thresh = cv2.threshold(blurred, self.thresh2, 255, 0)
		elif self.state == 2:
			ret, thresh = cv2.threshold(blurred, self.thresh1, 255, 0)
	        
		# find contour
	        cnts = cv2.findContours(thresh,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

	   
	        width, height = cv_image.shape 
	        centroid = (int(width/2), int(height/2))
	        best_cX = 0
	        for c in cnts:
		    #print('within contour for loop')
		    #area = cv2.contourArea(c)
		
		    #if not AREA/100 <area <AREA/20:
		    #    continue
		
                    #compute the center of the contour
                    M = cv2.moments(c)
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])

                    if cX > best_cX:
                        best_cX = cX
                        centroid = (cX,cY)
                        # draw the contour and center of the shape on teh image
                        '''
                    mask = np.zeros(thresh.shape, 'uint8')

                    cv2.drawContours(mask, [c], -1, 255, 2)
                    x,y,w,h = cv2.boundingRect(c)
                    mask = cv2.rectangle(mask,(x,y),(x+w,y+h),255,2)
                    cv2.circle(mask, (cX,cY), 2, 255, -1)
                    '''			

                def calculate_heading(centroid):
                    # calculate heading
                       width, height = cv_image.shape

                       camera_mid_offset_percent = 0.00  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
                       mid = int((width / 2 )* (1 + camera_mid_offset_percent))

                       # calculate centroid offset from mid
                       x_offset = centroid[0] - mid ; y_offset = height - centroid[1]

                       angle_to_mid_radian = math.atan(x_offset / float(y_offset))

                       #print('angle heading: %s' % (np.rad2deg(angle_to_mid_radian)))

                      # cv2.line(cv_image, (mid , height), (centroid[0], centroid[1]), (0,255,0), 2)

                       return max(-0.78, min(0.78 ,angle_to_mid_radian - (angle_to_mid_radian % 0.05)))


                def calculate_speed(speed):
                    return speed



                heading = calculate_heading(centroid)
#                speed = calculate_speed(-0.2)

                self.msg.angle = -heading
                self.msg.vel = -0.25#speed
                if self.centerDepth < 9000:
                    self.msg.vel = -0.1

	except CvBridgeError as e:
            print(e)

	except ZeroDivisionError as e:
	    print(e)	
	
	self.pub.publish(self.msg)
	#cv2.imshow("original image", cv_image)
        #cv2.waitKey(1)

	#cv2.imshow("Image window", thresh)
	#cv2.waitKey(1)
	#cv2.imshow("mask", mask)
	#cv2.waitKey(1)

	#print( time.time() - start_time)



    def calculate_heading(self):
	pass

    def pid(self):
	pass
   
    def ShapeDetector(self):
	pass

def main():
    rospy.init_node('image_listener', anonymous=True)
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
	print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
