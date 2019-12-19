#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from racecar_final.msg import Depth
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
	self.pub = rospy.Publisher('/depth_data', Depth, queue_size=1)
	self.pub_msg = Depth()

    def imageDepthCallback(self, data):
	
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
 #           self.center = cv_image[pix[1], pix[0]]
#	    self.right = cv_image[pix[1], pix[0]+150]
#	    self.left = cv_image[pix[1], pix[0]-150]
	    #self.right = [  cv_image[pix[1] , x] for x in range(550,600) ]
	    #self.right = sum(self.right)/len(self.right)
            self.right = self.left = 0
            
	    #self.left = [  cv_image[pix[1] , x] for x in range(50, 100) ]
	    #self.left = sum(self.left)/len(self.left)
            self.center = [ cv_image[pix[1]-30, x] for x in range(data.width)]
	    self.center = sum(self.center)/len(self.center)
	    #sys.stdout.write('%s: Depth at center(%d, %d): %f(mm) ___ Depth at right(%d, %d): %f(mm) ___ Depth at left(%d, %d): %f(mm)\r' \
	#	% (self.topic, pix[1], pix[0], self.center, pix[1], 639, self.right, pix[1], 0, self.left))
	    #sys.stdout.flush()

	    self.pub_msg.left = self.left
	    self.pub_msg.right = self.right
	    self.pub_msg.center = self.center
	    
	    self.pub.publish(self.pub_msg)
	    rospy.sleep(0.0333)
	    #rospy.loginfo(self.left)
	    #sys.stdout.write('%s: Depth at right(%d, %d): %f(mm)\r' % (self.topic, pix[1], 639, cv_image[pix[1], 639]))
            #sys.stdout.flush()
	    #sys.stdout.write('%s: Depth at left(%d, %d): %f(mm)\r' % (self.topic, pix[1], pix[0], cv_image[pix[1], 0]))
	    #rospy.loginfo(self.center)
	    #sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return

def main():
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
