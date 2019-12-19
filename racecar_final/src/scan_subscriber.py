#!/usr/bin/env python
import rospy 
#from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # implement msg  
from racecar_final.msg import Depth
from sensor_msgs.msg import LaserScan


class drive_forward:
    def __init__(self):
        self._proportional_turning_constant = 0.01
        self._angle_diff_thresh = None
	self.depthPrev = 1500	
	self.center = 2500
	self.thresh = 0
	self.jerkThresh = 800
	self.jerkRight = False
	self.jerkLeft = False
        self.vel_topic = '/cmd_vel'
	self.scan_topic = '/scan'
	self.turn_right = False
	self.go_straight = False
	self.turn_left = False
	self.cmd_pub = Twist_float()
        #create publisher passing it the vel_topic_name and msg_type
        self.pub = rospy.Publisher(self.vel_topic, Twist_float, queue_size = 10)

        #create subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)


    def scan_callback(self, data):
        self.depth = data
	#self.depth.right = self.depth.right + 500
	#rospy.loginfo('45 deg distance left: %s, center: %s, right: %s' % (self.depth.left, self.depth.center, self.depth.right))
	
	#if angle_diff < 0:
	#	rospy.loginfo('Turn Right')
	#else:
	#	rospy.loginfo('Turn Left')
	self.left = self.depth[0:200]/(mean(self.depth[0:200]))
	self.center = self.depth[300:340]/(mean(self.depth[300:340]))
	self.right = self.depth[435:635]/(mean(self.depth[435:635]))

	list = [ self.right, self.center, self.left]
	list = [ x for x in list if x != 0]
	self.depth.center = min(list)	
	self.cmd_pub.vel = -0.075
	if self.depth.center > self.center + self.thresh:
		self.turn_right = True
		self.go_straight = False
		self.turn_left = False	
	if  (self.center - self.thresh)< self.depth.center < (self.center + self.thresh):
		self.turn_right = False
		self.go_straight = True
		self.turn_left = False	
	if self.depth.center < (self.center - self.thresh):
		self.turn_right = False
		self.go_straight = False
		self.turn_left = True

	if self.depth.center < self.center - self.jerkThresh and self.turn_left: 
		self.jerkLeft = True
		self.jerkRight = False
	elif self.depth.center > self.center + self.jerkThresh and self.turn_right:
		self.jerkRight = True
		self.jerkLeft = False
	
	if self.depth.center != 0 and centerTrue < self.center + self.jerkThresh * 6:
		if self.turn_right:
			if not self.jerkLeft:
				self.cmd_pub.angle = -0.35
				self.pub.publish(self.cmd_pub)
			else:
				print("Jerk Right")
				self.cmd_pub.angle = -0.65
				self.jerkLeft = False
				self.jerkRight = False
				self.pub.publish(self.cmd_pub)
				rospy.sleep(0.2)	
		elif self.go_straight:
			self.pub.publish(self.cmd_pub)
		elif self.turn_left:
			if not self.jerkRight:
				self.cmd_pub.angle = 0.1
				self.pub.publish(self.cmd_pub)
			else:
				print("Jerk Left")
				self.cmd_pub.angle = .5
				self.jerkRight = False
				self.jerkLeft = False
				self.pub.publish(self.cmd_pub)
				rospy.sleep(0.2)
	elif self.depth.center != 0:
		#if frontTrue > self.center + self.jerkThresh * 2:
		print("Turning")
		self.cmd_pub.angle = -0.7
		self.pub.publish(self.cmd_pub)
		rospy.sleep(0.1)
		#else:
			#print("Avoiding")
			#self.cmd_pub.angle = 0.5
			#self.pub.publish(self.cmd_pub)
			#rospy.sleep(0.15)

	if centerTrue != 0 :
		self.depthPrev = centerTrue	
#try:
	#	self.image.data = [ ord(x) for x in list(data.data)] except: pass

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive_forward()
	rospy.spin()

