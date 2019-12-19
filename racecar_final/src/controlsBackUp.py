#!/usr/bin/env python
import rospy 
#from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # implement msg  
from realsense_pack.msg import Depth_real



class drive_forward:
    def __init__(self):
        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/depth'
        self.cmd_pub = Twist_float()
        self.angle_diff_prev = 0
        self.kp = 0.9
        self.kd = 0.01
        self.scale = 1
	self.turn_right = False
	self.rurn_left = False
	self.min_angle = 0.15
	self.max_angle = 0.7
         
        #create publisher passing it the vel_topic_name and msg_type
        self.pub = rospy.Publisher(self.vel_topic, Twist_float, queue_size = 10)

        #create subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic, Depth_real, self.scan_callback)


    def scan_callback(self, data):
        self.depth = data
        
        # control scheme
        self.min = 1
	self.max = 1.9
	if self.depth.right < self.min:
        	self.cmd_pub.angle = 0.2
		rospy.loginfo("Adjust Left")
	elif self.depth.right > self.max and self.depth.right < 5:
		self.cmd_pub.angle = -0.2
		rospy.loginfo("Adjust Right")
	elif self.depth.right > 5:
		self.cmd_pub.angle = -0.6
		rospy.loginfo("Turn Right HARD!")
	else:
		rospy.loginfo("Center")

	rospy.loginfo("Right: %s",self.depth.right)
        self.cmd_pub.vel = -0.1
        
	self.pub.publish(self.cmd_pub)
        rospy.sleep(0.1)

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive_forward()
	rospy.spin()
