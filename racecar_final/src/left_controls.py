#!/usr/bin/env python
import rospy 
#from geometry_msgs.msg import Twist
from racecar_flexbe_states.msg import Twist_float # implement msg  
from realsense_pack.msg import Depth_real



class drive_forward:
    def __init__(self):
        self.state = 0
        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/depth'
        self.cmd_pub = Twist_float()

	#Initialize default variables
        self.angle_diff_prev = 0
        self.kp = 1.1
        self.kd = 0.00
	self.error_prev = 0.0
	self.turn_right = False
	self.turn_left = False

	self.min_angle = 0.15
	self.max_angle = 0.7
	self.angle_range = self.max_angle - self.min_angle
	self.depth_max = 6.0
	self.depth_min = 0.2

	self.turn_flag = False

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = rospy.Publisher(self.vel_topic, Twist_float, queue_size = 10)

        #create subscriber
        self.scan_sub = rospy.Subscriber(self.scan_topic, Depth_real, self.scan_callback)


    def scan_callback(self, data):
        self.depth = data

        if self.state == 0:
            self.kp = 1.1
            self.kd = 0.15
            self.left_default = 2.0
            self.cmd_pub.vel = -0.3
        
	if self.state == 1:
            self.kp = 1.1
            self.kd = 0.15
            self.left_default = 2.1
            self.cmd_pub.vel = -0.3
        
	if self.state == 2:
            self.kp = 1.1
            self.kd = 0.00
            self.left_default = 1.9
            self.cmd_pub.vel = -0.3 
	
	if self.depth.left > self.depth_max:
		self.depth.left = self.depth_max
        
	if self.depth.left < self.depth_min:
		self.depth.left = self.depth_min

	# check if depth actual is less than depth min or greater than depth max
	#if self.depth.right < self.depth_min and self.depth.right > 0:
	#	self.depth_min = self.depth.right

        # PD control scheme
	self.error = self.left_default - self.depth.left #if error is neg -> turn left
        self.prop = self.kp * self.error
        self.deriv = self.kd * (self.error_prev - self.error)
        self.control_output = self.prop + self.deriv

	self.error_prev = self.error

	# convert depth output to angle output
	self.control_max = self.kp * (self.left_default - self.depth_max)
	self.control_min = self.kp * (self.left_default - self.depth_min) 
	self.control_range_min = abs(self.control_min)
	self.control_range_max = abs(self.control_max) #self.left_default - ...

	self.ratio_min = self.control_output/(self.control_range_min + self.control_min)
	self.ratio_max = self.control_output/(self.control_range_max - self.control_min)

	if self.error < 0:
		self.angle_output = (self.angle_range * self.ratio_min)
	else:
		self.angle_output = (self.angle_range * self.ratio_max)

#	rospy.loginfo('Control Out: %s, Control Range Max: %s, Control Range Min: %s, Ratio Max: %s, Ratio Min: %s', self.control_output, self.control_range_max, self.control_range_min, self.ratio_max, self.ratio_min)
	#rospy.loginfo('Depth Min: %s, Depth Max: %s', self.depth_min, self.depth_max)
#	rospy.loginfo('Prop: %s, Deriv: %s, Control Out: %s, Angle Out: %s', self.prop, self.deriv, self.control_output, self.angle_output)
	
	# command angle and velocity -> publish
	
	self.cmd_pub.angle = self.angle_output

        # First Turn 
	if self.depth.center < 7 and self.state == 0: #must be 7.0 meters with full batteries
		self.cmd_pub.vel = -0.08
		self.cmd_pub.angle = 0.78
		self.turn_flag = True 

        if self.state == 0 and self.turn_flag == True and self.depth.center > 7.0:
		self.state = 1
		self.turn_flag = False

        # Second Turn
      	if self.depth.center < 7 and self.state == 1:
		self.cmd_pub.vel = -0.05
		self.cmd_pub.angle = 0.78
                self.turn_flag = True

	if self.state == 1 and self.turn_flag == True and self.depth.center > 5.5:
		self.state = 2
		self.turn_flag = False


        
	self.pub.publish(self.cmd_pub)
        rospy.sleep(0.1)

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive_forward()
	rospy.spin()
