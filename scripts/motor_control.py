#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from motor_driver import MotorDriver

wheel_distance = 0.35
wheel_diameter = 0.15

class Robot_Mover(object):

	def __init__(self):
		rospy.Subscriber('/cmd_vel',Twist,self.cmd_vel_callback)
		rospy.loginfo("Robot Mover started...")	
		self.motor_driver = MotorDriver(wheel_distance,wheel_diameter)
		
	def cmd_vel_callback(self,msg):
		linear_speed=msg.linear.x
		angular_speed=msg.angular.z
		#print("Linear speed :")
		#print(linear_speed)
		#print("Angular speed :")
		#print(angular_speed)
		self.motor_driver.set_cmd_vel(linear_speed,angular_speed)
		
	def listener(self):
		rospy.spin()
	
if __name__ == '__main__':
	rospy.init_node('cmd_vel_listener',anonymous=True)
	try:
		robot_mover = Robot_Mover()
		robot_mover.listener()
	except rospy.ROSInterruptException:
		pass
