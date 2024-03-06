#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int16

class MotorDriver(object):
	def __init__(self,wheel_distance=0.35,wheel_diameter=0.15):
		self.wheel_distance= wheel_distance
		self.wheel_radius = wheel_diameter/2
		self.pwm1_pub = rospy.Publisher('/rm_pwm',Int16,queue_size=10)
		self.pwm2_pub = rospy.Publisher('/lm_pwm',Int16,queue_size=10)
	
	def __del__(self):
		pass
	def calculate_body_turn_radius(self,linear_speed,angular_speed):
		if angular_speed != 0.0 :
			body_turn_radius= linear_speed/angular_speed
		else:
			#Not turning 
			body_turn_radius = None
		#print("Body turn radius : %s"%(body_turn_radius))
		return body_turn_radius
	
	def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

		if body_turn_radius is not None:
		
		    if wheel == "right":
		        wheel_sign = 1
		    elif wheel == "left":
		        wheel_sign = -1
		    else:
		        assert False, "Wheel Name not supported, left or right only."

		    wheel_turn_radius = body_turn_radius + ( wheel_sign * (self.wheel_distance / 2.0))
		else:
		    wheel_turn_radius = None
		#print("Wheel turn radius: %s"%(wheel_turn_radius))
		return wheel_turn_radius
	
	def calculate_wheel_rpm(self,linear_speed,angular_speed,wheel_turn_radius):
		"""Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        	Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        	--> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        	:param angular_speed:
        	:param wheel_turn_radius:
        	:return:
        	"""
		if wheel_turn_radius is not None:
		    # The robot is turning
		    wheel_rpm = (angular_speed * wheel_turn_radius) / self.wheel_radius
		else:
		    # Its not turning therefore the wheel speed is the same as the body
		    #print(self.wheel_radius)
		    wheel_rpm = linear_speed / self.wheel_radius

		return wheel_rpm
	
	
	
	def set_wheel_movement(self,right_rpm,left_rpm):
		self.set_M1M2_speed(right_rpm,left_rpm)
	
	def set_M1M2_speed(self,right_rpm,left_rpm):
		self.set_M1_speed(right_rpm)
		self.set_M2_speed(left_rpm)
	
	def set_M1_speed(self,rpm):
		if rpm<0:
			self.PWM1=-(min(int(abs(rpm)*10),100))
		else:
			self.PWM1 = min(int(rpm*10),100)
		self.pwm1_pub.publish(self.PWM1)
		#rospy.loginfo("PWM1 : %s"%(self.PWM1))
		
	def set_M2_speed(self,rpm):
		if rpm<0:
			self.PWM2 = -(min(int(abs(rpm)*10),100))
		else:
			self.PWM2 = min(int(rpm*10),100)
		self.pwm2_pub.publish(self.PWM2)
		#rospy.loginfo("PWM2 : %s"%(self.PWM2))		
	
	def set_cmd_vel(self,linear_speed,angular_speed):
		#rospy.loginfo("Set cmd vel")
		body_turn_radius = self.calculate_body_turn_radius(linear_speed,angular_speed)
		wheel="right"
		right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,angular_speed,wheel)
		wheel="left"
		left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,angular_speed,wheel)
		
		right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
		#print("R wheel rpm : %d"%(right_wheel_rpm))
        	left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)
        	#print("L wheel rpm : %d"%(left_wheel_rpm))
        	self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)
