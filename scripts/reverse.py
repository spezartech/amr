#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

def publisher():
	pub=rospy.Publisher('/motor_cmd',String,queue_size=10)
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		motor_dir="reverse"
		rospy.loginfo(motor_dir)
		pub.publish(motor_dir)
		rate.sleep()
	
if __name__ == '__main__':
	global motor_dir
	rospy.init_node('motor_cmd_pub',anonymous=True)
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
