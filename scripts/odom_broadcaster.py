#!/usr/bin/env python2
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

br = tf.TransformBroadcaster()   
def callback(msg):
      rospy.loginfo(msg)
      br.sendTransform((msg.x, msg.y, 0),tf.transformations.quaternion_from_euler(0, 0,1),rospy.Time.now(),"world","plane")
      
      
      
def listener():
      # In ROS, nodes are uniquely named. If two nodes with the same
      # name are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
      rospy.init_node('tf_broadcaster', anonymous=True)
   
      rospy.Subscriber("/odom_msg",Quaternion,callback)
  
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
   
if __name__ == '__main__':
     listener()
