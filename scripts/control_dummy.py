#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from openwsn_ros.msg import Controls

def callback(inputMsg):
    rospy.loginfo(rospy.get_caller_id() + " Control Input received: %s", inputMsg.prop1)
    
def listener():
	rospy.Subscriber("quad_input", Controls, callback)
	rospy.init_node('mote_handler', anonymous=True)
	print "node initialized"


listener()
rospy.spin()