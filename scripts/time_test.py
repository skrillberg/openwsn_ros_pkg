#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from openwsn_ros.msg import Controls


    
def listener():
	
	rospy.init_node('mote_handler', anonymous=True)

	while not rospy.is_shutdown():
		now = rospy.Time.now()
		print "Current Time: ", float(now.nsecs)/1000000000


listener()
rospy.spin()