#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def openwsn_client(accelx, ip):
    rospy.wait_for_service('openserver1')
    try:
        open_server = rospy.ServiceProxy('openserver1', MimsyIMU)
        resp1 = open_server(accelx, ip)
        return resp1.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        accelx = int(sys.argv[1])
        ip = sys.argv[2]
    else:
        print usage()
        sys.exit(1)
    rospy.init_node('openwsn_client')
    rate=rospy.Rate(1)

    while not rospy.is_shutdown():
        
        print "Requesting %d+%s"%(accelx, ip)
        print "%d + %s = %s"%(accelx, ip, openwsn_client(accelx, ip))
        rate.sleep()
