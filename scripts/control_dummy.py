#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Bool,Float32
from openwsn_ros.msg import Controls
paused = False

def callback(inputMsg):
    rospy.loginfo(rospy.get_caller_id() + " Control Input received: %s at time %s", inputMsg.prop1, inputMsg.prop2)
    rate.sleep()
    msg = True
    pub.publish(msg)

def pause_callback(msg):
    global paused
    paused = msg.data
    #rospy.loginfo("Gazebo Paused Status: " + str(paused))	

  
#this code will simulate an update rate for gazebo time synchronization.
rospy.Subscriber("quad_input", Controls, callback)
pub = rospy.Publisher("sim_status",Bool,queue_size = 1)
pub_time = rospy.Publisher("gazebo_time_dummy",Float32, queue_size=1)
rospy.init_node('control_dummy', anonymous=True)
rate = rospy.Rate(1)
time_rate = 100.0
timeupdate = rospy.Rate(time_rate)
time = 0
s = rospy.Subscriber('gazebo_pause',Bool,pause_callback)
print "control dummy initialized"

#rospy.spin()
while(1):
    
    timeupdate.sleep()
   # print paused
    if not paused:
        #print "control node running"
        time = time+1/time_rate
        #rospy.loginfo("gazebo time updated: " + str(time))

      

    #else:
	#rospy.loginfo("paused, time update skipped")  

    pub_time.publish(time)
