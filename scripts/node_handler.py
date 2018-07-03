#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy
from sensor_msgs.msg import Imu
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from openwsn_ros.msg import Controls

#global variables
accelx = 1
accely = 2
accelz = 3

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

# Create server
server = SimpleXMLRPCServer(("localhost", 8001),
                            requestHandler=RequestHandler)
server.register_introspection_functions()

# Register pow() function; this will use the value of
# pow.__name__ as the name, which is just 'pow'.
server.register_function(pow)

# Register a function under a different name
def adder_function(x,y):
    return x + y
server.register_function(adder_function, 'add')



def imu_callback(imuMsg):
	rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)
	global accelx
	global accely
	global accelz

	accelx=imuMsg.linear_acceleration.x
	accely=imuMsg.linear_acceleration.y
	accelz=imuMsg.linear_acceleration.z
	#print "in callback"

# Register an instance; all the methods of the instance are
# published as XML-RPC methods (in this case, just 'div').
class MyFuncs:
    def div(self, x, y):
        return x // y

    def prt(self,x,y):
	print ord(x),ord(y)
	return "communicated"

    def prt2(self,mote_name,x,y,z):
		global accelx
		global accely
		global accelz
		print ""
		print "received ",(x),(y) ,z ,"from",mote_name
		print "sending ",accelx,accely ,accelz ,"to",mote_name
		rospy.loginfo(rospy.get_caller_id() + "Sending %s, %s, %s to %s",accelx,accely,accelz,mote_name)
		#server publishes controls that are received from the emulated mote
		pub = rospy.Publisher('quad_input',Controls,queue_size=10)
		inputMsg = Controls()
		inputMsg.prop1 = x
		pub.publish(inputMsg)

		return [int(accelx),int(accely),int(accelz)]


    def control(self,mote_name,control_input):
		pub = rospy.Publisher('quad_input',Controls,queue_size=10)
		inputMsg = Controls()
		inputMsg.prop1 = 0.6
		pub.publish(inputMsg)

server.register_instance(MyFuncs())

# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
rospy.init_node('mote_handler', anonymous=True)
print "node initialized"
rospy.Subscriber("imu_datastream", Imu, imu_callback)


# Run the server's main loop
server.serve_forever()
