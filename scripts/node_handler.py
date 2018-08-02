#!/usr/bin/env python

from beginner_tutorials.srv import *
from std_msgs.msg import Bool, Float32, Time, Float64
import rospy
from sensor_msgs.msg import Imu
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from openwsn_ros.msg import Controls
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import struct

#global variables
accelx = 1
accely = 2
accelz = 3
rosTime = 0# current time that the latest ros node is at (in openwsn time). This must be greater than openWSN time
simulating = False
gazebo_time =0
openTime = 0
mesh_pause = False
gazebo_pause = False
pauseOpenwsn = False
err_sum = 0
last_time = 0

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

def signify(x):
	if x > 127:
		signed = x-256
	else:
		signed = x
	return signed

def hector_imu(imuMsg):
	rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)
	global accelx
	global accely
	global accelz

	accelx=imuMsg.linear_acceleration.x
	accely=imuMsg.linear_acceleration.y
	accelz=imuMsg.linear_acceleration.z
	#print "in callback"

def imu_callback(imuMsg):
	rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)
	global accelx
	global accely
	global accelz

	accelx=imuMsg.linear_acceleration.x
	accely=imuMsg.linear_acceleration.y
	accelz=imuMsg.linear_acceleration.z
	#print "in callback"

def sim_status_callback(msg):
	global simulating
	rospy.loginfo("sim finished")
	simulating = False


def gazebo_time_callback(clock_msg):
	global gazebo_time
	global gazebo_pause
	#global openTime
	
	
	gazebo_time = clock_msg.clock.to_sec()
	#rospy.loginfo( "Sync Timestamps Always (OpenWSN,Gazebo): " + str(openTime)+", " + str(gazebo_time.data))
	#if (gazebo_time > openTime) and (gazebo_pause == False):
	#	gazebo_pause == True
	#	rospy.loginfo("Pause Gazebo, Gazebo Message")
	#	pause_pub.publish(True)
	#elif (gazebo_time > openTime) and (gazebo_pause == True):
	#	rospy.loginfo("Unpause Gazebo, Gazebo Message")		
	#	pause_pub.publish(False)
	#	gazebo_pause == False
	
# Register an instance; all the methods of the instance are
# published as XML-RPC methods (in this case, just 'div').
class MyFuncs:
    def div(self, x, y):
        return x // y

    def prt(self,x,y):
	print ord(x),ord(y)
	return "communicated"

    def prt2(self,mote_name,x,y,z,timestamp):
		global accelx
		global accely
		global accelz
		global rosTime
		global simulating
		rospy.loginfo("RPC control input received from openwsn")

		rosTime = timestamp
		simulating = True

		print ""
		print "received ",(x),(y) ,z, timestamp,"from",mote_name
		print "sending ",accelx,accely ,accelz ,timestamp,"to",mote_name
		print "Timestamp: ",timestamp
		rospy.loginfo(rospy.get_caller_id() + "Sending %s, %s, %s to %s",accelx,accely,accelz,mote_name)
		#server publishes controls that are received from the emulated mote
		#pub = rospy.Publisher('quad_input',Controls,queue_size=10)
		pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
		inputMsg = Twist()
		inputMsg.linear.x = signify(x)
		inputMsg.linear.y = signify(y)
		inputMsg.linear.z = signify(z)

		timestamp = timestamp+1
		pub.publish(inputMsg)
		print x, y, z
		rospy.loginfo("quad control input published to dummy node "+str(x)+str(y)+str(inputMsg.linear.z))

		#accelx = 0
		#accely= 0
		#accelz = 0
		longx = int(accelx*32767/16) #converts acceleration from float to hardware units int 
		
		longy = int(accely*32767/16)
		longz = int(accelz*32767/16)
		return [accelx,accely,accelz] , timestamp


    def control(self,mote_name,control_input):
		
		pub = rospy.Publisher('quad_input',Controls,queue_size=10)
		inputMsg = Controls()
		inputMsg.prop1 = 0.6

		pub.publish(inputMsg)
		
    def timeSync(self,timestamp):
		global pause_pub
		global openTime
		global mesh_pause
		global gazebo_time
		global gazebo_pause
		global pauseOpenwsn
		global lasttime
		global err_sum
		last_time= (gazebo_time+openTime)/2
		openTime = timestamp #time from openwsn timeline
		print "gazebo_pause: ", gazebo_pause

		if ((gazebo_time < timestamp)and(gazebo_pause==True)and(pauseOpenwsn==False) ):
			gazebo_pause = False
			print "unpausing gazebo"
			#rospy.loginfo("Unpause Gazebo")		
			pause_pub.publish(False)
			pauseOpenwsn = True #pause opewsn because gazebo is running

		elif ((gazebo_time > timestamp) and (gazebo_pause == False)and(pauseOpenwsn==True)):

			gazebo_pause = True
			#rospy.loginfo("Pause Gazebo")		
			pause_pub.publish(True)			
			pauseOpenwsn = False #run openwsn because gazebo is paused

		elif ((gazebo_time < timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = True	#pause openwsn if simulator starts up in this case
			gazebo_pause == False

		elif ((gazebo_time > timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = False	#pause gazebo if simulator starts up in this case
			gazebo_pause == True

		if (openTime+gazebo_time)/2 > 0:
			err_sum = err_sum + abs(openTime-gazebo_time)*((openTime+gazebo_time)/2-last_time)
    			rospy.loginfo_throttle(0.25, "Time Synchronization (OpenWSN Time,Gazebo Time,Synchronization Error,Avg Error): " + str(openTime)+", " + str(gazebo_time) +', ' +str(abs(openTime-gazebo_time)) +', ' +str(err_sum/((openTime+gazebo_time)/2)))
    		if simulating:
			print( "Sync Timestamp Simulating: " , timestamp , gazebo_time)
		#if simulating and (timestamp>rosTime):
		#	rospy.loginfo( "Sync Timestamp: " , timestamp , rosTime)
			#pauseOpenwsn = True
		print "pauseOpenwsn: ", pauseOpenwsn 
		return pauseOpenwsn
    def isRosSynced(self):
	synced = not simulating
	return synced
server.register_instance(MyFuncs())

# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
rospy.init_node('mote_handler', anonymous=True)
rate = rospy.Rate(.5)
print "node initialized"
rospy.Subscriber("raw_imu", Imu, imu_callback)
rospy.Subscriber("sim_status", Bool, sim_status_callback)
rospy.Subscriber("clock",Clock,gazebo_time_callback)
pause_pub = rospy.Publisher("gazebo_pause",Bool,queue_size = 1)
# Run the server's main loop
server.serve_forever()
