#!/usr/bin/env python

from beginner_tutorials.srv import *
from std_msgs.msg import Bool, Float32, Time, Float64
from std_srvs.srv import Empty
import rospy
from sensor_msgs.msg import Imu
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from openwsn_ros.msg import Controls
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import struct

#global variables
state_vars ={} #'uav name':[accelx, accely, accelz, gyrox,gyroy,gyroz]

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

def imu_callback(imuMsg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)

	global state_vars
	uav_id = "uav1"
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z

def imu_callback2(imuMsg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)

	global state_vars
	uav_id = "uav2"
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z

def imu_callback3(imuMsg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)

	global state_vars
	uav_id = "uav3"
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z

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
		#need a list of accelerations that are added to
		global state_vars
		global simulating
		global robot_dict
		mote_id = (mote_name.split('@')[1]) #mote_name is in this raw form: fromMoteProbe@emulatedx

		rospy.loginfo(mote_id)
		
		target_uav = robot_dict[mote_id]
		rospy.loginfo("RPC control input received from openwsn")

	

			
		simulating = True

		print ""
		print "received ",(x),(y) ,z, timestamp,"from",mote_name

		print "Timestamp: ",timestamp

		#server publishes controls that are received from the emulated mote
		#pub = rospy.Publisher('quad_input',Controls,queue_size=10)
		pub = rospy.Publisher(target_uav+'/cmd_vel',Twist,queue_size=10) #this is where i need to add the call to each uav based on mote name
		inputMsg = Twist()

		inputMsg.linear.x = x
		inputMsg.linear.y = y
		inputMsg.linear.z = z	
			
		#inputMsg.linear.x = signify(x)
		#inputMsg.linear.y = signify(y)
		#inputMsg.linear.z = signify(z)

		timestamp = timestamp+1
		pub.publish(inputMsg)
		print x, y, z
		rospy.loginfo("quad control input published to "+target_uav+": "+str(x)+str(y)+str(inputMsg.linear.z))


	
			


		return state_vars[target_uav][0:3] , timestamp 


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
		if gazebo_pause == False:
			unpause_srv

		if ((gazebo_time < timestamp)and(gazebo_pause==True)and(pauseOpenwsn==False) ):
			gazebo_pause = False
			print "unpausing gazebo"
			unpause_srv()
			#rospy.loginfo("Unpause Gazebo")		
			pause_pub.publish(False)
			pauseOpenwsn = True #pause opewsn because gazebo is running

		elif ((gazebo_time > timestamp) and (gazebo_pause == False)and(pauseOpenwsn==True)):

			gazebo_pause = True
			pause_srv()
			#rospy.loginfo("Pause Gazebo")		
			pause_pub.publish(True)			
			pauseOpenwsn = False #run openwsn because gazebo is paused

		elif ((gazebo_time < timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = True	#pause openwsn if simulator starts up in this case
			gazebo_pause == False
			unpause_srv()
		elif ((gazebo_time > timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = False	#pause gazebo if simulator starts up in this case
			gazebo_pause == True
			pause_srv()
		if (openTime+gazebo_time)/2 > 0:
			err_sum = err_sum + abs(openTime-gazebo_time)*((openTime+gazebo_time)/2-last_time)
    			rospy.loginfo_throttle(0.25, "Time Synchronization (OpenWSN Time,Gazebo Time,Synchronization Error,Avg Error): " + str(openTime)+", " + str(gazebo_time) +', ' +str(abs(openTime-gazebo_time)) +', ' +str(err_sum/((openTime+gazebo_time)/2)))
    		if simulating:
			print( "Sync Timestamp Simulating: " , timestamp , gazebo_time)

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
robot_dict = {"emulated1":"uav1","emulated2":"uav2","emulated3":"uav3"} #initialize dict with three robot mappings
state_vars = {"uav1":[0,0,0,0,0,0],"uav2":[0,0,0,0,0,0],"uav3":[0,0,0,0,0,0]} #initalize state dict for imu variables

rospy.Subscriber("uav1/raw_imu", Imu, imu_callback)
rospy.Subscriber("uav2/raw_imu", Imu, imu_callback2)
rospy.Subscriber("uav3/raw_imu", Imu, imu_callback3)

#rospy.Subscriber("sim_status", Bool, sim_status_callback)
rospy.Subscriber("clock",Clock,gazebo_time_callback)
pause_pub = rospy.Publisher("gazebo_pause",Bool,queue_size = 1)
pause_srv = rospy.ServiceProxy('gazebo/pause_physics',Empty)
unpause_srv = rospy.ServiceProxy('gazebo/unpause_physics',Empty)
unpause_srv() #unpause gazebo so it publishes to the clock topic and timekeeper can learn its current time
# Run the server's main loop
server.serve_forever()
