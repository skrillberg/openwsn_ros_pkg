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

#global variables ################################################

#List of IMU accelerations and angular velocities for each gazebo robot
state_vars ={} #'uav name':[accelx, accely, accelz, gyrox,gyroy,gyroz]


gazebo_time =0  	#current gazebo time, retrieved from /clock topic, updated in gazebo_time_callback 
openTime = 0    	#current openwsn time, retrieved from openWSN in the timeSync RPC function
gazebo_pause = False	#state variable for gazebo simulation state. True if gazebo is paused, False if it is running
pauseOpenwsn = False	#state variable for openWSN simulation state. True if openWSN simulator is currently paused
err_sum = 0		#Keeps track of the err_sum from the time synchronization difference
last_time = 0		#Keeps track of the previous average time of openwsn and gazebo time

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

# Create RPC server
server = SimpleXMLRPCServer(("localhost", 8001),
                            requestHandler=RequestHandler)
server.register_introspection_functions()

#suscriber callbacks#######################################################3

#calback for uav1 imu messages
def imu_callback(imuMsg,uav_arg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)
	#rospy.loginfo("uav_arg: " + uav_arg)
	global state_vars
	uav_id = "uav1"
	uav_id = uav_arg
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z
#callback for imu messages from uav2
def imu_callback2(imuMsg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)

	global state_vars
	uav_id = "uav2"
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z

#callback for imu messages from uav3
def imu_callback3(imuMsg):
	#rospy.loginfo(rospy.get_caller_id() + "IMU message received: %s, %s, %s ", imuMsg.linear_acceleration.x,imuMsg.linear_acceleration.y,imuMsg.linear_acceleration.z)

	global state_vars
	uav_id = "uav3"
	state_vars[uav_id][0] = imuMsg.linear_acceleration.x
	state_vars[uav_id][1] = imuMsg.linear_acceleration.y
	state_vars[uav_id][2] = imuMsg.linear_acceleration.z

#callback for clock messages from gazebo simulation time. Keeps track of gazebo current time
def gazebo_time_callback(clock_msg):
	global gazebo_time
	global gazebo_pause
	global openTime

	gazebo_time = clock_msg.clock.to_sec() #obtain gazebot time in seconds 
	#rospy.loginfo( "Sync Timestamps Always (OpenWSN,Gazebo): " + str(openTime)+", " + str(gazebo_time.data))


#Register RPC functions that are called by moteProbe.py in openwsn #################################################	
# Register an instance; all the methods of the instance are
# published as XML-RPC methods (in this case, just 'div').
class MyFuncs:

    #prt2###############################################################
    #called by moteProbe.py when a uart tx from a mote is received. Its 
    #purpose is to route control inputs from openwsn motes to quadcopters
    #
    #args
    #mote_name: name of mote that made the RPC request
    #x: linear x velocity control input
    #y: y linear velocity control input
    #z: z linear velocity control input
    #timestamp: current openwsn time

    def prt2(self,mote_name,x,y,z,timestamp):

		global state_vars
		global robot_dict

		mote_id = (mote_name.split('@')[1]) #mote_name is in this raw form: fromMoteProbe@emulatedx, we want the "emulatedx" part

		rospy.loginfo(mote_id)
		
		target_uav = robot_dict[mote_id]   #looks up the name of the target uav that is linked to the openwsn mote name: "emulated1" -> "uav1"
		rospy.loginfo("RPC control input received from " + mote_id + "routing to " +target_uav)

		#server publishes controls that are received from the emulated mote

		pub = rospy.Publisher(target_uav+'/cmd_vel',Twist,queue_size=10) #this is where i need to add the call to each uav based on mote name. constantly reconstructing the publisher isn't a great idea
		inputMsg = Twist() #create twist message for quadcopter controls
		
		#put linear accelerations into twist message
		inputMsg.linear.x = x
		inputMsg.linear.y = y
		inputMsg.linear.z = z	
			
		pub.publish(inputMsg)
		rospy.loginfo("quad control input from "+mote_id+" published to "+target_uav+": "+str(x)+str(y)+str(inputMsg.linear.z))

		#return imu state to openwsn mote that called this RPC function
		#TODO: create another timer in openwsn that requests IMU data
		return state_vars[target_uav][0:3] , timestamp 

    #timeSync#####################################################################	
    #Synchronizes openWSN time and gazebo time. Called by openWSN everytime a new
    #event is added to the timeline
    # 
    #timestamp: current openWSN time
    def timeSync(self,timestamp):

		global openTime

		global gazebo_time
		global gazebo_pause
		global pauseOpenwsn
		global lasttime
		global err_sum

		last_time= (gazebo_time+openTime)/2 #previous time, used to compute average error
		openTime = timestamp #time from openwsn timeline
		rospy.loginfo("gazebo_pause: " + str(gazebo_pause))
		
		#this was added because at one point gazebo_pause had been set to false without the upause gazebo service being called 
		if gazebo_pause == False:
			unpause_srv
		
		#if openWSN gets ahead of gazebo, pause openWSN and unpause gazebo
		if ((gazebo_time < timestamp)and(gazebo_pause==True)and(pauseOpenwsn==False) ):
			gazebo_pause = False
			print "unpausing gazebo"
			unpause_srv()
			#rospy.loginfo("Unpause Gazebo")		

			pauseOpenwsn = True #pause opewsn because gazebo is running

		#if gazebo gets ahead of openwsn, pause gazebo and unpause openwsn
		elif ((gazebo_time > timestamp) and (gazebo_pause == False)and(pauseOpenwsn==True)):

			gazebo_pause = True
			pause_srv()
			#rospy.loginfo("Pause Gazebo")		

			pauseOpenwsn = False #run openwsn because gazebo is paused

		#if gazebo is behind openWSN and both are running, pause openwsn
		elif ((gazebo_time < timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = True	#pause openwsn if simulator starts up in this case
			gazebo_pause == False
			unpause_srv()

		#if gazebo is ahead of openWSN and both are running, pause gazebo
		elif ((gazebo_time > timestamp)and (gazebo_pause == False)and(pauseOpenwsn==False)):
			pauseOpenwsn = False	#pause gazebo if simulator starts up in this case
			gazebo_pause == True
			pause_srv()

		#compute average difference in time. if statement avoids divide by zero edge case
		if (openTime+gazebo_time)/2 > 0:
			err_sum = err_sum + abs(openTime-gazebo_time)*((openTime+gazebo_time)/2-last_time)
    			rospy.loginfo_throttle(0.25, "Time Synchronization (OpenWSN Time,Gazebo Time,Synchronization Error,Avg Error): " + str(openTime)+", " + str(gazebo_time) +', ' +str(abs(openTime-gazebo_time)) +', ' +str(err_sum/((openTime+gazebo_time)/2)))


		print "pauseOpenwsn: ", pauseOpenwsn 
		
		#return pauseOpenwsn parameter to TimeLine.py. TimeLine.py will stay in a while loop as long as pauseOpenWSN is true
		return pauseOpenwsn  


server.register_instance(MyFuncs())

#initialize ros node
rospy.init_node('node_handler', anonymous=True)
rate = rospy.Rate(.5)
print "node initialized"

#TODO: the following two line should be done programatically and not be hardcoded
robot_dict={}
state_vars={}
#create robot dict
for i in range(1,4):
	robot_dict["emulated"+str(i)]="uav"+str(i)
	rospy.Subscriber("uav"+str(i)+"/raw_imu", Imu, imu_callback,"uav"+str(i))
	state_vars["uav"+str(i)] = [0,0,0,0,0,0]
'''
robot_dict = {"emulated1":"uav1","emulated2":"uav2","emulated3":"uav3"} #initialize dict with three robot mappings
state_vars = {"uav1":[0,0,0,0,0,0],"uav2":[0,0,0,0,0,0],"uav3":[0,0,0,0,0,0]} #initalize state dict for imu variables
'''

#suscribe to IMU topics for each robot
#TODO: this should be done programatically so I can suscribe to n IMU topics
'''
rospy.Subscriber("uav1/raw_imu", Imu, imu_callback,"uav1")
rospy.Subscriber("uav2/raw_imu", Imu, imu_callback,"uav2")
rospy.Subscriber("uav3/raw_imu", Imu, imu_callback,"uav3")
'''
#Suscribe to clock topic to receive gazebo time. Gazebo needs to be publishing
#to the clock topic for this to work
rospy.Subscriber("clock",Clock,gazebo_time_callback)

#creating proxies so the pause and unpause gazebo services
#can be used

pause_srv = rospy.ServiceProxy('gazebo/pause_physics',Empty)
unpause_srv = rospy.ServiceProxy('gazebo/unpause_physics',Empty)
unpause_srv() #unpause gazebo so it publishes to the clock topic and timekeeper can learn its current time

# Run the server's main loop
server.serve_forever()
