#!/usr/bin/env python

import rospy
from std_msgs.msg import String
print "hi"
from sensor_msgs.msg import Imu
import random
print "hi"
def imu_example():
	pub = rospy.Publisher('imu_datastream',Imu,queue_size=10)
	rospy.init_node("imu_example",anonymous=True)
	rate = rospy.Rate(1)
	gyro_fsr = 2000 
	accel_fsr = 16
	imuMsg = Imu()

	while not rospy.is_shutdown():

		random_accel = (random.uniform(0,accel_fsr),random.uniform(0,accel_fsr),random.uniform(0,accel_fsr))
		imuMsg.linear_acceleration.x = random_accel[0]
		imuMsg.linear_acceleration.y = random_accel[1]
		imuMsg.linear_acceleration.z = random_accel[2]

		random_gyro = (random.uniform(0,gyro_fsr),random.uniform(0,gyro_fsr),random.uniform(0,gyro_fsr))
		imuMsg.angular_velocity.x = random_gyro[0]
		imuMsg.angular_velocity.y = random_gyro[1]
		imuMsg.angular_velocity.z = random_gyro[2]

		imu_string = "Acceleration messages sent: "+str(random_accel[0])+", "+str(random_accel[1])+", " + str(random_accel[2])
		rospy.loginfo(imu_string)
		pub.publish(imuMsg)
		rate.sleep()



print "trying node"
imu_example()
print "node intialized"

