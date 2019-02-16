#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
import rospy
import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray
from math import fabs
import cv2

class send_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_automaion')
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		rospy.Subscriber('/input_key', Int16, self.indentify_key )
		rospy.Subscriber("/whycon/poses",PoseArray, self.callback)
		
		self.key_value =0
		self.cmd = PlutoMsg()
		self.cmd.rcRoll =1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1000
		
		
		
	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)
	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
	
	def indentify_key(self, msg):
		self.key_value = msg.data
###########################################################################################################################	
	def callback(self,msg):
        
		prev_data = dict()
		prev_data['last_time'] = time.time()

		pid_coeff = list()
		pid_coeff=[1000., 1800, 0.00005] 
		prev_data['Kp'] = np.array([ pid_coeff[0], pid_coeff[0], 100])
		prev_data['Ki'] = np.array([ pid_coeff[1], pid_coeff[1], 180])  ## z value 0.0025
		prev_data['Kd'] = np.array([ pid_coeff[2], pid_coeff[2], 0.0005])     ## z value 0.15
		prev_data['lastError'] = np.array([0., 0., 0.])
		prev_data['int'] = np.array([0., 0., 0.])
		prev_data['derv'] = np.array([0., 0., 0.])
		hold_pos=np.array([(0., 0., 22.)])
		x=msg.poses[0].position.x
		y=msg.poses[0].position.y
		z=msg.poses[0].position.z
		curr_data=np.array([x,y,z])
		prev_data = self.pid(curr_data, hold_pos, prev_data)
		# twist_vel.publish(pid_twist)




	def pid(self,curr_data, waypoint, prev_data):

		twist=Twist()
		current_time = time.time()
		dt = current_time - prev_data['last_time']
		error =  np.array([curr_data[0], curr_data[1], curr_data[2]]) -waypoint
		error = np.around(error, decimals=2)
		
		prev_data['int'] = prev_data['int'] + (error*dt)
		prev_data['derv'] = (error - prev_data['lastError']) / dt
		
		f = (prev_data['Kp'] * error) + (prev_data['Ki'] * prev_data['int']) + (prev_data['Kd'] * prev_data['derv'])
		np.clip(f,-400,400,out=f)
		# print f[0]
		prev_data['lastError'] = error
		prev_data['last_time'] = current_time
		x_corr=1600-f[0][0]
		y_corr=1600+f[0][1]
		z_corr=1600+f[0][2]
		print x_corr,y_corr,z_corr
# X correction
		self.cmd.rcPitch =x_corr     
		self.command_pub.publish(self.cmd)
# Y correction
		self.cmd.rcRoll =y_corr
		self.command_pub.publish(self.cmd)
# Z correction 
		self.cmd.rcThrottle = z_corr
		self.command_pub.publish(self.cmd)


		return prev_data
    	
		



####################################################################################################################################



	def control_drone(self):

		while True:
			if self.key_value == 0:         
				self.disarm()
			if self.key_value == 70:
				self.arm()
			if self.key_value == 10:
			
			self.command_pub.publish(self.cmd)
		


if __name__ == '__main__':
	while not rospy.is_shutdown():
		test = send_data()
		test.control_drone()
		rospy.spin()
		sys.exit(1)


