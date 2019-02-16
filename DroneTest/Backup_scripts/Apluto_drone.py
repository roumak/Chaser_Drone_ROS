#!/usr/bin/env python


import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray
from math import fabs








def pid(curr_data, waypoint, prev_data):
   
   
	twist=Twist() 
	current_time = time.time()
	dt = current_time - prev_data['last_time']
	

	error =  np.array([curr_data[0], curr_data[1], curr_data[2]]) -waypoint
	error = np.around(error, decimals=2)

    # publish errors for plotting.
	

	prev_data['int'] = prev_data['int'] + (error*dt)
	prev_data['derv'] = (error - prev_data['lastError']) / dt

	f = (prev_data['Kp'] * error) + (prev_data['Ki'] * prev_data['int']) + (prev_data['Kd'] * prev_data['derv'])

	prev_data['lastError'] = error
	prev_data['last_time'] = current_time
	print f[0][0],f[0][1],f[0][2]
    
    
   
    
	# twist.linear.x = f[1] 
	# twist.linear.y = f[0] 
	# twist.linear.z = f[2]
	# # twist.angular.z = 0#-f[3]
	# print twist
	return twist, prev_data


def callback(msg):
        
        twist_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        Empty_twist=Twist() 
		
        
        prev_data = dict()
        prev_data['last_time'] = time.time()

        pid_coeff = [1., 1., 1.]    #1.5, 1 ,0
      
        prev_data['Kp'] = np.array([pid_coeff[0], pid_coeff[0], 0.5])
        prev_data['Ki'] = np.array([pid_coeff[1], pid_coeff[1], 0.9])  ## z value 0.0025
        prev_data['Kd'] = np.array([pid_coeff[2], pid_coeff[2], 0.0])     ## z value 0.15
                

        
        prev_data['lastError'] = np.array([0., 0., 0.])
        prev_data['int'] = np.array([0., 0., 0.])
        prev_data['derv'] = np.array([0., 0., 0.])

       

        hold_pos=np.array([(0., 0., 22.)])
        


        x=msg.poses[0].position.x
        y=msg.poses[0].position.y
        z=msg.poses[0].position.z
       
        curr_data=np.array([x,y,z])
              
                                   
   #****************************HERE IS THE PID CALL**************************************************   
        
            
        pid_twist, prev_data = pid(curr_data, hold_pos, prev_data)
        twist_vel.publish(pid_twist)
       
            


def main():
    
    rospy.init_node('drone_automation')
    rospy.Subscriber("/whycon/poses",PoseArray, callback)
    rospy.spin()





if __name__ == '__main__':
   
    main()