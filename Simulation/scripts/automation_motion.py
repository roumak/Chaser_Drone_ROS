#!/usr/bin/env python
"""Generic PID implimentation."""

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
  

    prev_data['int'] += error * dt
    prev_data['derv'] = (error - prev_data['lastError']) / dt

    f = prev_data['Kp'] * error + prev_data['Ki'] * prev_data['int'] + prev_data['Kd'] * prev_data['derv']

    prev_data['lastError'] = error
    prev_data['last_time'] = current_time

    
    
    
   
    
    twist.linear.x = f[1] 
    twist.linear.y = f[0] 
    twist.linear.z = f[2]
    # twist.angular.z = 0#-f[3]
    print twist
    return twist, prev_data


def callback(msg):
        global i
        twist_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        Empty_twist=Twist() 





        if i==0:
            takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
            takeoff.publish(Empty())

        if i>4:
            land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
            twist_vel.publish(Empty_twist)
            

            land.publish(Empty())
            return
        

        waypoint=np.array([ (4.25, 4.05, 11.), (-4.0, 3.25, 11.), (-3.8, -4.2, 11.), (4.25, -4.2, 11.), (0., 0., 11.) ])
        
        
        
            
        
        
        prev_data = dict()
        prev_data['last_time'] = time.time()

        pid_coeff = [1., 1.8, 0.00005]    #1.5, 1 ,0
      
        prev_data['Kp'] = np.array([pid_coeff[0], pid_coeff[0], 0.5])
        prev_data['Ki'] = np.array([pid_coeff[1], pid_coeff[1], 0.9])  ## z value 0.0025
        prev_data['Kd'] = np.array([pid_coeff[2], pid_coeff[2], 0.0])     ## z value 0.15
                

        
        prev_data['lastError'] = np.array([0., 0., 0.])
        prev_data['int'] = np.array([0., 0., 0.])
        prev_data['derv'] = np.array([0., 0., 0.])

       

        
        


        x=msg.poses[0].position.x
        y=msg.poses[0].position.y
        z=msg.poses[0].position.z
        #trying to stabilize the yaw axis 
        

        # az1=msg.poses[0].orientation.x
        # az1a=np.sin(az1)
          ######################r
        #curr_data=np.array([x,y,z,az1a])
        curr_data=np.array([x,y,z])
        

 #in problem statement it was given an error of 0.2 is permitted thus we are using value 0.15 ie<0.2                          
        
        if (  np.absolute(curr_data-waypoint[i])<=0.1 ).all():  
            i=i+1

                                   
   #****************************HERE IS THE PID CALL**************************************************   
        # if i>4:
            # return
        else:    
            pid_twist, prev_data = pid(curr_data, waypoint[i], prev_data)
            twist_vel.publish(pid_twist)
       
            


def main():
    
    #[ (4.25, 4.05, 11), (-4.0, 3.25, 11), (-3.8, -4.2, 11), (4.25, -4.2, 11), (0, 0, 11) ]
     
    
    rospy.init_node('automation_waypoint')
    # take_off = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    # take_off.publish(Empty())    
    rospy.Subscriber("/whycon/poses",PoseArray, callback)
    if i>4:
        return
        
    else:
        rospy.spin()

if __name__ == '__main__':
    i=0
    main()