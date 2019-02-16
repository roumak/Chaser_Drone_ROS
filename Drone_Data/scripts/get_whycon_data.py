#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseArray



def callback(msg):
	 for i in range(0,5):
		x=msg.poses[i].position.x
		y=msg.poses[i].position.y
		z=msg.poses[i].position.z

		
	
		rospy.loginfo("\nMarker%d (x,y,z) := %f  , %f  , %f  ",i,x,y,z)
		
		# if True:
			# return 2
	


def main():
	rospy.init_node('marker_position')
	p=rospy.Subscriber("/whycon/poses",PoseArray, callback)

	# print p
	rospy.spin(); # intentianally commented out
	# used so that rqt_graph can successfuly capture the node


if __name__== '__main__':
	main() 