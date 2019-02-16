#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
import rospy


def main():
	rospy.init_node('just_arm')
	command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

	cmd=PlutoMsg()
	while True:
		cmd.rcRoll=1500
		cmd.rcYaw=1500
		cmd.rcPitch =1500
		cmd.rcThrottle =1000
		cmd.rcAUX1 =0
		cmd.rcAUX2 =0
		cmd.rcAUX3 =0
		cmd.rcAUX4 =1500
		command_pub.publish(cmd)
		rospy.sleep(.1)


if __name__ == '__main__':
	main()