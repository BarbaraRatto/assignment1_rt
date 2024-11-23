#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn


# --------------------------------------- TURTLES CONTROL ---------------------------------------


def turtles_control():
			


# --------------------------------------- MAIN ---------------------------------------

def main():
	global pub1, pub2
	
	# Initializing the node
	rospy.init_node('UserInterfaceAssignment', anonymous=True)
	
	# Defining the 2 publisher for the respective turtles 
	pub1 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)	# topic: turtle1/cmd_vel
	pub2 = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)	# topic: turtle2/cmd_vel
    
    
	# initializing the service client and call it to spawn turtle2
	client1 = rospy.ServiceProxy('/spawn', Spawn)				# service to spawn
	client1(5.5, 4.0, 0.0, "turtle2")					# spawn
    
	
	while not rospy.is_shutdown():
		turtles_control()


if __name__ == '__main__':
	main()
	
	
	
