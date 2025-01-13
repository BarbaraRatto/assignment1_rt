#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

# -----------------------------------------------------------------------------------------------

def turtle1_callback(msg):
	global my_vel
	threshold_value = 1
	
	for i in range(16) :
		if  msg.vect[i] < threshold_value :
			my_vel.linear.x = 0
			my_vel.linear.y = 0
			my_vel.angular.z = 0     	
			# stopping turtle1
			pubturtle1.publish(my_vel)
	
	
# -----------------------------------------------------------------------------------------------
	
def turtle2_callback(msg):
	global my_vel
	threshold_value = 1
	
	for i in range(16) :
		if  msg.vect[i] < threshold_value :
			my_vel.linear.x = 0
			my_vel.linear.y = 0
			my_vel.angular.z = 0     	
			# stopping turtle2
			pubturtle2.publish(my_vel)



# -----------------------------------------------------------------------------------------------

def main():
	global pubturtle1, pubturtle2, my_vel
    
	rospy.init_node('Assignment_Distance', anonymous=True)

	pubturtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	pubturtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)
	
	rospy.Subscriber('/turtle1/obstacle', Float32MultiArray, turtle1_callback)
	rospy.Subscriber('/turtle2/obstacle', Float32MultiArray, turtle2_callback)

	# msg to stop
	my_vel = Twist()

	print("\nNODE RUNNING\n")
	
	rospy.spin()

# -----------------------------------------------------------------------------------------------

# Running main()
if __name__ == '__main__':
	main()

