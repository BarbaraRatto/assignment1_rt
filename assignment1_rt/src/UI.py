#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn


# --------------------------------------- TURTLES CONTROL ---------------------------------------


def turtles_control():
	global pub1, pub2
	
	# definying my_vel (an object of type Twist)
	my_vel = Twist()
	
	# asking the user which turtle he wants to control (turtle1 or turtle2)
	chosenTurtle = int(input("\nWhich turtle do you want to control? Insert 1 or 2: "))
	
	if (chosenTurtle == 1) or (chosenTurtle == 2):
		print(f"Perfect! You selected turtle{chosenTurtle}\n")
	else:
		print("\nWARNING: You didn't insert the correct number. Try again.\n")
		return
    	
	# asking the user how he wants to move the turtle
	print("Set the 3 components of the velocity for the chosen turtle:")
	
	# setting the velocities as desidered & building my_vel message
	my_vel.linear.x = float(input("Linear velocity along x-axis: "))
	my_vel.linear.y = float(input("Linear velocity along y-axis: "))
	my_vel.angular.z = float(input("Angular velocity (along z-axis): "))
	print("\n")
	
	# test to print the velocities
	# print(f"\nvel_lin_x = {chosenVel_x} \nvel_lin_y = {chosenVel_y} \nvel_ang_z = {chosenVel_theta}")
	
	# moving turtle1 as requested
	if chosenTurtle == 1:
		pub1.publish(my_vel)

	# moving turtle2 as requested
	elif chosenTurtle == 2:
		pub2.publish(my_vel)
	
	# wait for 1 second
	rospy.sleep(1)				


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
	
	
	
