#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_msgs.msg import Float32
from turtlesim.srv import TeleportAbsolute

# --------------------------------------- CALLBACK 1 ---------------------------------------
def turtle_callback1(msg):
	global x1, y1, theta1
	
	x1 = msg.x
	y1 = msg.y
	theta1 = msg.theta
	
	# computing the distance
	distance()
	
	# stoppig turtle1 if its position is too close to the boundaries
	if ( x1 <= 1.0 or x1 >= 10.0 or y1 <= 1.0 or y1 >= 10.0 ):
		my_vel.linear.x = 0;
		my_vel.linear.y = 0;
		my_vel.angular.z = 0;
		 
		pub1.publish(my_vel)
		
		# calling the teleport service to take back turtle1 inside the boundaries
		if( x1 <= 1.0 ):
			teleportTurtle1(1.1, y1, theta1)
		if( x1 >= 10.0 ):
			teleportTurtle1(9.9, y1, theta1)
		if( y1 <= 1.0 ):
			teleportTurtle1(x1, 1.1, theta1)
		if( y1 >= 10.0 ):
			teleportTurtle1(x1, 9.9, theta1)
		

# --------------------------------------- CALLBACK 2 ---------------------------------------
def turtle_callback2(msg):
	global x2, y2, theta2
	
	x2 = msg.x
	y2 = msg.y
	theta2 = msg.theta

	# computing the distance
	distance()

	# stoppig turtle2 if its position is too close to the boundaries
	if ( x2 <= 1.0 or x2 >= 10.0 or y2 <= 1.0 or y2 >= 10.0 ):
		my_vel.linear.x = 0;
		my_vel.linear.y = 0;
		my_vel.angular.z = 0;
		
		pub2.publish(my_vel)
		
		# calling the teleport service to take back turtle2 inside the boundaries
		if( x2 <= 1.0 ):
			teleportTurtle2(1.1, y2, theta2)
		if( x2 >= 10.0 ):
			teleportTurtle2(9.9, y2, theta2)
		if( y2 <= 1.0 ):
			teleportTurtle2(x2, 1.1, theta2)
		if( y2 >= 10.0 ):
			teleportTurtle2(x2, 9.9, theta2)
		

# --------------------------------------- FUNCTION DISTANCE ---------------------------------------
def distance():
	global threshold
	
	# setting the threshold
	threshold = 1
	
	# computing the relative distance between the turtles
	distance = math.sqrt( pow(( x2 - x1 ), 2) + pow(( y2 - y1 ), 2) )
	
	# building a message of type Float32
	distanceMsg = Float32()
	distanceMsg.data = distance
	
	# publishing on a topic the relative distance between the turtles
	pubDistance.publish(distanceMsg)
	
	# stopping the turtles if they are too close
	if distance <= threshold:
		
		# teleporting the moving turtle out of the 'threshold zone'
		# computing the angle
		deltaX = x2 - x1
		deltaY = y2 - y1
		alpha = math.atan2(deltaY, deltaX)
		
		# distance to do in reverse
		reverseDistance = threshold - distance + 0.1
		
		if vel_x1 != 0 or vel_y1 != 0 or vel_theta1 != 0:
			# computing the new coordinates in which teleporting turtle1
			x1_teleport = x1 - reverseDistance * math.cos(alpha)
			y1_teleport = y1 - reverseDistance * math.sin(alpha)
			
			teleportTurtle1(x1_teleport, y1_teleport, theta1)
			
		elif vel_x2 != 0 or vel_y2 != 0 or vel_theta2 != 0:
			# computing the new coordinates in which teleporting turtle2
			x2_teleport = x2 + reverseDistance * math.cos(alpha)
			y2_teleport = y2 + reverseDistance * math.sin(alpha)
			
			teleportTurtle2(x2_teleport, y2_teleport, theta2)
				
		
		my_vel.linear.x = 0;
		my_vel.linear.y = 0;
		my_vel.angular.z = 0;
		# stopping both the 2 turtles
		pub1.publish(my_vel)
		pub2.publish(my_vel)
		
		
		
# --------------------------------------- SAVE VELOCITY FUNCTIONS ---------------------------------------
def turtle_velocity1(msg):
	global vel_x1, vel_y1, vel_theta1
	
	vel_x1 = msg.linear.x
	vel_y1 = msg.linear.y
	vel_theta1 = msg.angular.z
	

def turtle_velocity2(msg):
	global vel_x2, vel_y2, vel_theta2
	
	vel_x2 = msg.linear.x
	vel_y2 = msg.linear.y
	vel_theta2 = msg.angular.z



# --------------------------------------- MAIN ---------------------------------------

def main():
	global my_vel, pub1, pub2, pubDistance, x1, y1, theta1, x2, y2, theta2, teleportTurtle1, teleportTurtle2, turtle_velocity1, turtle_velocity2, vel_x1, vel_y1, vel_theta1, vel_x2, vel_y2, vel_theta2
	
	# initialization of global variables
	x1 = 5.5
	y1 = 5.5
	theta1 = 0.0
	x2 = 5.5
	y2 = 4.0
	theta2 = 0.0
	
	vel_x1 = 0
	vel_y1 = 0
	vel_theta1 = 0
	vel_x2 = 0
	vel_y2 = 0
	vel_theta2 = 0
	
	
	# Initialize the node
	rospy.init_node('UserInterfaceAssignment', anonymous=True)
	
	# defining my_vel
	my_vel = Twist()			# my_vel is an object of type Twist 
	
	# Defining the publisher and subscriber
	pub1 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)	# topic: turtle1/cmd_vel
	pub2 = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)	# topic: turtle2/cmd_vel
	
	rospy.Subscriber('turtle1/pose', Pose, turtle_callback1)	# topic to read position of turtle1
	rospy.Subscriber('turtle2/pose', Pose, turtle_callback2)	# topic to read position of turtle2
	
	# topic where to publish the distance between the turtles
	pubDistance = rospy.Publisher('distanceTopic', Float32, queue_size=10)
	
	# initializing the services to teleport turtle1 and turtle2
	teleportTurtle1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teleportTurtle2 = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
	
	# subscribers to read the turtles velocities
	rospy.Subscriber('turtle1/cmd_vel', Twist, turtle_velocity1)	# topic to read velocity of turtle1
	rospy.Subscriber('turtle2/cmd_vel', Twist, turtle_velocity2)	# topic to read velocity of turtle2	

	
	print("The node Distance.py is running")
	
	rospy.spin()		# infinite spin
	

if __name__ == '__main__':
	main()
	
	
