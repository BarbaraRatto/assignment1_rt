# How to run the code

The first thing required to run the following repository is to have ROS noetic.
In order to use these ROS nodes you have to run the following commands in 4 different terminals. 

Terminal 1:

	roscore
Terminal 2: launch the simulator

	rosrun turtlesim turtlesim_node
Terminal 3: go in the folder of your workspace and run the first node

	cd my_ros
	rosrun assignment1_rt UI.py
Terminal 4: go in the folder of your workspace and run the second node

	cd my_ros
	rosrun assignment1_rt Distance.py

If you also want to 'listen' to the messages published on the topic *distanceTopic*, you have to open another terminal and to run the command:

	rostopic echo /distanceTopic

# What I have done

In the folder *src* there are two files .py that correspond to the two nodes of the package *assignment_rt*.

## Node 1: UI

This node does the following things:
- spawning a new turtle (turtle2), in addition to turtle1;
- implementing a textual interface to retrieve a user command.

The user is able to choose which turtle he wants to move (inserting 1 or 2) and to set the following things:
- the linear velocity along x;
- the linear velocity along y;
- the angular velocity along z.

A message named *my_vel* is built with these velocities and it is published on the right topic (topic of turtle1 or of turtle2). Then the chosen turtle moves as requested for 1 second. After that the moving turtle is stopped and user is again able to choose the turtle he wants to move and so on.



## Node 2: Distance

This node has to objective to read the position of the turtles and to stop them if they are too close to the boundaries or to each other.
More in details, the following reasoning has been done to handle the situation where the turtles are too close to each other. 

The subscription to the topics *turtle1/pose* and *turtle2/pose* allows reading the position of the turtles, in order to compute the relative distance between them. This distance is published on the topic *distanceTopic* and it is compared with a threshold (set to 1). If the turtles are too close to each other (*distance <= threshold*) the moving turtle is stopped. In addition, exploiting the teleporting service, the turtle that was moving is teleported back on its trajectory by a small stretch, in order to have the distance between the turtles > threshold. 


Instead, the following reasoning has been done to handle the situation where the turtles are getting too close to the boundaries.

The control that prevents the turtles crashing into the boundaries is implemented in the functions *turtle_callback1* and *turtle_callback2*. If a turtle crosses the limit distance, it is not only stopped but also teleported back on its trajectory by a small stretch, as done for the previous case. 
