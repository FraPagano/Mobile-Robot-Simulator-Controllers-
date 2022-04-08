#! /usr/bin/env python3

"""
.. module:: go_to_desired_pos
	:platform: Unix
	:synopsis: Python module for controlling the robot providing a desired position to reach. 
.. moduleauthor:: Francesco Pagano <francescopagano1999@outlook.it>

This is the First Robot Controlling Modality.
This node makes the robot autonomously reach a x,y position inserted by the user. 
The robot can reach the user defined x,y coordinates thanks to the 'move_base' action server. 
The robot is going to plan the path through the Dijkstra's algorithm. 

ROS parameters: 
	1.	"active": (type: int) parameter for activate the desired control modality 
	2.	"des_pos_x": (type: double) parameter for the desired X coordinate 
	3.	"des_pos_y": (type: double) parameter for the desired Y coordinate 

These ROS parameters too and they are set by the :mod:`UI` node.
"""

# IMPORTS
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *

#COLORS 
class bcolors:
	"""
	This class is used for printing colors on the temrinal
	"""

	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKCYAN = '\033[96m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'
	ORANGE = '\033[33m' 
	PURPLE  = '\033[35m'

# Explanatory message
msg = """ 
""" + bcolors.BOLD + """
This node makes the robot autonomously reach a x,y position inserted by the user.
The user's x,y coordinates are reached thanks to the 'move_base' action server. 
The robot is going to plan the path through the Dijkstra's algorithm. 
""" +bcolors.ENDC + """
"""


goal_msg=MoveBaseGoal()
"""
Global action message
"""	


active_ = 0 
"""
Global ROS poarameter to block/unlock the modality 
"""	
		
desired_position_x =  0 
"""
Global X desired coordinate 
"""		

desired_position_y = 0 
"""
Global Y desired coordinate 
"""	

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)	# Action client
"""
Global action client. 
"""


achieved = False
"""
Global bool for defining if a goal was achieved or not. Useful in order to differentiate the case in which a goal was achieved 
(in this case the cacel request of an already canceled goal may cause an error, so I avoided to send the cancel request to the action server), 
and the case in which the user decides to send a cancel request before the goal achievement (in this case we must send a cancel request to the server).
"""


goal_cont = 1	
"""
Global Goal counter. Takes into account the number of requests. 
"""	

cont = 1			
"""
Feedback index. Takes into account the number of feedback for each request. 
"""	

def action_client():
	"""
	This function is called for both wait until we are connected to the action server and 
	to set some parameters of the action message.

		No Args. 

		No Returns. 
	"""

	global goal_msg
	global client

	client.wait_for_server()	# Waits until we are connected to the action server

	# Setting some goal's fields
	goal_msg.target_pose.header.frame_id = 'map'			
	goal_msg.target_pose.header.stamp = rospy.Time.now()	
	goal_msg.target_pose.pose.orientation.w = 1				


def done_cb(status, result):
	"""
	This is a callback function called after the execution of the action server. It gives the client information about the termination of the goal process. 
	In particular, this callback function puts a value that into the argument ``status``. 
	Depending on the value of this variable the client knows the status of the goal processing after the execution. 

    Args:
        status (actionlib_GoalStatus): 
        	terminal state (as an integer from actionlib_msgs/GoalStatus)
        result (MoveBaseResult): 
        	result of the goal processing. 

    No Returns. 

	
	"""
	global client
	global achieved
	global goal_cont

	goal_cont += 1 # Increment goal counter

	if status == 2:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing. Execution terminated." + bcolors.ENDC)
		cont = 1
		return
	if status == 3:
		print(bcolors.OKGREEN + bcolors.UNDERLINE + bcolors.BOLD + "Goal successfully achieved" + bcolors.ENDC)
		cont = 1
		achieved = True
		return
	if status == 4:
		print(bcolors.FAIL + "Timeout expired, the desired poition is not reachable. Goal aborted."  + bcolors.ENDC)
		cont = 1
		return
	if status == 5:
		print(bcolors.FAIL + "The goal was rejected" + bcolors.ENDC)
		cont = 1 
		return
	if status == 6:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing and has not yet completed execution"+ bcolors.ENDC)
		cont = 1
		return
	if status == 8:
		print(bcolors.FAIL + "The goal received a cancel request before it started executing and was successfully cancelled."+ bcolors.ENDC)
		cont = 1
		return


def active_cb():
	"""
	Callback that gets called on transitions to Active.

	No Args.

	No Returns. 
	"""
	print(bcolors.OKBLUE + bcolors.BOLD +"Goal number "+ str(goal_cont) + " is being processed..."  + bcolors.ENDC)

def feedback_cb(feedback):
	"""
	Callback that gets called whenever feedback for this goal is received. 

	Args: 
		feedback (move_base_msgs/MoveBaseActionFeedback.msg): information about the robot status during the the action server execution.

	No Returns.  
	"""
	global cont
	cont += 1	# Increment index
	print(str(cont) + ")\tFeedback from goal number " + str(goal_cont) + " received!")

def set_goal(x, y):
	"""
	This function fills the x, y fields of the goal message and sends a goal request to the action server.

	Args:
		x (double): x coordinate of the position that we want the robot to reach.  
		y (double): y coordinate of the position that we want the robot to reach.

	No Returns
	"""
	global goal_msg
	global client
	goal_msg.target_pose.pose.position.x = x
	goal_msg.target_pose.pose.position.y = y
	client.send_goal(goal_msg, done_cb, active_cb, feedback_cb)

def update_variables():
	"""
	Function for updating the ROS parameters: active, des_pos_x, des_pos_y. 

	No Args

	No Returns
	"""

	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('/active')
	desired_position_x = rospy.get_param('/des_pos_x')
	desired_position_y = rospy.get_param('/des_pos_y')

def main():
	"""
	In the main funciton some goals parameters of the goal message are set, updated variables and, if the current modality 
	is chosen, the ``set_goal()`` function is called. Finally, the the case in which a goal was achieved and the one in which the user 
	decides to send a cancel request before the goal achievement is managed. 
	"""
	global client
	global achieved
	global desired_position_x
	global desired_position_y
	global active_

	rospy.init_node('go_to_desired_pos') # Init node
	action_client() # Setting some goals' parameter
	flag=0 # Flag used in order to know if the previous state was Idle or not
	desired_position_x = rospy.get_param('/des_pos_x')
	desired_position_y = rospy.get_param('/des_pos_y')
	active_ = rospy.get_param('/active')	
	print(msg) 
	while (1):
		
		update_variables() # Update Ros parameters

		if active_==1: # If the current modality is active the code can be executed
			
			if flag == 1:	# If the prevoius state was Idle then we can set a new goal
				print(bcolors.OKGREEN + bcolors.UNDERLINE + "The robot is moving towards your desired target" + bcolors.ENDC)
				set_goal(desired_position_x, desired_position_y)	# Set a new goal
				flag = 0	# If this modality will be blocked, then must be put in Idle state

		else:
			if flag == 0 and achieved == False: # If we are in Idle state but a goal was not achieved we need to cancel the goal
				print(bcolors.OKBLUE + "Modality 1 is currently in idle state\n" + bcolors.ENDC)
				client.cancel_goal()	# Send a cancel request
				flag = 1	# Ready to set a new goal if this modality is unlocked

			if achieved == True: # If a goal was achieved there's no need to cancel the goal
				flag = 1
				achieved = False


if __name__ == '__main__':
    main()




	

