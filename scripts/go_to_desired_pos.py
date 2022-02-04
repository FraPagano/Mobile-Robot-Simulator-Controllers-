#! /usr/bin/env python3

# IMPORTS
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *

#COLORS 
class bcolors:
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

goal_msg=MoveBaseGoal()	# Action message

active_ = rospy.get_param('active')					# ROS poarameter to block/unlock the modality 
desired_position_x = rospy.get_param('des_pos_x')	# X desired coordinate 
desired_position_y = rospy.get_param('des_pos_y')	# Y desired coordinate 

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)	# Action client

achieved = False	# Variable for defining if a goal was achieved or not
goal_cont = 1		# Goal counter
cont = 1			# Feedback index

def action_client():
	# Sets some parameters of the action message

	global goal_msg
	global client

	client.wait_for_server()	# Waits until we are connected to the action server

	# Setting some goal's fields
	goal_msg.target_pose.header.frame_id = 'map'			
	goal_msg.target_pose.header.stamp = rospy.Time.now()	
	goal_msg.target_pose.pose.orientation.w = 1				


def done_cb(status, result):
	# Function called after goal was processed. It is used to notify the client on the current status of every goal in the system.
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
	# Function called before goal processing
	print(bcolors.OKBLUE + bcolors.BOLD +"Goal number "+ str(goal_cont) + " is being processed..."  + bcolors.ENDC)

def feedback_cb(feedback):
	# Function called for sending client feedback, that are some auxiliary information
	global cont
	cont += 1	# Increment index
	print(str(cont) + ")\tFeedback from goal number " + str(goal_cont) + " received!")

def set_goal(x, y):
	# Creates a goal and sends it to the action server. 
	global goal_msg
	global client
	goal_msg.target_pose.pose.position.x = x
	goal_msg.target_pose.pose.position.y = y
	client.send_goal(goal_msg, done_cb, active_cb, feedback_cb)

def update_variables():
	# Function for updating the ROS parameters

	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')

def main():

	global client
	global goal_msg
	global achieved

	rospy.init_node('go_to_desired_pos') # Init node

	action_client() # Setting some goals' parameter

	flag=0 # Flag used in order to know if the previous state was Idle or not

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




	

