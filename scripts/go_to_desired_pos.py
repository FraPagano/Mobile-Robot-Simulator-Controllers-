#! /usr/bin/env python3

# import ros stuff
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *

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

msg = """ 
""" + bcolors.BOLD + """
This node makes the robot autonomously reach a x,y position inserted by the user.
The user's x,y coordinates are reached thanks to the 'move_base' action server. 
The robot is going to plan the path through the Dijkstra's algorithm. 
""" +bcolors.ENDC + """
"""

goal_msg=MoveBaseGoal()

my_timer = 0
active_ = rospy.get_param('active')
desired_position_x = rospy.get_param('des_pos_x')
desired_position_y = rospy.get_param('des_pos_y')
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
achieved = False
goal_cont = 1
cont = 1

def action_client():

	global goal_msg
	global client

	
	client.wait_for_server()

	goal_msg.target_pose.header.frame_id = 'map'
	goal_msg.target_pose.header.stamp = rospy.Time.now()
	goal_msg.target_pose.pose.orientation.w = 1


def done_cb(status, result):
	global client
	global achieved
	global goal_cont

	goal_cont += 1

	if status == 2:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing. Execution terminated." + bcolors.ENDC)
		return
	if status == 3:
		print(bcolors.OKGREEN + bcolors.UNDERLINE + bcolors.BOLD + "Goal successfully achieved" + bcolors.ENDC)
		cont = 1
		achieved = True
		return
	if status == 4:
		print(bcolors.FAIL + "Timeout expired, the desired poition is not reachable. Goal aborted."  + bcolors.ENDC)
		return
	if status == 5:
		print(bcolors.FAIL + "The goal was rejected" + bcolors.ENDC)
		return
	if status == 6:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing and has not yet completed execution"+ bcolors.ENDC)
		return
	if status == 8:
		print(bcolors.FAIL + "The goal received a cancel request before it started executing and was successfully cancelled."+ bcolors.ENDC)
		return


def active_cb():
	print(bcolors.OKBLUE + bcolors.BOLD +"Goal number "+ str(goal_cont) + " is being processed..."  + bcolors.ENDC)

def feedback_cb(feedback):
	global cont
	cont += 1
	print(str(cont) + ")\tFeedback from goal number " + str(goal_cont) + " received!")

def set_goal(x, y):

	global goal_msg
	global client
	goal_msg.target_pose.pose.position.x = x
	goal_msg.target_pose.pose.position.y = y
	client.send_goal(goal_msg, done_cb, active_cb, feedback_cb)

def update_variables():
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')

def main():

	global client
	global goal_msg
	global achieved

	flag=0
	
	rospy.init_node('go_to_desired_pos')
	action_client()
	rate = rospy.Rate(10)
	print(msg)
	while (1):
		
		update_variables()

		if active_==1:
			
			if flag == 1:
				print(bcolors.OKGREEN + bcolors.UNDERLINE + "The robot is moving towards your desired target" + bcolors.ENDC)
				set_goal(desired_position_x, desired_position_y)
				flag = 0

		else:
			if flag == 0 and achieved == False:
				print(bcolors.OKBLUE + "Modality 1 is currently in idle state\n" + bcolors.ENDC)
				client.cancel_goal()
				flag = 1

			if achieved == True:
				flag = 1
				achieved = False

		rate.sleep()


if __name__ == '__main__':
    main()




	

