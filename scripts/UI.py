#!/usr/bin/python3
import rospy
import os
import signal

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


def interpreter():

	command = int(input(bcolors.HEADER + 'Choose your modality or press [4] for quit \n' + bcolors.ENDC))

	if command == 0:
		rospy.set_param('active', 0)
		print(bcolors.OKGREEN + "Idle" + bcolors.ENDC)
		active_=rospy.get_param("/active")
		print(active_)

	elif command == 1:

		rospy.set_param('active', 0)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 1 is active," + bcolors.ENDC + " press '0' to cancel the target.")
		active_=rospy.get_param("/active")
		print(bcolors.OKBLUE + bcolors.BOLD + "Where do you want the robot to go?" + bcolors.ENDC)
		des_x_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired x position: " + bcolors.ENDC))
		des_y_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired y position: " + bcolors.ENDC))
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Okay, let's reach the psotion x= " + str(des_x_input) + " y= " + str(des_y_input) + bcolors.ENDC)
		rospy.set_param('des_pos_x', des_x_input)
		rospy.set_param('des_pos_y', des_y_input)
		rospy.set_param('active', 1)

	elif command == 2:
		rospy.set_param('active', 2)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 2 is active." + bcolors.ENDC)
		active_=rospy.get_param("/active")
		
	elif command == 3:
		rospy.set_param('active', 3)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 3 is active." + bcolors.ENDC)
		active_=rospy.get_param("/active")

	elif command == 4:
		print(bcolors.WARNING + bcolors.BOLD + "Exiting..." + bcolors.ENDC)
		os.kill(os.getpid(), signal.SIGKILL)
		
	else:
		print(bcolors.FAIL + "Wrong key" + bcolors.ENDC)


def main():
	print(bcolors.HEADER + bcolors.BOLD + "Hi! This is your User Interface" + bcolors.ENDC) 
	print(bcolors.UNDERLINE + "You can move the robot through three different modalities:" + bcolors.ENDC)
	print("\n[1] " + bcolors.UNDERLINE + "By selecting your desired position\n\n" + bcolors.ENDC + "[2] " + bcolors.UNDERLINE + "By controlling it with your keyboard \n\n" + bcolors.ENDC + "[3] " + bcolors.UNDERLINE + "By controlling it with your keyboard assisted by an obstacle avoidance algorithm\n" + bcolors.ENDC)

	while not rospy.is_shutdown():
		interpreter()

main()

