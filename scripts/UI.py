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

intro = """ 
""" + bcolors.HEADER + bcolors.BOLD + """
Hi! This is your User Interface """ + bcolors.ENDC + bcolors.UNDERLINE + """
You can move the robot through three different modalities:
"""

menu_msg = """
""" + bcolors.ENDC + """
----------------------------------------------------------------
[1] """ + bcolors.UNDERLINE + """Insert your desired position """ + bcolors.ENDC + """
[2] """ + bcolors.UNDERLINE + """Free Drive the robot with your keyboard """ + bcolors.ENDC +"""
[3] """ + bcolors.UNDERLINE + """Free Drive the robot with your keyboard assisted by an obstacle avoidance algorithm """ + bcolors.ENDC + """
[4] """ + bcolors.UNDERLINE + bcolors.FAIL + """Quit the simulaiton
"""

flag = False
def interpreter():
	global flag 
	print(menu_msg)

	if flag == True:
		print(bcolors.FAIL + bcolors.BOLD + "Press [0] for canceling the target" + bcolors.ENDC)
		flag = False
	command = input(bcolors.HEADER + 'Instert a command \n' + bcolors.ENDC)
	if command == "0":
		rospy.set_param('active', 0)
		print(bcolors.OKGREEN + "Idle" + bcolors.ENDC)
		active_=rospy.get_param("/active")
		print(active_)

	elif command == "1":

		rospy.set_param('active', 0)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 1 is active.")
		active_=rospy.get_param("/active")
		print(bcolors.OKBLUE + bcolors.BOLD + "Where do you want the robot to go?" + bcolors.ENDC)
		des_x_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired x position: " + bcolors.ENDC))
		des_y_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired y position: " + bcolors.ENDC))
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Okay, let's reach the psotion x= " + str(des_x_input) + " y= " + str(des_y_input) + bcolors.ENDC)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "The robot is moving towards your desired target" + bcolors.ENDC)		
		rospy.set_param('des_pos_x', des_x_input)
		rospy.set_param('des_pos_y', des_y_input)
		rospy.set_param('active', 1)
		flag=True

	elif command == "2":
		rospy.set_param('active', 2)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 2 is active." + bcolors.ENDC)
		active_=rospy.get_param("/active")
		
	elif command == "3":
		rospy.set_param('active', 3)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 3 is active." + bcolors.ENDC)
		active_=rospy.get_param("/active")

	elif command == "4":
		print(bcolors.WARNING + bcolors.BOLD + "Exiting..." + bcolors.ENDC)
		os.kill(os.getpid(), signal.SIGKILL)
		
	else:
		print(bcolors.FAIL + "Wrong key" + bcolors.ENDC)


def main():
	
	print(intro)
	while not rospy.is_shutdown():
		interpreter()

main()

