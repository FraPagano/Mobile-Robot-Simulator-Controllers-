#!/usr/bin/python3

"""
.. module:: UI
	:platform: Unix
	:synopsis: Python module for the User Interface
.. moduleauthor:: Francesco Pagano <francescopagano1999@outlook.it>

This module implements an user interface that allows the user to switch among the three modalities.
ROS parameter are used in order to activate / deactivate the chosen modality. 

ROS parameters: 
	1.	"active": (type: int) parameter for activate the desired control modality 
	2.	"des_pos_x": (type: double) parameter for the desired X coordinate 
	3.	"des_pos_y": (type: double) parameter for the desired Y coordinate 

"""

# IMPORTS
import rospy
import os
import signal

# COLORS
class bcolors:
	"""
	This class is used for  printing colors on the terminal
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

ascii_art_message = """ """ + bcolors.BOLD + bcolors.HEADER + """
				 _________
			|\     /|\ _   __/
			| )   ( |   ) (   
			| |   | |   | |   
			| |   | |   | |   
			| |   | |   | |   
			| (___) |___) (___
			(_______)\_______/
""" + bcolors.ENDC +""" """

# Intro message
intro = """ 
""" + bcolors.HEADER + bcolors.BOLD + """
Hi! This is your User Interface """ + bcolors.ENDC + bcolors.UNDERLINE + """
You can move the robot through three different modalities:
"""
# Menu message showing the three modalities
menu_msg = """
""" + bcolors.ENDC + """
----------------------------------------------------------------
[1] """ + bcolors.UNDERLINE + """Insert your desired position """ + bcolors.ENDC + """
[2] """ + bcolors.UNDERLINE + """Free Drive the robot with your keyboard """ + bcolors.ENDC +"""
[3] """ + bcolors.UNDERLINE + """Free Drive the robot with your keyboard assisted by an obstacle avoidance algorithm """ + bcolors.ENDC + """
[4] """ + bcolors.UNDERLINE + bcolors.FAIL + """Quit the simulaiton
"""



flag = False 
"""
Global bool for knowing if the prevoius modality was the first one so that a goal can be canceled online
"""

def interpreter():
	"""
	This function gets the keyboard user input and changes the ROS parameter active depending on which modality was chosen.

	- '1' keyboard key is used for choosing the autonomously reaching modality;

	- '2' keyboard key is used for the free keyboard driving modality;

	- '3' keyboard key is used for the free keyboard driving modality with a collision avoidance algorithm;

	- '4' keyboard key is used for quitting the application and terminates all nodes.

		No Args. 

		No Returns. 
	"""
	global flag 
	print(menu_msg)

	if flag == True:
		print(bcolors.FAIL + bcolors.BOLD + "Press [0] for canceling the goal" + bcolors.ENDC)
		flag = False


	command = input(bcolors.HEADER + 'Choose a modality: \n' + bcolors.ENDC) # Stores the input key


	if command == "0":
		rospy.set_param('active', 0)	# if the active parameter is 0 the current goal is canceled
		print(bcolors.OKGREEN + "No modality is active, please choose one for controlling the robot" + bcolors.ENDC) # Sysytem in idle state
		active_=rospy.get_param("/active")

	elif command == "1": # Modality one chosen

		rospy.set_param('active', 0) # Useful for changing goal 
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 1 is active.")
		active_=rospy.get_param("/active")
		print(bcolors.OKBLUE + bcolors.BOLD + "Where do you want the robot to go?" + bcolors.ENDC)

		# Receive the desired cooridnates as input
		des_x_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired x position: " + bcolors.ENDC))
		des_y_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired y position: " + bcolors.ENDC))

		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Okay, let's reach the psotion x= " + str(des_x_input) + " y= " + str(des_y_input) + bcolors.ENDC)
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "\nThe robot is moving towards your desired target" + bcolors.ENDC)	

		rospy.set_param('des_pos_x', des_x_input) # Update the desired X coordinate
		rospy.set_param('des_pos_y', des_y_input) # Update the desired Y coordinate
		rospy.set_param('active', 1) # Modality 1 active

		flag=True

	elif command == "2": # Modality two chosen
		rospy.set_param('active', 2) # Modality two active
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 2 is active." + bcolors.ENDC)
		print(bcolors.BOLD + bcolors.HEADER + "Use the 'my_teleop_twist_keyboard' xterm terminal to control the robot" + bcolors.ENDC)
		active_=rospy.get_param("/active")
		
	elif command == "3": # Modality three chosen
		rospy.set_param('active', 3) # # Modality three active
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 3 is active." + bcolors.ENDC)
		print(bcolors.BOLD + bcolors.OKBLUE + "Use the 'teleop_avoid' xterm terminal to control the robot" + bcolors.ENDC)
		active_=rospy.get_param("/active")

	elif command == "4": # Exit command
		print(bcolors.WARNING + bcolors.BOLD + "Exiting..." + bcolors.ENDC)
		os.kill(os.getpid(), signal.SIGKILL) # Kill the current process
		
	else:
		print(bcolors.FAIL + bcolors.BOLD + "Wrong key! Use the shown commands " + bcolors.ENDC)


def main():
	"""
	In the main() function the ``interpreter()`` function is looped and some introductory messages are printed on the terminal. "
	"""
	print(ascii_art_message)
	print(intro)
	while not rospy.is_shutdown():
		interpreter()

if __name__ == '__main__':
    main()

