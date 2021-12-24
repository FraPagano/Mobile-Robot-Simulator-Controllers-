#!/usr/bin/python3
import rospy


def interpreter():

	command = int(input('Choose modality: \n'))

	if command == 0:
		rospy.set_param('active', 0)
		print("Idle")
		active_=rospy.get_param("/active")
		print(active_)

	elif command == 1:

		print("Modality 1 is active, press '0' to cancel the target.")
		active_=rospy.get_param("/active")
		print("Where do you want the robot to go?")
		des_x_input = float(input("Insert the desired x position: "))
		des_y_input = float(input("Insert the desired y position: "))
		rospy.set_param('des_pos_x', des_x_input)
		rospy.set_param('des_pos_y', des_y_input)
		rospy.set_param('active', 1)

	elif command == 2:
		rospy.set_param('active', 2)
		print("Modality 2 is active.")
		active_=rospy.get_param("/active")
		
	elif command == 3:
		rospy.set_param('active', 3)
		print("Modality 3 is active.")
		active_=rospy.get_param("/active")
		
	else:
		print("Wrong key")


def main():
	while not rospy.is_shutdown():
		interpreter()


main()

