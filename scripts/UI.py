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
		rospy.set_param('active', 1)
		print("Modality 1 is active")
		active_=rospy.get_param("/active")
		print(active_)

	elif command == 2:
		rospy.set_param('active', 2)
		print("Modality 2 is active")
		active_=rospy.get_param("/active")
		print(active_)
	elif command == 3:
		rospy.set_param('active', 3)
		print("Modality 3 is active")
		active_=rospy.get_param("/active")
		print(active_)
	else:
		print("Wrong key")


def main():
	while not rospy.is_shutdown():
		interpreter()


main()

