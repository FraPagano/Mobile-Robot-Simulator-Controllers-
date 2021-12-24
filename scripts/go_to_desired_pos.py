#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal

from actionlib_msgs.msg import GoalID 

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math

goal_msg=MoveBaseActionGoal()
goal_cancel=GoalID()

goal_msg.goal.target_pose.header.frame_id = 'map'
goal_msg.goal.target_pose.pose.orientation.w = 1
active_ = rospy.get_param('active')
desired_position_x = rospy.get_param('des_pos_x')
desired_position_y = rospy.get_param('des_pos_y')

	


def set_goal(x, y):
	goal_msg.goal.target_pose.pose.position.x = x
	goal_msg.goal.target_pose.pose.position.y = y
	pub_goal.publish(goal_msg)



def main():

	global pub_vel 
	global pub_goal

	flag=0
	rospy.init_node('go_to_desired_pos')
	
	pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
	pub_cancel_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	pub_goal.publish(goal_msg)
	# sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	while (1):
		active_ = rospy.get_param('active')
		desired_position_x = rospy.get_param('des_pos_x')
		desired_position_y = rospy.get_param('des_pos_y')

		if active_==1:
			print(active_)
			if flag == 1:
				print("x= " + str(desired_position_x))
				print("y= " + str(desired_position_y))
				set_goal(desired_position_x, desired_position_y)
				flag = 0

		else:
			if flag == 0:
				print("IDLE MODALITY 1\n")
				pub_cancel_goal.publish(goal_cancel)
				flag = 1


if __name__ == '__main__':
    main()




	

