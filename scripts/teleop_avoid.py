#!/usr/bin/env python3

"""
.. module:: teleop_avoid
    :platform: Unix
    :synopsis: Python module for controlling the robot using the keyboard assisted by an obstacle avoidance algorithm
.. moduleauthor:: Francesco Pagano <francescopagano1999@outlook.it>

Subscribes to:
    /scan

Publishes to:
    /cmd_vel

This is the Third Controlling Modality.
Reads inputs from the keyboard and makes the robot navigate in the environment with an obstacle avoidance algorithm. 
Messages of type Twist() are published to the '/cmd_vel' topic. 

The functionality is quite similar to the teleop_twist_keyboad's one. 

"""


# IMPORTS
from __future__ import print_function
import threading
from sensor_msgs.msg import LaserScan
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import time
from std_srvs.srv import *
import sys, select, termios, tty

# COLORS
class bcolors:
    """
    This class is used for printing colors on the terminal. 
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

# Instructions message
msg = """
""" + bcolors.BOLD +"""
Reading from the keyboard and Publishing to Twist!
---------------------------
""" + bcolors.ENDC + bcolors.OKBLUE + bcolors.BOLD + """
[i] go straight    
[j] turn left
[l] turn right
[k] go backwards

""" + bcolors.HEADER + bcolors.BOLD +"""
[q]/[z] : increase/decrease max speeds by 10%
[w]/[x] : increase/decrease only linear speed by 10%
[e]/[c] : increase/decrease only angular speed by 10%
""" + bcolors.ENDC + """

"""

# Bool variables for taking into account where obstacles are


ok_left = True 
"""
Bool global variable that is True when there's no wall on the left of the robot, otherwise is False
"""
 
ok_right = True
"""
Bool global variable that is True when there's no wall on the right of the robot, otherwise is False
"""  
    
ok_straight = True  
"""
Bool global variable that is True when there's no wall in front of the robot, otherwise is False
""" 

# Dictionary for movement commands
moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
    }
""" 
Dictionary for allowed movement commands. The values in the key/value pair represents the direction in which the robot 
should move.
"""


# Dictionary for velocities
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
""" 
Dictionary for velocities commands. The values in the key/value pair represents the linear and angular 
velocity combinations that the robot should assume after that an input occurrs.
"""

class PublishThread(threading.Thread):
    """
    """
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #Publisher on the 'cmd_vel' topic
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def my_stop(self):
        # Class funtion for stopping the robot movement 
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        # Publish.
        self.publisher.publish(twist)

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout): # Get input key
    settings_old = termios.tcgetattr(sys.stdin) 
    settings_new = settings_old
    settings_new[3] = settings_new[3] & ~termios.ECHO
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings_new)
        key = sys.stdin.read(1) # Get input key from standard input
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings_old)
    return key

def clbk_laser(msg):

    """
    This callback function is for detecting the obstacles from laser scan information. 
    This subscription allowed me to detect the obstacles in the simulation environment but also their position with 
    respect to the robot by slicing the LaserScan array in three parts: front, left and right direction. 

    * If no obstacles are detected on the right of the robot the variable ``ok_right`` is set to True, otherwise to False.
    * If no obstacles are detected on the left of the robot the variable ``ok_left`` is set to True, otherwise to False.
    * If no obstacles are detected in front of the robot the variable ``ok_straight`` is set to True, otherwise to False.

    Args:
        msg: sensor_msgs/LaserScan.msg, scan from a planar laser range-finder. 
    No Returns. 
    """

    global ok_left
    global ok_right
    global ok_straight

    right = min(min(msg.ranges[0:143]), 1)      # Detects obstacles on the right of the robot
    front = min(min(msg.ranges[288:431]), 1)    # Detects obstacles in front of the robot
    left = min(min(msg.ranges[576:719]), 1)     # Detects obstacles on the left of the robot

    if right != 1.0:    # No obstacles detected on the right at a distance less than 1 meter
        ok_right =False
    else:               # Obstacle detected on the right of the robot
        ok_right =True

    if front != 1.0:    # No obstacles detected in the front direction at a distance less than 1 meter
        ok_straight =False
    else:               # Obstacle detected in front of the robot
        ok_straight =True

    if left != 1.0:     # No obstacles detected on the left at a distance less than 1 meter
        ok_left =False
    else:               # Obstacle detected on the left of the robot
        ok_left =True


def pop_dict(dictionary):
    """
    The obstacle avoidance algorithm is based on the modification of the Dictionary for the allowed movements.
    Such collision avoidance algorithm just uses the ``.pop(key)`` method that removes and returns the element idexed by key of the dictionary. 
    When an obstacle is detected, thanks to the ``/scan`` subscription we also know its direction with respect to the robot, and
    this function just pops the index of such direction from the dictionary. In this way the obsatcle
    direction is no more allowed and it will be impossible to publish on the /cmd_vel in that direction. 
    The ``.pop(key)`` methos is applied by considering all the combinations that the robot could face. 

    Args:
        dictionary (dict): allowed movements dictionary

    No Returns


    """

    global ok_left
    global ok_right
    global ok_straight
    
    # All the cases the robot could face

    if not ok_straight and not ok_right and not ok_left: # Obstacles in every direction
        # Disable all the three commands 
        popped1 = dictionary.pop('i')   # Disable the front movement 
        popped2 = dictionary.pop('j')   # Disable the left turn movement   
        popped3 = dictionary.pop('l')   # Disable the right turn movement 
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and not ok_straight and ok_right: # Obstacles on the left and in the front direction, so right direction is free
        popped1 = dictionary.pop('i')   # Disable the front movement 
        popped2 = dictionary.pop('j')   # Disable the left turn movement
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and not ok_straight and not ok_right: # Obstacles on the right and in the front direction, so left direction is free
        popped1 = dictionary.pop('i')   # Disable the front movement 
        popped2 = dictionary.pop('l')   # Disable the right turn movement
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and ok_straight and not ok_right: # Obstacles on the right and on the left, so the front direction is free
        popped1 = dictionary.pop('l')   # Disable the right turn movement
        popped2 = dictionary.pop('j')   # Disable the left turn movement
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and not ok_straight and ok_right: # Obstacles only in the front direction, so the left and right directions are free
        popped1 = dictionary.pop('i')   # Disable the front movement 
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and ok_straight and ok_right: # Obstacles only in the left direction, so the front and right directions are free
        popped1 = dictionary.pop('j')   # Disable the left turn movement 
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and ok_straight and not ok_right: # Obstacles only in the right direction, so the front and left directions are free
        popped1 = dictionary.pop('l')   # Disable the right turn movement 
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


def main():

    """
    In the ``main()`` function I made some other changes from the ``teleop_twist_keyboard`` code, that are:

        * Insertion of an if(active == 3) statement in order to block the code when another modality is running.
        * The keys now must be kept pressed in order to move the robot. I did this by setting the key_timeout variable to 0.1. Such variable was the select() timeout. That means that the select() function waits 0.1 seconds for new inputs at every loop
        * Added the above descripted functions: ``clbk_laser`` and ``pop_dict``
    """

    # Init node
    rospy.init_node('teleop_avoid') 
    active_=rospy.get_param("/active") # We want a local variable that is equal to the ROS parameter 
    flag = 1 
    
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    rate = rospy.Rate(5)
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)
    moveBindings_temp = {}
    print(msg)
    print(vels(speed,turn))
    while(1):
        active_=rospy.get_param("/active")
        moveBindings_temp = moveBindings.copy()
        if active_ == 3:
            key = getKey(key_timeout)
            
            pop_dict(moveBindings_temp)

            if key in moveBindings_temp.keys():

                x = moveBindings_temp[key][0] 
                y = moveBindings_temp[key][1]
                z = moveBindings_temp[key][2]
                th = moveBindings_temp[key][3]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)
            flag = 1

        else:
            if flag == 1:
                pub_thread.my_stop() 
                print(bcolors.OKGREEN + bcolors.BOLD + "Modality 3 is currently in idle state\n" + bcolors.ENDC)
            flag = 0

        rate.sleep()




if __name__=="__main__":
    main()

    

