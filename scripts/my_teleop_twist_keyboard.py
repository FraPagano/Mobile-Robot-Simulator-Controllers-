#!/usr/bin/env python3


"""
.. module:: my_teleop_twist_keyboard
    :platform: Unix
    :synopsis: Python module for controlling the robot using the keyboard
.. moduleauthor:: Francesco Pagano <francescopagano1999@outlook.it>

Publishes to:
    /cmd_vel
    
This is the Second Robot Controlling Modality.
This node reads inputs from the keyboard and makes the robot freely navigate in the environment.
Messages of type Twist() are published to the '/cmd_vel' topic. 

The functionality is quite similar to the teleop_twist_keyboad's one. 


"""

# IMPORTS
from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
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
Moving around:
   u    i    o
   j    k    l
   m    ,    .

""" + bcolors.ENDC + bcolors.OKCYAN + bcolors.BOLD + """
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)
""" + bcolors.ENDC + """
anything else : stop
""" + bcolors.HEADER + bcolors.BOLD +"""
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
"""+ bcolors.ENDC +""" 

"""


moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }
""" 
Dictionary for movement commands. The values in the key/value pair represents the direction in which the robot 
should move.
"""

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
        self.publisher.publish(twist) # Publishes the twist message 


def getKey(key_timeout): 
    settings = termios.tcgetattr(sys.stdin) # Settings for avoid printing commands on terminal
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1) # Get input key from standard input
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


def main():
    """
    In the ``main()`` function I made the most important changes from ``teleop_twist_keyboard`` code, that are:

        * Insertion of an if(active == 2) statement in order to block the code when another modality is running.
        * The keys now must be kept pressed in order to move the robot. I did this by setting the key_timeout variable to 0.1. Such variable was the select() timeout. That means that the select() function waits 0.1 seconds for new inputs at every loop.

    """
    

    rospy.init_node('my_teleop_twist_kb')   # Init node
    active_=rospy.get_param("/active")      # We want a local variable that is equal to the ROS parameter   
    flag = 1                                # Useful flag for determine an idle status
    
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)

    # Timeout set at 0.1 seconds. That means that the user needs to keep the key pushed for moving the robot. 
    key_timeout = rospy.get_param("~key_timeout", 0.1) 

    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0


    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)

    print(msg)
    print(vels(speed,turn))

    while(1):

        active_=rospy.get_param("/active") # Update the system status

        if active_ == 2: # If this modality is active
            key = getKey(key_timeout) #Get the input command
            if key in moveBindings.keys(): # If the input command is in the dictionary set the x, y, z, th variables
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
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
            if flag == 1: # Put the current modality in Idle state
                pub_thread.my_stop() 
                print(bcolors.OKGREEN + bcolors.BOLD + "Modality 2 is currently in idle state\n" + bcolors.ENDC)
            flag = 0



if __name__=="__main__":
    main()


            


