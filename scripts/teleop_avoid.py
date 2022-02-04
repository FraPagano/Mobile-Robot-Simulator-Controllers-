#!/usr/bin/env python3

from __future__ import print_function

import threading
from sensor_msgs.msg import LaserScan
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy


from geometry_msgs.msg import Twist
import time
from std_srvs.srv import *
import sys, select, termios, tty

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

ok_left = True
ok_right = True
ok_straight = True

moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
    }


speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
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


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clbk_laser(msg):


    global ok_left
    global ok_right
    global ok_straight

    right = min(min(msg.ranges[0:143]), 1)
    front = min(min(msg.ranges[288:431]), 1)
    left = min(min(msg.ranges[576:719]), 1)

    if right != 1.0:
        ok_right =False
    else:
        ok_right =True

    if front != 1.0:
        ok_straight =False
    else:
        ok_straight =True

    if left != 1.0:
        ok_left =False
    else:
        ok_left =True


def pop_dict(dictionary):

    global ok_left
    global ok_right
    global ok_straight
    

    if not ok_straight and not ok_right and not ok_left:
        popped1 = dictionary.pop('i')
        popped2 = dictionary.pop('j')
        popped3 = dictionary.pop('l')
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and not ok_straight and ok_right:
        popped1 = dictionary.pop('i')
        popped2 = dictionary.pop('j')
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and not ok_straight and not ok_right:
        popped1 = dictionary.pop('i')
        popped2 = dictionary.pop('l')
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and ok_straight and not ok_right:
        popped1 = dictionary.pop('l')
        popped2 = dictionary.pop('j')
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and not ok_straight and ok_right:
        popped1 = dictionary.pop('i')
        print(bcolors.FAIL + "Command 'i' disabled" + bcolors.ENDC , end="\r")
    elif not ok_left and ok_straight and ok_right:
        popped1 = dictionary.pop('j')
        print(bcolors.FAIL + "Command 'j' disabled" + bcolors.ENDC , end="\r")
    elif ok_left and ok_straight and not ok_right:
        popped1 = dictionary.pop('l')
        print(bcolors.FAIL + "Command 'l' disabled" + bcolors.ENDC , end="\r")


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_avoid')
    active_=rospy.get_param("/active")
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


