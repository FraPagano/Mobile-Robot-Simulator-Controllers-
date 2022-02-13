## Research Track 1, final assignment

================================

### Introduction

--------------------------------

The final assignment of the [Research Track 1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) class is about a mobile robot simulator using the ROS framework.  The last course lectures were about the software architecture for the control of a mobile robot and this assignment is about the development of such software architecture. The software will rely on the move_base  and gmapping packages for localizing the robot and plan the motion. 
The software architecture must control the robot in three different ways:

 1. Autonomously reach a x,y coordinate inserted by the user;
 2. Let the user drive the robot with the keyboard;
 3. Let the user drive the robot with the keyboard assisting them to avoid collisions.

In order to accomplish such task I wrote four nodes in python:

 1. **UI.py**
 2. **go_to_desired_pos.py**
 3. **my_teleop_twist_keyboard.py**
 4. **teleop_avoid**

Here's a picture that shows the simulation enviroment provided us by profesor [Carmine Recchiuto](https://github.com/CarmineD8):
<p align="center">
 <img src="https://github.com/FraPagano/final_assignment/blob/main/Images/Simulation_Env.JPG" height=320 width=500>
</p>


The structure is quite simple: The UI.py node is the user interface that the user should use when he wants to change the controlling modality or cancel a goal. The other three nodes represent the three robot controlling modalities. The activation of a nodes rather than the deactivation the other two is made through a ROS parameter called: ***active***. 
This parameter is set to three different values, each of them represent a m=controlling modality:

 1. ***active == 1***, for activating the first modality;
 2. ***active == 2***, for activating the second modality;
 3. ***active == 3***, for activating the third modality;

So that when the user chooses one modality through the user interface node, the ***active*** parameter is set to one these three values and the other two modalities will be blocked. Moreover, to set an idle state of the whole system, the ***active*** parameter can be set to 0. 

```python
#PSEUDOCODE

input = get_input()
if input == 1
	active = 1	# activates the first modality
if input == 2
	active = 2	# activates the second modality
if input == 3
	active = 3	# activates the third modality

```

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/scheme.jpg" height=350 width=450>
</p>

The greatest issues that I faced with during the implementation of the project were:

 - Become familiar with ROS parameter usage;

 - Become familiar with the simulation environment (Gazebo, RVIZ).

### Code description
---------------------------

#### User Interface Node
The User Interface node handles the user keyboard inputs. Here's a legend of the allowed commands:

 1. *'1'* keyboard key is used for choosing the  autonomously reaching modality;
 2. *'2'* keyboard key is used for the free keyboard driving modality;
 3. *'3'* keyboard key is used for the free keyboard driving modality with a collision avoidance algorithm;
 4. *'4'* keyboard key is used for quitting the application and terminates all nodes;
This node is very simply designed. Essentially, a function called `interpreter()` is looped inside a `while not rospy.is_shutdown():` loop. This function gets the keyboard user input and changes the ROS parameter ***active*** depending on which modality was chosen. 
Here's the `interpreter()` code:
``` python
def interpreter():
	#Function that receives inputs and sets all the ROS parameters
	
	command = input(bcolors.HEADER + 'Choose a modality: \n' + bcolors.ENDC) # Stores the input key
	
	if command == "0":
		rospy.set_param('active', 0) # if the active parameter is 0 the current goal is canceled
		print(bcolors.OKGREEN + "No modality is active, please choose one for controlling the robot" + bcolors.ENDC) # Sysytem in idle state
		active_=rospy.get_param("/active")
	elif command == "1": # Modality one chosen
		rospy.set_param('active', 0) # Useful for changing goal
		active_=rospy.get_param("/active")
		# Receive the desired cooridnates as input
		des_x_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired x position: " + bcolors.ENDC))
		des_y_input = float(input(bcolors.UNDERLINE + bcolors.OKBLUE +"Insert the desired y position: " + bcolors.ENDC))
		rospy.set_param('des_pos_x', des_x_input) # Update the desired X coordinate
		rospy.set_param('des_pos_y', des_y_input) # Update the desired Y coordinate
		rospy.set_param('active', 1) # Modality 1 active
	elif command == "2": # Modality two chosen
		rospy.set_param('active', 2) # Modality two active
		print(bcolors.OKGREEN + bcolors.UNDERLINE + "Modality 2 is active." + bcolors.ENDC)
		print(bcolors.BOLD + bcolors.HEADER + "Use the 'my_teleop_twist_keyboard' xterm terminal to control the robot" + bcolors.ENDC)
		active_=rospy.get_param("/active")
	elif command == "3": # Modality three chosen
		rospy.set_param('active', 3) # # Modality three active
		active_=rospy.get_param("/active")
	elif command == "4": # Exit command
		print(bcolors.WARNING + bcolors.BOLD + "Exiting..." + bcolors.ENDC)
		os.kill(os.getpid(), signal.SIGKILL) # Kill the current process
	else:
		print(bcolors.FAIL + bcolors.BOLD + "Wrong key! Use the shown commands " + bcolors.ENDC)
```
####  Autonomously reaching node (First Modality)
This node makes the robot autonomously reach a x,y position inserted by the user. The robot can reach the user's x,y coordinates thanks to the **'move_base' action server**.
The robot is going to plan the path through the Dijkstra's algorithm.
When the first modality is selected, the UI.py node sets the active ROS parameter to 1 letting the first modality's loop to run all the necessary code for sending the desired goal. 
The desired x, y coordinates are ROS parameters too and they are set by the UI.py node. 

When the first modality is running and a goal is received, this node uses the `send_goal(goal_msg, done_cb(), active_cb(), feedback_cb())` function for asking the action server to compute the path planning for the desired goal. 
 
 
 1.  ***goal_msg*** is a  MoveBaseGoal() action message containing all the information about the desired goal (i.e. x, y coordinates, referencing frame, etc... )
 2. ***done_cb(status, result)*** is a callback function called after the execution of the action server. It gives the client information about the termination of the goal process. In particular, this callback function returns a value that is stored in am `int` variable that I called *status*. Depending on the value of this variable the client knows the status of the goal processing. 
 3. ***active_cb()*** is a callback funtion called before the execution of the action server. I used this callback funtion in order to take into account the number of processed goals.
 4. ***feedback_cb(feedback)*** is a callback funtion called durning the execution of the action server. It returns feedbacks about the goal processing. 

Here's a picture that clarifies this concept: 

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/Action_server.JPG" height=250 width=480>
</p>

Thanks to the ***done_cb(status, result)*** function I could manage the result of the goal achievement.
Here's the ***done_cb(status, result)*** code:

```python
def done_cb(status, result):
# Function called after goal was processed. It is used to notify the client on the current status of every goal in the system.
global client
global achieved
global goal_cont
goal_cont += 1 # Increment goal counter
if status == 2:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing. Execution terminated." + bcolors.ENDC)
		cont = 1
		return
if status == 3:
		print(bcolors.OKGREEN + bcolors.UNDERLINE + bcolors.BOLD + "Goal successfully achieved" + bcolors.ENDC)
		cont = 1
		achieved = True
		return
if status == 4:
		print(bcolors.FAIL + "Timeout expired, the desired poition is not reachable. Goal aborted." + bcolors.ENDC)
		cont = 1
		return
if status == 5:
		print(bcolors.FAIL + "The goal was rejected" + bcolors.ENDC)
		cont = 1
		return
if status == 6:
		print(bcolors.FAIL + "The goal received a cancel request after it started executing and has not yet completed execution"+ bcolors.ENDC)
		cont = 1
		return
if status == 8:
		print(bcolors.FAIL + "The goal received a cancel request before it started executing and was successfully cancelled."+ bcolors.ENDC)
		cont = 1
		return
```

A concept that I want to point out is the that the achievement of the goal implies a cancel of such goal. So, I had to use a flag (`bool achieved`) to differentiate the case in which a goal was achieved (in this case the cacel request of an already canceled goal may cause an error, so I avoided to send the cancel request to the action server) and the case in which the user decides to send a cancel request before the goal achievement (in this case we must send a cancel request to the server).

This is a piece of code about the differentiation that I just pointed out:  
```python
if flag == 0 and achieved == False: # If we are in Idle state but a goal was not achieved we need to cancel the goal
		print("Modality 1 is currently in idle state\n")
		client.cancel_goal() # Send a cancel request
		flag = 1 # Ready to set a new goal if this modality is unlocked
if achieved == True: # If a goal was achieved there's no need to cancel the goal
		flag = 1
		achieved = False
```
 
####  Free drive with keyboard node  (Second Modality)
This node reads inputs from the keyboar and publishes a Twist() message to the `cmd_vel` topic. 
Basically I relied on the ***teleop_twist_keyboard***  code. So, the functionality is the same:
```
# Instructions
Moving around:
				u i o
				j k l
				m , .
For Holonomic mode (strafing), hold down the shift key:
---------------------------

				U I O
				J K L
				M < >
							
t : up (+z)
b : down (-z)
anything else : stop

q/z : increase/decrease max speeds by 10%

w/x : increase/decrease only linear speed by 10%

e/c : increase/decrease only angular speed by 10%
```

I modified some lines of code in order to adapt the process on my needs. This modality is a total free driving algorithm, that means that the robot can obviously bump into the obstacles. 
The main changes I've made from the ***teleop_twist_keyboard*** node are:

 1. Insertion of an `if(active == 2)` statement in order to block the code when another modality is running.
 2. The keys now must be kept pressed in order to move the robot. I did this by setting the `key_timeout` variable  to 0.1. Such variable was the `select()` timeout. That means that the `select()` function waits 0.1 seconds for new inputs at every loop. 

####  Free drive with keyboard and collision avoidance algorithm node  (Third Modality)
This node reads inputs from the keyboard and publishes a Twist() message to the `/cmd_vel` topic. 
Basically I relied on the ***teleop_twist_keyboard***  code and therefore the functionality is quite the same:
```
# Instructions
Reading from the keyboard and Publishing to Twist!
---------------------------
[i] go straight
[j] turn left
[l] turn right
[k] go backwards

[q]/[z] : increase/decrease max speeds by 10%
[w]/[x] : increase/decrease only linear speed by 10%
[e]/[c] : increase/decrease only angular speed by 10%
```

I modified some lines of code in order to adapt the process on my needs. This modality has also a collision avoidance algorithm that I implemented with my colleagues: [Alessandro Perri](https://github.com/PerriAlessandro), [Matteo Carlone](https://github.com/MatteoCarlone), [Luca Predieri](https://github.com/LucaPredieri) and [Fabio Conti](https://github.com/Fabioconti99). 
This algorithm is based on the modification of a Dictionary. 
Dictionary is adata type similar to a list and it contains a collection of objects. These objects are idexed with the key/value pair. In this case the contained value are  the linear and angular velocity combinations that the robot should assume after that an input occurrs. 
Such collision avoidance algorithm just uses the`.pop(key)` method that removes and returns the element idexed by *key* of  the dictionary. In this way, when the distance between the robot and the obstacle is less than a threshold value, a command is disabled just by popping such index from the dictionary. Moreover, by this way the robot cannot move in the direction in which the obstacle is detected. 
For implementing such algorithm, I needed the subscription to the `/scan` topic. This subscription allowed me to detect the obstacles in the simulation environment but also their position with respect to the robot by slicing the LaserScan array in three parts: front, left and right direction:

Here's a picture that represent how I sliced the laser scan span:

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/laser_scan.jpg" height=380 width=450>
</p>

```python
def clbk_laser(msg):
		# Callback for detecting obstacles from laser scan
		global ok_left		#This variable is set to True when there are no obstacles on the left of the robot
		global ok_right		#This variable is set to True when there are no obstacles on the right of the robot
		global ok_straight	#This variable is set to True when there are no obstacles in front of the robot
		
		right = min(min(msg.ranges[0:143]), 1) # Detects obstacles on the right of the robot
		front = min(min(msg.ranges[288:431]), 1) # Detects obstacles in front of the robot
		left = min(min(msg.ranges[576:719]), 1) # Detects obstacles on the left of the robot
		
		if right != 1.0: # No obstacles detected on the right at a distance less than 1 meter
				ok_right =False
		else: # Obstacle detected on the right of the robot
				ok_right =True
		if front != 1.0: # No obstacles detected in the front direction at a distance less than 1 meter
				ok_straight =False
		else: # Obstacle detected in front of the robot
				ok_straight =True
		if left != 1.0: # No obstacles detected on the left at a distance less than 1 meter
				ok_left =False
		else: # Obstacle detected on the left of the robot
				ok_left =True
``` 
Then, I considered all the eight combination that the robot could face:
 
<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/8_combinations.jpg" height=380 width=400>
</p>

and finally I popped the dictionary indexes that would have moved the robot towards the detected obstacle:
```python
def pop_dict(dictionary):
# Function that removes commands from the dictionary when an obstacle is detected. 
#In this way, when an obstacle is detected, the movement in that direction is disabled
		global ok_left
		global ok_right
		global ok_straight
		# All the cases the robot could face
		if not ok_straight and not ok_right and not ok_left: # Obstacles in every direction
				# Disable all the three commands
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('j') # Disable the left turn movement
				dictionary.pop('l') # Disable the right turn movement
		elif not ok_left and not ok_straight and ok_right: # Obstacles on the left and in the front direction, so right direction is free
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and not ok_straight and not ok_right: # Obstacles on the right and in the front direction, so left direction is free
				dictionary.pop('i') # Disable the front movement
				dictionary.pop('l') # Disable the right turn movement
		elif not ok_left and ok_straight and not ok_right: # Obstacles on the right and on the left, so the front direction is free
				dictionary.pop('l') # Disable the right turn movement
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and not ok_straight and ok_right: # Obstacles only in the front direction, so the left and right directions are free
				dictionary.pop('i') # Disable the front movement
		elif not ok_left and ok_straight and ok_right: # Obstacles only in the left direction, so the front and right directions are free
				dictionary.pop('j') # Disable the left turn movement
		elif ok_left and ok_straight and not ok_right: # Obstacles only in the right direction, so the front and left directions are free
				dictionary.pop('l') # Disable the right turn movement
```

###  Installing and running 
----------------------

Here's some useful informations regarding how to the nodes and the simulation environment.
First of all, [xterm](https://it.wikipedia.org/wiki/Xterm), a standard terminal emulator, is needed. You can install xterm by entering the following commands in the terminal:
```
sudo apt update
sudo apt-get install xterm
```
I created a launch file in the launch directory that executes all the necessary nodes and the simulation environment. You can run the programs by entering the following command:

```
roslaunch final_assignment launchAll.launch 
```

The three modalities and the UI nodes will run on different xterm terminals. You can control the robot with the three different modalities by reerring to each xterm terminal that will appear after the launch file execution. 

If any of the three node terminates, the launch file will terminates all the nodes.

### Rqt Graph
--------------------------------
In order to have a GUI plugin for visualizing the ROS computation graph, here's a *rqt_graph* about the project:

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/rqt_graph.JPG" height=380 width=750>
</p>


### Flowcharts
--------------------------------

For a more precise description of what the four nodes do you can consult the following flowcharts, created with [Lucidchart](https://www.lucidchart.com/pages/it/landing?utm_source=google&utm_medium=cpc&utm_campaign=_chart_it_allcountries_mixed_search_brand_bmm_&km_CPC_CampaignId=9589672283&km_CPC_AdGroupID=99331286392&km_CPC_Keyword=%2Blucidcharts&km_CPC_MatchType=b&km_CPC_ExtensionID=&km_CPC_Network=g&km_CPC_AdPosition=&km_CPC_Creative=424699413299&km_CPC_TargetID=kwd-334618660008&km_CPC_Country=1008337&km_CPC_Device=c&km_CPC_placement=&km_CPC_target=&mkwid=sKwFuAgHb_pcrid_424699413299_pkw_%2Blucidcharts_pmt_b_pdv_c_slid__pgrid_99331286392_ptaid_kwd-334618660008_&gclid=CjwKCAjw5c6LBhBdEiwAP9ejG86DblinG5ivYRvMmKSvI8Dl7as9i2oINlmgqIDoj0gpLX6WfnCenRoCxxQQAvD_BwE):

This is the UI.py node's flowchart:
<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/UI_flowchart.jpeg" height=670 width=750>
</p>

This is the go_to_desired_pos.py node's flowchart:
<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/go_to_desire_pos_flowchart.jpeg" height=670 width=700>
</p>

This is the my_teleop_twist_keyboard.py node's flowchart:

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/my_teleop_twist_keyboard_flowchart.jpeg" height=670 width=750>
</p>

This is the teleop_avoid.py node's flowchart:

<p align="center">
<img src="https://github.com/FraPagano/final_assignment/blob/main/Images/teleop_avoid_flowchart.jpeg" height=760 width=710>
</p>

### Results
--------------------------------

The final result is that the robot correctly runs around the simulation environment switching among the three modalities. 

### Possible Improvements
--------------------------------
A possible improvement that could be done is the usage of an algorithm such that the robot could map the entire environment from the beginning of the simulation and so that it can immediately detect all the x, y coordinates that cannot be reached. 
