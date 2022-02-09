# ## Research Track 1, final assignment

================================

### Introduction

--------------------------------

The second assignment of the [Research Track 1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) class is about a mobile robot simulator using the ROS framework.  The last course lectures were about the software architecture for the control of a mobile robot and this assignment is about the development of such software architecture. The software will rely on the move_base  and gmapping packages for localizing the robot and plan the motion. 
The software architecture must control the robot in three different ways:

 1. Autonomously reach a x,y coordinate inserted by the user;
 2. Let the user drive the robot with the keyboard;
 3. Let the user drive the robot assisting them to avoid collisions.

In order to accomplish such task I wrote four nodes in python:

 1. **UI.py**
 2. **go_to_des_pos.py**
 3. **my_teleop_twist_keyboard.py**
 4. **teleop_avoid**

Here's some pictures that show the simulation enviroment provided us by profesor [Carmine Recchiuto](https://github.com/CarmineD8):
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Robot.JPG" height=320 width=380>
</p>
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Circuit.JPG" height=320 width=380>
</p>

The structure is quite simple: The UI.py node is the user interface that the user should use when he wants to change the controlling modality or cancel a goal. The other three nodes represent the three robot controlling modalities. The activation of a nodes rather than the deactivation the other two is made through a ROS parameter called: ***active***. 
This parameter is set to value:

 1. ***active == 1***, for activating the first modality;
 2. ***active == 2***, for activating the second modality;
 3. ***active == 3***, for activating the third modality;

So that when the user chooses one modality through the user interface node, the ***active*** parameter is set to one these three values and the other two modalities will be blocked. Moreover, to set an idle state of the whole system, the ***active*** parameter can be set to 0. 

The greaest issues that I faced with during the implementation of the project were:

 - Become familiar with ROS parameter usage;

 - Become familiar with the simulation environment.

### Code description
---------------------------

#### User Interface Node
The User Interface node handles the user keyboard inputs. Here's a legend of the allowed commands:

 - *'1'* keyboard key is used for choosing the  autonomously reaching modality;
 - *'2'* keyboard key is used for the free keyboard driving modality;
 - *'3'* keyboard key is used for the free keyboard driving modality with a collision avoidance algorithm;
 - *'4'* keyboard key is used for quitting the application and terminates all nodes;
 
This node is very simply designed. Essentially, a function called `interpreter()` is looped insiude a `while not rospy.is_shutdown():` loop. This function gets the keyboard user input and changes the ROS parameter ***active*** depending on which modality was chosen. 
Here's the `interpreter()` code:
``` python
def interpreter():
	#Function that receives inputs and sets all the ROS parameters
	global flag
	if flag == True:
		print(bcolors.FAIL + bcolors.BOLD + "Press [0] for canceling the goal" + bcolors.ENDC)
		flag = False
	command = input(bcolors.HEADER + 'Choose a modality: \n' + bcolors.ENDC) # Stores the input key
	if command == "0":
		rospy.set_param('active', 0) # if the active parameter is 0 the current goal is canceled
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
```
####  Autonomously reaching node (First Modality)
####  Free drive with keyboard node  (Second Modality)
####  Free drive with keyboard and collision avoidance algorithm node  (Third Modality)


###  Installing and running 
----------------------

Here's some useful informations regarding how to the nodes and the simulation environment.
First of all, [xterm](https://it.wikipedia.org/wiki/Xterm), a standard terminal emulator, is needed. You can install xterm by entering the following commands in the terminal:
```
sudo apt update
sudo apt-get install xterm
```
I created a launch file in the launch directory that executes all the necessary nodes and the simulation environment. You can run the program by entering the following command:

```
roslaunch final_assignment launchAll.launch 
```

The three modalities and the UI nodes will run on an xterm terminal. 

If any of the three node terminates, the launch file will terminates all the nodes.

### Rqt Graph
--------------------------------
In order to have a GUI plugin for visualizing the ROS computation graph, here's a *rqt_graph* about the project:

<p align="center">
<img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos%2C%20gifs%20%20and%20images/rqt_graph.jpg" height=250 width=500>
</p>

As you can see, the *control* node publishes both the linear and the angular velocity to the robot in the environment on the */cmd_vel* topic.  At the same time, the control node is subscibed to the */stage_ros* topic that provides the robot's distances from the wall. The *ui*  node, instead, handles inputs from the user and sends requests in order to let the *control* node to modify the robot velocity.

### Flowcharts
--------------------------------

For a more precise description of what the two nodes do you can consult the following flowcharts, created with [Lucidchart](https://www.lucidchart.com/pages/it/landing?utm_source=google&utm_medium=cpc&utm_campaign=_chart_it_allcountries_mixed_search_brand_bmm_&km_CPC_CampaignId=9589672283&km_CPC_AdGroupID=99331286392&km_CPC_Keyword=%2Blucidcharts&km_CPC_MatchType=b&km_CPC_ExtensionID=&km_CPC_Network=g&km_CPC_AdPosition=&km_CPC_Creative=424699413299&km_CPC_TargetID=kwd-334618660008&km_CPC_Country=1008337&km_CPC_Device=c&km_CPC_placement=&km_CPC_target=&mkwid=sKwFuAgHb_pcrid_424699413299_pkw_%2Blucidcharts_pmt_b_pdv_c_slid__pgrid_99331286392_ptaid_kwd-334618660008_&gclid=CjwKCAjw5c6LBhBdEiwAP9ejG86DblinG5ivYRvMmKSvI8Dl7as9i2oINlmgqIDoj0gpLX6WfnCenRoCxxQQAvD_BwE):

This is the controller node's flowchart:
<p align="center">
<img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos%2C%20gifs%20%20and%20images/controller_flowchart.jpeg" height=670 width=650>
</p>

This one, instead, is the UI node's flowchart:
<p align="center">
<img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos%2C%20gifs%20%20and%20images/ui_flowchart.jpeg" height=670 width=750>
</p>


I created also a global flowchart in order to have a more precise general idea of the project's implementation:

<p align="center">
<img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos%2C%20gifs%20%20and%20images/global_flowchart.jpeg" height=600 width=750>
</p>

### Results
--------------------------------

The final result is that the robot correctly runs around the circuit and, despite there are some things that could be improved in the future, I am satisfied with the work that I've done specially because that was my first approach with the ROS framework. 

In order to make you understand how my code works, I recorded this video:


https://user-images.githubusercontent.com/91267426/143897374-b2abea45-ce71-4e31-89b0-dc77842e4775.mp4






### Possible Improvements
--------------------------------
A possible improvement could be the usage of an algorithm such that it can map the entire environment from the beginning of the simulation and so that it can immediately detect all the x, y coordinates that cannot be reached. 
